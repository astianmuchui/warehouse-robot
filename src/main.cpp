#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "defines.h"

static const BaseType_t CPU0 = 0;
static const BaseType_t CPU1 = 1;

QueueHandle_t g_dht_queue;
QueueHandle_t g_imu_queue;
QueueHandle_t g_sonic_queue;
QueueHandle_t g_mq135_queue;
QueueHandle_t g_gps_queue;
QueueHandle_t g_event_queue;
QueueHandle_t g_motor_cmd_queue;
QueueHandle_t g_drive_cmd_queue;
QueueHandle_t g_servo_cmd_queue;
QueueHandle_t g_pose_cmd_queue;
QueueHandle_t g_color_queue;

static SemaphoreHandle_t s_led_sem;
static void led_timer_cb(TimerHandle_t) { xSemaphoreGive(s_led_sem); }

/* Scan the I2C bus and print every address that ACKs.  Run once at boot before
   any I2C device is initialised, so we can tell from the serial log whether the
   PCA9685 (0x40), IMU (0x68) and colour sensor (0x29) are actually reachable
   — the usual reason "nothing moves" is the PCA9685 simply not answering. */
static void i2c_scan()
{
    Serial.println("[I2C] Scanning bus...");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            const char *name = "";
            if (addr == PCA9685_ADDR) name = " (PCA9685)";
            else if (addr == 0x68)    name = " (IMU)";
            else if (addr == TCS34725_ADDR) name = " (TCS34725 colour)";
            Serial.printf("[I2C]   found device at 0x%02X%s\n", addr, name);
            found++;
        }
    }
    if (found == 0)
        Serial.println("[I2C] No devices found — check SDA/SCL wiring, pull-ups and power.");
    else
        Serial.printf("[I2C] Scan done — %u device(s).\n", found);
}

static void led_task(void *)
{
    bool state = false;
    while (true)
    {
        xSemaphoreTake(s_led_sem, portMAX_DELAY);
        state = !state;
        digitalWrite(LED_PIN_1, state ? HIGH : LOW);
    }
}

void setup()
{
    Serial.begin(115200);

    initialize_pins();
    InitBuzzer();          /* quiet PWM buzzer — must precede any Pulsate/Beep */
    /* PCF8574 I/O expander disabled — no longer fitted (all I/O moved to PCA9685) */

    /* Bring up I2C explicitly (with the correct ESP32 pins) before any device
       init, then scan so the serial log shows what's actually on the bus. */
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    i2c_scan();

    /* Soft boot chirp (was three loud 80/60/40 ms blasts) */
    Beep(2);

    /* PCA9685 first — both the L298N control lines and the servos hang off it,
       so MotorInit() needs a live PWM driver to write zeros to. */
    InitServoDriver();
    MotorInit();

    /* Boot motor self-test: forward 1 s, backward 1 s, then stop.  Runs here
       before the FreeRTOS motor tasks start, so nothing else is driving the
       motors.  Duty is set directly (the ramping motor_speed_task isn't up yet)
       at the default cruise speed. */
    Serial.println("[Motor] Self-test: forward 1s, backward 1s");
    MotorSetDirection(MOTOR_FORWARD);
    MotorSetDuty(MOTOR_SPEED_DEFAULT);
    delay(1000);
    MotorSetDirection(MOTOR_BACKWARD);
    delay(1000);
    MotorSetDirection(MOTOR_STOP);
    MotorSetDuty(0);
    Serial.println("[Motor] Self-test done");

    /* ── Servo-sweep boot diagnostic (servo walk) ────────────────────────────
       Walk each arm servo through its range once at boot so you can confirm all
       four joints respond and the new 1.0–2.0 ms pulse calibration is correct.
       Runs before the FreeRTOS tasks start, so nothing else is driving the bus.
       Each joint sweeps 60→120→90 then releases. Servos are driven one at a time
       to keep the 5 V rail's peak current down. */
    {
        const struct { uint8_t ch; const char *name; } servowalk[] = {
            { SERVO_CH_BASE,     "base"     },
            { SERVO_CH_SHOULDER, "shoulder" },
            { SERVO_CH_ELBOW,    "elbow"    },
            { SERVO_CH_GRIPPER,  "gripper"  },
        };
        Serial.println("[ServoWalk] Sweeping each joint 60→120→90");
        for (uint8_t i = 0; i < 4; i++) {
            Serial.printf("[ServoWalk] %s\n", servowalk[i].name);
            SetServoAngle(servowalk[i].ch, 60);
            delay(SERVO_SETTLE_MS);
            SetServoAngle(servowalk[i].ch, 120);
            delay(SERVO_SETTLE_MS);
            SetServoAngle(servowalk[i].ch, 90);
            delay(SERVO_SETTLE_MS);
            DisableServo(servowalk[i].ch);   /* release before the next joint */
            delay(150);
        }
        Serial.println("[ServoWalk] Done — all four joints should have moved");
    }

    InitializeDHT();
    InitializeIMU();
    ConfigureIMUEvents();
    InitializeColorSensor();   /* optional — robot runs fine if absent */

    g_dht_queue       = xQueueCreate(1, sizeof(dht_data_t));
    g_imu_queue       = xQueueCreate(1, sizeof(imu_reading_t));
    g_sonic_queue     = xQueueCreate(1, sizeof(float));
    g_mq135_queue     = xQueueCreate(1, sizeof(mq135_data_t));
    g_gps_queue       = xQueueCreate(1, sizeof(gps_data_t));
    g_event_queue     = xQueueCreate(5, sizeof(imu_event_t));
    g_motor_cmd_queue = xQueueCreate(1, sizeof(motor_cmd_t));
    g_drive_cmd_queue = xQueueCreate(1, sizeof(motor_cmd_t)); /* MQTT → patrol */
    g_servo_cmd_queue = xQueueCreate(4, sizeof(servo_cmd_t));
    g_pose_cmd_queue  = xQueueCreate(2, sizeof(arm_pose_cmd_t));
    g_color_queue     = xQueueCreate(1, sizeof(color_data_t));

    init_sensor_timers();

    s_led_sem = xSemaphoreCreateBinary();
    xTimerStart(xTimerCreate("led_tmr", pdMS_TO_TICKS(500), pdTRUE, NULL, led_timer_cb), 0);

    MQTT_INITIALIZE();

    xTaskCreatePinnedToCore(led_task,        "LED",      2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(dht_task,        "DHT",      2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(imu_task,        "IMU",      4096, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(ultrasonic_task, "US",       2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(mq135_task,      "MQ135",    2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(motor_cmd_task,   "MotorCmd",  2048, NULL, 2, NULL, CPU0);
    xTaskCreatePinnedToCore(motor_speed_task, "MotorSpd",  2048, NULL, 3, NULL, CPU0);
    xTaskCreatePinnedToCore(servo_cmd_task,   "ServoCmd",  3072, NULL, 2, NULL, CPU0);
    xTaskCreatePinnedToCore(pose_cmd_task,    "PoseCmd",   4096, NULL, 2, NULL, CPU0);
    xTaskCreatePinnedToCore(color_task,       "Color",     3072, NULL, 1, NULL, CPU0);
    /* Line following + obstacle avoidance. Higher priority than the motor_cmd
       task it feeds so its steering decisions aren't starved; it drives only
       through g_motor_cmd_queue, so motor_cmd_task remains the sole motor owner. */
    xTaskCreatePinnedToCore(line_follow_task, "LineFollow", 3072, NULL, 2, NULL, CPU1);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
