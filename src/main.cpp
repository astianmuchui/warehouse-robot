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

/**
 * i2c_scan - print every I2C address that ACKs. Run at boot before any device
 * init so the serial log shows whether the PCA9685/IMU/colour sensor are
 * reachable; "nothing moves" is usually the PCA9685 not answering.
 */
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
    InitBuzzer();          /* must precede any Pulsate/Beep */

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    i2c_scan();

    Beep(2);

    /* PCA9685 before MotorInit: the servos hang off it and it must be live. */
    InitServoDriver();
    MotorInit();

    /* Motor self-test, before the FreeRTOS motor tasks exist so nothing else
       drives the wheels. Duty is set directly (no ramp task yet). */
    Serial.println("[Motor] Self-test: forward 1s, backward 1s");
    MotorSetDirection(MOTOR_FORWARD);
    MotorSetDuty(MOTOR_SPEED_DEFAULT);
    delay(1000);
    MotorSetDirection(MOTOR_BACKWARD);
    delay(1000);
    MotorSetDirection(MOTOR_STOP);
    MotorSetDuty(0);
    Serial.println("[Motor] Self-test done");

    /* Servo-walk boot diagnostic: sweep each joint 60->120->90 so you can see
       all four respond. One at a time to keep 5V rail peak current down. */
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
            DisableServo(servowalk[i].ch);
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
    g_drive_cmd_queue = xQueueCreate(1, sizeof(motor_cmd_t)); /* MQTT -> patrol */
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
    /* Patrol task drains the MQTT drive queue, so it must run even with
       line-following off or drive commands never reach the motors. */
    xTaskCreatePinnedToCore(line_follow_task, "Patrol", 3072, NULL, 2, NULL, CPU1);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
