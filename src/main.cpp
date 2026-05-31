#include <Arduino.h>
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
QueueHandle_t g_servo_cmd_queue;
QueueHandle_t g_pose_cmd_queue;
QueueHandle_t g_color_queue;

static SemaphoreHandle_t s_led_sem;
static void led_timer_cb(TimerHandle_t) { xSemaphoreGive(s_led_sem); }

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
    InitializeExpander();

    /* Soft boot chirp (was three loud 80/60/40 ms blasts) */
    Beep(2);

    /* PCA9685 first — both the L298N control lines and the servos hang off it,
       so MotorInit() needs a live PWM driver to write zeros to. */
    InitServoDriver();
    MotorInit();

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
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
