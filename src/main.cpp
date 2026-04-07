#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <ESP32Servo.h>

#include "defines.h"

Servo base, shoulder, elbow, gripper;

static const BaseType_t CPU0 = 0;
static const BaseType_t CPU1 = 1;

QueueHandle_t g_dht_queue;
QueueHandle_t g_imu_queue;
QueueHandle_t g_sonic_queue;
QueueHandle_t g_mq135_queue;
QueueHandle_t g_gps_queue;
QueueHandle_t g_event_queue;

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

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    shoulder.attach(SHOULDER_SERVO, 500, 2400);
    elbow.attach(ELBOW_SERVO, 500, 2400);
    gripper.attach(GRIPPER_SERVO, 500, 2400);
    base.attach(BASE_SERVO, 500, 2400);
    shoulder.write(0);
    elbow.write(0);
    gripper.write(0);
    base.write(0);

    initialize_pins();
    InitializeExpander();
    Pulsate(BUZZER_PIN, 2, 300);

    InitializeDHT();
    InitializeIMU();
    ConfigureIMUEvents();

    g_dht_queue = xQueueCreate(1, sizeof(dht_data_t));
    g_imu_queue = xQueueCreate(1, sizeof(imu_reading_t));
    g_sonic_queue = xQueueCreate(1, sizeof(float));
    g_mq135_queue = xQueueCreate(1, sizeof(mq135_data_t));
    g_gps_queue = xQueueCreate(1, sizeof(gps_data_t));
    g_event_queue = xQueueCreate(5, sizeof(imu_event_t));

    init_sensor_timers();

    s_led_sem = xSemaphoreCreateBinary();
    xTimerStart(xTimerCreate("led_tmr", pdMS_TO_TICKS(500), pdTRUE, NULL, led_timer_cb), 0);

    MQTT_INITIALIZE();

    xTaskCreatePinnedToCore(led_task, "LED", 2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(dht_task, "DHT", 2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(imu_task, "IMU", 4096, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(ultrasonic_task, "US", 2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(mq135_task, "MQ135", 2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(gps_task, "GPS", 4096, NULL, 1, NULL, CPU0);
}

void loop()
{

    vTaskDelay(pdMS_TO_TICKS(1000));
}
