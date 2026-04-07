#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <ESP32Servo.h>

#include "defines.h"

// ─── Servo objects ────────────────────────────────────────────────────────────
Servo base, shoulder, elbow, gripper;

// ─── CPU affinity ─────────────────────────────────────────────────────────────
// CPU0 (PRO): WiFi/BT stack + sensor tasks
// CPU1 (APP): Network / MQTT / publish tasks + setup/loop
static const BaseType_t CPU0 = 0;
static const BaseType_t CPU1 = 1;

// ─── FreeRTOS queue handles ───────────────────────────────────────────────────
// Each sensor task overwrites its single-slot queue with the latest reading.
// publish_task peeks (non-destructively) every 30 s to build the JSON payload.
QueueHandle_t g_dht_queue;    // dht_data_t
QueueHandle_t g_imu_queue;    // imu_reading_t
QueueHandle_t g_sonic_queue;  // float  (distance_cm; -1 = invalid)
QueueHandle_t g_mq135_queue;  // mq135_data_t
QueueHandle_t g_gps_queue;    // gps_data_t
QueueHandle_t g_event_queue;  // imu_event_t  (depth 5 — event FIFO)

// ─── LED heartbeat task ───────────────────────────────────────────────────────
// Timer-driven blink on the built-in LED at 1 Hz (500 ms on / 500 ms off).
static SemaphoreHandle_t s_led_sem;
static void led_timer_cb(TimerHandle_t) { xSemaphoreGive(s_led_sem); }

static void led_task(void *)
{
    bool state = false;
    while (true) {
        xSemaphoreTake(s_led_sem, portMAX_DELAY);
        state = !state;
        digitalWrite(LED_PIN_1, state ? HIGH : LOW);
    }
}

// ─── setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // ── Servo PWM timer allocation ────────────────────────────────────────────
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    shoulder.attach(SHOULDER_SERVO, 500, 2400);
    elbow.attach(ELBOW_SERVO,       500, 2400);
    gripper.attach(GRIPPER_SERVO,   500, 2400);
    base.attach(BASE_SERVO,         500, 2400);
    shoulder.write(0); elbow.write(0); gripper.write(0); base.write(0);

    // ── GPIO, GPS UART, I/O expander ─────────────────────────────────────────
    initialize_pins();
    InitializeExpander();
    Pulsate(BUZZER_PIN, 2, 300);

    // ── Sensor initialisation ─────────────────────────────────────────────────
    InitializeDHT();
    InitializeIMU();
    ConfigureIMUEvents();   // arm motion / freefall / zero-motion interrupts

    // ── FreeRTOS queues ───────────────────────────────────────────────────────
    g_dht_queue   = xQueueCreate(1, sizeof(dht_data_t));
    g_imu_queue   = xQueueCreate(1, sizeof(imu_reading_t));
    g_sonic_queue = xQueueCreate(1, sizeof(float));
    g_mq135_queue = xQueueCreate(1, sizeof(mq135_data_t));
    g_gps_queue   = xQueueCreate(1, sizeof(gps_data_t));
    g_event_queue = xQueueCreate(5, sizeof(imu_event_t));

    // ── Sensor timers ─────────────────────────────────────────────────────────
    // Creates one FreeRTOS software timer + binary semaphore per sensor.
    // Timers fire periodically and give the semaphore; tasks block on it.
    init_sensor_timers();

    // ── LED timer ─────────────────────────────────────────────────────────────
    s_led_sem = xSemaphoreCreateBinary();
    xTimerStart(xTimerCreate("led_tmr", pdMS_TO_TICKS(500), pdTRUE, NULL, led_timer_cb), 0);

    // ── Network / MQTT tasks (CPU1) ───────────────────────────────────────────
    // Creates: network_init_task (prio 3), publish_task (prio 2), event_task (prio 2)
    MQTT_INITIALIZE();

    // ── Sensor tasks (CPU0) ───────────────────────────────────────────────────
    xTaskCreatePinnedToCore(led_task,        "LED",    2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(dht_task,        "DHT",    2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(imu_task,        "IMU",    4096, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(ultrasonic_task, "US",     2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(mq135_task,      "MQ135",  2048, NULL, 1, NULL, CPU0);
    xTaskCreatePinnedToCore(gps_task,        "GPS",    4096, NULL, 1, NULL, CPU0);
}

void loop()
{
    // All work is done inside FreeRTOS tasks. loop() just yields.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
