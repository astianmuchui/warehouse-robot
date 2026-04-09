

#include <Arduino.h>
#include <time.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>

#include "defines.h"

static SemaphoreHandle_t s_dht_sem;
static SemaphoreHandle_t s_imu_sem;
static SemaphoreHandle_t s_sonic_sem;
static SemaphoreHandle_t s_mq135_sem;
static SemaphoreHandle_t s_gps_sem;

static void dht_timer_cb(TimerHandle_t) { xSemaphoreGive(s_dht_sem); }
static void imu_timer_cb(TimerHandle_t) { xSemaphoreGive(s_imu_sem); }
static void sonic_timer_cb(TimerHandle_t) { xSemaphoreGive(s_sonic_sem); }
static void mq135_timer_cb(TimerHandle_t) { xSemaphoreGive(s_mq135_sem); }
static void gps_timer_cb(TimerHandle_t) { xSemaphoreGive(s_gps_sem); }

void init_sensor_timers()
{
    s_dht_sem = xSemaphoreCreateBinary();
    s_imu_sem = xSemaphoreCreateBinary();
    s_sonic_sem = xSemaphoreCreateBinary();
    s_mq135_sem = xSemaphoreCreateBinary();
    s_gps_sem = xSemaphoreCreateBinary();

    xTimerStart(xTimerCreate("dht", pdMS_TO_TICKS(2000), pdTRUE, NULL, dht_timer_cb), 0);
    xTimerStart(xTimerCreate("imu", pdMS_TO_TICKS(500), pdTRUE, NULL, imu_timer_cb), 0);
    xTimerStart(xTimerCreate("sonic", pdMS_TO_TICKS(1000), pdTRUE, NULL, sonic_timer_cb), 0);
    xTimerStart(xTimerCreate("mq135", pdMS_TO_TICKS(2000), pdTRUE, NULL, mq135_timer_cb), 0);
    xTimerStart(xTimerCreate("gps", pdMS_TO_TICKS(1000), pdTRUE, NULL, gps_timer_cb), 0);
}

void dht_task(void *)
{
    while (true)
    {
        xSemaphoreTake(s_dht_sem, portMAX_DELAY);

        dht_data_t d = ReadDHT();
        if (!isnan(d.temperature) && !isnan(d.humidity))
        {
            xQueueOverwrite(g_dht_queue, &d);
            Serial.printf("[DHT] %.1f°C  %.1f %%RH\n", d.temperature, d.humidity);
        }
        else
        {
            Serial.println("[DHT] Read failed (NaN) — sensor not ready?");
        }
        vTaskDelay(PUBLISH_INTERVAL_MS * 0.5 / portTICK_PERIOD_MS);
    }
}

void imu_task(void *)
{
    while (true)
    {
        xSemaphoreTake(s_imu_sem, portMAX_DELAY);

        imu_reading_t r = ReadIMU();
        xQueueOverwrite(g_imu_queue, &r);

        Serial.printf("[IMU] A(%.2f, %.2f, %.2f) m/s²  G(%.3f, %.3f, %.3f) rad/s  %.1f°C\n",
                      r.accel.x, r.accel.y, r.accel.z,
                      r.gyro.x, r.gyro.y, r.gyro.z,
                      r.temperature);

        imu_event_type_t ev_type = CheckIMUEvents();
        if (ev_type != IMU_EVENT_NONE)
        {
            imu_event_t ev;
            ev.type = ev_type;
            ev.timestamp_s = (uint32_t)time(nullptr);
            ev.uptime_s = millis() / 1000;
            ev.accel_x = r.accel.x;
            ev.accel_y = r.accel.y;
            ev.accel_z = r.accel.z;

            if (xQueueSend(g_event_queue, &ev, 0) == pdTRUE)
                Serial.printf("[IMU] Event type %d queued\n", (int)ev_type);
            else
                Serial.println("[IMU] Event queue full — event dropped");
        }
        vTaskDelay(PUBLISH_INTERVAL_MS * 0.5 / portTICK_PERIOD_MS);
    }
}

void ultrasonic_task(void *)
{
    while (true)
    {
        xSemaphoreTake(s_sonic_sem, portMAX_DELAY);

        float dist = ReadUltrasonic();
        xQueueOverwrite(g_sonic_queue, &dist);

        if (dist > 0.0f)
            Serial.printf("[US] %.1f cm\n", dist);
        else
            Serial.println("[US] No echo (out of range or timeout)");

        vTaskDelay(PUBLISH_INTERVAL_MS * 0.5 / portTICK_PERIOD_MS);
    }
}

void mq135_task(void *)
{
    Serial.println("[MQ135] Warm-up: waiting 2 minutes before first read...");
    vTaskDelay(pdMS_TO_TICKS(120000));
    Serial.println("[MQ135] Warm-up complete");

    while (true)
    {
        xSemaphoreTake(s_mq135_sem, portMAX_DELAY);

        mq135_data_t d = ReadMQ135();
        xQueueOverwrite(g_mq135_queue, &d);

        Serial.printf("[MQ135] %.1f ppm  (%.3f V)\n", d.ppm, d.voltage);
        vTaskDelay(PUBLISH_INTERVAL_MS * 0.5 / portTICK_PERIOD_MS);
    }
}

void gps_task(void *)
{
    while (true)
    {

        BaseType_t tick = xSemaphoreTake(s_gps_sem, pdMS_TO_TICKS(100));

        GPS_DRAIN();

        if (tick == pdTRUE)
        {
            gps_data_t d = READ_GPS();
            xQueueOverwrite(g_gps_queue, &d);

            if (d.valid)
                Serial.printf("[GPS] %.6f, %.6f  alt=%.1f m  %.1f km/h  sats=%u\n",
                              d.latitude, d.longitude,
                              d.altitude, d.speed, d.satellites);
            else
                Serial.println("[GPS] Waiting for fix...");

            vTaskDelay(PUBLISH_INTERVAL_MS * 0.5 / portTICK_PERIOD_MS);
        }
    }
}
