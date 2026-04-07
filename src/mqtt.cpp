#include <WiFi.h>
#include <time.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include "PubSubClient.h"
#include "defines.h"
#include "mqtt.h"

static Network g_net(Transport::WIFI);

static SemaphoreHandle_t g_publish_sem;
static EventGroupHandle_t g_net_events;
#define NET_READY_BIT (EventBits_t)(1 << 0)

Network::Network(Transport t) : _transport(t), _mqtt(_wifi_client)
{
    _mqtt.setBufferSize(1024);
}

bool Network::begin(const char *ssid_or_apn, const char *password)
{
    return wifi_connect(ssid_or_apn, password);
}

bool Network::wifi_connect(const char *ssid, const char *password)
{
    Serial.printf("[Net] Connecting to WiFi: %s\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    const uint32_t timeout_ms = 20000;
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - start > timeout_ms)
        {
            Serial.println("[Net] WiFi timeout");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.printf("[Net] WiFi OK — IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    configTime(0, 0, "pool.ntp.org", "time.google.com");
    Serial.print("[Net] NTP sync");
    time_t now = 0;
    for (uint8_t tries = 0; now < 1000000000UL && tries < 20; ++tries)
    {
        time(&now);
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print('.');
    }
    if (now >= 1000000000UL)
        Serial.printf("\n[Net] NTP synced: %lu\n", (unsigned long)now);
    else
        Serial.println("\n[Net] NTP sync failed — timestamps will show 0");

    return true;
}

bool Network::connect_mqtt(const char *broker, uint16_t port,
                           const char *client_id,
                           const char *user, const char *pass)
{
    _broker = broker;
    _port = port;
    _client_id = client_id;
    _user = (user && user[0]) ? user : nullptr;
    _pass = (pass && pass[0]) ? pass : nullptr;

    _mqtt.setServer(broker, port);
    return reconnect_mqtt();
}

bool Network::reconnect_mqtt()
{
    if (_mqtt.connected())
        return true;

    Serial.printf("[MQTT] Connecting to %s:%u as %s\n", _broker, _port, _client_id);

    bool ok = (_user)
                  ? _mqtt.connect(_client_id, _user, _pass)
                  : _mqtt.connect(_client_id);

    if (ok)
    {
        Serial.println("[MQTT] Connected");
        _mqtt.subscribe(TOPIC_CMD);
    }
    else
    {
        Serial.printf("[MQTT] Failed (state=%d)\n", _mqtt.state());
    }
    return ok;
}

bool Network::publish(const char *topic, const char *payload)
{
    if (!_mqtt.connected() && !reconnect_mqtt())
        return false;
    bool ok = _mqtt.publish(topic, payload, /*retained=*/false);
    if (!ok)
        Serial.printf("[MQTT] Publish failed on %s\n", topic);
    return ok;
}

bool Network::subscribe(const char *topic)
{
    if (!_mqtt.connected() && !reconnect_mqtt())
        return false;
    return _mqtt.subscribe(topic);
}

void Network::set_callback(void (*cb)(char *, uint8_t *, unsigned int))
{
    _mqtt.setCallback(cb);
}

void Network::loop()
{
    if (!_mqtt.connected())
        reconnect_mqtt();
    _mqtt.loop();
}

int Network::signal_strength() { return WiFi.RSSI(); }
bool Network::is_mqtt_connected() { return _mqtt.connected(); }
bool Network::is_net_connected() { return WiFi.isConnected(); }
const char *Network::transport_name() const { return "wifi"; }

void printPayload(char *topic, byte *message, unsigned int length)
{
    Serial.printf("[CMD] %s → ", topic);
    for (unsigned int i = 0; i < length; i++)
        Serial.print((char)message[i]);
    Serial.println();
}

void callback(char *topic, byte *message, unsigned int length)
{
    printPayload(topic, message, length);
}

static uint32_t unix_ts()
{
    time_t t = time(nullptr);
    return (t >= 1000000000UL) ? (uint32_t)t : 0;
}
static uint32_t uptime_s() { return millis() / 1000; }

static void on_publish_timer(TimerHandle_t) { xSemaphoreGive(g_publish_sem); }

static void network_init_task(void *)
{

    g_net.set_callback(callback);

    bool radio_ok = g_net.begin(WIFI_SSID, WIFI_PASSWORD);
    if (!radio_ok)
        Serial.println("[Net] Radio init failed");

    const char *user = MQTT_USER[0] ? MQTT_USER : nullptr;
    const char *pass = MQTT_PASS[0] ? MQTT_PASS : nullptr;
    g_net.connect_mqtt(MQTT_BROKER, MQTT_PORT, MQTT_CLIENT_ID, user, pass);
    g_net.subscribe(TOPIC_CMD);

    {
        JsonDocument doc;
        doc["device_id"] = DEVICE_ID;
        doc["event"] = "boot";
        doc["timestamp"] = (long long)unix_ts();
        doc["firmware"] = "1.0.0";
        doc["transport"] = g_net.transport_name();
        doc["ip"] = WiFi.localIP().toString();
        doc["rssi"] = g_net.signal_strength();
        doc["uptime_ms"] = (long long)millis();

        char buf[512];
        size_t len = serializeJson(doc, buf, sizeof(buf));
        if (len > 0 && len < sizeof(buf))
        {
            g_net.publish(TOPIC_BOOT, buf);
            Serial.printf("[Boot] %s\n", buf);
        }
    }

    TimerHandle_t timer = xTimerCreate("pub_tmr",
                                       pdMS_TO_TICKS(PUBLISH_INTERVAL_MS),
                                       pdTRUE, nullptr, on_publish_timer);
    if (timer)
        xTimerStart(timer, 0);
    else
        Serial.println("[Timer] Failed to create publish timer!");

    Serial.printf("[Net] Ready — publishing every %d s\n", PUBLISH_INTERVAL_MS / 1000);

    xEventGroupSetBits(g_net_events, NET_READY_BIT);

    vTaskDelete(nullptr);
}

static void publish_task(void *)
{

    xEventGroupWaitBits(g_net_events, NET_READY_BIT,
                        pdFALSE,
                        pdTRUE,
                        portMAX_DELAY);

    char buf[1024];

    while (true)
    {

        bool publish_now =
            (xSemaphoreTake(g_publish_sem, pdMS_TO_TICKS(5000)) == pdTRUE);

        g_net.loop();

        if (!publish_now)
            continue;

        dht_data_t dht = {};
        imu_reading_t imu = {};
        float dist = -1.0f;
        mq135_data_t mq135 = {};
        gps_data_t gps = {};

        xQueuePeek(g_dht_queue, &dht, 0);
        xQueuePeek(g_imu_queue, &imu, 0);
        xQueuePeek(g_sonic_queue, &dist, 0);
        xQueuePeek(g_mq135_queue, &mq135, 0);
        xQueuePeek(g_gps_queue, &gps, 0);

        JsonDocument doc;
        doc["device_id"] = DEVICE_ID;
        doc["timestamp"] = (long long)unix_ts();
        doc["uptime_s"] = (long long)uptime_s();

        JsonObject env = doc["environment"].to<JsonObject>();
        env["temperature_c"] = dht.temperature;
        env["humidity_pct"] = dht.humidity;
        env["air_quality_ppm"] = mq135.ppm;
        env["air_quality_v"] = mq135.voltage;

        JsonObject imu_obj = doc["imu"].to<JsonObject>();
        imu_obj["accel_x"] = imu.accel.x;
        imu_obj["accel_y"] = imu.accel.y;
        imu_obj["accel_z"] = imu.accel.z;
        imu_obj["gyro_x"] = imu.gyro.x;
        imu_obj["gyro_y"] = imu.gyro.y;
        imu_obj["gyro_z"] = imu.gyro.z;
        imu_obj["temperature_c"] = imu.temperature;

        JsonObject prox = doc["proximity"].to<JsonObject>();
        prox["distance_cm"] = dist;
        prox["valid"] = (dist > 0.0f);

        JsonObject loc = doc["location"].to<JsonObject>();
        loc["lat"] = gps.latitude;
        loc["lon"] = gps.longitude;
        loc["altitude_m"] = gps.altitude;
        loc["speed_kmph"] = gps.speed;
        loc["satellites"] = gps.satellites;
        loc["valid"] = gps.valid;

        JsonObject sys = doc["system"].to<JsonObject>();
        sys["rssi"] = g_net.signal_strength();
        sys["transport"] = g_net.transport_name();

        size_t len = serializeJson(doc, buf, sizeof(buf));
        if (len == 0 || len >= sizeof(buf))
        {
            Serial.println("[Publish] JSON overflow — skipping");
            continue;
        }

        bool ok = g_net.publish(TOPIC_READINGS, buf);
        Serial.printf("[Publish] readings → %s\n", ok ? "OK" : "FAIL");

        JsonDocument hb;
        hb["device_id"] = DEVICE_ID;
        hb["timestamp"] = (long long)unix_ts();
        hb["uptime_s"] = (long long)uptime_s();
        hb["rssi"] = g_net.signal_strength();

        char hb_buf[256];
        serializeJson(hb, hb_buf, sizeof(hb_buf));
        g_net.publish(TOPIC_HEARTBEAT, hb_buf);
    }
}

static void event_task(void *)
{
    xEventGroupWaitBits(g_net_events, NET_READY_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    char buf[512];

    while (true)
    {
        imu_event_t ev;

        if (xQueueReceive(g_event_queue, &ev, portMAX_DELAY) != pdTRUE)
            continue;

        const char *event_name = nullptr;
        const char *topic = nullptr;

        switch (ev.type)
        {
        case IMU_EVENT_MOTION:
            event_name = "motion";
            topic = TOPIC_EVENT_MOTION;
            break;
        case IMU_EVENT_FREEFALL:
            event_name = "freefall";
            topic = TOPIC_EVENT_FREEFALL;
            break;
        case IMU_EVENT_ZERO_MOTION:
            event_name = "zero_motion";
            topic = TOPIC_EVENT_ZERO_MOTION;
            break;
        default:
            continue;
        }

        JsonDocument doc;
        doc["device_id"] = DEVICE_ID;
        doc["event"] = event_name;
        doc["timestamp"] = (long long)ev.timestamp_s;
        doc["uptime_s"] = (long long)ev.uptime_s;
        doc["accel_x"] = ev.accel_x;
        doc["accel_y"] = ev.accel_y;
        doc["accel_z"] = ev.accel_z;

        size_t len = serializeJson(doc, buf, sizeof(buf));
        if (len > 0 && len < sizeof(buf))
        {
            bool ok = g_net.publish(topic, buf);
            Serial.printf("[Event] %s → %s\n", event_name, ok ? "OK" : "FAIL");
        }

        g_net.loop();
    }
}

void MQTT_INITIALIZE()
{
    g_publish_sem = xSemaphoreCreateBinary();
    g_net_events = xEventGroupCreate();

    xTaskCreatePinnedToCore(publish_task, "Publish", 8192, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(event_task, "Events", 4096, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(network_init_task, "NetInit", 8192, nullptr, 3, nullptr, 1);
}
