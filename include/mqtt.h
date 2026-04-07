#ifndef MQTT_H
#define MQTT_H

#pragma once

#include <Arduino.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

enum class Transport {
    WIFI,
    GSM
};

class Network {
public:
    explicit Network(Transport t);

    bool begin(const char *ssid_or_apn, const char *password = nullptr);

    bool connect_mqtt(const char *broker, uint16_t port,
                      const char *client_id,
                      const char *user = nullptr,
                      const char *pass = nullptr);

    bool publish(const char *topic, const char *payload);
    bool subscribe(const char *topic);
    void set_callback(void (*cb)(char *, uint8_t *, unsigned int));

    void loop();

    bool        is_mqtt_connected();
    bool        is_net_connected();
    int         signal_strength();
    const char *transport_name() const;

private:
    Transport    _transport;
    WiFiClient   _wifi_client;
    PubSubClient _mqtt;

    const char *_broker    = nullptr;
    uint16_t    _port      = 1883;
    const char *_client_id = nullptr;
    const char *_user      = nullptr;
    const char *_pass      = nullptr;

    bool wifi_connect(const char *ssid, const char *password);
    bool reconnect_mqtt();
};

#endif /* MQTT_H */
