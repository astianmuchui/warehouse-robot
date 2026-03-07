#include <WiFi.h>
#include "PubSubClient.h"
#include "defines.h"

char ssid[] = "undeniable";
char pass[] = "$ArcherOfHalcyon62";

const char *mqtt_broker = "hustlemania-home-1.local";
const char *mqtt_username = "astian";
const char *mqtt_password = "muchui";
const int mqtt_port = 1883;

const char *topic_publish = "robot/data";
const char *topic_subsrcibe = "robot/cmd";

WiFiClient wifi_client;
PubSubClient client(wifi_client);

void MQTT_INITIALIZE()
{
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }

    Serial.println("Connected to the Wi-Fi network");
    Serial.println("Before Connect MQTT");
    client.setServer(mqtt_broker, mqtt_port);

    while (!client.connected())
    {
        String client_id = "warehouse-robot-";
        client_id = client_id + String(WiFi.macAddress());

        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
        {
            Serial.println("Public MQTT broker connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    Serial.println("After Connect MQTT");
    client.publish(topic_publish, "Hi, I'm ESP32 ^^");
    client.subscribe(topic_subsrcibe);
    client.setCallback(callback);
}

void callback(char *topic, byte *message, unsigned int length)
{
    printPayload(topic, message, length);
}

void printPayload(char *topic, byte *message, unsigned int length)
{

    Serial.print("Message received on topic: ");
    Serial.println(topic);

    Serial.print("Message: ");
    for (unsigned int i = 0; i < length; i++)
    {
        Serial.print((char)message[i]);
    }
    Serial.println();
}