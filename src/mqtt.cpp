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

    Pulsate(BUZZER_PIN, 2, 300);
    Pulsate(LED_PIN_2, 2, 300);

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
            Pulsate(BUZZER_PIN, 4, 200);
            Pulsate(LED_PIN_2, 4, 200);
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            Pulsate(BUZZER_PIN, 8, 100);
            Pulsate(LED_PIN_3, 8, 100);
            delay(2000);
        }
    }

    client.publish(topic_publish, "Boot Notification");
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

class RobotMQTT
{
    private:
        PubSubClient mqtt_client ;
    public:
        RobotMQTT(char ssid[], char password[])
        {
            WiFi.begin(ssid, password);

            while (WiFi.status() != WL_CONNECTED)
            {
                delay(500);
                Serial.println("Connecting to WiFi..");
            }

            Serial.println("Connected to the Wi-Fi network");

            Pulsate(BUZZER_PIN, 2, 300);
            Pulsate(LED_PIN_2, 2, 300);

            Serial.println("Before Connect MQTT");
            client.setServer(mqtt_broker, mqtt_port);

            while (!client.connected())
            {
                String client_id = "warehouse-robot-";
                client_id = client_id + String(WiFi.macAddress());

                Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
                if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
                {
                    mqtt_client = client;
                    Serial.println("Public MQTT broker connected");
                    Pulsate(BUZZER_PIN, 4, 200);
                    Pulsate(LED_PIN_2, 4, 200);
                }
                else
                {
                    Serial.print("failed with state ");
                    Serial.print(client.state());
                    Pulsate(BUZZER_PIN, 8, 100);
                    Pulsate(LED_PIN_3, 8, 100);
                    delay(2000);
                }
            }
        };

};
