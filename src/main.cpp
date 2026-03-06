#include <Arduino.h>
#include "defines.h"

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    Serial.println("Hello, Warehouse Robot!");

    ledcWrite(LED_PIN, 255);
    delay(1000);
    ledcWrite(LED_PIN, 0);
    delay(1000);
}