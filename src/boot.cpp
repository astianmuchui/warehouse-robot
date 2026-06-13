#include <Arduino.h>
#include <Wire.h>
#include "defines.h"

/** initialize_pins - set up serial, I2C, and the GPIO directions. */
void initialize_pins()
{
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    pinMode(HC_SR04_TRIG, OUTPUT);
    pinMode(HC_SR04_ECHO, INPUT);
    pinMode(MQ_PIN, INPUT);
    digitalWrite(HC_SR04_TRIG, LOW);

    pinMode(LED_PIN_1, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    GPS_INIT();
}
