#include <Arduino.h>
#include "defines.h"

/**
 * @initialize_pins - Initializes the device pins
 *
 */

void initialize_pins()
{

    Serial.begin(115200);
    // Ultrasonic sensor pins
    pinMode(HC_SR04_TRIG, OUTPUT);
    pinMode(HC_SR04_ECHO, INPUT);
    digitalWrite(HC_SR04_TRIG, LOW);

    // DHT11 Pin

    // pinMode(BUZZER_PIN, OUTPUT);

    pinMode(LED_PIN_1, OUTPUT);
}

