#include <Arduino.h>
#include "defines.h"

/**
 * @initialize_pins - Initializes the device pins
 *
 */

void initialize_pins()
{

    Serial.begin(115200);

    pinMode(HC_SR04_TRIG, OUTPUT);
    pinMode(HC_SR04_ECHO, INPUT);
    pinMode(MQ_PIN, INPUT);
    digitalWrite(HC_SR04_TRIG, LOW);

    pinMode(LED_PIN_1, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    GPS_INIT();
}
