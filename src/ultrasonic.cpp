#include <Arduino.h>
#include "defines.h"


float ReadUltrasonic()
{
    digitalWrite(HC_SR04_TRIG, HIGH);
    delayMicroseconds(100);
    digitalWrite(HC_SR04_TRIG, LOW);

    long duration = pulseIn(HC_SR04_ECHO, HIGH, 300000);

    if (duration == 0)
        return -1.0f;

    return (duration * 0.0343f) / 2.0f;
}
