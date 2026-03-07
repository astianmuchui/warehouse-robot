#include <Arduino.h>
#include "defines.h"

mq135_data_t *ReadMQ135()
{
    float air_quality = analogRead(MQ_PIN);
    float voltage = air_quality * (5.0 / 1023.0);

    float ppm = 116.6020682 * pow(voltage, -2.769034857);

    mq135_data_t data;
    data.ppm = ppm;
    data.voltage = voltage;

    return &data;
}

