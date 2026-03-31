#include <Arduino.h>
#include "defines.h"

HardwareSerial gpsSerial(2);

void GPS_INIT()
{
    Serial.println("Initializing GPS .......");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
}

void READ_GPS()
{
    unsigned long startTime = millis();
    while (millis() - startTime < 10000 && gpsSerial.available() > 0)
    {

        char gpsData = gpsSerial.read();
        Serial.print(gpsData);
    }
}
