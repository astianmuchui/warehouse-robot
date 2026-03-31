#include <Arduino.h>
#include "defines.h"

HardwareSerial gpsSerial(2);

void GPS_INIT()
{
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
}

char READ_GPS()
{
    if (gpsSerial.available() <= 0)
        return '\0';

    return gpsSerial.read();
}
