#include <Arduino.h>
#include "defines.h"
#include <TinyGPS++.h>

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

void GPS_INIT()
{
    Serial.println("Initializing GPS .......");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
}

gps_data_t READ_GPS()
{
    while (gpsSerial.available() > 0)
        gps.encode(gpsSerial.read());

    gps_data_t data = {};
    data.valid      = gps.location.isValid();
    data.latitude   = gps.location.lat();
    data.longitude  = gps.location.lng();
    data.altitude   = gps.altitude.meters();
    data.speed      = gps.speed.kmph();
    data.satellites = gps.satellites.value();
    return data;
}
