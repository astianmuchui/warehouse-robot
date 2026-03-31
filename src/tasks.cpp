#include <Arduino.h>
#include "defines.h"

void SensorTask(void *ptr)
{
    while (1)
    {
        dht_data_t dht_data;
        accel_data_t accel_data;
        gyro_data_t gyro_data;
        char gps_data;


        Serial.println("Reading DHT11");
        dht_data = ReadDHT();

        Serial.printf("dht data: temp=%.2f humidity=%.2f\n", dht_data.temperature, dht_data.humidity);

        vTaskDelay(1500 / portTICK_PERIOD_MS);

        Serial.println("Reading IMU");
        gyro_data = ReadGyro();
        accel_data = ReadAccel();

        Serial.printf("accel data: x=%.2f y=%.2f z=%.2f\n", accel_data.x, accel_data.y, accel_data.z);
        Serial.printf("gyro data: x=%.2f y=%.2f z=%.2f\n", gyro_data.x, gyro_data.y, gyro_data.z);

        float distance = ReadUltrasonic();
        if (distance < 0)
            Serial.println("ultrasonic: timeout");
        else
            Serial.printf("ultrasonic: %.2f cm\n", distance);

        vTaskDelay(500 / portTICK_PERIOD_MS);

        gps_data = READ_GPS();
        Serial.printf("gps data: %c\n", gps_data);
        vTaskDelay(1500 / portTICK_PERIOD_MS);

    }
}