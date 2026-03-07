#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "defines.h"

Adafruit_MPU6050 mpu;

void InitializeIMU()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Initialized!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

gyro_data_t ReadGyro()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro_data_t data;
  data.x = g.gyro.x;
  data.y = g.gyro.y;
  data.z = g.gyro.z;

  return data;
}

accel_data_t ReadAccel()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel_data_t data;
  data.x = a.acceleration.x;
  data.y = a.acceleration.y;
  data.z = a.acceleration.z;

  return data;
}

double ReadIMUTemp()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  return temp.temperature;
}