#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "defines.h"

Adafruit_MPU6050 mpu;




#define MPU6050_ADDR        0x68
#define MPU6050_FF_THR      0x1D   
#define MPU6050_FF_DUR      0x1E   
#define MPU6050_MOT_THR     0x1F   
#define MPU6050_MOT_DUR     0x20   
#define MPU6050_ZRMOT_THR   0x21   
#define MPU6050_ZRMOT_DUR   0x22   
#define MPU6050_INT_PIN_CFG 0x37   
#define MPU6050_INT_ENABLE  0x38   
#define MPU6050_INT_STATUS  0x3A   

static void writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t readReg(uint8_t reg)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}


void InitializeIMU()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!mpu.begin()) {
        Serial.println("[IMU] MPU6050 not found — check wiring (SDA=21, SCL=22)");
        while (1) vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.println("[IMU] MPU6050 OK");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

















void ConfigureIMUEvents()
{
    writeReg(MPU6050_MOT_THR,   5);
    writeReg(MPU6050_MOT_DUR,   1);

    writeReg(MPU6050_FF_THR,    17);
    writeReg(MPU6050_FF_DUR,    100);

    writeReg(MPU6050_ZRMOT_THR, 4);
    writeReg(MPU6050_ZRMOT_DUR, 4);

    
    writeReg(MPU6050_INT_PIN_CFG, 0x00);

    
    writeReg(MPU6050_INT_ENABLE, (1 << 7) | (1 << 6) | (1 << 5));

    Serial.println("[IMU] Motion / freefall / zero-motion detection armed");
}




imu_event_type_t CheckIMUEvents()
{
    uint8_t status = readReg(MPU6050_INT_STATUS);
    if (status & (1 << 7)) return IMU_EVENT_FREEFALL;
    if (status & (1 << 6)) return IMU_EVENT_MOTION;
    if (status & (1 << 5)) return IMU_EVENT_ZERO_MOTION;
    return IMU_EVENT_NONE;
}




imu_reading_t ReadIMU()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    imu_reading_t r;
    r.accel.x    = a.acceleration.x;
    r.accel.y    = a.acceleration.y;
    r.accel.z    = a.acceleration.z;
    r.gyro.x     = g.gyro.x;
    r.gyro.y     = g.gyro.y;
    r.gyro.z     = g.gyro.z;
    r.temperature = temp.temperature;
    return r;
}


accel_data_t ReadAccel()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accel_data_t d;
    d.x = a.acceleration.x;
    d.y = a.acceleration.y;
    d.z = a.acceleration.z;
    return d;
}

gyro_data_t ReadGyro()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_data_t d;
    d.x = g.gyro.x;
    d.y = g.gyro.y;
    d.z = g.gyro.z;
    return d;
}

double ReadIMUTemp()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return temp.temperature;
}
