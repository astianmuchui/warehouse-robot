#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "defines.h"

Adafruit_MPU6050 mpu;

// ─── MPU6050 Register Map (advanced interrupt features) ──────────────────────
// The Adafruit library does not expose free-fall or zero-motion detection, so
// we write the relevant registers directly via Wire after mpu.begin().
#define MPU6050_ADDR        0x68
#define MPU6050_FF_THR      0x1D   // Free-fall threshold (1 LSB = ~1 mg)
#define MPU6050_FF_DUR      0x1E   // Free-fall duration  (1 LSB = 1 ms)
#define MPU6050_MOT_THR     0x1F   // Motion threshold    (1 LSB = 32 mg @ ±2g)
#define MPU6050_MOT_DUR     0x20   // Motion duration     (1 LSB = 1 ms)
#define MPU6050_ZRMOT_THR   0x21   // Zero-motion threshold
#define MPU6050_ZRMOT_DUR   0x22   // Zero-motion duration
#define MPU6050_INT_PIN_CFG 0x37   // Interrupt pin config
#define MPU6050_INT_ENABLE  0x38   // Interrupt enable mask
#define MPU6050_INT_STATUS  0x3A   // Interrupt status — reading this clears all flags

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

// ─── InitializeIMU ────────────────────────────────────────────────────────────
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

// ─── ConfigureIMUEvents ───────────────────────────────────────────────────────
// Must be called AFTER InitializeIMU(). Configures three interrupt sources:
//
//  Motion / collision
//    MOT_THR = 5  → 5 × 32 mg = 160 mg threshold (lower = more sensitive)
//    MOT_DUR = 1  → must exceed threshold for 1 ms
//
//  Free-fall (drop / launch)
//    FF_THR  = 17 → 17 mg (total-accel must be BELOW this for FF_DUR)
//    FF_DUR  = 100 → 100 consecutive ms to confirm free-fall
//
//  Zero-motion (robot parked / stationary)
//    ZRMOT_THR = 4, ZRMOT_DUR = 4
//
// INT_STATUS (0x3A) is READ-CLEAR: a single readReg() call atomically reads and
// clears all flags, so CheckIMUEvents() is safe to call from the IMU task loop.
void ConfigureIMUEvents()
{
    writeReg(MPU6050_MOT_THR,   5);
    writeReg(MPU6050_MOT_DUR,   1);

    writeReg(MPU6050_FF_THR,    17);
    writeReg(MPU6050_FF_DUR,    100);

    writeReg(MPU6050_ZRMOT_THR, 4);
    writeReg(MPU6050_ZRMOT_DUR, 4);

    // INT pin: active-high, push-pull, 50 µs pulse, read-clear on status read
    writeReg(MPU6050_INT_PIN_CFG, 0x00);

    // Enable: FF (bit7) | MOT (bit6) | ZMOT (bit5)
    writeReg(MPU6050_INT_ENABLE, (1 << 7) | (1 << 6) | (1 << 5));

    Serial.println("[IMU] Motion / freefall / zero-motion detection armed");
}

// ─── CheckIMUEvents ───────────────────────────────────────────────────────────
// Polls INT_STATUS and returns the highest-priority pending event (or NONE).
// Reading INT_STATUS clears all flags atomically (no separate clear needed).
imu_event_type_t CheckIMUEvents()
{
    uint8_t status = readReg(MPU6050_INT_STATUS);
    if (status & (1 << 7)) return IMU_EVENT_FREEFALL;
    if (status & (1 << 6)) return IMU_EVENT_MOTION;
    if (status & (1 << 5)) return IMU_EVENT_ZERO_MOTION;
    return IMU_EVENT_NONE;
}

// ─── ReadIMU ─────────────────────────────────────────────────────────────────
// Single getEvent() call populates accel, gyro, and die temperature together.
// Prefer this over calling ReadAccel() + ReadGyro() + ReadIMUTemp() separately.
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

// ─── Legacy single-field accessors (kept for compatibility) ──────────────────
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
