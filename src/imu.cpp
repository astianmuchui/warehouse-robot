#include <Arduino.h>
#include <Wire.h>
#include "defines.h"

/* ── MPU6050 / MPU6500 driver (raw register access) ──────────────────────────
 *  We talk to the IMU directly over I2C rather than via Adafruit_MPU6050.  The
 *  Adafruit library's begin() rejects any chip whose WHO_AM_I != 0x68, so it
 *  refuses to initialise an MPU6500/MPU9250 (which report 0x70) even though the
 *  register map is otherwise compatible.  The board on this robot is a 6500-class
 *  part, so we use the proven raw burst-read sequence instead.
 *
 *  Downstream consumers (tasks.cpp, mqtt.cpp) expect SI units — acceleration in
 *  m/s² and angular rate in rad/s — matching the old Adafruit contract, so we
 *  convert the raw counts here.
 * ──────────────────────────────────────────────────────────────────────────── */

#define IMU_ADDR            0x68

/* Register map (shared by MPU6050 / MPU6500) */
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define REG_ACCEL_XOUT_H    0x3B

/* Default full-scale ranges after reset:
 *   accel ±2 g    -> 16384 LSB/g
 *   gyro  ±250 dps -> 131 LSB/(deg/s)                                          */
#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    131.0f
#define STANDARD_GRAVITY    9.80665f          /* m/s² per g                     */
#define DEG_TO_RAD_F        0.01745329252f    /* π / 180                        */

static bool s_imu_present = false;

bool is_imu_detected() { return s_imu_present; }

/* Read the WHO_AM_I register. Returns 0xFF on a bus error (no ACK). */
static uint8_t imu_who_am_i()
{
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(REG_WHO_AM_I);
    if (Wire.endTransmission(true) != 0) return 0xFF;

    if (Wire.requestFrom(IMU_ADDR, 1) != 1) return 0xFF;
    return Wire.read();
}

void InitializeIMU()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    /* Some breakout boards need a short settle time after power-up; probe a few
       times so a present device isn't latched as absent on the first attempt. */
    s_imu_present = false;
    for (uint8_t attempt = 0; attempt < 5; ++attempt) {
        uint8_t who = imu_who_am_i();
        /* MPU6050 -> 0x68, MPU6500 -> 0x70, MPU9250 -> 0x71. Accept any of the
           compatible parts; reject only a dead bus (0xFF) / wrong device. */
        if (who == 0x68 || who == 0x70 || who == 0x71) {
            s_imu_present = true;
            break;
        }
        delay(100);
    }

    if (!s_imu_present) {
        return;
    }

    /* Wake the device: clear the SLEEP bit in PWR_MGMT_1. */
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delay(10);
}


void ConfigureIMUEvents()
{
}




imu_event_type_t CheckIMUEvents()
{
    return IMU_EVENT_NONE;
}


/* Burst-read all six accel + gyro axes (and temperature) in one transaction.
 * Returns false on a bus error; on success the raw 16-bit signed counts are
 * written to the out-params (any of which may be nullptr if not needed). */
static bool imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                         int16_t *gx, int16_t *gy, int16_t *gz,
                         int16_t *temp)
{
    if (!s_imu_present) return false;

    uint8_t b[14];

    Wire.beginTransmission(IMU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;   /* repeated start */

    if (Wire.requestFrom(IMU_ADDR, 14) != 14) return false;

    for (int i = 0; i < 14; i++) b[i] = Wire.read();

    if (ax)   *ax   = (int16_t)((b[0]  << 8) | b[1]);
    if (ay)   *ay   = (int16_t)((b[2]  << 8) | b[3]);
    if (az)   *az   = (int16_t)((b[4]  << 8) | b[5]);
    if (temp) *temp = (int16_t)((b[6]  << 8) | b[7]);
    if (gx)   *gx   = (int16_t)((b[8]  << 8) | b[9]);
    if (gy)   *gy   = (int16_t)((b[10] << 8) | b[11]);
    if (gz)   *gz   = (int16_t)((b[12] << 8) | b[13]);
    return true;
}

/* Raw temperature count -> °C. The MPU6500 formula differs from the 6050; we
   use the 6500 form (raw/333.87 + 21.0) to match the board on this robot. */
static inline float imu_temp_c(int16_t raw)
{
    return raw / 333.87f + 21.0f;
}


imu_reading_t ReadIMU()
{
    imu_reading_t r = {};
    int16_t ax, ay, az, gx, gy, gz, t;
    if (!imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz, &t)) return r;

    /* counts -> g -> m/s² */
    r.accel.x = (ax / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    r.accel.y = (ay / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    r.accel.z = (az / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;

    /* counts -> dps -> rad/s */
    r.gyro.x = (gx / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    r.gyro.y = (gy / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    r.gyro.z = (gz / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;

    r.temperature = imu_temp_c(t);
    return r;
}


accel_data_t ReadAccel()
{
    accel_data_t d = {};
    int16_t ax, ay, az;
    if (!imu_read_raw(&ax, &ay, &az, nullptr, nullptr, nullptr, nullptr)) return d;
    d.x = (ax / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    d.y = (ay / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    d.z = (az / ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
    return d;
}

gyro_data_t ReadGyro()
{
    gyro_data_t d = {};
    int16_t gx, gy, gz;
    if (!imu_read_raw(nullptr, nullptr, nullptr, &gx, &gy, &gz, nullptr)) return d;
    d.x = (gx / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    d.y = (gy / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    d.z = (gz / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    return d;
}

double ReadIMUTemp()
{
    int16_t t;
    if (!imu_read_raw(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &t)) return 0.0;
    return imu_temp_c(t);
}
