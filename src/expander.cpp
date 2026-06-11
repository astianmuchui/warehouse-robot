#include <Arduino.h>
#include "defines.h"

/* ── PCF8574 I/O expander: DISABLED ──────────────────────────────────────────
 *  The PCF8574 is no longer fitted/used on this robot, and the L298N control
 *  lines no longer live on the PCA9685 either — every motor signal now wires
 *  directly to an ESP32 GPIO.  IN1–IN4 are plain digital direction lines;
 *  ENA/ENB are driven with native LEDC PWM (see MotorSetDuty in servo_arm.cpp).
 *  The PCA9685 is now servos-only.
 * ──────────────────────────────────────────────────────────────────────────── */

void MotorInit()
{
    /* Direction lines: plain digital outputs, start LOW (motors coasting). */
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, LOW);

    /* Enable lines: LEDC PWM for real speed control. setup() also calls
       MotorSetDuty(0) shortly after to make sure the wheels are stopped. */
    ledcSetup(MOTOR_LEDC_CH_A, MOTOR_LEDC_FREQ, MOTOR_LEDC_RES_BITS);
    ledcSetup(MOTOR_LEDC_CH_B, MOTOR_LEDC_FREQ, MOTOR_LEDC_RES_BITS);
    ledcAttachPin(MOTOR_PWM_ENA, MOTOR_LEDC_CH_A);
    ledcAttachPin(MOTOR_PWM_ENB, MOTOR_LEDC_CH_B);
    ledcWrite(MOTOR_LEDC_CH_A, 0);
    ledcWrite(MOTOR_LEDC_CH_B, 0);

    Serial.printf("[Motor] L298N on native GPIO (ENA/IN1/IN2=%d,%d,%d  ENB/IN3/IN4=%d,%d,%d)\n",
                  MOTOR_PWM_ENA, MOTOR_IN1, MOTOR_IN2, MOTOR_PWM_ENB, MOTOR_IN3, MOTOR_IN4);
}

/* Set the IN1–IN4 direction lines only (speed is the speed task's job via
   MotorSetDuty).  STOP and BRAKE both clear all four direction bits. */
void MotorSetDirection(motor_dir_t dir)
{
    switch (dir)
    {
    case MOTOR_FORWARD:
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
        break;

    case MOTOR_BACKWARD:
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, HIGH);
        break;

    case MOTOR_LEFT:
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
        break;

    case MOTOR_RIGHT:
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, HIGH);
        break;

    case MOTOR_STOP:
    case MOTOR_BRAKE:
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, LOW);
        break;
    }
}

/* Set ENA/ENB PWM duty 0–100 %.  Now that the enables are on native ESP32
   GPIOs driven by LEDC at MOTOR_LEDC_FREQ, this is real proportional speed
   control (unlike the old PCA9685 wiring, which could only do on/off). */
void MotorSetDuty(uint8_t duty_pct)
{
    if (duty_pct > 100) duty_pct = 100;

    /* Scale 0–100 % to the LEDC resolution (0–255 at 8 bits). */
    const uint32_t max_duty = (1u << MOTOR_LEDC_RES_BITS) - 1;
    uint32_t level = (uint32_t)duty_pct * max_duty / 100u;

    ledcWrite(MOTOR_LEDC_CH_A, level);
    ledcWrite(MOTOR_LEDC_CH_B, level);
}
