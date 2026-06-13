#include <Arduino.h>
#include "defines.h"

/*
 * L298N motor driver. Every motor signal wires straight to an ESP32 GPIO: IN1-IN4
 * are digital direction lines, ENA/ENB are LEDC PWM (MotorSetDuty). The old
 * PCF8574 expander and PCA9685 motor wiring are gone; the PCA9685 is servos-only.
 */

/** MotorInit - direction pins LOW (coasting), enable pins on LEDC PWM at zero. */
void MotorInit()
{
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, LOW);

    ledcSetup(MOTOR_LEDC_CH_A, MOTOR_LEDC_FREQ, MOTOR_LEDC_RES_BITS);
    ledcSetup(MOTOR_LEDC_CH_B, MOTOR_LEDC_FREQ, MOTOR_LEDC_RES_BITS);
    ledcAttachPin(MOTOR_PWM_ENA, MOTOR_LEDC_CH_A);
    ledcAttachPin(MOTOR_PWM_ENB, MOTOR_LEDC_CH_B);
    ledcWrite(MOTOR_LEDC_CH_A, 0);
    ledcWrite(MOTOR_LEDC_CH_B, 0);

    Serial.printf("[Motor] L298N on native GPIO (ENA/IN1/IN2=%d,%d,%d  ENB/IN3/IN4=%d,%d,%d)\n",
                  MOTOR_PWM_ENA, MOTOR_IN1, MOTOR_IN2, MOTOR_PWM_ENB, MOTOR_IN3, MOTOR_IN4);
}

/**
 * MotorSetDirection - set IN1-IN4 only; speed is MotorSetDuty's job. STOP and
 * BRAKE both clear all four direction bits.
 */
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

/** MotorSetDuty - set both enable channels to duty 0-100%, scaled to LEDC bits. */
void MotorSetDuty(uint8_t duty_pct)
{
    if (duty_pct > 100) duty_pct = 100;

    const uint32_t max_duty = (1u << MOTOR_LEDC_RES_BITS) - 1;
    uint32_t level = (uint32_t)duty_pct * max_duty / 100u;

    ledcWrite(MOTOR_LEDC_CH_A, level);
    ledcWrite(MOTOR_LEDC_CH_B, level);
}
