#include <Arduino.h>
#include "PCF8574.h"
#include "defines.h"

static void IRAM_ATTR keyPressInterruptHandler();

PCF8574 pcf8574(PCF_ADDR, PCF_INTERRUPT_PIN, keyPressInterruptHandler);

static volatile bool keyPressed = false;

static void IRAM_ATTR keyPressInterruptHandler()
{
    keyPressed = true;
}

void InitializeExpander()
{
    pinMode(PCF_INTERRUPT_PIN, INPUT_PULLUP);

    pcf8574.pinMode(P0, OUTPUT);
    pcf8574.pinMode(P1, OUTPUT);
    pcf8574.pinMode(P2, OUTPUT);
    pcf8574.pinMode(P3, OUTPUT);
    pcf8574.pinMode(P4, OUTPUT);
    pcf8574.pinMode(P5, OUTPUT);
    pcf8574.pinMode(P6, OUTPUT);
    pcf8574.pinMode(P7, OUTPUT);

    if (pcf8574.begin()) {
        Serial.println("[PCF] PCF8574 OK");
    } else {
        Serial.println("[PCF] PCF8574 not found – check PCF_ADDR");
    }


    for (uint8_t p = 0; p <= 7; p++)
        pcf8574.digitalWrite(p, LOW);
}

void PCF_Pulsate(uint8_t pcf_pin, uint8_t iter, uint16_t duration)
{
    for (int i = 0; i < iter; i++) {
        pcf8574.digitalWrite(pcf_pin, HIGH);
        delay(duration);
        pcf8574.digitalWrite(pcf_pin, LOW);
        delay(duration);
    }
    pcf8574.digitalWrite(pcf_pin, LOW);
}



/* L298N IN1–IN4 are digital, but they now share the PCA9685 with the
   ENA/ENB PWMs and the servos.  We treat them as one-bit PWM: 0 = LOW,
   PCA_PWM_MAX = HIGH (the L298N input threshold sees a steady level). */
static inline void PcaDigital(uint8_t channel, bool high)
{
    SetPCAPwm(channel, high ? PCA_PWM_MAX : 0);
}

void MotorInit()
{
    /* All L298N control lines (ENA/IN1/IN2/ENB/IN3/IN4) live on the PCA9685.
       Speed is initialised to 0 by MotorSetDuty(0) from servo_arm.cpp. */
    PcaDigital(MOTOR_IN1, false);
    PcaDigital(MOTOR_IN2, false);
    PcaDigital(MOTOR_IN3, false);
    PcaDigital(MOTOR_IN4, false);
    Serial.println("[Motor] L298N ready via PCA9685 (ENA/IN1/IN2=0,1,2  ENB/IN3/IN4=4,5,6)");
}

/* Drive IN1–IN4 direction lines via the PCA9685 without touching ENA/ENB.
   STOP and BRAKE clear the direction bits; speed zeroing is the speed task's
   job (MotorSetDuty). */
void MotorSetDirection(motor_dir_t dir)
{
    switch (dir)
    {
    case MOTOR_FORWARD:
        PcaDigital(MOTOR_IN1, true);
        PcaDigital(MOTOR_IN2, false);
        PcaDigital(MOTOR_IN3, true);
        PcaDigital(MOTOR_IN4, false);
        break;

    case MOTOR_BACKWARD:
        PcaDigital(MOTOR_IN1, false);
        PcaDigital(MOTOR_IN2, true);
        PcaDigital(MOTOR_IN3, false);
        PcaDigital(MOTOR_IN4, true);
        break;

    case MOTOR_LEFT:
        PcaDigital(MOTOR_IN1, false);
        PcaDigital(MOTOR_IN2, true);
        PcaDigital(MOTOR_IN3, true);
        PcaDigital(MOTOR_IN4, false);
        break;

    case MOTOR_RIGHT:
        PcaDigital(MOTOR_IN1, true);
        PcaDigital(MOTOR_IN2, false);
        PcaDigital(MOTOR_IN3, false);
        PcaDigital(MOTOR_IN4, true);
        break;

    case MOTOR_STOP:
    case MOTOR_BRAKE:
        PcaDigital(MOTOR_IN1, false);
        PcaDigital(MOTOR_IN2, false);
        PcaDigital(MOTOR_IN3, false);
        PcaDigital(MOTOR_IN4, false);
        break;
    }
}
