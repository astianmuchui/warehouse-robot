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
        Serial.println("PCF8574 OK");
    } else {
        Serial.println("PCF8574 not found - check PCF_ADDR");
    }

    digitalWrite(P0, LOW);
    digitalWrite(P1, LOW);
    digitalWrite(P2, LOW);
    digitalWrite(P3, LOW);
    digitalWrite(P4, LOW);
    digitalWrite(P5, LOW);
    digitalWrite(P6, LOW);
    digitalWrite(P7, LOW);
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
