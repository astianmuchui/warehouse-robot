#include <Arduino.h>
#include "pitches.h"
#include "defines.h"

/*
 * Buzzer, driven with LEDC PWM at a low duty so it's a soft chirp instead of the
 * full-volume digitalWrite it used to be.
 */

static bool s_buzzer_ready = false;

/** InitBuzzer - attach the buzzer pin to an LEDC channel, silent to start. */
void InitBuzzer()
{
    ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_TONE_HZ, BUZZER_LEDC_RES_BITS);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
    ledcWrite(BUZZER_LEDC_CHANNEL, 0);
    s_buzzer_ready = true;
}

/**
 * Pulsate - `iter` soft beeps of `duration` ms on/off. The `pin` argument is
 * only used as a fallback if InitBuzzer() hasn't run yet.
 */
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration)
{
    for (uint8_t i = 0; i < iter; i++)
    {
        if (s_buzzer_ready)
            ledcWrite(BUZZER_LEDC_CHANNEL, BUZZER_DUTY);
        else
            digitalWrite(pin, HIGH);

        vTaskDelay(pdMS_TO_TICKS(duration));

        if (s_buzzer_ready)
            ledcWrite(BUZZER_LEDC_CHANNEL, 0);
        else
            digitalWrite(pin, LOW);

        vTaskDelay(pdMS_TO_TICKS(duration));
    }

    if (s_buzzer_ready)
        ledcWrite(BUZZER_LEDC_CHANNEL, 0);
    else
        digitalWrite(pin, LOW);
}

/** Beep - `iter` short beeps at the default length. */
void Beep(uint8_t iter)
{
    Pulsate(BUZZER_PIN, iter, BUZZER_BEEP_MS);
}
