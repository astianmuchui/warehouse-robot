#include <Arduino.h>
#include "pitches.h"
#include "defines.h"

/* ── Buzzer 
 *  The buzzer was previously driven with a flat digitalWrite(HIGH), i.e. 100 %
 *  duty — that's why the robot was so loud.  We now drive it with LEDC PWM at a
 *  low duty cycle (BUZZER_DUTY of 255), which is far quieter while still
 *  audible.  A tone frequency is set so a passive buzzer makes a clean pitch and
 *  an active buzzer simply chirps softly.
 * ──────── */

static bool s_buzzer_ready = false;

void InitBuzzer()
{
    ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_TONE_HZ, BUZZER_LEDC_RES_BITS);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
    ledcWrite(BUZZER_LEDC_CHANNEL, 0);   /* silent until first beep */
    s_buzzer_ready = true;
}

/* Drive `iter` soft beeps of `duration` ms on/off.  Kept the old signature so
   every existing call site stays valid; the `pin` argument is honoured only as
   a fallback when InitBuzzer() hasn't run yet. */
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration)
{
    for (uint8_t i = 0; i < iter; i++)
    {
        if (s_buzzer_ready)
            ledcWrite(BUZZER_LEDC_CHANNEL, BUZZER_DUTY);   /* quiet tone on */
        else
            digitalWrite(pin, HIGH);                       /* pre-init fallback */

        vTaskDelay(pdMS_TO_TICKS(duration));

        if (s_buzzer_ready)
            ledcWrite(BUZZER_LEDC_CHANNEL, 0);             /* tone off */
        else
            digitalWrite(pin, LOW);

        vTaskDelay(pdMS_TO_TICKS(duration));
    }

    if (s_buzzer_ready)
        ledcWrite(BUZZER_LEDC_CHANNEL, 0);
    else
        digitalWrite(pin, LOW);
}

/* Convenience: `iter` short, soft beeps at the default beep length. */
void Beep(uint8_t iter)
{
    Pulsate(BUZZER_PIN, iter, BUZZER_BEEP_MS);
}
