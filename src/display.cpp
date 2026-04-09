#include <Arduino.h>
#include "pitches.h"
#include "defines.h"

void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration)
{
  for (int i = 0; i < iter; i++)
  {
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(duration));
    digitalWrite(pin, LOW);
    vTaskDelay(pdMS_TO_TICKS(duration));
  }
  digitalWrite(pin, LOW);
}
