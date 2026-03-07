#include <Arduino.h>
#include <DHT.h>
#include "defines.h"

DHT dht(DHT_PIN, DHT22);

void InitializeDHT()
{
  dht.begin();
}

dht_data_t *ReadDHT()
{
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  dht_data_t data;
  data.temperature = temperature;
  data.humidity = humidity;

  return &data;
}
