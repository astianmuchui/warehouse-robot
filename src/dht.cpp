#include <Arduino.h>
#include <DHT.h>
#include "defines.h"

DHT dht(DHT_PIN, DHT11);

void InitializeDHT()
{
  return dht.begin();
}

/** ReadDHT - temperature + humidity (either may be NaN on a failed read). */
dht_data_t ReadDHT()
{
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  dht_data_t data;
  data.temperature = temperature;
  data.humidity = humidity;

  return data;
}
