#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>
#include "defines.h"
#include <ESP32Servo.h>

Servo base, shoulder, elbow, gripper;


static const BaseType_t servo_cpu = 1;
static const BaseType_t sensor_cpu = 0;

void toggleLED1(void *ptr)
{
  while (1)
  {
    digitalWrite(LED_PIN_1, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN_1, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}


void test_servos(void *ptr)
{
  base.write(100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  base.write(100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  shoulder.write(180);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  shoulder.write(100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  elbow.write(100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  elbow.write(180);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gripper.write(100);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gripper.write(180);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  while (1)
  {
    base.write(rand() % 181);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    shoulder.write(rand() % 181);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    elbow.write(rand() % 181);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    gripper.write(rand() % 181);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(9600);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ESP32PWM::allocateTimer(4);

  shoulder.attach(SHOULDER_SERVO, 500, 2400);
  elbow.attach(ELBOW_SERVO, 500, 2400);
  gripper.attach(GRIPPER_SERVO, 500, 2400);
  base.attach(BASE_SERVO, 500, 2400);

  shoulder.write(0);
  elbow.write(0);
  gripper.write(0);
  base.write(0);

  initialize_pins();
  InitializeExpander();
  Pulsate(BUZZER_PIN, 2, 500);
  MQTT_INITIALIZE();
  InitializeDHT();
  InitializeIMU();


  // xTaskCreatePinnedToCore(test_servos, "Test Servos", 4096, NULL, 1, NULL, servo_cpu);
  xTaskCreatePinnedToCore(toggleLED1, "LED Task", 2048, NULL, 1, NULL, sensor_cpu);
  xTaskCreatePinnedToCore(SensorTask, "Sensor Task", 4096, NULL, 1, NULL, sensor_cpu);

  // xTaskCreatePinnedToCore(toggleLED3, "Toggle LED3", 2048, NULL, 1, NULL, sensor_cpu);
  // xTaskCreatePinnedToCore(toggleLED4, "Toggle LED4", 2048, NULL, 1, NULL, sensor_cpu);


}

void loop()
{
}
