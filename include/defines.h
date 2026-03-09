#ifndef DEFINES_H
#define DEFINES_H

typedef struct gyro_data_s
{
    float x;
    float y;
    float z;
} gyro_data_t;

typedef struct dht_data_s
{
    float temperature;
    float humidity;
} dht_data_t;

typedef struct accel_data_s
{
    float x;
    float y;
    float z;
} accel_data_t;

typedef struct mq135_data_s
{
    float ppm;
    float voltage;
} mq135_data_t;

#define MQ_PIN 4

#define GYRO_SCL
#define GYRO_SDA
#define GYRO_XDA
#define GYRO_XCL
#define GYRO_INT

#define DHT_PIN 15

#define HC_SR04_TRIG 22
#define HC_SR04_ECHO 23

#define BASE_SERVO 5
#define SHOULDER_SERVO 27
#define ELBOW_SERVO 19
#define GRIPPER_SERVO 18

#define BUZZER_PIN 13

#define ENABLE_PIN_1
#define ENABLE_PIN_2

#define MOTOR1_IN1
#define MOTOR1_IN2

#define MOTOR2_IN1
#define MOTOR2_IN2

#define LED_PIN_1 LED_BUILTIN
#define LED_PIN_2 12
#define LED_PIN_3 14

#define LED_RED 14

void initialize_pins();
void Buzz(uint8_t iter, uint16_t duration);
accel_data_t ReadAccel();
double ReadIMUTemp();
gyro_data_t ReadGyro();
void InitializeIMU();
void MQTT_INITIALIZE();
void printPayload(char *topic, byte *message, unsigned int length);
void callback(char *topic, byte *message, unsigned int length);
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration);


#endif