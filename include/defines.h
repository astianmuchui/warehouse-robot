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

#define PCF_ADDR          0x20
#define PCF_INTERRUPT_PIN 34
#define I2C_SDA           16
#define I2C_SCL           17


#define PCF_P0 0
#define PCF_P1 1
#define PCF_P2 2
#define PCF_P3 3
#define PCF_P4 4
#define PCF_P5 5
#define PCF_P6 6
#define PCF_P7 7

#define MQ_PIN 4

#define GYRO_SCL
#define GYRO_SDA
#define GYRO_XDA
#define GYRO_XCL
#define GYRO_INT

#define DHT_PIN 15

#define HC_SR04_TRIG 12
#define HC_SR04_ECHO 23

#define BASE_SERVO 14
#define SHOULDER_SERVO 27
#define ELBOW_SERVO 19
#define GRIPPER_SERVO 21

#define BUZZER_PIN 13

#define ENABLE_PIN_1
#define ENABLE_PIN_2

#define MOTOR1_IN1
#define MOTOR1_IN2

#define MOTOR2_IN1
#define MOTOR2_IN2

#define LED_PIN_1 LED_BUILTIN
#define LED_PIN_2 32
#define LED_PIN_3 33

#define LED_RED 25

void initialize_pins();
void Buzz(uint8_t iter, uint16_t duration);
accel_data_t ReadAccel();
dht_data_t ReadDHT();

double ReadIMUTemp();
gyro_data_t ReadGyro();
void InitializeIMU();
void MQTT_INITIALIZE();

void SensorTask(void *ptr);
void DisplayTask(void *ptr);
void ServoTask(void *ptr);
void WheelsTask(void *ptr);
void InitializeDHT();
float ReadUltrasonic();

void InitializeExpander();
void PCF_Pulsate(uint8_t pcf_pin, uint8_t iter, uint16_t duration);

void printPayload(char *topic, byte *message, unsigned int length);
void callback(char *topic, byte *message, unsigned int length);
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration);

#endif