#ifndef DEFINES_H
#define DEFINES_H

typedef struct gyro_data_s {
    float x;
    float y;
    float z;
} gyro_data_t;

#define GYRO_SCL
#define GYRO_SDA
#define GYRO_XDA
#define GYRO_XCL
#define GYRO_INT

#define DHT_PIN

#define HC_SR04_TRIG
#define HC_SR04_ECHO

#define BASE_SERVO
#define SHOULDER_SERVO
#define ELBOW_SERVO
#define WRIST_SERVO

#define BUZZER_PIN

#define ENABLE_PIN_1
#define ENABLE_PIN_2

#define MOTOR1_IN1
#define MOTOR1_IN2

#define MOTOR2_IN1
#define MOTOR2_IN2

#endif