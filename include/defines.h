#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>



typedef struct { float x, y, z; } gyro_data_t;
typedef struct { float x, y, z; } accel_data_t;
typedef struct { float temperature, humidity; } dht_data_t;
typedef struct { float ppm, voltage; } mq135_data_t;

typedef struct {
    double   latitude;
    double   longitude;
    double   altitude;
    double   speed;
    uint32_t satellites;
    bool     valid;
} gps_data_t;


typedef struct {
    accel_data_t accel;
    gyro_data_t  gyro;
    float        temperature;
} imu_reading_t;



typedef enum {
    IMU_EVENT_NONE        = 0,
    IMU_EVENT_MOTION,       
    IMU_EVENT_FREEFALL,     
    IMU_EVENT_ZERO_MOTION,  
} imu_event_type_t;

typedef struct {
    imu_event_type_t type;
    uint32_t         timestamp_s;  
    uint32_t         uptime_s;     
    float            accel_x;
    float            accel_y;
    float            accel_z;
} imu_event_t;





#define I2C_SDA 21
#define I2C_SCL 22


#define PCF_ADDR          0x20
#define PCF_INTERRUPT_PIN 34

#define PCF_P0 0
#define PCF_P1 1
#define PCF_P2 2
#define PCF_P3 3
#define PCF_P4 4
#define PCF_P5 5
#define PCF_P6 6
#define PCF_P7 7





#define MQ_PIN 4


#define DHT_PIN 15


#define HC_SR04_TRIG 12
#define HC_SR04_ECHO 23


#define BASE_SERVO     5
#define SHOULDER_SERVO 27
#define ELBOW_SERVO    19
#define GRIPPER_SERVO  18










#define BUZZER_PIN 25
#define LED_PIN_1  2    /* GPIO2 – built-in blue LED */
#define LED_PIN_2  32
#define LED_PIN_3  33


#define RXD2     16
#define TXD2     17
#define GPS_BAUD 9600


#define GSM_RX_PIN 32
#define GSM_TX_PIN 33
#define GSM_BAUD   9600
#define GSM_APN    "safaricom"


#define WIFI_SSID     "DXS"
#define WIFI_PASSWORD "$Codex2016@Phoenix-FR1507"


#define MQTT_BROKER    "hustlemania-home-1.local"
#define MQTT_PORT      1883
#define MQTT_USER      "astian"
#define MQTT_PASS      "muchui"
#define DEVICE_ID      "IRK17352YV2026"
#define MQTT_CLIENT_ID DEVICE_ID

#define PUBLISH_INTERVAL_MS 30000  /* 30 s periodic readings + heartbeat */


#define TOPIC_BASE              "robot/devices/" DEVICE_ID
#define TOPIC_BOOT              TOPIC_BASE "/boot"
#define TOPIC_READINGS          TOPIC_BASE "/readings"
#define TOPIC_HEARTBEAT         TOPIC_BASE "/heartbeat"
#define TOPIC_EVENT_MOTION      TOPIC_BASE "/events/motion"
#define TOPIC_EVENT_FREEFALL    TOPIC_BASE "/events/freefall"
#define TOPIC_EVENT_ZERO_MOTION TOPIC_BASE "/events/zero_motion"
#define TOPIC_CMD               "robot/cmd"


extern QueueHandle_t g_dht_queue;
extern QueueHandle_t g_imu_queue;
extern QueueHandle_t g_sonic_queue;
extern QueueHandle_t g_mq135_queue;
extern QueueHandle_t g_gps_queue;
extern QueueHandle_t g_event_queue;


void initialize_pins();
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration);

void InitializeDHT();
dht_data_t   ReadDHT();

void InitializeIMU();
void ConfigureIMUEvents();
imu_event_type_t CheckIMUEvents();
imu_reading_t    ReadIMU();
accel_data_t     ReadAccel();
gyro_data_t      ReadGyro();
double           ReadIMUTemp();

float ReadUltrasonic();

mq135_data_t ReadMQ135();

void       GPS_INIT();
void       GPS_DRAIN();
gps_data_t READ_GPS();

void InitializeExpander();
void PCF_Pulsate(uint8_t pcf_pin, uint8_t iter, uint16_t duration);

void MQTT_INITIALIZE();
void callback(char *topic, byte *message, unsigned int length);
void printPayload(char *topic, byte *message, unsigned int length);

void init_sensor_timers();
void dht_task(void *);
void imu_task(void *);
void ultrasonic_task(void *);
void mq135_task(void *);
void gps_task(void *);

#endif /* DEFINES_H */
