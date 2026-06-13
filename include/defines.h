#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef unsigned char byte;

typedef struct
{
    float x, y, z;
} gyro_data_t;
typedef struct
{
    float x, y, z;
} accel_data_t;
typedef struct
{
    float temperature, humidity;
} dht_data_t;
typedef struct
{
    float ppm, voltage;
} mq135_data_t;

typedef enum
{
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_WHITE,
    COLOR_BLACK,
} color_name_t;

typedef struct
{
    uint16_t r, g, b, c;   /* raw 16-bit channels */
    float color_temp;      /* Kelvin */
    float lux;
    color_name_t dominant;
    bool valid;            /* false when the sensor is absent/failed */
} color_data_t;

typedef struct
{
    double latitude;
    double longitude;
    double altitude;
    double speed;
    uint32_t satellites;
    bool valid;
} gps_data_t;

typedef struct
{
    accel_data_t accel;
    gyro_data_t gyro;
    float temperature;
} imu_reading_t;

typedef enum
{
    IMU_EVENT_NONE = 0,
    IMU_EVENT_MOTION,
    IMU_EVENT_FREEFALL,
    IMU_EVENT_ZERO_MOTION,
} imu_event_type_t;

typedef struct
{
    imu_event_type_t type;
    uint32_t timestamp_s;
    uint32_t uptime_s;
    float accel_x;
    float accel_y;
    float accel_z;
} imu_event_t;

typedef enum
{
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_STOP,
    MOTOR_BRAKE,
} motor_dir_t;

typedef struct
{
    motor_dir_t dir;
    int16_t speed; /* 0-100% duty, or -1 for cruise default */
    bool pivot;    /* LEFT/RIGHT: true = discrete ~90° pivot, false = continuous turn */
} motor_cmd_t;

typedef struct
{
    uint8_t channel; /* SERVO_CH_* */
    uint8_t angle;   /* 0-180 */
    bool hold;       /* keep PWM on after settling (default depends on joint) */
} servo_cmd_t;

typedef struct
{
    float x, y, z;   /* mm in the arm base frame; z is height above the shoulder */
    int16_t gripper; /* 0-180, or -1 to leave unchanged */
} arm_pose_cmd_t;

/* I2C bus */
#define I2C_SDA 21
#define I2C_SCL 22

/*
 * L298N motor driver. Every control line wires straight to an ESP32 GPIO:
 * IN1-IN4 are digital direction, ENA/ENB are native LEDC PWM for real
 * proportional speed. The PCA9685 carries no motor signals (servos-only).
 */
#define MOTOR_PWM_ENA 25
#define MOTOR_IN1 4
#define MOTOR_IN2 5
#define MOTOR_PWM_ENB 27
#define MOTOR_IN3 33
#define MOTOR_IN4 32

/* ~20 kHz keeps the motors out of the audible whine range. LEDC ch 0 is the
   buzzer's, so the enables use 2/3. */
#define MOTOR_LEDC_CH_A 2
#define MOTOR_LEDC_CH_B 3
#define MOTOR_LEDC_FREQ 20000
#define MOTOR_LEDC_RES_BITS 8

/* PCA9685 PWM driver (servos only). */
#define PCA9685_ADDR 0x40
#define SERVO_CH_GRIPPER 3
#define SERVO_CH_SHOULDER 2
#define SERVO_CH_ELBOW 1
#define SERVO_CH_BASE 0

/* Servos want a 1.0-2.0 ms pulse in the 20 ms frame, NOT a full-range duty:
   at 50 Hz / 12-bit that's ~205-410 counts. Driving 0-4095 is outside the
   valid pulse window and the servo just jitters or stalls. */
#define SERVO_PULSE_MIN 205 /* 1.0 ms -> 0° */
#define SERVO_PULSE_MAX 410 /* 2.0 ms -> 180° */
#define SERVO_SETTLE_MS 350

#define PCA_PWM_FREQ 50
#define PCA_PWM_MAX 4095

/* Arm geometry + interpolation. */
#define ARM_SHOULDER_LEN_MM 100.0f
#define ARM_ELBOW_LEN_MM 100.0f
#define ARM_STEP_DEG 2.0f
#define ARM_STEP_MS 15

/* Shoulder/elbow MG90S buzz and overheat holding against gravity, so we cut
   PWM after a move and let gearbox friction hold the pose (a little droop). */
#define ARM_HOLD_SHOULDER false
#define ARM_HOLD_ELBOW false

/* Hard cap on continuous energized time, so a stalled servo can't sit at full
   stall current humming until its gears strip. */
#define SERVO_MAX_ENERGIZED_MS 600

/*
 * Scripted pick-and-place angles. These are SCRIPTED, not IK, and every value
 * needs calibrating on the real arm. Gripper opens at 0°, clamps at ~40°.
 */
#define GRIP_OPEN_DEG   0
#define GRIP_CLOSED_DEG 40

#define ARM_BASE_FRONT_DEG 90  /* over the object */
#define ARM_BASE_LEFT_DEG  0   /* 90° left drop-off (180 = right) */

#define ARM_SHOULDER_DOWN_DEG 140
#define ARM_ELBOW_DOWN_DEG    120
#define ARM_SHOULDER_UP_DEG   60
#define ARM_ELBOW_UP_DEG      80

/* The gripper has no interpolation, so give the jaws time to actually move. */
#define GRIP_SETTLE_MS 500

/* Wheel speed control (open-loop ramp; PID gains wait on encoders). */
#define MOTOR_SPEED_DEFAULT 100
#define MOTOR_SPEED_MIN 0
#define MOTOR_RAMP_STEP 4
#define MOTOR_RAMP_MS 12
#define MOTOR_PID_KP 1.2f
#define MOTOR_PID_KI 0.4f
#define MOTOR_PID_KD 0.05f

/* Discrete ~90° pivot for a LEFT/RIGHT command: open-loop, so TURN_90_MS is a
   calibration value — tune until one command gives a square corner. */
#define TURN_DUTY_PCT 80
#define TURN_90_MS    600

/* How long a FORWARD/BACKWARD command drives before auto-stopping (open-loop
   distance knob). Shared by the timed-drive dwell and the patrol hold window. */
#define DRIVE_RUN_MS 1000

/* TCS34725 colour sensor (optional). */
#define TCS34725_ADDR 0x29

#define MQ_PIN 35
#define DHT_PIN 15
#define HC_SR04_TRIG 12
#define HC_SR04_ECHO 23

/* Line following: set to 0 to compile out the IR steering. MUST be 1/0, not
   TRUE/FALSE — the preprocessor reads an unknown TRUE as 0 and silently
   disables it (the original "line following disabled" boot bug). */
#define LINE_FOLLOWING_ENABLED 0

/* GPIO34/39 are input-only with no pull-ups, fine since the IR modules drive
   the line actively. */
#define IR_LEFT_PIN 34
#define IR_RIGHT_PIN 39
#define IR_ON_LINE HIGH /* modules read HIGH over the line; flip for active-low */

/* Obstacle avoidance: stop and re-check anything closer than this. */
#define OBSTACLE_THRESHOLD_CM 50.0f
#define OBSTACLE_BEEP_MS 2000
#define OBSTACLE_REVERSE_MS 400

/* On a non-RED obstacle: pivot RIGHT ~90° and re-check; if still blocked,
   U-turn (~180°, ending left of the original heading) to escape a corner. */
#define OBSTACLE_TURN_90_MS  TURN_90_MS
#define OBSTACLE_UTURN_MS   (TURN_90_MS * 2)

#define LINE_LOOP_MS 30

/* Patrol command-hold windows: forward/backward run for a full drive, other
   commands use the short default. */
#define PATROL_CMD_HOLD_MS   1000
#define PATROL_DRIVE_HOLD_MS DRIVE_RUN_MS

/* Buzzer / LEDs. The buzzer is LEDC PWM at a low duty so it stays quiet. */
#define BUZZER_PIN 26
#define LED_PIN_1 2 /* built-in blue LED */
#define LED_PIN_2 32
#define LED_PIN_3 33
#define BUZZER_LEDC_CHANNEL 0
#define BUZZER_LEDC_RES_BITS 8
#define BUZZER_TONE_HZ 2300
#define BUZZER_DUTY 12 /* ~5% of 255 */
#define BUZZER_BEEP_MS 25

/* GPS (UART2). */
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

/* GSM. */
#define GSM_RX_PIN 32
#define GSM_TX_PIN 33
#define GSM_BAUD 9600
#define GSM_APN "safaricom"

/* WiFi. */
#define WIFI_SSID "DXS"
#define WIFI_PASSWORD "$Codex2016@Phoenix-FR1507"

/* MQTT. */
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_USER "astian"
#define MQTT_PASS "muchui"
#define DEVICE_ID "WRBT202642"
#define MQTT_CLIENT_ID DEVICE_ID

#define PUBLISH_INTERVAL_MS 5000

/* PubSubClient isn't thread-safe, so the network task is the ONLY place the
   socket is serviced. Keep this short so inbound commands dispatch promptly. */
#define MQTT_POLL_MS 50

/* MQTT topics. */
#define TOPIC_BASE "robot/devices/" DEVICE_ID
#define TOPIC_BOOT TOPIC_BASE "/boot"
#define TOPIC_READINGS TOPIC_BASE "/readings"
#define TOPIC_HEARTBEAT TOPIC_BASE "/heartbeat"
#define TOPIC_EVENT_MOTION TOPIC_BASE "/events/motion"
#define TOPIC_EVENT_FREEFALL TOPIC_BASE "/events/freefall"
#define TOPIC_EVENT_ZERO_MOTION TOPIC_BASE "/events/zero_motion"

/* Inbound command topics. Payload schemas:
   drive: {"cmd":"forward|backward|left|right|stop|brake","speed":0-100}
   arm:   {"joint":"base|shoulder|elbow|gripper","angle":0-180,"hold":bool}
   pose:  {"x":mm,"y":mm,"z":mm,"gripper":0-180}  (inverse kinematics) */
#define TOPIC_CMD_DRIVE "robot/cmd/drive"
#define TOPIC_CMD_ARM "robot/cmd/arm"
#define TOPIC_CMD_POSE "robot/cmd/pose"

/* FreeRTOS queues. */
extern QueueHandle_t g_dht_queue;
extern QueueHandle_t g_imu_queue;
extern QueueHandle_t g_sonic_queue;
extern QueueHandle_t g_mq135_queue;
extern QueueHandle_t g_gps_queue;
extern QueueHandle_t g_event_queue;
extern QueueHandle_t g_motor_cmd_queue; /* patrol -> motor_cmd_task (sole producer) */
extern QueueHandle_t g_drive_cmd_queue; /* MQTT -> patrol (external drive overrides) */
extern QueueHandle_t g_servo_cmd_queue;
extern QueueHandle_t g_pose_cmd_queue;
extern QueueHandle_t g_color_queue;

/* Function declarations. */
void initialize_pins();
void InitBuzzer();
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration);
void Beep(uint8_t iter);

void InitializeDHT();
dht_data_t ReadDHT();

void InitializeIMU();
bool is_imu_detected();
void ConfigureIMUEvents();
imu_event_type_t CheckIMUEvents();
imu_reading_t ReadIMU();
accel_data_t ReadAccel();
gyro_data_t ReadGyro();
double ReadIMUTemp();

float ReadUltrasonic();

void InitLineSensors();
bool ReadIRLeft();   /* true when the left sensor is over the line */
bool ReadIRRight();

mq135_data_t ReadMQ135();

void GPS_INIT();
void GPS_DRAIN();
gps_data_t READ_GPS();

void MotorInit();
void MotorSetDirection(motor_dir_t dir); /* IN1-IN4 only, no speed change */
void MotorSetDuty(uint8_t duty_pct);     /* ENA/ENB PWM duty 0-100% */
void MotorSetFeedback(float left_rpm, float right_rpm); /* encoder hook (stub) */

void InitServoDriver();
void SetServoAngle(uint8_t channel, uint8_t angle);
void MoveServoSmooth(uint8_t channel, uint8_t target);
/* Steps two joints together so the arm stays tucked (lower peak shoulder
   torque) instead of one joint finishing before the other starts. */
void MoveTwoServosSmooth(uint8_t ch_a, uint8_t tgt_a, uint8_t ch_b, uint8_t tgt_b);
void DisableServo(uint8_t channel);
void SetPCAPwm(uint8_t channel, uint16_t on_count); /* raw duty 0-4095 */

/* Returns false if the Cartesian target is out of the arm's reach. */
bool ArmSolveIK(float x, float y, float z,
                uint8_t *base_deg, uint8_t *shoulder_deg, uint8_t *elbow_deg);

void PickPlaceRedObject();
void RedObjectGripSequence();

/* InitializeColorSensor probes the bus once; the rest of the firmware runs
   unchanged when the sensor is absent. */
void InitializeColorSensor();
bool is_color_sensor_enabled();
color_data_t ReadColor();
const char *ColorName(color_name_t c);

void MQTT_INITIALIZE();
void callback(char *topic, byte *message, unsigned int length);
void printPayload(char *topic, byte *message, unsigned int length);

void init_sensor_timers();
void dht_task(void *);
void imu_task(void *);
void ultrasonic_task(void *);
void mq135_task(void *);
void motor_cmd_task(void *);
void motor_speed_task(void *);
void servo_cmd_task(void *);
void pose_cmd_task(void *);
void color_task(void *);
void line_follow_task(void *);

#endif /* DEFINES_H */
