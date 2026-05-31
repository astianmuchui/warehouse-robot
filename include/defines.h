#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef unsigned char byte;

typedef struct { float x, y, z; } gyro_data_t;
typedef struct { float x, y, z; } accel_data_t;
typedef struct { float temperature, humidity; } dht_data_t;
typedef struct { float ppm, voltage; } mq135_data_t;

/* ── TCS34725 colour sensor ────────────────────────────────────────────────── */
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_WHITE,
    COLOR_BLACK,
} color_name_t;

typedef struct {
    uint16_t   r, g, b, c;   /* raw 16-bit channels                    */
    float      color_temp;   /* correlated colour temperature (Kelvin) */
    float      lux;          /* illuminance                            */
    color_name_t dominant;   /* classified dominant colour             */
    bool       valid;        /* false when the sensor is absent/failed */
} color_data_t;

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

/* ── Motor command ─────────────────────────────────────────────────────────── */
typedef enum {
    MOTOR_FORWARD  = 0,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_STOP,
    MOTOR_BRAKE,
} motor_dir_t;

/* A drive command: a direction plus a target speed (0–100 %).  speed < 0 means
   "use the default cruise speed".  The speed task ramps duty toward this target
   so direction changes are smooth rather than jerky. */
typedef struct {
    motor_dir_t dir;
    int16_t     speed;   /* 0–100 % duty, or -1 for default cruise */
} motor_cmd_t;

/* ── Servo (arm) command ───────────────────────────────────────────────────── */
typedef struct {
    uint8_t channel;   /* SERVO_CH_*   */
    uint8_t angle;     /* 0–180 degrees */
    bool    hold;      /* keep PWM active after settling (default depends on joint) */
} servo_cmd_t;

/* A Cartesian arm target, solved with inverse kinematics into joint angles and
   reached with smooth interpolation.  Units are millimetres in the arm's base
   frame; z is height above the shoulder pivot. */
typedef struct {
    float x, y, z;     /* target end-effector position (mm) */
    int16_t gripper;   /* gripper angle 0–180, or -1 to leave unchanged */
} arm_pose_cmd_t;


/* ── I2C bus ───────────────────────────────────────────────────────────────── */
#define I2C_SDA 21
#define I2C_SCL 22

/* ── PCF8574 I/O expander ──────────────────────────────────────────────────── */
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

/* ── L298N motor driver ──────────────────────────────────────────────────────
 *  All six L298N control lines live on the PCA9685.  ENA/ENB get real PWM
 *  speed control; IN1–IN4 are driven as digital (full-on / full-off) via the
 *  same chip.  PCF8574 P0–P3 are now free.
 * ──────────────────────────────────────────────────────────────────────────── */
#define MOTOR_PWM_ENA     8     /* Motor A enable (PWM speed)      */
#define MOTOR_IN1         9     /* Motor A forward                 */
#define MOTOR_IN2         10    /* Motor A reverse                 */
#define MOTOR_PWM_ENB     11    /* Motor B enable (PWM speed)      */
#define MOTOR_IN3         12    /* Motor B forward                 */
#define MOTOR_IN4         13    /* Motor B reverse                 */

/* ── PCA9685 I2C PWM driver (servos + motor enables) ───────────────────────── */
#define PCA9685_ADDR      0x40
/* Arm servos live on the first four channels (next to the bulk capacitor).
   Base = MG996R, Shoulder/Elbow = MG90S, Gripper = micro servo. */
#define SERVO_CH_BASE     0
#define SERVO_CH_SHOULDER 1
#define SERVO_CH_ELBOW    2
#define SERVO_CH_GRIPPER  3
#define SERVO_PULSE_MIN   102   /* ~500 µs at 50 Hz, 12-bit resolution */
#define SERVO_PULSE_MAX   492   /* ~2400 µs                            */
#define SERVO_SETTLE_MS   350   /* time for a servo to reach target before PWM is cut */

/* PCA9685 runs at 50 Hz for the servos; the motor-enable channels share that
   frequency, which is plenty for L298N speed control.  Duty is set as a 12-bit
   on-count (0–4095). */
#define PCA_PWM_FREQ      50
#define PCA_PWM_MAX       4095

/* ── Arm geometry / motion (inverse kinematics + interpolation) ────────────── */
#define ARM_SHOULDER_LEN_MM  100.0f   /* shoulder pivot → elbow pivot   */
#define ARM_ELBOW_LEN_MM     100.0f   /* elbow pivot → gripper tip      */
#define ARM_STEP_DEG         2.0f     /* max joint step per interp tick  */
#define ARM_STEP_MS          15       /* interpolation tick period (ms)  */
/* Which joints stay energised when idle.  The MG90S used on shoulder/elbow
   stalls and buzzes (and overheats) when asked to hold against gravity at
   the arm's weight, so we cut PWM after every move and let gearbox friction
   hold the pose.  A small amount of droop is the trade-off. */
#define ARM_HOLD_SHOULDER    false
#define ARM_HOLD_ELBOW       false

/* ── Wheel speed control (PID-ready, open-loop ramp for now) ───────────────── */
#define MOTOR_SPEED_DEFAULT  80     /* default cruise duty (%)             */
#define MOTOR_SPEED_MIN      35     /* below this the motors stall         */
#define MOTOR_RAMP_STEP      4      /* duty % change per ramp tick         */
#define MOTOR_RAMP_MS        12     /* ramp tick period (ms)               */
/* PID gains — used once wheel encoders provide feedback (see MotorSetFeedback) */
#define MOTOR_PID_KP         1.2f
#define MOTOR_PID_KI         0.4f
#define MOTOR_PID_KD         0.05f

/* ── TCS34725 colour sensor (optional — robot runs fine without it) ────────── */
#define TCS34725_ADDR     0x29

/* ── Analog sensors ────────────────────────────────────────────────────────── */
#define MQ_PIN 35

/* ── DHT11 ─────────────────────────────────────────────────────────────────── */
#define DHT_PIN 15

/* ── HC-SR04 ultrasonic ────────────────────────────────────────────────────── */
#define HC_SR04_TRIG 12
#define HC_SR04_ECHO 23

/* ── Buzzer / LEDs ─────────────────────────────────────────────────────────── */
#define BUZZER_PIN 26
#define LED_PIN_1  2    /* GPIO2 – built-in blue LED */
#define LED_PIN_2  32
#define LED_PIN_3  33

/* Buzzer volume / tone control.
 * The buzzer is driven with LEDC PWM; a low duty cycle makes it much quieter
 * than the previous full-on digitalWrite.  Tone uses a fixed frequency at a
 * gentle duty so the robot stops being so loud. */
#define BUZZER_LEDC_CHANNEL  0
#define BUZZER_LEDC_RES_BITS 8        /* 0–255 duty resolution               */
#define BUZZER_TONE_HZ       2300     /* pleasant mid tone                   */
#define BUZZER_DUTY          12       /* ~5 % of 255 — quiet (was full-on)   */
#define BUZZER_BEEP_MS       25       /* short, soft beep by default         */

/* ── GPS (UART2) ───────────────────────────────────────────────────────────── */
#define RXD2     16
#define TXD2     17
#define GPS_BAUD 9600

/* ── GSM ───────────────────────────────────────────────────────────────────── */
#define GSM_RX_PIN 32
#define GSM_TX_PIN 33
#define GSM_BAUD   9600
#define GSM_APN    "safaricom"

/* ── WiFi ──────────────────────────────────────────────────────────────────── */
#define WIFI_SSID     "DXS"
#define WIFI_PASSWORD "$Codex2016@Phoenix-FR1507"

/* ── MQTT ──────────────────────────────────────────────────────────────────── */
#define MQTT_BROKER    "hustlemania-home-1.local"
#define MQTT_PORT      1883
#define MQTT_USER      "astian"
#define MQTT_PASS      "muchui"
#define DEVICE_ID      "WRBT202642"
#define MQTT_CLIENT_ID DEVICE_ID

#define PUBLISH_INTERVAL_MS 5000

/* ── MQTT topics ───────────────────────────────────────────────────────────── */
#define TOPIC_BASE              "robot/devices/" DEVICE_ID
#define TOPIC_BOOT              TOPIC_BASE "/boot"
#define TOPIC_READINGS          TOPIC_BASE "/readings"
#define TOPIC_HEARTBEAT         TOPIC_BASE "/heartbeat"
#define TOPIC_EVENT_MOTION      TOPIC_BASE "/events/motion"
#define TOPIC_EVENT_FREEFALL    TOPIC_BASE "/events/freefall"
#define TOPIC_EVENT_ZERO_MOTION TOPIC_BASE "/events/zero_motion"

/* Inbound command topics */
#define TOPIC_CMD_DRIVE  "robot/cmd/drive"   /* {"cmd":"forward|backward|left|right|stop|brake","speed":0-100} */
#define TOPIC_CMD_ARM    "robot/cmd/arm"     /* {"joint":"base|shoulder|elbow|gripper","angle":0-180,"hold":bool} */
#define TOPIC_CMD_POSE   "robot/cmd/pose"    /* {"x":mm,"y":mm,"z":mm,"gripper":0-180} — inverse kinematics */


/* ── FreeRTOS queues ───────────────────────────────────────────────────────── */
extern QueueHandle_t g_dht_queue;
extern QueueHandle_t g_imu_queue;
extern QueueHandle_t g_sonic_queue;
extern QueueHandle_t g_mq135_queue;
extern QueueHandle_t g_gps_queue;
extern QueueHandle_t g_event_queue;
extern QueueHandle_t g_motor_cmd_queue;
extern QueueHandle_t g_servo_cmd_queue;
extern QueueHandle_t g_pose_cmd_queue;
extern QueueHandle_t g_color_queue;


/* ── Function declarations ─────────────────────────────────────────────────── */
void initialize_pins();
void InitBuzzer();
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration);   /* quiet PWM beeps */
void Beep(uint8_t iter);   /* convenience: `iter` short soft beeps */

void InitializeDHT();
dht_data_t   ReadDHT();

void InitializeIMU();
bool is_imu_detected();
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

/* Motor driver: direction via PCF8574, speed (duty 0–100 %) via PCA9685 PWM. */
void MotorInit();
void MotorSetDirection(motor_dir_t dir);   /* set IN1–IN4 only (no speed change) */
void MotorSetDuty(uint8_t duty_pct);       /* set ENA/ENB PWM duty 0–100 %        */
/* Hook for closed-loop control once wheel encoders exist; left as a stub. */
void MotorSetFeedback(float left_rpm, float right_rpm);

/* PCA9685 PWM driver (shared by servos and motor enables). */
void InitServoDriver();
void SetServoAngle(uint8_t channel, uint8_t angle);
void MoveServoSmooth(uint8_t channel, uint8_t target);   /* interpolated move */
void DisableServo(uint8_t channel);
void SetPCAPwm(uint8_t channel, uint16_t on_count);   /* raw duty 0–4095 */

/* Arm inverse kinematics: solve a Cartesian target into joint angles. Returns
   false if the target is unreachable (out of the arm's reach envelope). */
bool ArmSolveIK(float x, float y, float z,
                uint8_t *base_deg, uint8_t *shoulder_deg, uint8_t *elbow_deg);

/* ── TCS34725 colour sensor ─────────────────────────────────────────────────
 *  InitializeColorSensor() probes the bus and remembers whether the sensor is
 *  present, so the rest of the firmware can run unchanged when it is absent.
 *  is_color_sensor_enabled() reflects that result.
 * ──────────────────────────────────────────────────────────────────────────── */
void         InitializeColorSensor();
bool         is_color_sensor_enabled();
color_data_t ReadColor();
const char  *ColorName(color_name_t c);

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

#endif /* DEFINES_H */
