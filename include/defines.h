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

/*  TCS34725 colour sensor  */
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
    uint16_t r, g, b, c;   /* raw 16-bit channels                    */
    float color_temp;      /* correlated colour temperature (Kelvin) */
    float lux;             /* illuminance                            */
    color_name_t dominant; /* classified dominant colour             */
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

/*  Motor command ─ */
typedef enum
{
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_STOP,
    MOTOR_BRAKE,
} motor_dir_t;

/* A drive command: a direction plus a target speed (0–100 %).  speed < 0 means
   "use the default cruise speed".  The speed task ramps duty toward this target
   so direction changes are smooth rather than jerky. */
typedef struct
{
    motor_dir_t dir;
    int16_t speed; /* 0–100 % duty, or -1 for default cruise */
    /* For LEFT/RIGHT only: true → discrete ~90° in-place pivot then stop
       (external commands); false → continuous turn (line-follow nudges). */
    bool pivot;
} motor_cmd_t;

/*  Servo (arm) command ─ */
typedef struct
{
    uint8_t channel; /* SERVO_CH_*   */
    uint8_t angle;   /* 0–180 degrees */
    bool hold;       /* keep PWM active after settling (default depends on joint) */
} servo_cmd_t;

/* A Cartesian arm target, solved with inverse kinematics into joint angles and
   reached with smooth interpolation.  Units are millimetres in the arm's base
   frame; z is height above the shoulder pivot. */
typedef struct
{
    float x, y, z;   /* target end-effector position (mm) */
    int16_t gripper; /* gripper angle 0–180, or -1 to leave unchanged */
} arm_pose_cmd_t;

/*  I2C bus ─ */
#define I2C_SDA 21
#define I2C_SCL 22

/*  PCF8574 I/O expander: DISABLED 
 *  No longer fitted; all I/O moved to the PCA9685.  Address/pin/channel defines
 *  removed along with the driver. GPIO34 (was PCF_INTERRUPT_PIN) is now free. */

/*  L298N motor driver 
 *  All six L298N control lines now wire directly to ESP32 GPIOs (the PCA9685
 *  no longer carries any motor signals — it is servos-only).  ENA/ENB are
 *  driven with native LEDC PWM, giving real proportional speed control at a
 *  proper DC-motor frequency; IN1–IN4 are plain digital direction lines.
 *  */
#define MOTOR_PWM_ENA 25 /* Motor A enable (LEDC PWM speed) */
#define MOTOR_IN1 4     /* Motor A forward                 */
#define MOTOR_IN2 5     /* Motor A reverse                 */
#define MOTOR_PWM_ENB 27 /* Motor B enable (LEDC PWM speed) */
#define MOTOR_IN3 33     /* Motor B forward                 */
#define MOTOR_IN4 32     /* Motor B reverse                 */

/* LEDC PWM for the motor enables.  ~20 kHz keeps the DC motors out of the
   audible whine range and is well within the L298N's switching ability.
   Channels 0 is taken by the buzzer (see BUZZER_LEDC_CHANNEL), so use 2/3. */
#define MOTOR_LEDC_CH_A 2
#define MOTOR_LEDC_CH_B 3
#define MOTOR_LEDC_FREQ 20000
#define MOTOR_LEDC_RES_BITS 8 /* 0–255 duty resolution */

/*  PCA9685 I2C PWM driver (servos + motor enables) ─ */
#define PCA9685_ADDR 0x40
/* Arm servos live on the last block of four PCA9685 channels (12–15).
   Base/Shoulder/Elbow = MG90S, Gripper = micro servo. */
// #define SERVO_CH_GRIPPER 12
// #define SERVO_CH_SHOULDER 13
// #define SERVO_CH_ELBOW 14
// #define SERVO_CH_BASE 15

#define SERVO_CH_GRIPPER 3
#define SERVO_CH_SHOULDER 2
#define SERVO_CH_ELBOW 1
#define SERVO_CH_BASE 0
/* Hobby servos expect a 1.0–2.0 ms pulse inside the 20 ms (50 Hz) frame, NOT a
   full-range duty. At 50 Hz / 12-bit, one count ≈ 20 ms / 4096 ≈ 4.88 µs, so:
     1.0 ms (0°)   ≈ 205 counts
     1.5 ms (90°)  ≈ 307 counts
     2.0 ms (180°) ≈ 410 counts
   Driving the full 0–4095 range sends a ~0–20 ms pulse, which is outside any
   servo's valid window — the servo ignores it, jitters, or stalls at an end
   stop. (Torque comes from the supply voltage/current, not a wider pulse.) */
#define SERVO_PULSE_MIN 205 /* 1.0 ms @ 50 Hz → 0°   */
#define SERVO_PULSE_MAX 410 /* 2.0 ms @ 50 Hz → 180° */
#define SERVO_SETTLE_MS 350 /* time for a servo to reach target before PWM is cut */

/* PCA9685 runs at 50 Hz for the servos; the motor-enable channels share that
   frequency, which is plenty for L298N speed control.  Duty is set as a 12-bit
   on-count (0–4095). */
#define PCA_PWM_FREQ 50
#define PCA_PWM_MAX 4095

/*  Arm geometry / motion (inverse kinematics + interpolation)  */
#define ARM_SHOULDER_LEN_MM 100.0f /* shoulder pivot → elbow pivot   */
#define ARM_ELBOW_LEN_MM 100.0f    /* elbow pivot → gripper tip      */
#define ARM_STEP_DEG 2.0f          /* max joint step per interp tick  */
#define ARM_STEP_MS 15             /* interpolation tick period (ms)  */
/* Which joints stay energised when idle.  The MG90S used on shoulder/elbow
   stalls and buzzes (and overheats) when asked to hold against gravity at
   the arm's weight, so we cut PWM after every move and let gearbox friction
   hold the pose.  A small amount of droop is the trade-off. */
#define ARM_HOLD_SHOULDER false
#define ARM_HOLD_ELBOW false

/* Stall protection: an MG90S commanded to a position it can't reach (e.g. a
   binding joint) holds at full stall torque and hums, which overheats the coil
   and strips the plastic gears within minutes.  Even "held" joints get their
   PWM cut after this long so a stalled servo can never sit at full current
   indefinitely.  Cutting PWM lets gearbox friction hold the pose. */
#define SERVO_MAX_ENERGIZED_MS 600 /* hard cap on continuous energized time */

/*  Pick-and-place sequence (fixed joint angles) ─
 *  When the patrol finds an obstacle it treats it as an object: the downward
 *  colour sensor checks it, and a RED object is picked, carried, and placed
 *  90° to the LEFT by swinging the arm base (the chassis stays put).
 *
 *  These are SCRIPTED joint angles, not IK — every value below is a starting
 *  guess and MUST be CALIBRATED on the actual arm. Angles are 0–180° servo
 *  degrees on the channels in [[project-pca9685-channel-map]].
 *
 *  Gripper convention: OPEN = high angle, CLOSED = low angle. */
#define GRIP_OPEN_DEG   120 /* gripper open  — CALIBRATE */
#define GRIP_CLOSED_DEG 40  /* gripper closed (holding) — CALIBRATE */

/* Base yaw: forward (object in front) vs 90° left (drop-off beside robot). */
#define ARM_BASE_FRONT_DEG 90  /* arm pointing forward, over the object */
#define ARM_BASE_LEFT_DEG  0   /* 90° left of forward — CALIBRATE (180 = right) */

/* Shoulder/elbow poses. "DOWN" reaches to the object on the floor; "UP" is the
   raised carry pose so the object clears the ground while the base swings. */
#define ARM_SHOULDER_DOWN_DEG 140 /* reach down to floor — CALIBRATE */
#define ARM_ELBOW_DOWN_DEG    120 /* reach down to floor — CALIBRATE */
#define ARM_SHOULDER_UP_DEG   60  /* raised carry pose   — CALIBRATE */
#define ARM_ELBOW_UP_DEG      80  /* raised carry pose   — CALIBRATE */

/* Settle time after a grip open/close so the jaws actually move before the next
   step. Longer than a joint sweep because the gripper has no interpolation. */
#define GRIP_SETTLE_MS 500

/*  Wheel speed control (PID-ready, open-loop ramp for now) ─ */
#define MOTOR_SPEED_DEFAULT 100 /* default cruise duty (%) — maximum */
#define MOTOR_SPEED_MIN 0      /* allow full range down to 0%%         */
#define MOTOR_RAMP_STEP 4      /* duty % change per ramp tick         */
#define MOTOR_RAMP_MS 12       /* ramp tick period (ms)               */
/* PID gains — used once wheel encoders provide feedback (see MotorSetFeedback) */
#define MOTOR_PID_KP 1.2f
#define MOTOR_PID_KI 0.4f
#define MOTOR_PID_KD 0.05f

/*  Discrete 90° turns ─
 *  A LEFT/RIGHT command is treated as a single ~90° in-place pivot rather than
 *  a continuous spin: drive the pivot at TURN_DUTY_PCT for TURN_90_MS, then
 *  stop. Without wheel encoders this is open-loop time-based, so TURN_90_MS is
 *  a calibration value — tune it on the actual robot until a command yields a
 *  square-corner 90°. The pivot uses ~80 % duty (not full speed) for control. */
#define TURN_DUTY_PCT 80   /* ~80 % of full speed (≈204/255) for a controlled turn */
#define TURN_90_MS    600  /* time at TURN_DUTY_PCT for ~90° — CALIBRATE on robot   */

/*  TCS34725 colour sensor (optional — robot runs fine without it)  */
#define TCS34725_ADDR 0x29

/*  Analog sensors  */
#define MQ_PIN 35

/*  DHT11 ─0 */
#define DHT_PIN 15

/*  HC-SR04 ultrasonic  */
#define HC_SR04_TRIG 12
#define HC_SR04_ECHO 23

/*  Line following / obstacle avoidance 
 *  Two IR reflectance sensors (e.g. TCRT5000) wired to input-only GPIOs.
 *  Master switch below: set LINE_FOLLOWING_ENABLED to 0 to compile out the
 *  whole behaviour (the task self-deletes and never touches the motors).
 *  MUST be 1/0, not TRUE/FALSE: this feeds `#if`, where the preprocessor treats
 *  an unknown identifier like TRUE as 0 — so `#define ... TRUE` silently
 *  DISABLED the feature (that was the "line following disabled" boot bug). */
#define LINE_FOLLOWING_ENABLEcd dev

/* IR sensor pins — GPIO34/39 are input-only on the ESP32 (no pull-ups), which is
   fine because the IR modules drive the line actively HIGH/LOW. */
#define IR_LEFT_PIN 34
#define IR_RIGHT_PIN 39

/* Sensor polarity: the modules read HIGH when over the (black) line. Flip this
   one define if a future module is active-low instead. */
#define IR_ON_LINE HIGH

/* Obstacle avoidance: when the ultrasonic sees something closer than this, the
   robot stops, backs off a bit, beeps for OBSTACLE_BEEP_MS, and re-checks —
   repeating until the path is clear again. */
#define OBSTACLE_THRESHOLD_CM 50.0f
#define OBSTACLE_BEEP_MS 2000

/* On detecting an obstacle the robot reverses briefly to back off it before
   waiting for the path to clear. Open-loop time-based (no encoders), so this is
   a calibration value — tune for "a bit" on the actual robot. */
#define OBSTACLE_REVERSE_MS 400

/* How often the line-follow control loop runs. Fast enough to react, slow
   enough not to starve the rest of the system. */
#define LINE_LOOP_MS 30

/*  Patrol mode 
 *  The robot patrols: it drives FORWARD by default, follows the line when the
 *  IR sensors see one, and pauses for obstacles. An external (MQTT) drive
 *  command takes over for a hold window, then patrol resumes. An obstacle
 *  always overrides — even during a command window — for safety.
 *
 *  Hold window depends on the command: forward/backward drive for longer
 *  (PATROL_DRIVE_HOLD_MS) so a single command covers real ground; everything
 *  else (e.g. stop/brake) uses the short default. Left/right are discrete
 *  pivots handled in motor_cmd_task, so their window just needs to outlast the
 *  pivot. */
#define PATROL_CMD_HOLD_MS   1000  /* default hold for non-drive commands (ms) */
#define PATROL_DRIVE_HOLD_MS 3000  /* forward/backward run this long before patrol resumes */

/*  Buzzer / LEDs ─ */
#define BUZZER_PIN 26
#define LED_PIN_1 2 /* GPIO2 – built-in blue LED */
#define LED_PIN_2 32
#define LED_PIN_3 33

/* Buzzer volume / tone control.
 * The buzzer is driven with LEDC PWM; a low duty cycle makes it much quieter
 * than the previous full-on digitalWrite.  Tone uses a fixed frequency at a
 * gentle duty so the robot stops being so loud. */
#define BUZZER_LEDC_CHANNEL 0
#define BUZZER_LEDC_RES_BITS 8 /* 0–255 duty resolution               */
#define BUZZER_TONE_HZ 2300    /* pleasant mid tone                   */
#define BUZZER_DUTY 12         /* ~5 % of 255 — quiet (was full-on)   */
#define BUZZER_BEEP_MS 25      /* short, soft beep by default         */

/*  GPS (UART2) ─ */
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

/*  GSM ─ */
#define GSM_RX_PIN 32
#define GSM_TX_PIN 33
#define GSM_BAUD 9600
#define GSM_APN "safaricom"

/*  WiFi  */
#define WIFI_SSID "DXS"
#define WIFI_PASSWORD "$Codex2016@Phoenix-FR1507"

/*  MQTT  */
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_USER "astian"
#define MQTT_PASS "muchui"
#define DEVICE_ID "WRBT202642"
#define MQTT_CLIENT_ID DEVICE_ID

#define PUBLISH_INTERVAL_MS 5000

/* How often the network task polls the MQTT socket for inbound commands. Must
   be short so commands dispatch promptly (not just at the publish interval) —
   PubSubClient isn't thread-safe, so this is the ONLY place the socket is
   serviced. 50 ms keeps command latency low without hammering the CPU. */
#define MQTT_POLL_MS 50

/*  MQTT topics ─ */
#define TOPIC_BASE "robot/devices/" DEVICE_ID
#define TOPIC_BOOT TOPIC_BASE "/boot"
#define TOPIC_READINGS TOPIC_BASE "/readings"
#define TOPIC_HEARTBEAT TOPIC_BASE "/heartbeat"
#define TOPIC_EVENT_MOTION TOPIC_BASE "/events/motion"
#define TOPIC_EVENT_FREEFALL TOPIC_BASE "/events/freefall"
#define TOPIC_EVENT_ZERO_MOTION TOPIC_BASE "/events/zero_motion"

/* Inbound command topics */
#define TOPIC_CMD_DRIVE "robot/cmd/drive" /* {"cmd":"forward|backward|left|right|stop|brake","speed":0-100} */
#define TOPIC_CMD_ARM "robot/cmd/arm"     /* {"joint":"base|shoulder|elbow|gripper","angle":0-180,"hold":bool} */
#define TOPIC_CMD_POSE "robot/cmd/pose"   /* {"x":mm,"y":mm,"z":mm,"gripper":0-180} — inverse kinematics */

/*  FreeRTOS queues ─ */
extern QueueHandle_t g_dht_queue;
extern QueueHandle_t g_imu_queue;
extern QueueHandle_t g_sonic_queue;
extern QueueHandle_t g_mq135_queue;
extern QueueHandle_t g_gps_queue;
extern QueueHandle_t g_event_queue;
extern QueueHandle_t g_motor_cmd_queue; /* patrol task → motor_cmd_task (sole producer) */
extern QueueHandle_t g_drive_cmd_queue; /* MQTT → patrol task: external drive overrides   */
extern QueueHandle_t g_servo_cmd_queue;
extern QueueHandle_t g_pose_cmd_queue;
extern QueueHandle_t g_color_queue;

/*  Function declarations ─ */
void initialize_pins();
void InitBuzzer();
void Pulsate(uint8_t pin, uint8_t iter, uint16_t duration); /* quiet PWM beeps */
void Beep(uint8_t iter);                                    /* convenience: `iter` short soft beeps */

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

/*  Line following / obstacle avoidance  */
void InitLineSensors();
bool ReadIRLeft();   /* true when the left sensor is over the line  */
bool ReadIRRight();  /* true when the right sensor is over the line */

mq135_data_t ReadMQ135();

void GPS_INIT();
void GPS_DRAIN();
gps_data_t READ_GPS();

/* PCF8574 I/O expander disabled — driver removed (all I/O moved to PCA9685). */

/* Motor driver: direction via PCF8574, speed (duty 0–100 %) via PCA9685 PWM. */
void MotorInit();
void MotorSetDirection(motor_dir_t dir); /* set IN1–IN4 only (no speed change) */
void MotorSetDuty(uint8_t duty_pct);     /* set ENA/ENB PWM duty 0–100 %        */
/* Hook for closed-loop control once wheel encoders exist; left as a stub. */
void MotorSetFeedback(float left_rpm, float right_rpm);

/* PCA9685 PWM driver (shared by servos and motor enables). */
void InitServoDriver();
void SetServoAngle(uint8_t channel, uint8_t angle);
void MoveServoSmooth(uint8_t channel, uint8_t target); /* interpolated move */
void DisableServo(uint8_t channel);
void SetPCAPwm(uint8_t channel, uint16_t on_count); /* raw duty 0–4095 */

/* Arm inverse kinematics: solve a Cartesian target into joint angles. Returns
   false if the target is unreachable (out of the arm's reach envelope). */
bool ArmSolveIK(float x, float y, float z,
                uint8_t *base_deg, uint8_t *shoulder_deg, uint8_t *elbow_deg);

/* Scripted pick-and-place: reach down in front, close the gripper on the
   object, raise it, swing the base 90° left, lower, release, then park. Uses
   the fixed ARM_ / GRIP_ angle constants above. Blocks for the whole sequence. */
void PickPlaceRedObject();
void RedObjectGripSequence();

/*  TCS34725 colour sensor ─
 *  InitializeColorSensor() probes the bus and remembers whether the sensor is
 *  present, so the rest of the firmware can run unchanged when it is absent.
 *  is_color_sensor_enabled() reflects that result.
 *  */
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
