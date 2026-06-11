#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "defines.h"

static Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);

/* Last commanded angle, indexed by PCA9685 channel (0–15).  The arm servos now
   live on channels 12–15, so this must be sized to the full channel count — a
   4-element array indexed by channel 12+ would be an out-of-bounds write.
   Seeded to 90° so the first interpolated move starts from the parked pose. */
static uint8_t s_last_angle[16] = {
    90, 90, 90, 90, 90, 90, 90, 90,
    90, 90, 90, 90, 90, 90, 90, 90,
};

/* True for the four arm-servo channels (12–15), which get angle tracking and
   smooth interpolation; other channels (e.g. motor enables) do not. */
static inline bool is_servo_channel(uint8_t ch)
{
    return ch == SERVO_CH_GRIPPER  || ch == SERVO_CH_SHOULDER ||
           ch == SERVO_CH_ELBOW    || ch == SERVO_CH_BASE;
}

void InitServoDriver()
{
    pca9685.begin();
    pca9685.setPWMFreq(PCA_PWM_FREQ);
    delay(10);

    /* Adafruit's begin() returns void, so verify the chip actually ACKs on the
       bus. If this fails, no servo or motor will ever move — surface it loudly
       rather than silently driving a chip that isn't there. */
    Wire.beginTransmission(PCA9685_ADDR);
    if (Wire.endTransmission() == 0)
        Serial.printf("[Servo] PCA9685 ACK at 0x%02X\n", PCA9685_ADDR);
    else
        Serial.printf("[Servo] PCA9685 NOT responding at 0x%02X — nothing will move!\n", PCA9685_ADDR);

    /* Zero the motor enable channels so wheels don't spin on boot */
    MotorSetDuty(0);

    /* Park all joints at 90°. Load-bearing joints (shoulder/elbow) stay
       energised for holding torque; base/gripper release to stay cool & quiet. */

    SetServoAngle(SERVO_CH_BASE,     90);
    SetServoAngle(SERVO_CH_SHOULDER, 90);
    SetServoAngle(SERVO_CH_ELBOW,    90);
    SetServoAngle(SERVO_CH_GRIPPER,  90);
    delay(SERVO_SETTLE_MS);

    DisableServo(SERVO_CH_BASE);
    if (!ARM_HOLD_SHOULDER) DisableServo(SERVO_CH_SHOULDER);
    if (!ARM_HOLD_ELBOW)    DisableServo(SERVO_CH_ELBOW);
    DisableServo(SERVO_CH_GRIPPER);

    Serial.println("[Servo] PCA9685 ready — parked at 90°, shoulder/elbow holding, motor enables at 0");
}

void SetServoAngle(uint8_t channel, uint8_t angle)
{
    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    pca9685.setPWM(channel, 0, pulse);
    if (channel < 16)
        s_last_angle[channel] = angle;
}

void DisableServo(uint8_t channel)
{
    /* Full LOW on both on/off counts cuts PWM without changing the
       physical register state — servo coasts silently to held position */
    pca9685.setPWM(channel, 0, 0);
}

/* ── Smooth single-joint move ──────────────────────────────────────────────────
 *  Interpolates from the joint's current angle to `target` in small steps so the
 *  motion is gentle instead of slamming to position.  The servo is held at full
 *  power for the whole sweep (no PWM gaps), which is also why the weak
 *  shoulder/elbow feel stronger.  Re-disabling idle joints is left to the caller.
 * ──────────────────────────────────────────────────────────────────────────── */
void MoveServoSmooth(uint8_t channel, uint8_t target)
{
    if (!is_servo_channel(channel)) { SetServoAngle(channel, target); return; }

    target = constrain(target, 0, 180);
    int from = s_last_angle[channel];
    int step = (target >= from) ? (int)ARM_STEP_DEG : -(int)ARM_STEP_DEG;

    for (int a = from; (step > 0) ? (a < target) : (a > target); a += step)
    {
        SetServoAngle(channel, (uint8_t)a);
        vTaskDelay(pdMS_TO_TICKS(ARM_STEP_MS));
    }
    SetServoAngle(channel, target);   /* land exactly on target */
}

/* ── Raw PCA9685 PWM helper ────────────────────────────────────────────────── */

/* Set any PCA9685 channel to a 12-bit duty (0–4095 ticks active out of 4096).
   Use setPin() rather than setPWM(ch,0,count): setPin() applies the PCA9685's
   special full-on (4096) / full-off bits at the 0 and 4095 endpoints, so an
   L298N enable/IN line driven to 4095 is a clean steady HIGH instead of a
   pulse with a one-tick dropout every cycle. */
void SetPCAPwm(uint8_t channel, uint16_t on_count)
{
    on_count = (on_count > PCA_PWM_MAX) ? PCA_PWM_MAX : on_count;
    pca9685.setPin(channel, on_count);
}

/* ── Scripted pick-and-place ──────────────────────────────────────────────────
 *  Called when the patrol confirms a RED object ahead. Sequence:
 *    1. Open gripper, swing base to front, reach down to the floor.
 *    2. Close gripper on the object.
 *    3. Raise to the carry pose (object clears the ground).
 *    4. Swing base 90° left to the drop-off.
 *    5. Lower, open gripper to release.
 *    6. Raise and park back at front/90°.
 *  All angles are the fixed ARM_ / GRIP_ constants (calibrate on the robot).
 *  Blocks the caller for the whole move; joints are released between steps so a
 *  weak/stalling MG90S never sits at full current. */
void PickPlaceRedObject()
{
    Serial.println("[Arm] RED object — pick-and-place start");

    /* 1. Approach: open jaws, face front, reach down. */
    SetServoAngle(SERVO_CH_GRIPPER, GRIP_OPEN_DEG);
    MoveServoSmooth(SERVO_CH_BASE,     ARM_BASE_FRONT_DEG);
    MoveServoSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_DOWN_DEG);
    MoveServoSmooth(SERVO_CH_ELBOW,    ARM_ELBOW_DOWN_DEG);
    delay(GRIP_SETTLE_MS);

    /* 2. Grip. Hold the gripper energised while carrying so it keeps its clamp. */
    SetServoAngle(SERVO_CH_GRIPPER, GRIP_CLOSED_DEG);
    delay(GRIP_SETTLE_MS);

    /* 3. Lift to carry pose. */
    MoveServoSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_UP_DEG);
    MoveServoSmooth(SERVO_CH_ELBOW,    ARM_ELBOW_UP_DEG);

    /* 4. Carry: swing the base 90° left to the drop-off. */
    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_LEFT_DEG);

    /* 5. Lower and release. */
    MoveServoSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_DOWN_DEG);
    MoveServoSmooth(SERVO_CH_ELBOW,    ARM_ELBOW_DOWN_DEG);
    delay(GRIP_SETTLE_MS);
    SetServoAngle(SERVO_CH_GRIPPER, GRIP_OPEN_DEG);
    delay(GRIP_SETTLE_MS);

    /* 6. Raise clear, swing back to front, park. */
    MoveServoSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_UP_DEG);
    MoveServoSmooth(SERVO_CH_ELBOW,    ARM_ELBOW_UP_DEG);
    MoveServoSmooth(SERVO_CH_BASE,     ARM_BASE_FRONT_DEG);

    /* Release every joint so nothing sits at stall current. */
    DisableServo(SERVO_CH_GRIPPER);
    DisableServo(SERVO_CH_ELBOW);
    DisableServo(SERVO_CH_SHOULDER);
    DisableServo(SERVO_CH_BASE);

    Serial.println("[Arm] Pick-and-place done — object placed 90° left");
}

/* ── Red-object gripper routine ────────────────────────────────────────────────
 *  Called when the colour sensor sees RED. Simple scripted sequence, no checks:
 *    1. Open the gripper.
 *    2. Close it after 3 s.
 *    3. Turn the base 90° (to the left drop-off position).
 *    4. Open the gripper again to release.
 *  Blocks the caller for the whole sequence. Joints are released at the end so
 *  nothing sits at stall current.
 * ──────────────────────────────────────────────────────────────────────────── */
void RedObjectGripSequence()
{
    Serial.println("[Arm] RED detected — grip sequence start");

    /* 1. Open gripper */
    SetServoAngle(SERVO_CH_GRIPPER, GRIP_OPEN_DEG);
    delay(3000);

    /* 2. Close gripper after 3 s */
    SetServoAngle(SERVO_CH_GRIPPER, GRIP_CLOSED_DEG);
    delay(GRIP_SETTLE_MS);

    /* 3. Turn the base 90° */
    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_LEFT_DEG);

    /* 4. Open gripper again to release */
    SetServoAngle(SERVO_CH_GRIPPER, GRIP_OPEN_DEG);
    delay(GRIP_SETTLE_MS);

    /* Park the base back to front and release every joint */
    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_FRONT_DEG);
    DisableServo(SERVO_CH_GRIPPER);
    DisableServo(SERVO_CH_BASE);

    Serial.println("[Arm] RED grip sequence done");
}

/* ── Closed-loop stub ──────────────────────────────────────────────────────── */

/* Placeholder for wheel encoder feedback once encoders are fitted.
   Currently a no-op; a PID controller will live here once left_rpm / right_rpm
   are available from hardware interrupts on the encoder GPIO lines. */
void MotorSetFeedback(float left_rpm, float right_rpm)
{
    (void)left_rpm;
    (void)right_rpm;
    /* TODO: PID speed correction using MOTOR_PID_KP/KI/KD */
}

/* ── Arm inverse kinematics ────────────────────────────────────────────────── */

/* Solve a 2-link planar arm with a yaw base into joint angles.
 *
 *  Geometry (arm at 0° base, viewed from above / side):
 *    - Base   rotates the arm's plane around the vertical axis (yaw).
 *    - Shoulder and elbow are revolute joints in that vertical plane.
 *    - L1 = ARM_SHOULDER_LEN_MM  (shoulder pivot → elbow pivot)
 *    - L2 = ARM_ELBOW_LEN_MM     (elbow pivot → gripper tip)
 *
 *  Coordinate system:
 *    - x, y  horizontal plane measured from the base pivot.
 *    - z     height above the shoulder pivot.
 *
 *  Returns false when the target lies outside the arm's reach envelope or
 *  is mathematically unsolvable (e.g. directly above the pivot). */
bool ArmSolveIK(float x, float y, float z,
                uint8_t *base_deg, uint8_t *shoulder_deg, uint8_t *elbow_deg)
{
    const float L1 = ARM_SHOULDER_LEN_MM;
    const float L2 = ARM_ELBOW_LEN_MM;

    /* ── 1. Base yaw from horizontal projection ──────────────────────────── */
    float base_rad = atan2f(y, x);           /* −π … +π                   */
    float base_d   = base_rad * 180.0f / M_PI + 90.0f;  /* map to 0–180 servo range  */
    if (base_d < 0.0f || base_d > 180.0f)
        return false;   /* target is behind the robot's reachable arc */

    /* ── 2. Reach distance in the arm's vertical plane ───────────────────── */
    float r   = sqrtf(x * x + y * y);       /* horizontal reach (mm) */
    float r2  = r * r + z * z;              /* squared 3D distance   */
    float reach = sqrtf(r2);

    if (reach > (L1 + L2) || reach < fabsf(L1 - L2))
        return false;   /* out of reach envelope */

    /* ── 3. Elbow angle via cosine rule ──────────────────────────────────── */
    /* cos(elbow) = (r² + z² - L1² - L2²) / (2·L1·L2) */
    float cos_e = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cos_e = constrain(cos_e, -1.0f, 1.0f);          /* numerical safety clamp */
    float elbow_rad = acosf(cos_e);                  /* 0 … π (elbow-up)       */

    /* ── 4. Shoulder angle ───────────────────────────────────────────────── */
    float k1 = L1 + L2 * cosf(elbow_rad);
    float k2 = L2 * sinf(elbow_rad);
    float shoulder_rad = atan2f(z, r) - atan2f(k2, k1);

    /* Convert to servo degrees (0–180). Shoulder 0 = arm straight down,
       90 = horizontal, 180 = straight up.  We offset by 90° so that 90
       maps to the horizontal "park" position. */
    float shoulder_d = shoulder_rad * 180.0f / M_PI + 90.0f;
    float elbow_d    = elbow_rad    * 180.0f / M_PI;

    if (shoulder_d < 0.0f || shoulder_d > 180.0f) return false;
    if (elbow_d    < 0.0f || elbow_d    > 180.0f) return false;

    *base_deg     = (uint8_t)constrain((int)base_d,     0, 180);
    *shoulder_deg = (uint8_t)constrain((int)shoulder_d, 0, 180);
    *elbow_deg    = (uint8_t)constrain((int)elbow_d,    0, 180);
    return true;
}
