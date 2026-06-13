#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "defines.h"

static Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);

/* Last commanded angle per PCA9685 channel. Full 16 wide (not 4) so indexing by
   raw channel number is always in bounds. Seeded to 90° = parked. */
static uint8_t s_last_angle[16] = {
    90, 90, 90, 90, 90, 90, 90, 90,
    90, 90, 90, 90, 90, 90, 90, 90,
};

/** is_servo_channel - true for the four arm joints (they get angle tracking). */
static inline bool is_servo_channel(uint8_t ch)
{
    return ch == SERVO_CH_GRIPPER  || ch == SERVO_CH_SHOULDER ||
           ch == SERVO_CH_ELBOW    || ch == SERVO_CH_BASE;
}

/** InitServoDriver - start the PCA9685, check it ACKs, park all joints at 90°. */
void InitServoDriver()
{
    pca9685.begin();
    pca9685.setPWMFreq(PCA_PWM_FREQ);
    delay(10);

    /* begin() returns void, so confirm the chip is actually there. If it isn't,
       nothing moves — say so loudly. */
    Wire.beginTransmission(PCA9685_ADDR);
    if (Wire.endTransmission() == 0)
        Serial.printf("[Servo] PCA9685 ACK at 0x%02X\n", PCA9685_ADDR);
    else
        Serial.printf("[Servo] PCA9685 NOT responding at 0x%02X — nothing will move!\n", PCA9685_ADDR);

    MotorSetDuty(0);

    SetServoAngle(SERVO_CH_BASE,     90);
    SetServoAngle(SERVO_CH_SHOULDER, 90);
    SetServoAngle(SERVO_CH_ELBOW,    90);
    SetServoAngle(SERVO_CH_GRIPPER,  90);
    delay(SERVO_SETTLE_MS);

    DisableServo(SERVO_CH_BASE);
    if (!ARM_HOLD_SHOULDER) DisableServo(SERVO_CH_SHOULDER);
    if (!ARM_HOLD_ELBOW)    DisableServo(SERVO_CH_ELBOW);
    DisableServo(SERVO_CH_GRIPPER);

    Serial.println("[Servo] PCA9685 ready — parked at 90°");
}

/** SetServoAngle - map 0-180° into the valid pulse band and write it. */
void SetServoAngle(uint8_t channel, uint8_t angle)
{
    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    pca9685.setPWM(channel, 0, pulse);
    if (channel < 16)
        s_last_angle[channel] = angle;
}

/** DisableServo - cut PWM so the servo coasts silently to its held position. */
void DisableServo(uint8_t channel)
{
    pca9685.setPWM(channel, 0, 0);
}

/**
 * MoveServoSmooth - interpolate from the joint's current angle to target in
 * small steps so the motion is gentle. Stays energised the whole sweep;
 * re-disabling idle joints is the caller's job.
 */
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
    SetServoAngle(channel, target);
}

/**
 * MoveTwoServosSmooth - step two joints toward their targets together. Doesn't
 * add torque (each servo still fights its own load); it keeps the arm tucked so
 * the shoulder sees a lower PEAK load than a sequential shoulder-then-elbow move.
 */
void MoveTwoServosSmooth(uint8_t ch_a, uint8_t tgt_a, uint8_t ch_b, uint8_t tgt_b)
{
    if (!is_servo_channel(ch_a) || !is_servo_channel(ch_b))
    {
        MoveServoSmooth(ch_a, tgt_a);
        MoveServoSmooth(ch_b, tgt_b);
        return;
    }

    tgt_a = constrain(tgt_a, 0, 180);
    tgt_b = constrain(tgt_b, 0, 180);

    int a = s_last_angle[ch_a];
    int b = s_last_angle[ch_b];

    while (a != (int)tgt_a || b != (int)tgt_b)
    {
        if (a < (int)tgt_a) a = min(a + (int)ARM_STEP_DEG, (int)tgt_a);
        else if (a > (int)tgt_a) a = max(a - (int)ARM_STEP_DEG, (int)tgt_a);

        if (b < (int)tgt_b) b = min(b + (int)ARM_STEP_DEG, (int)tgt_b);
        else if (b > (int)tgt_b) b = max(b - (int)ARM_STEP_DEG, (int)tgt_b);

        SetServoAngle(ch_a, (uint8_t)a);
        SetServoAngle(ch_b, (uint8_t)b);
        vTaskDelay(pdMS_TO_TICKS(ARM_STEP_MS));
    }
}

/** SetPCAPwm - raw 12-bit duty. setPin() gives clean full-on/off at the ends. */
void SetPCAPwm(uint8_t channel, uint16_t on_count)
{
    on_count = (on_count > PCA_PWM_MAX) ? PCA_PWM_MAX : on_count;
    pca9685.setPin(channel, on_count);
}

/* Drive the gripper exactly the way a manual MQTT `robot/cmd/arm` command does:
   post a servo_cmd_t to g_servo_cmd_queue and let servo_cmd_task move it. The
   queue path is the one confirmed to drive the gripper correctly, so the
   sequences reuse it; the existing delays pace the async move. */
static void grip_via_queue(uint8_t angle)
{
    servo_cmd_t scmd;
    scmd.channel = SERVO_CH_GRIPPER;
    scmd.angle   = angle;
    scmd.hold    = false;
    if (xQueueSend(g_servo_cmd_queue, &scmd, 0) != pdTRUE)
        Serial.println("[Arm] Servo queue full — gripper command dropped");
}

/**
 * PickPlaceRedObject - reach down, grip, lift, swing the base 90° left, lower,
 * release, park. Shoulder+elbow move together so the arm stays tucked through
 * the heavy part. Blocks for the whole sequence. All angles need calibrating.
 */
void PickPlaceRedObject()
{
    Serial.println("[Arm] RED object — pick-and-place start");

    /* 1. Open, face front, reach down (tucked). */
    grip_via_queue(GRIP_OPEN_DEG);
    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_FRONT_DEG);
    MoveTwoServosSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_DOWN_DEG,
                        SERVO_CH_ELBOW,    ARM_ELBOW_DOWN_DEG);
    delay(GRIP_SETTLE_MS);

    /* 2. Grip. */
    grip_via_queue(GRIP_CLOSED_DEG);
    delay(GRIP_SETTLE_MS);

    /* 3. Lift to carry pose. */
    MoveTwoServosSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_UP_DEG,
                        SERVO_CH_ELBOW,    ARM_ELBOW_UP_DEG);

    /* 4. Swing base 90° left. */
    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_LEFT_DEG);

    /* 5. Lower and release. */
    MoveTwoServosSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_DOWN_DEG,
                        SERVO_CH_ELBOW,    ARM_ELBOW_DOWN_DEG);
    delay(GRIP_SETTLE_MS);
    grip_via_queue(GRIP_OPEN_DEG);
    delay(GRIP_SETTLE_MS);

    /* 6. Raise clear, swing back, park. */
    MoveTwoServosSmooth(SERVO_CH_SHOULDER, ARM_SHOULDER_UP_DEG,
                        SERVO_CH_ELBOW,    ARM_ELBOW_UP_DEG);
    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_FRONT_DEG);

    DisableServo(SERVO_CH_GRIPPER);
    DisableServo(SERVO_CH_ELBOW);
    DisableServo(SERVO_CH_SHOULDER);
    DisableServo(SERVO_CH_BASE);

    Serial.println("[Arm] Pick-and-place done — object placed 90° left");
}

/**
 * RedObjectGripSequence - simpler routine the colour task runs on a RED edge:
 * open, close after 3 s, swing base 90° left, open to release, park.
 */
void RedObjectGripSequence()
{
    Serial.println("[Arm] RED detected — grip sequence start");

    grip_via_queue(GRIP_OPEN_DEG);
    delay(3000);

    grip_via_queue(GRIP_CLOSED_DEG);
    delay(GRIP_SETTLE_MS);

    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_LEFT_DEG);

    grip_via_queue(GRIP_OPEN_DEG);
    delay(GRIP_SETTLE_MS);

    MoveServoSmooth(SERVO_CH_BASE, ARM_BASE_FRONT_DEG);
    DisableServo(SERVO_CH_GRIPPER);
    DisableServo(SERVO_CH_BASE);

    Serial.println("[Arm] RED grip sequence done");
}

/** MotorSetFeedback - encoder/PID hook, no-op until encoders are fitted. */
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
