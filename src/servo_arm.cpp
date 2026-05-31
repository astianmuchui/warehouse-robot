#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "defines.h"

static Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);

/* Last commanded angle per servo channel.  Seeded to 90° so the first
   interpolated move starts from the parked pose instead of jumping. */
static uint8_t s_last_angle[4] = { 90, 90, 90, 90 };

void InitServoDriver()
{
    pca9685.begin();
    pca9685.setPWMFreq(PCA_PWM_FREQ);
    delay(10);

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
    if (channel < 4)
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
    if (channel >= 4) { SetServoAngle(channel, target); return; }

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

/* Set any PCA9685 channel to an arbitrary 12-bit on-count (0–4095).
   on=0 means always-off; on=4096 means always-on. */
void SetPCAPwm(uint8_t channel, uint16_t on_count)
{
    on_count = (on_count > PCA_PWM_MAX) ? PCA_PWM_MAX : on_count;
    pca9685.setPWM(channel, 0, on_count);
}

/* ── Motor speed control via PCA9685 ───────────────────────────────────────── */

/* Set both motor enable channels (ENA = ch14, ENB = ch15) to the same duty.
   duty_pct is 0–100; anything > 100 is clamped.
   MOTOR_SPEED_MIN (35 %) is the stall threshold — below that both channels
   are zeroed to avoid motor growling at a stall-level duty. */
void MotorSetDuty(uint8_t duty_pct)
{
    duty_pct = (duty_pct > 100) ? 100 : duty_pct;

    uint16_t on_count = 0;
    if (duty_pct == 0)
    {
        on_count = 0;
    }
    else if (duty_pct < MOTOR_SPEED_MIN)
    {
        /* Clamp to minimum to prevent stalling — or zero if caller wants full stop */
        on_count = (uint16_t)((MOTOR_SPEED_MIN * (uint32_t)PCA_PWM_MAX) / 100);
    }
    else
    {
        on_count = (uint16_t)((duty_pct * (uint32_t)PCA_PWM_MAX) / 100);
    }

    SetPCAPwm(MOTOR_PWM_ENA, on_count);
    SetPCAPwm(MOTOR_PWM_ENB, on_count);
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
