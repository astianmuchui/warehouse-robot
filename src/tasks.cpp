#include <Arduino.h>
#include <time.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>

#include "defines.h"

static SemaphoreHandle_t s_dht_sem;
static SemaphoreHandle_t s_imu_sem;
static SemaphoreHandle_t s_sonic_sem;
static SemaphoreHandle_t s_mq135_sem;
static SemaphoreHandle_t s_color_sem;

static void dht_timer_cb(TimerHandle_t)   { xSemaphoreGive(s_dht_sem);   }
static void imu_timer_cb(TimerHandle_t)   { xSemaphoreGive(s_imu_sem);   }
static void sonic_timer_cb(TimerHandle_t) { xSemaphoreGive(s_sonic_sem); }
static void mq135_timer_cb(TimerHandle_t) { xSemaphoreGive(s_mq135_sem); }
static void color_timer_cb(TimerHandle_t) { xSemaphoreGive(s_color_sem); }

void init_sensor_timers()
{
    s_dht_sem   = xSemaphoreCreateBinary();
    s_imu_sem   = xSemaphoreCreateBinary();
    s_sonic_sem = xSemaphoreCreateBinary();
    s_mq135_sem = xSemaphoreCreateBinary();
    s_color_sem = xSemaphoreCreateBinary();

    xTimerStart(xTimerCreate("dht",   pdMS_TO_TICKS(2000), pdTRUE, NULL, dht_timer_cb),   0);
    xTimerStart(xTimerCreate("imu",   pdMS_TO_TICKS(500),  pdTRUE, NULL, imu_timer_cb),   0);
    xTimerStart(xTimerCreate("sonic", pdMS_TO_TICKS(1000), pdTRUE, NULL, sonic_timer_cb), 0);
    xTimerStart(xTimerCreate("mq135", pdMS_TO_TICKS(2000), pdTRUE, NULL, mq135_timer_cb), 0);
    xTimerStart(xTimerCreate("color", pdMS_TO_TICKS(1000), pdTRUE, NULL, color_timer_cb), 0);
}

/* ── Sensor tasks ──────────────────────────────────────────────────────────── */

void dht_task(void *)
{
    while (true)
    {
        xSemaphoreTake(s_dht_sem, portMAX_DELAY);

        dht_data_t d = ReadDHT();
        if (!isnan(d.temperature) && !isnan(d.humidity))
        {
            xQueueOverwrite(g_dht_queue, &d);
            Serial.printf("[DHT] %.1f°C  %.1f %%RH\n", d.temperature, d.humidity);

            /* Warn with rapid double-beep if temperature is dangerously high */
            if (d.temperature > 50.0f)
                Pulsate(BUZZER_PIN, 2, 80);
        }
        else
        {
            Serial.println("[DHT] Read failed (NaN)");
        }
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS / 2));
    }
}

void imu_task(void *)
{
    if (!is_imu_detected())
    {
        Serial.println("[IMU] Sensor disabled — task not running");
        vTaskDelete(nullptr);
        return;
    }

    while (true)
    {
        xSemaphoreTake(s_imu_sem, portMAX_DELAY);

        imu_reading_t r = ReadIMU();
        xQueueOverwrite(g_imu_queue, &r);

        Serial.printf("[IMU] A(%.2f, %.2f, %.2f) m/s²  G(%.3f, %.3f, %.3f) rad/s  %.1f°C\n",
                      r.accel.x, r.accel.y, r.accel.z,
                      r.gyro.x,  r.gyro.y,  r.gyro.z,
                      r.temperature);

        imu_event_type_t ev_type = CheckIMUEvents();
        if (ev_type != IMU_EVENT_NONE)
        {
            imu_event_t ev;
            ev.type        = ev_type;
            ev.timestamp_s = (uint32_t)time(nullptr);
            ev.uptime_s    = millis() / 1000;
            ev.accel_x     = r.accel.x;
            ev.accel_y     = r.accel.y;
            ev.accel_z     = r.accel.z;

            if (xQueueSend(g_event_queue, &ev, 0) == pdTRUE)
                Serial.printf("[IMU] Event type %d queued\n", (int)ev_type);
            else
                Serial.println("[IMU] Event queue full — dropped");

            /* Beep pattern per event type */
            if (ev_type == IMU_EVENT_FREEFALL)
                Pulsate(BUZZER_PIN, 3, 60);   /* freefall: triple urgent */
            else
                Pulsate(BUZZER_PIN, 1, 80);   /* motion / zero-motion: single */
        }
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS / 2));
    }
}

void ultrasonic_task(void *)
{
    while (true)
    {
        xSemaphoreTake(s_sonic_sem, portMAX_DELAY);

        float dist = ReadUltrasonic();
        xQueueOverwrite(g_sonic_queue, &dist);

        if (dist > 0.0f)
        {
            Serial.printf("[US] %.1f cm\n", dist);

            if (dist < 15.0f)
                Pulsate(BUZZER_PIN, 3, 50);   /* danger-close */
            else if (dist < 30.0f)
                Pulsate(BUZZER_PIN, 1, 80);   /* caution */
        }
        else
        {
            Serial.println("[US] No echo (out of range or timeout)");
        }
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS / 2));
    }
}

void mq135_task(void *)
{
    Serial.println("[MQ135] Warm-up: 2 minutes...");
    vTaskDelay(pdMS_TO_TICKS(120000));
    Serial.println("[MQ135] Warm-up complete");

    while (true)
    {
        xSemaphoreTake(s_mq135_sem, portMAX_DELAY);

        mq135_data_t d = ReadMQ135();
        xQueueOverwrite(g_mq135_queue, &d);

        Serial.printf("[MQ135] %.1f ppm  (%.3f V)\n", d.ppm, d.voltage);

        /* Warn if air quality crosses a threshold */
        if (d.ppm > 400.0f)
            Pulsate(BUZZER_PIN, 2, 100);

        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS / 2));
    }
}

/* ── Drive tasks ───────────────────────────────────────────────────────────── */

/* Shared target duty between motor_cmd_task (writer) and motor_speed_task
   (reader).  Declared volatile so the compiler never caches the value across
   the task boundary.  Both tasks run on CPU0 so a single 32-bit write is
   atomic on Xtensa, but the ISR-safe critical section is cheap insurance. */
static volatile uint8_t  s_target_duty = 0;
static volatile motor_dir_t s_current_dir = MOTOR_STOP;
static portMUX_TYPE      s_motor_mux = portMUX_INITIALIZER_UNLOCKED;

/* motor_cmd_task: receives {dir, speed} commands from the MQTT callback,
   commits the direction to the PCA9685 immediately, and posts the target
   speed for motor_speed_task to ramp toward. */
void motor_cmd_task(void *)
{
    static const char *names[] = {
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "BRAKE"
    };

    motor_cmd_t cmd;
    motor_dir_t prev_dir = MOTOR_STOP;

    /* Ensure motors start stopped */
    MotorSetDirection(MOTOR_STOP);
    portENTER_CRITICAL(&s_motor_mux);
    s_target_duty = 0;
    s_current_dir = MOTOR_STOP;
    portEXIT_CRITICAL(&s_motor_mux);

    while (true)
    {
        if (xQueueReceive(g_motor_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        bool dir_changed = (cmd.dir != prev_dir);

        /* ── Discrete 90° turn ────────────────────────────────────────────────
           An *external* LEFT/RIGHT command (cmd.pivot == true) is a single
           in-place pivot, not a continuous spin: pivot at TURN_DUTY_PCT for
           TURN_90_MS, then stop. We drive the duty directly (bypassing the slow
           ramp task) because the turn window is short — the ramp would eat most
           of it and the robot would barely move. After the dwell we hand control
           back to STOP. Line-follow corrections set pivot=false and fall through
           to the continuous-turn path below, so steering stays smooth. */
        if ((cmd.dir == MOTOR_LEFT || cmd.dir == MOTOR_RIGHT) && cmd.pivot)
        {
            MotorSetDirection(cmd.dir);

            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = TURN_DUTY_PCT;   /* keep ramp task in sync, no fight */
            s_current_dir = cmd.dir;
            portEXIT_CRITICAL(&s_motor_mux);

            MotorSetDuty(TURN_DUTY_PCT);     /* full 80 % immediately, no ramp */
            Serial.printf("[Motor] %s 90° pivot @ %u%% for %u ms\n",
                          names[(int)cmd.dir], (unsigned)TURN_DUTY_PCT,
                          (unsigned)TURN_90_MS);
            if (dir_changed) Pulsate(BUZZER_PIN, 1, 60);

            vTaskDelay(pdMS_TO_TICKS(TURN_90_MS));

            /* Stop and settle back into the ramp-managed STOP state */
            MotorSetDirection(MOTOR_STOP);
            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = 0;
            s_current_dir = MOTOR_STOP;
            portEXIT_CRITICAL(&s_motor_mux);
            MotorSetDuty(0);

            prev_dir = MOTOR_STOP;   /* turn left us stopped, not turning */
            continue;
        }

        /* ── Timed forward/backward ───────────────────────────────────────────
           On a FORWARD/BACKWARD command just drive at full duty immediately and
           run for 3 s, then stop. No ramp, no checks — same direct-drive + dwell
           pattern as the LEFT/RIGHT pivot above. */
        if (cmd.dir == MOTOR_FORWARD || cmd.dir == MOTOR_BACKWARD)
        {
            MotorSetDirection(cmd.dir);

            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = 100;             /* keep ramp task in sync, no fight */
            s_current_dir = cmd.dir;
            portEXIT_CRITICAL(&s_motor_mux);

            MotorSetDuty(100);               /* full speed immediately, no ramp */
            Serial.printf("[Motor] %s @ 100%% for 3000 ms\n", names[(int)cmd.dir]);
            if (dir_changed) Pulsate(BUZZER_PIN, 1, 60);

            vTaskDelay(pdMS_TO_TICKS(3000));

            /* Stop and settle back into the ramp-managed STOP state */
            MotorSetDirection(MOTOR_STOP);
            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = 0;
            s_current_dir = MOTOR_STOP;
            portEXIT_CRITICAL(&s_motor_mux);
            MotorSetDuty(0);

            prev_dir = MOTOR_STOP;
            continue;
        }

        /* Resolve speed: -1 means "use cruise default" */
        uint8_t duty = (cmd.speed < 0) ? MOTOR_SPEED_DEFAULT
                                       : (uint8_t)constrain(cmd.speed, 0, 100);

        /* STOP and BRAKE always zero the speed target */
        if (cmd.dir == MOTOR_STOP || cmd.dir == MOTOR_BRAKE)
            duty = 0;

        MotorSetDirection(cmd.dir);

        portENTER_CRITICAL(&s_motor_mux);
        s_target_duty = duty;
        s_current_dir = cmd.dir;
        portEXIT_CRITICAL(&s_motor_mux);

        prev_dir = cmd.dir;

        const char *name = (cmd.dir <= MOTOR_BRAKE) ? names[(int)cmd.dir] : "???";
        Serial.printf("[Motor] %s @ %u%%\n", name, duty);

        if (dir_changed)
        {
            if (cmd.dir == MOTOR_STOP || cmd.dir == MOTOR_BRAKE)
                Pulsate(BUZZER_PIN, 2, 40);   /* double-beep: halted */
            else
                Pulsate(BUZZER_PIN, 1, 60);   /* single-beep: moving */
        }
    }
}

/* motor_speed_task: smoothly ramps the PCA9685 PWM duty toward s_target_duty.
   Runs every MOTOR_RAMP_MS ms, stepping by MOTOR_RAMP_STEP % per tick.
   This gives the robot smooth acceleration and deceleration instead of
   abrupt speed changes that can cause wheel slip or mechanical shock.

   Example with defaults (step=4%, tick=12 ms):
     0→80% cruise takes 20 ticks × 12 ms = 240 ms
     80→0% braking takes the same ramp down                                  */
void motor_speed_task(void *)
{
    uint8_t current_duty = 0;

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(MOTOR_RAMP_MS));

        uint8_t  target;
        motor_dir_t dir;
        portENTER_CRITICAL(&s_motor_mux);
        target = s_target_duty;
        dir    = s_current_dir;
        portEXIT_CRITICAL(&s_motor_mux);

        /* Ramp current_duty toward target by at most MOTOR_RAMP_STEP per tick */
        if (current_duty < target)
        {
            current_duty = (uint8_t)min((int)current_duty + MOTOR_RAMP_STEP,
                                        (int)target);
        }
        else if (current_duty > target)
        {
            current_duty = (uint8_t)max((int)current_duty - MOTOR_RAMP_STEP,
                                        (int)target);
        }

        /* For BRAKE: keep enable HIGH (current_duty stays at whatever the
           ramp left it) but direction bits are already LOW — that's the
           active-brake state on an L298N. */
        if (dir == MOTOR_BRAKE && current_duty == 0)
        {
            /* Drive ENA/ENB high directly so the L298N holds both outputs LOW */
            MotorSetDuty(100);
        }
        else
        {
            MotorSetDuty(current_duty);
        }

        /* Provide encoder feedback once hardware is wired (no-op stub for now) */
        MotorSetFeedback(0.001f, 0.001f);
    }
}

/* ── Arm joint task ────────────────────────────────────────────────────────── */

/* Map a PCA9685 channel (now 12–15) back to a joint name for logging. */
static const char *joint_name_for(uint8_t ch)
{
    switch (ch)
    {
    case SERVO_CH_BASE:     return "base";
    case SERVO_CH_SHOULDER: return "shoulder";
    case SERVO_CH_ELBOW:    return "elbow";
    case SERVO_CH_GRIPPER:  return "gripper";
    default:                return "?";
    }
}

void servo_cmd_task(void *)
{
    servo_cmd_t cmd;

    while (true)
    {
        if (xQueueReceive(g_servo_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        const char *name = joint_name_for(cmd.channel);

        /* Smooth, full-power sweep to the target (the interpolation *is* the
           settle, so there's no extra blocking delay holding up the next
           command). The servo stays energised throughout for max torque. */
        MoveServoSmooth(cmd.channel, cmd.angle);
        Beep(1);

        /* Decide whether to keep this joint energised when idle.
           Shoulder/elbow are weak and load-bearing → hold by default; the
           caller can still force-hold any joint via cmd.hold. */
        bool keep_energised = cmd.hold
            || (cmd.channel == SERVO_CH_SHOULDER && ARM_HOLD_SHOULDER)
            || (cmd.channel == SERVO_CH_ELBOW    && ARM_HOLD_ELBOW);

        Serial.printf("[Arm] %s → %d° %s\n", name, cmd.angle,
                      keep_energised ? "(holding)" : "(released)");

        if (!keep_energised)
        {
            DisableServo(cmd.channel);
        }
        else
        {
            /* Stall protection: even a "held" joint gets PWM cut after a bounded
               window, so a servo that can't reach position (binding joint) can't
               sit at full stall torque humming until its gears strip. Gearbox
               friction holds the pose afterward. */
            vTaskDelay(pdMS_TO_TICKS(SERVO_MAX_ENERGIZED_MS));
            DisableServo(cmd.channel);
        }
    }
}

/* ── Arm pose task (IK) ────────────────────────────────────────────────────── */

/* pose_cmd_task: receives arm_pose_cmd_t {x, y, z, gripper} Cartesian targets,
   solves inverse kinematics into joint angles, then interpolates each joint
   in small steps of ARM_STEP_DEG degrees every ARM_STEP_MS ms.
   Interpolation prevents sudden jumps that could damage the arm linkage or
   knock over a payload.

   How to send a pose command via:
     Topic: robot/cmd/pose
     Body:  { "x": 80, "y": 0, "z": 50, "gripper": 30 }

   Coordinates are in mm in the arm's base frame.  x/y is the horizontal
   plane from the shoulder pivot; z is height above it.  The arm has a reach
   envelope of 100+100 = 200 mm; targets outside are silently rejected.      */
void pose_cmd_task(void *)
{
    /* Track current angles so we can interpolate from where we are now */
    uint8_t cur_base     = 90;
    uint8_t cur_shoulder = 90;
    uint8_t cur_elbow    = 90;

    arm_pose_cmd_t pose;

    while (true)
    {
        if (xQueueReceive(g_pose_cmd_queue, &pose, portMAX_DELAY) != pdTRUE)
            continue;

        uint8_t tgt_base, tgt_shoulder, tgt_elbow;
        bool ok = ArmSolveIK(pose.x, pose.y, pose.z,
                              &tgt_base, &tgt_shoulder, &tgt_elbow);

        if (!ok)
        {
            Serial.printf("[Pose] IK unsolvable for (%.1f, %.1f, %.1f)\n",
                          pose.x, pose.y, pose.z);
            Pulsate(BUZZER_PIN, 2, 50);   /* two beeps: rejected target */
            continue;
        }

        Serial.printf("[Pose] Target (%.0f,%.0f,%.0f) → base=%u° shoulder=%u° elbow=%u°\n",
                      pose.x, pose.y, pose.z, tgt_base, tgt_shoulder, tgt_elbow);

        /* Smooth interpolation — move all joints together one step at a time.
           Each tick we step each joint by up to ARM_STEP_DEG degrees toward
           its target.  The loop runs until all joints have settled. */
        while (cur_base != tgt_base || cur_shoulder != tgt_shoulder || cur_elbow != tgt_elbow)
        {
            auto step = [](uint8_t cur, uint8_t tgt) -> uint8_t {
                if (cur < tgt) return (uint8_t)min((int)cur + (int)ARM_STEP_DEG, (int)tgt);
                if (cur > tgt) return (uint8_t)max((int)cur - (int)ARM_STEP_DEG, (int)tgt);
                return cur;
            };

            cur_base     = step(cur_base,     tgt_base);
            cur_shoulder = step(cur_shoulder, tgt_shoulder);
            cur_elbow    = step(cur_elbow,    tgt_elbow);

            SetServoAngle(SERVO_CH_BASE,     cur_base);
            SetServoAngle(SERVO_CH_SHOULDER, cur_shoulder);
            SetServoAngle(SERVO_CH_ELBOW,    cur_elbow);

            vTaskDelay(pdMS_TO_TICKS(ARM_STEP_MS));
        }

        /* Handle gripper if specified (-1 means leave it unchanged) */
        if (pose.gripper >= 0 && pose.gripper <= 180)
        {
            SetServoAngle(SERVO_CH_GRIPPER, (uint8_t)pose.gripper);
            vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
            DisableServo(SERVO_CH_GRIPPER);   /* stall protection: don't hold a grip indefinitely */
        }

        /* Stall protection: cap how long held joints stay energised, then cut
           PWM on every joint so a binding/stalled servo can't hum at full
           current until its gears strip. Gearbox friction holds the pose. */
        if (ARM_HOLD_SHOULDER || ARM_HOLD_ELBOW)
            vTaskDelay(pdMS_TO_TICKS(SERVO_MAX_ENERGIZED_MS));
        DisableServo(SERVO_CH_SHOULDER);
        DisableServo(SERVO_CH_ELBOW);
        DisableServo(SERVO_CH_BASE);

        Beep(1);
        Serial.println("[Pose] Reached target");
    }
}

/* ── Colour sensor task ────────────────────────────────────────────────────────
 *  Reads the TCS34725 periodically and publishes into g_color_queue for the
 *  MQTT layer to pick up.  If the sensor is absent (is_color_sensor_enabled()
 *  is false) the task self-deletes so it costs nothing — the robot runs
 *  perfectly without it.
 * ──────────────────────────────────────────────────────────────────────────── */
void color_task(void *)
{
    if (!is_color_sensor_enabled())
    {
        Serial.println("[Color] Sensor disabled — task not running");
        vTaskDelete(nullptr);
        return;
    }

    bool was_red = false;   /* edge-trigger so the sequence runs once per red */

    while (true)
    {
        xSemaphoreTake(s_color_sem, portMAX_DELAY);

        color_data_t d = ReadColor();
        if (!d.valid)
            continue;

        xQueueOverwrite(g_color_queue, &d);
        Serial.printf("[Color] R=%u G=%u B=%u C=%u  %.0fK  %.0f lux  → %s\n",
                      d.r, d.g, d.b, d.c, d.color_temp, d.lux,
                      ColorName(d.dominant));

        /* On RED: open gripper, close after 3 s, turn base 90°, open again.
           Edge-triggered so it fires once when red appears, not every read. */
        if (d.dominant == COLOR_RED && !was_red)
        {
            Serial.println("[Color] RED edge → running grip sequence");
            RedObjectGripSequence();
        }
        was_red = (d.dominant == COLOR_RED);

        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS / 2));
    }
}
