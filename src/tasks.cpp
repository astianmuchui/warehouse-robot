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

/** init_sensor_timers - create the periodic timers that tick each sensor task. */
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

/* Each sensor task blocks on its timer's semaphore, reads, and overwrites its
   one-slot queue with the latest value for publish_task to pick up. */

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

            if (d.temperature > 50.0f)
                Pulsate(BUZZER_PIN, 2, 80);   /* too hot */
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

            if (ev_type == IMU_EVENT_FREEFALL)
                Pulsate(BUZZER_PIN, 3, 60);   /* freefall: urgent */
            else
                Pulsate(BUZZER_PIN, 1, 80);
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
                Pulsate(BUZZER_PIN, 1, 80);
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

        if (d.ppm > 400.0f)
            Pulsate(BUZZER_PIN, 2, 100);   /* poor air */

        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS / 2));
    }
}

/* motor_cmd_task (writer) hands the target duty/direction to motor_speed_task
   (reader) through these. volatile + a short critical section keeps the pair
   from tearing across the task boundary. */
static volatile uint8_t  s_target_duty = 0;
static volatile motor_dir_t s_current_dir = MOTOR_STOP;
static portMUX_TYPE      s_motor_mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * motor_cmd_task - the sole owner of motor state. Takes drive commands off
 * g_motor_cmd_queue, sets direction, and posts the target speed for the ramp
 * task. LEFT/RIGHT pivots and FORWARD/BACKWARD are timed direct-drive moves
 * (no ramp, since the window is too short for it); everything else just sets a
 * target and lets motor_speed_task ramp toward it.
 */
void motor_cmd_task(void *)
{
    static const char *names[] = {
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "BRAKE"
    };

    motor_cmd_t cmd;
    motor_dir_t prev_dir = MOTOR_STOP;

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

        /* Discrete ~90° pivot: drive duty directly for TURN_90_MS then stop.
           The ramp would eat most of so short a window. */
        if ((cmd.dir == MOTOR_LEFT || cmd.dir == MOTOR_RIGHT) && cmd.pivot)
        {
            MotorSetDirection(cmd.dir);

            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = TURN_DUTY_PCT;
            s_current_dir = cmd.dir;
            portEXIT_CRITICAL(&s_motor_mux);

            MotorSetDuty(TURN_DUTY_PCT);
            Serial.printf("[Motor] %s 90° pivot @ %u%% for %u ms\n",
                          names[(int)cmd.dir], (unsigned)TURN_DUTY_PCT,
                          (unsigned)TURN_90_MS);
            if (dir_changed) Pulsate(BUZZER_PIN, 1, 60);

            vTaskDelay(pdMS_TO_TICKS(TURN_90_MS));

            MotorSetDirection(MOTOR_STOP);
            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = 0;
            s_current_dir = MOTOR_STOP;
            portEXIT_CRITICAL(&s_motor_mux);
            MotorSetDuty(0);

            prev_dir = MOTOR_STOP;
            continue;
        }

        /* Timed forward/backward: full duty for DRIVE_RUN_MS then stop. */
        if (cmd.dir == MOTOR_FORWARD || cmd.dir == MOTOR_BACKWARD)
        {
            MotorSetDirection(cmd.dir);

            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = 100;
            s_current_dir = cmd.dir;
            portEXIT_CRITICAL(&s_motor_mux);

            MotorSetDuty(100);
            Serial.printf("[Motor] %s @ 100%% for %u ms\n",
                          names[(int)cmd.dir], (unsigned)DRIVE_RUN_MS);
            if (dir_changed) Pulsate(BUZZER_PIN, 1, 60);

            vTaskDelay(pdMS_TO_TICKS(DRIVE_RUN_MS));

            MotorSetDirection(MOTOR_STOP);
            portENTER_CRITICAL(&s_motor_mux);
            s_target_duty = 0;
            s_current_dir = MOTOR_STOP;
            portEXIT_CRITICAL(&s_motor_mux);
            MotorSetDuty(0);

            prev_dir = MOTOR_STOP;
            continue;
        }

        /* Continuous case: -1 speed = cruise default; STOP/BRAKE force 0. */
        uint8_t duty = (cmd.speed < 0) ? MOTOR_SPEED_DEFAULT
                                       : (uint8_t)constrain(cmd.speed, 0, 100);
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
                Pulsate(BUZZER_PIN, 2, 40);   /* halted */
            else
                Pulsate(BUZZER_PIN, 1, 60);   /* moving */
        }
    }
}

/**
 * motor_speed_task - ramps the live PWM duty toward s_target_duty a few percent
 * per tick, so the robot accelerates/decelerates smoothly instead of lurching.
 */
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

        /* BRAKE with direction bits already LOW + enable HIGH is the L298N's
           active-brake state, so drive the enables high. */
        if (dir == MOTOR_BRAKE && current_duty == 0)
            MotorSetDuty(100);
        else
            MotorSetDuty(current_duty);

        MotorSetFeedback(0.001f, 0.001f);   /* no-op until encoders exist */
    }
}

/** joint_name_for - PCA9685 channel -> joint name, for logging. */
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

/**
 * servo_cmd_task - move one joint to a commanded angle (robot/cmd/arm). After
 * the move, hold or release per cmd.hold and the joint defaults; even a held
 * joint gets PWM cut after SERVO_MAX_ENERGIZED_MS so a stalled servo can't cook.
 */
void servo_cmd_task(void *)
{
    servo_cmd_t cmd;

    while (true)
    {
        if (xQueueReceive(g_servo_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        const char *name = joint_name_for(cmd.channel);

        MoveServoSmooth(cmd.channel, cmd.angle);
        Beep(1);

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
            vTaskDelay(pdMS_TO_TICKS(SERVO_MAX_ENERGIZED_MS));
            DisableServo(cmd.channel);
        }
    }
}

/**
 * pose_cmd_task - take a Cartesian target (robot/cmd/pose), solve IK, and
 * interpolate base/shoulder/elbow together. Targets outside the 200 mm reach
 * are rejected with two beeps.
 */
void pose_cmd_task(void *)
{
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
            Pulsate(BUZZER_PIN, 2, 50);   /* rejected target */
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

        if (pose.gripper >= 0 && pose.gripper <= 180)
        {
            SetServoAngle(SERVO_CH_GRIPPER, (uint8_t)pose.gripper);
            vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
            DisableServo(SERVO_CH_GRIPPER);
        }

        if (ARM_HOLD_SHOULDER || ARM_HOLD_ELBOW)
            vTaskDelay(pdMS_TO_TICKS(SERVO_MAX_ENERGIZED_MS));
        DisableServo(SERVO_CH_SHOULDER);
        DisableServo(SERVO_CH_ELBOW);
        DisableServo(SERVO_CH_BASE);

        Beep(1);
        Serial.println("[Pose] Reached target");
    }
}

/**
 * color_task - read the TCS34725, publish to g_color_queue, and run the grip
 * sequence on a rising RED edge (independent of obstacles). Self-deletes if the
 * sensor isn't fitted.
 */
void color_task(void *)
{
    if (!is_color_sensor_enabled())
    {
        Serial.println("[Color] Sensor disabled — task not running");
        vTaskDelete(nullptr);
        return;
    }

    bool was_red = false;   /* edge-trigger: fire once per red, not every read */

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

        if (d.dominant == COLOR_RED && !was_red)
        {
            Serial.println("[Color] RED edge → running grip sequence");
            RedObjectGripSequence();
        }
        was_red = (d.dominant == COLOR_RED);

        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS / 2));
    }
}
