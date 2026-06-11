#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "defines.h"

/* ── Patrol: line following + obstacle avoidance + command override ───────────
 *  The robot patrols autonomously. Priority, highest first:
 *
 *    1. Obstacle      — ultrasonic closer than OBSTACLE_THRESHOLD_CM → stop and
 *                       beep, re-check until clear. Overrides everything,
 *                       including an active external command (safety).
 *    2. External cmd  — a drive command from MQTT (g_drive_cmd_queue) takes over
 *                       for PATROL_CMD_HOLD_MS, then patrol resumes.
 *    3. Line follow   — when the IR sensors see a line, steer along it.
 *    4. Patrol        — default: no line, no command → drive FORWARD.
 *
 *  Two IR reflectance sensors steer; the HC-SR04 ultrasonic gives obstacle
 *  avoidance. All driving goes through g_motor_cmd_queue so motor_cmd_task
 *  stays the single owner of motor state (and we get its smooth speed ramp).
 *
 *  Line truth table (IR_ON_LINE == HIGH means "over the black line"):
 *    L on,  R on   → both on the line  → drive FORWARD
 *    L on,  R off  → drifted right     → steer LEFT
 *    L off, R on   → drifted left      → steer RIGHT
 *    L off, R off  → no line           → patrol FORWARD (not stop)
 * ──────────────────────────────────────────────────────────────────────────── */

void InitLineSensors()
{
    /* GPIO34/39 are input-only and have no internal pull resistors, so a plain
       INPUT is correct — the IR module actively drives the line. */
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);
}

bool ReadIRLeft()  { return digitalRead(IR_LEFT_PIN)  == IR_ON_LINE; }
bool ReadIRRight() { return digitalRead(IR_RIGHT_PIN) == IR_ON_LINE; }

/* Post a drive command to motor_cmd_task. speed < 0 → cruise default.
   pivot defaults to false: patrol/line-follow turns are continuous nudges,
   not the discrete 90° pivot that external commands request. */
static void drive(motor_dir_t dir, int16_t speed = -1, bool pivot = false)
{
    motor_cmd_t cmd = { dir, speed, pivot };
    xQueueOverwrite(g_motor_cmd_queue, &cmd);
}

void line_follow_task(void *)
{
#if !LINE_FOLLOWING_ENABLED
    Serial.println("[Line] Line following disabled — task not running");
    vTaskDelete(nullptr);
    return;
#else
    InitLineSensors();
    Serial.println("[Patrol] Patrol active — forward by default, line-follow + obstacle pause");

    static const char *names[] = {
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "BRAKE"
    };

    motor_dir_t last_dir = MOTOR_STOP;
    bool obstacle_active = false;
    /* Set once we've already handled the current obstacle (picked a red object
       or decided to wait it out), so we don't re-run the arm on every loop tick
       while the same object is still in front of the sensor. Cleared when the
       path goes clear. */
    bool obstacle_handled = false;

    /* External-command override: when an MQTT drive command arrives we honour it
       until this deadline (millis), then patrol resumes. 0 = no active command. */
    uint32_t cmd_until_ms = 0;
    motor_cmd_t ext_cmd   = { MOTOR_STOP, -1, false };

    while (true)
    {
        /* ── 1. Obstacle avoidance (highest priority, overrides commands too) ──
           A negative reading means "no echo / out of range", which is NOT an
           obstacle — only treat a positive distance under the threshold as one. */
        float dist = ReadUltrasonic();
        if (dist > 0.0f && dist <= OBSTACLE_THRESHOLD_CM)
        {
            if (!obstacle_active)
            {
                obstacle_active = true;
                Serial.printf("[Patrol] Obstacle %.1f cm — stopping\n", dist);
            }

            /* Always stop first — the object is right in front of us. */
            drive(MOTOR_STOP);
            last_dir = MOTOR_STOP;

            /* Treat the obstacle as an object: inspect it with the downward
               colour sensor exactly once per obstacle. A RED object is picked
               and placed 90° left; anything else we just wait out. The
               obstacle_handled latch stops us re-running the arm every tick
               while the same object sits in front of the sensor. */
            if (!obstacle_handled)
            {
                obstacle_handled = true;

                color_data_t col = ReadColor();
                if (col.valid && col.dominant == COLOR_RED)
                {
                    /* RED → pick it up. Don't reverse first: backing off would
                       move the object out of the arm's reach. */
                    Serial.println("[Patrol] Object is RED — picking up");
                    PickPlaceRedObject();
                    /* Object removed; fall through next loop to re-check distance
                       (path should now be clear) and resume patrol. */
                }
                else
                {
                    /* Not a target → back off a bit, then wait for it to clear.
                       Reverse open-loop for OBSTACLE_REVERSE_MS, then stop. */
                    Serial.printf("[Patrol] Object is %s (not red) — reversing then waiting\n",
                                  col.valid ? ColorName(col.dominant) : "unreadable");
                    drive(MOTOR_BACKWARD);
                    vTaskDelay(pdMS_TO_TICKS(OBSTACLE_REVERSE_MS));
                    drive(MOTOR_STOP);
                    last_dir = MOTOR_STOP;
                }
            }

            /* Beep while blocked, then loop back to re-check distance. */
            Pulsate(BUZZER_PIN, 1, OBSTACLE_BEEP_MS);
            continue;
        }

        if (obstacle_active)
        {
            obstacle_active = false;
            obstacle_handled = false;   /* ready to inspect the next object */
            Serial.println("[Patrol] Path clear — resuming");
        }

        /* ── 2. External command override ─────────────────────────────────────
           A new MQTT command starts/refreshes a PATROL_CMD_HOLD_MS window. While
           that window is open the command's direction is what we drive (patrol
           and line-follow are suspended), then we fall back to patrol. */
        motor_cmd_t incoming;
        if (xQueueReceive(g_drive_cmd_queue, &incoming, 0) == pdTRUE)
        {
            ext_cmd = incoming;
            /* Forward/backward get a longer window so one command covers real
               ground; other commands use the short default. */
            uint32_t hold_ms = (ext_cmd.dir == MOTOR_FORWARD ||
                                ext_cmd.dir == MOTOR_BACKWARD)
                                   ? PATROL_DRIVE_HOLD_MS
                                   : PATROL_CMD_HOLD_MS;
            cmd_until_ms = millis() + hold_ms;
            Serial.printf("[Patrol] Command %s held for %u ms\n",
                          (ext_cmd.dir <= MOTOR_BRAKE) ? names[(int)ext_cmd.dir] : "???",
                          (unsigned)hold_ms);
        }

        if (cmd_until_ms != 0)
        {
            if ((int32_t)(millis() - cmd_until_ms) < 0)
            {
                /* Command window still open — drive it and skip patrol. Re-post
                   each tick is cheap (xQueueOverwrite) and keeps the duty fresh;
                   motor_cmd_task ignores repeats of the same direction. */
                if (ext_cmd.dir != last_dir)
                {
                    drive(ext_cmd.dir, ext_cmd.speed, ext_cmd.pivot);
                    last_dir = ext_cmd.dir;
                }
                vTaskDelay(pdMS_TO_TICKS(LINE_LOOP_MS));
                continue;
            }
            /* Window expired — clear it and fall through to patrol */
            cmd_until_ms = 0;
            last_dir     = MOTOR_STOP;   /* force a fresh patrol drive command */
            Serial.println("[Patrol] Command expired — resuming patrol");
        }

        /* ── 3/4. Line following, else patrol forward ─────────────────────────
           No line (both sensors off) is the patrol case: keep moving FORWARD
           rather than stopping, so the robot patrols when off-line. */
        bool left  = ReadIRLeft();
        bool right = ReadIRRight();

        motor_dir_t dir;
        if (left && right)       dir = MOTOR_FORWARD;   /* centred on line */
        else if (left && !right) dir = MOTOR_LEFT;      /* drifted right → correct left  */
        else if (!left && right) dir = MOTOR_RIGHT;     /* drifted left  → correct right */
        else                     dir = MOTOR_FORWARD;   /* no line → patrol forward */

        if (dir != last_dir)
        {
            Serial.printf("[Patrol] L=%d R=%d → %s\n", left, right, names[(int)dir]);
            drive(dir);
            last_dir = dir;
        }

        vTaskDelay(pdMS_TO_TICKS(LINE_LOOP_MS));
    }
#endif /* LINE_FOLLOWING_ENABLED */
}
