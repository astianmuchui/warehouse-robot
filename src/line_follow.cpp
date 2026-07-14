#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "defines.h"

void InitLineSensors()
{
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);
}

bool ReadIRLeft()  { return digitalRead(IR_LEFT_PIN)  == IR_ON_LINE; }
bool ReadIRRight() { return digitalRead(IR_RIGHT_PIN) == IR_ON_LINE; }

static void drive(motor_dir_t dir, int16_t speed = -1, bool pivot = false)
{
    motor_cmd_t cmd = { dir, speed, pivot };
    xQueueOverwrite(g_motor_cmd_queue, &cmd);
}

static void pivot_blocking(motor_dir_t dir, uint32_t dwell_ms)
{
    drive(dir, TURN_DUTY_PCT, /*pivot=*/true);
    vTaskDelay(pdMS_TO_TICKS(dwell_ms + 150));
}

void line_follow_task(void *)
{
#if LINE_FOLLOWING_ENABLED
    InitLineSensors();
    Serial.println("[Patrol] Active — line-follow steering ON, MQTT override, obstacle turn-away");
#else
    Serial.println("[Patrol] Active — MQTT-controlled movement, obstacle turn-away (line-follow OFF)");
#endif

    static const char *names[] = {
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "BRAKE"
    };

    motor_dir_t last_dir = MOTOR_STOP;
    bool obstacle_active = false;
    bool obstacle_handled = false;

    uint32_t cmd_until_ms = 0;
    motor_cmd_t ext_cmd   = { MOTOR_STOP, -1, false };

    while (true)
    {
        /* 1. Obstacle. A negative reading is "no echo", not an obstacle. */
        float dist = ReadUltrasonic();
        if (dist > 0.0f && dist <= OBSTACLE_THRESHOLD_CM)
        {
            if (!obstacle_active)
            {
                obstacle_active = true;
                Serial.printf("[Patrol] Obstacle %.1f cm — stopping\n", dist);
            }

            drive(MOTOR_STOP);
            last_dir = MOTOR_STOP;

            if (!obstacle_handled)
            {
                obstacle_handled = true;

                uint32_t handle_start = millis();

                color_data_t col = ReadColor();
                if (col.valid && col.dominant == COLOR_RED)
                {
                    Serial.println("[Patrol] Object is RED — picking up");
                    PickPlaceRedObject();
                }
                else
                {
                    Serial.printf("[Patrol] Object is %s (not red) — turning RIGHT to avoid\n",
                                  col.valid ? ColorName(col.dominant) : "unreadable");
                    pivot_blocking(MOTOR_RIGHT, OBSTACLE_TURN_90_MS);

                    float after = ReadUltrasonic();
                    if (after > 0.0f && after <= OBSTACLE_THRESHOLD_CM)
                    {
                        Serial.printf("[Patrol] Still blocked at %.1f cm — U-turn (left of original)\n",
                                      after);
                        pivot_blocking(MOTOR_RIGHT, OBSTACLE_UTURN_MS);
                    }
                }

                if (cmd_until_ms != 0)
                    cmd_until_ms += (millis() - handle_start);
            }

            Pulsate(BUZZER_PIN, 1, OBSTACLE_BEEP_MS);
            continue;
        }

        if (obstacle_active)
        {
            obstacle_active = false;
            obstacle_handled = false;
            last_dir = MOTOR_STOP;
            Serial.println("[Patrol] Path clear — resuming");
        }

        /* 2. External command. A new MQTT command opens/refreshes a hold window. */
        motor_cmd_t incoming;
        if (xQueueReceive(g_drive_cmd_queue, &incoming, 0) == pdTRUE)
        {
            ext_cmd = incoming;
            uint32_t hold_ms = (ext_cmd.dir == MOTOR_FORWARD ||
                                ext_cmd.dir == MOTOR_BACKWARD)
                                   ? PATROL_DRIVE_HOLD_MS
                                   : PATROL_CMD_HOLD_MS;
            cmd_until_ms = millis() + hold_ms;
            last_dir     = MOTOR_STOP;
            Serial.printf("[Patrol] Command %s held for %u ms\n",
                          (ext_cmd.dir <= MOTOR_BRAKE) ? names[(int)ext_cmd.dir] : "???",
                          (unsigned)hold_ms);
        }

        if (cmd_until_ms != 0)
        {
            if ((int32_t)(millis() - cmd_until_ms) < 0)
            {
                if (ext_cmd.dir != last_dir)
                {
                    drive(ext_cmd.dir, ext_cmd.speed, ext_cmd.pivot);
                    last_dir = ext_cmd.dir;
                }
                vTaskDelay(pdMS_TO_TICKS(LINE_LOOP_MS));
                continue;
            }
            cmd_until_ms = 0;
            ext_cmd.dir  = MOTOR_STOP;
            if (last_dir != MOTOR_STOP)
            {
                drive(MOTOR_STOP);
                last_dir = MOTOR_STOP;
            }
            Serial.println("[Patrol] Command expired — stopping (awaiting next command)");
        }

#if LINE_FOLLOWING_ENABLED
        bool left  = ReadIRLeft();
        bool right = ReadIRRight();

        motor_dir_t dir;
        if (left && right)       dir = MOTOR_FORWARD;
        else if (left && !right) dir = MOTOR_LEFT; 
        else if (!left && right) dir = MOTOR_RIGHT; 
        else                     dir = MOTOR_FORWARD;

        if (dir != last_dir)
        {
            Serial.printf("[Patrol] L=%d R=%d → %s\n", left, right, names[(int)dir]);
            drive(dir);
            last_dir = dir;
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(LINE_LOOP_MS));
    }
}
