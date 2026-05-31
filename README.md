# Warehouse Robot Firmware

ESP32-based firmware for a warehouse robot. Reads six sensors, detects IMU
events (collision, free-fall, stationary), drives a two-wheel base via an
L298N (direction over PCF8574 + PWM speed over PCA9685), and drives a
four-joint arm via the same PCA9685 — with both per-joint and Cartesian
(inverse-kinematics) control. All telemetry is published to an MQTT broker
through a FreeRTOS, queue-based architecture.

---

## Hardware

### Sensors

| Sensor | Interface | GPIO | Sample Rate |
| --- | --- | --- | --- |
| DHT11 (temp + humidity) | 1-Wire | 15 | 2 s |
| MPU6050 (accel + gyro + temp) | I2C (SDA 21, SCL 22) | 21 / 22 | 500 ms |
| HC-SR04 ultrasonic (proximity) | GPIO | TRIG 12 / ECHO 23 | 1 s |
| MQ-135 (air quality / CO₂) | ADC | 35 | 2 s (after 2 min warm-up) |
| GPS (NMEA via UART2) | UART2 | RX 16 / TX 17 | 1 s |
| TCS34725 (RGB colour, optional) | I2C (`0x29`) | 21 / 22 | 1 s |

The colour sensor is probed at boot. If it is absent, [color_task](src/tasks.cpp#L427)
self-deletes and the rest of the firmware runs unchanged.

### Actuators

#### L298N Motor Driver

Direction lives on a PCF8574 I/O expander (`0x20`, INT on GPIO 34). Speed
(ENA / ENB) is driven by PWM on the **PCA9685**, so the wheels get real
analog speed control instead of on/off.

| Channel | L298N pin | Role |
| --- | --- | --- |
| PCF P0 | IN1 | Motor A forward |
| PCF P1 | IN2 | Motor A reverse |
| PCF P2 | IN3 | Motor B forward |
| PCF P3 | IN4 | Motor B reverse |
| PCA9685 ch 14 | ENA | Motor A PWM speed |
| PCA9685 ch 15 | ENB | Motor B PWM speed |
| PCF P4–P7 | — | Free (general purpose) |

Speed is ramped, not stepped — see [Smooth speed ramping](#smooth-speed-ramping).

#### PCA9685 I²C PWM Driver

Address `0x40` on the shared I²C bus. Runs at 50 Hz for servos; the motor
enable channels share that frequency. 12-bit duty resolution (0–4095).

| Channel | Use |
| --- | --- |
| 0 | Arm base |
| 1 | Arm shoulder |
| 2 | Arm elbow |
| 3 | Gripper |
| 14 | Motor A enable (PWM) |
| 15 | Motor B enable (PWM) |

Servo pulse range: 500 µs (0°) → 2400 µs (180°). All joints park at **90°** on boot.

By default, after a joint reaches its target the PWM is **cut** so the servo
goes silent. The shoulder and elbow are exceptions — they bear load and would
sag — so they hold by default ([defines.h:151-152](include/defines.h#L151-L152)).
Any joint can also be force-held via `"hold": true` in the MQTT command.

### Indicators

| Peripheral | GPIO |
| --- | --- |
| Buzzer | 26 (LEDC PWM) |
| LED 1 (built-in) | 2 |
| LED 2 | 32 |
| LED 3 | 33 |

The buzzer is driven by **LEDC PWM** at ~5 % duty / 2300 Hz so it is much
quieter than a `digitalWrite`-style buzzer. The previous loud 80/60/40 ms
boot blast was replaced with two soft chirps.

#### Buzzer Feedback

| Event | Pattern |
| --- | --- |
| Power-on | 2 soft chirps |
| WiFi connected | 2 × 80 ms |
| MQTT connected | 2 soft chirps |
| WiFi / MQTT error | 3–4 × 50 ms rapid |
| Motor direction change (moving) | 1 × 60 ms |
| Motor stopped / braked | 2 × 40 ms |
| Each servo joint move | 1 soft chirp |
| Pose target unreachable | 2 × 50 ms |
| Pose target reached | 1 soft chirp |
| Obstacle < 30 cm | 1 × 80 ms caution |
| Obstacle < 15 cm | 3 × 50 ms urgent |
| IMU motion / zero-motion event | 1 × 80 ms |
| IMU free-fall event | 3 × 60 ms urgent |
| Temperature > 50 °C | 2 × 80 ms |
| Air quality > 400 ppm | 2 × 100 ms |

---

## MQTT Topics

All telemetry is rooted under `robot/devices/<DEVICE_ID>/`.
`DEVICE_ID = WRBT202642` ([defines.h:213](include/defines.h#L213)).

### Outbound (device → broker)

| Topic | Trigger | Description |
| --- | --- | --- |
| `robot/devices/{id}/boot` | Once at boot | Device online notification |
| `robot/devices/{id}/readings` | Every 5 s | Full sensor snapshot |
| `robot/devices/{id}/heartbeat` | Every 5 s | Liveness ping |
| `robot/devices/{id}/events/motion` | Immediate | Collision / impact |
| `robot/devices/{id}/events/freefall` | Immediate | Robot dropped |
| `robot/devices/{id}/events/zero_motion` | Immediate | Robot became stationary |

### Inbound (broker → device)

| Topic | Description |
| --- | --- |
| `robot/cmd/drive` | Wheel drive command (direction + optional speed) |
| `robot/cmd/arm` | Single arm-joint command |
| `robot/cmd/pose` | Cartesian end-effector target (inverse kinematics) |

---

## JSON Payload Schemas

### `…/boot`

```json
{
  "device_id":  "WRBT202642",
  "event":      "boot",
  "timestamp":  1712500000,
  "firmware":   "1.0.0",
  "transport":  "wifi",
  "ip":         "192.168.1.42",
  "rssi":       -52,
  "uptime_ms":  8340
}
```

---

### `…/readings`

```json
{
  "device_id": "WRBT202642",
  "timestamp": 1712500030,
  "uptime_s":  30,

  "environment": {
    "temperature_c":   24.5,
    "humidity_pct":    61.2,
    "air_quality_ppm": 412.3,
    "air_quality_v":   1.24
  },

  "imu": {
    "accel_x":        0.12,
    "accel_y":       -0.05,
    "accel_z":        9.79,
    "gyro_x":         0.001,
    "gyro_y":        -0.003,
    "gyro_z":         0.000,
    "temperature_c":  28.4
  },

  "proximity": {
    "distance_cm": 42.5,
    "valid":       true
  },

  "location": {
    "lat":        -1.286389,
    "lon":        36.817223,
    "altitude_m": 1650.1,
    "speed_kmph": 0.0,
    "satellites": 8,
    "valid":      true
  },

  "color": {
    "present":    true,
    "valid":      true,
    "r":          1240,
    "g":           980,
    "b":           512,
    "clear":      2730,
    "color_temp": 4200,
    "lux":         145,
    "dominant":   "RED"
  },

  "system": {
    "rssi":      -52,
    "transport": "wifi"
  }
}
```

| Field | Unit | Source |
| --- | --- | --- |
| `environment.temperature_c` | °C | DHT11 |
| `environment.humidity_pct` | % RH | DHT11 |
| `environment.air_quality_ppm` | ppm CO₂ (estimated) | MQ-135 |
| `environment.air_quality_v` | V | MQ-135 raw ADC |
| `imu.accel_*` | m/s² | MPU6050 (±8 g) |
| `imu.gyro_*` | rad/s | MPU6050 (±500 °/s) |
| `imu.temperature_c` | °C | MPU6050 die |
| `proximity.distance_cm` | cm | HC-SR04 (-1 = no echo) |
| `location.*` | — | GPS / TinyGPS++ |
| `color.r/g/b/clear` | 16-bit raw | TCS34725 |
| `color.color_temp` | K | TCS34725 |
| `color.lux` | lux | TCS34725 |
| `color.dominant` | enum | `RED` · `GREEN` · `BLUE` · `YELLOW` · `WHITE` · `BLACK` · `UNKNOWN` |
| `color.present` | bool | `false` when the sensor is absent |
| `system.rssi` | dBm | WiFi |

When `color.present` is `false`, the rest of the `color` block is omitted —
consumers can use this to distinguish "sensor absent" from "dark reading".

---

### `…/heartbeat`

```json
{
  "device_id": "WRBT202642",
  "timestamp": 1712500030,
  "uptime_s":  30,
  "rssi":      -52
}
```

---

### `…/events/motion` · `…/events/freefall` · `…/events/zero_motion`

```json
{
  "device_id": "WRBT202642",
  "event":     "motion",
  "timestamp": 1712500045,
  "uptime_s":  45,
  "accel_x":   3.12,
  "accel_y":  -1.45,
  "accel_z":   9.81
}
```

---

### `robot/cmd/drive` (inbound)

Controls the two-wheel base. The firmware pushes the command to a size-1
overwrite queue so the latest instruction always wins.

```json
{ "cmd": "forward" }
{ "cmd": "forward", "speed": 60 }
```

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `cmd` | string | required | See table below |
| `speed` | integer | `-1` (cruise) | Target duty 0 – 100 %. Omit to use `MOTOR_SPEED_DEFAULT` (80 %). |

| `cmd` value | Behaviour |
| --- | --- |
| `"forward"` | Both wheels forward |
| `"backward"` | Both wheels reverse |
| `"left"` | Left wheel reverse, right forward (pivot left) |
| `"right"` | Left wheel forward, right reverse (pivot right) |
| `"stop"` | Coast — speed forced to 0, direction cleared |
| `"brake"` | Active brake — ENA/ENB high, both directions LOW |

#### Smooth speed ramping

`motor_speed_task` runs every `MOTOR_RAMP_MS` (12 ms) and steps the live PCA
duty toward the latest target by at most `MOTOR_RAMP_STEP` (4 %) per tick.
The result is smooth acceleration and deceleration instead of jerky direction
changes that cause wheel slip or mechanical shock. With defaults, 0 → 80 %
cruise takes ~240 ms; the same applies on braking.

A hook (`MotorSetFeedback(left_rpm, right_rpm)`) is in place for closed-loop
PID control once wheel encoders are wired — the gains are already defined in
[defines.h:160-162](include/defines.h#L160-L162).

---

### `robot/cmd/arm` (inbound)

Moves one joint on the PCA9685 servo arm. Multiple commands can be queued
(depth 4), so a full pose can be set with four successive messages.

```json
{ "joint": "shoulder", "angle": 45 }
{ "joint": "gripper",  "angle": 20, "hold": true }
```

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `joint` | string | required | `"base"` · `"shoulder"` · `"elbow"` · `"gripper"` |
| `angle` | integer | required | 0 – 180 degrees |
| `hold` | boolean | depends on joint | Keep PWM active after settling. Shoulder/elbow hold by default; base/gripper release by default. Use `true` on the gripper when it must clamp a load. |

The interpolation in `MoveServoSmooth` *is* the settle, so successive joint
commands dispatch back-to-back without an artificial delay.

---

### `robot/cmd/pose` (inbound) — inverse kinematics

Solves a Cartesian target into joint angles and moves all three joints (base,
shoulder, elbow) together with smooth interpolation. Optionally sets the
gripper in the same command.

```json
{ "x": 80, "y": 0, "z": 50 }
{ "x": 80, "y": 0, "z": 50, "gripper": 30 }
```

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `x` | float | 0 | mm in the arm's base frame, horizontal axis |
| `y` | float | 0 | mm in the arm's base frame, horizontal axis |
| `z` | float | 0 | mm above the shoulder pivot |
| `gripper` | integer | `-1` | 0 – 180 °, or `-1` to leave unchanged |

#### Arm geometry

| Constant | Value | Meaning |
| --- | --- | --- |
| `ARM_SHOULDER_LEN_MM` | 100 | shoulder pivot → elbow pivot |
| `ARM_ELBOW_LEN_MM` | 100 | elbow pivot → gripper tip |
| `ARM_STEP_DEG` | 2 ° | max joint step per interpolation tick |
| `ARM_STEP_MS` | 15 ms | interpolation tick period |

Reach envelope = `ARM_SHOULDER_LEN_MM + ARM_ELBOW_LEN_MM` = 200 mm. Targets
outside it are silently rejected with two short beeps; `ArmSolveIK()` returns
`false` and the pose is dropped.

The pose queue has depth 2, so a follow-up pose can be queued while the
current one is interpolating.

---

## IMU Event Detection (MPU6050 Register Configuration)

The Adafruit MPU6050 library does not expose free-fall or zero-motion detection,
so the firmware writes directly to the relevant registers after `mpu.begin()`.
`ConfigureIMUEvents()` in [src/imu.cpp](src/imu.cpp) sets:

| Register | Address | Value | Meaning |
| --- | --- | --- | --- |
| `MOT_THR` | 0x1F | 5 | Motion threshold: 5 × 32 mg = 160 mg |
| `MOT_DUR` | 0x20 | 1 | Must exceed threshold for 1 ms |
| `FF_THR` | 0x1D | 17 | Free-fall threshold: 17 mg |
| `FF_DUR` | 0x1E | 100 | Must be below threshold for 100 ms |
| `ZRMOT_THR` | 0x21 | 4 | Zero-motion threshold |
| `ZRMOT_DUR` | 0x22 | 4 | 4 consecutive samples stationary |
| `INT_PIN_CFG` | 0x37 | 0x00 | Active-high, push-pull, 50 µs pulse |
| `INT_ENABLE` | 0x38 | 0xE0 | FF (bit7) + MOT (bit6) + ZMOT (bit5) |

`INT_STATUS` (0x3A) is read-clear. `CheckIMUEvents()` is called after every IMU
sample; detected events are pushed to `g_event_queue` (depth 5) for immediate
publishing by `event_task` on CPU1.

---

## FreeRTOS Architecture

```text
CPU0                               CPU1
───────────────────────────────    ──────────────────────────────────────────
led_task         500 ms timer      network_init_task  (once, self-deletes)
dht_task        2000 ms timer        WiFi + NTP sync
imu_task         500 ms timer        MQTT connect + subscribe cmd topics
ultrasonic_task 1000 ms timer        Publish boot message → …/boot
mq135_task      2000 ms timer        Start 5 s pub_timer
gps_task        1000 ms timer        Set NET_READY_BIT
                 100 ms serial poll
color_task      1000 ms timer      publish_task   (waits on g_publish_sem)
  (self-deletes if absent)           peek all sensor queues → build JSON
motor_cmd_task  (blocks on queue)    → …/readings + …/heartbeat
  ← robot/cmd/drive                event_task     (blocks on g_event_queue)
  → MotorSetDirection()              → …/events/motion
motor_speed_task 12 ms ramp tick     → …/events/freefall
  → MotorSetDuty() via PCA9685       → …/events/zero_motion
servo_cmd_task  (blocks on queue)
  ← robot/cmd/arm
  → MoveServoSmooth() via PCA9685
pose_cmd_task   (blocks on queue)
  ← robot/cmd/pose
  → ArmSolveIK() + interpolated move
```

### Queue and Semaphore Map

| Handle | Type | Depth | Item | Producer | Consumer |
| --- | --- | --- | --- | --- | --- |
| `g_dht_queue` | Queue | 1 | `dht_data_t` | `dht_task` | `publish_task` |
| `g_imu_queue` | Queue | 1 | `imu_reading_t` | `imu_task` | `publish_task` |
| `g_sonic_queue` | Queue | 1 | `float` | `ultrasonic_task` | `publish_task` |
| `g_mq135_queue` | Queue | 1 | `mq135_data_t` | `mq135_task` | `publish_task` |
| `g_gps_queue` | Queue | 1 | `gps_data_t` | `gps_task` | `publish_task` |
| `g_color_queue` | Queue | 1 | `color_data_t` | `color_task` | `publish_task` |
| `g_event_queue` | Queue | 5 | `imu_event_t` | `imu_task` | `event_task` |
| `g_motor_cmd_queue` | Queue | 1 | `motor_cmd_t` | MQTT `callback` | `motor_cmd_task` |
| `g_servo_cmd_queue` | Queue | 4 | `servo_cmd_t` | MQTT `callback` | `servo_cmd_task` |
| `g_pose_cmd_queue` | Queue | 2 | `arm_pose_cmd_t` | MQTT `callback` | `pose_cmd_task` |
| `g_publish_sem` | Binary semaphore | — | — | `pub_timer` (5 s) | `publish_task` |
| `s_*_sem` | Binary semaphores | — | — | sensor timers | sensor tasks |
| `g_net_events` | Event group | — | Bit 0 = NET_READY | `network_init_task` | `publish_task`, `event_task` |

Sensor queues use `xQueueOverwrite()` (latest wins). The motor queue also uses
`xQueueOverwrite()` so stale direction commands never accumulate. The servo
queue is FIFO (depth 4) for atomic full-arm poses; the pose queue is FIFO
(depth 2) so a follow-up IK target can be lined up while the first one is
interpolating.

Direction/duty hand-off between `motor_cmd_task` and `motor_speed_task` is
guarded by a `portMUX_TYPE` critical section ([tasks.cpp:188-190](src/tasks.cpp#L188-L190)).

---

## Optional Build-Flag Features

Two extra integration paths ship in the tree but are disabled by default;
they activate only when their `build_flags` are set in `platformio.ini`.

| Flag | File | What it adds |
| --- | --- | --- |
| `-D ENABLE_BLE` | [src/ble.cpp](src/ble.cpp) | BLE GATT server with WRITE `cmd` + NOTIFY `status` characteristics — second control channel when WiFi is out of range. |
| `-D ENABLE_NETWIZARD` | [src/wifi_config.cpp](src/wifi_config.cpp) | NetWizard captive-portal WiFi provisioning with **triple-reset** credential wipe via `RTC_DATA_ATTR`. Drop-in replacement for the hardcoded `WIFI_SSID` / `WIFI_PASSWORD`. |

Neither is enabled in the default build.

---

## Build

```bash
pio run                          # compile
pio run -t upload                # flash to ESP32
pio device monitor -b 115200     # serial output
```

Dependencies (auto-installed by PlatformIO via [platformio.ini](platformio.ini)):

- `Adafruit PWM Servo Driver Library` — PCA9685 (servos + motor enables)
- `Adafruit MPU6050` — IMU driver
- `Adafruit TCS34725` — RGB colour sensor
- `DHT sensor library` — DHT11 temperature/humidity
- `PubSubClient` — MQTT client
- `ArduinoJson` v7 — JSON serialization
- `TinyGPSPlus` — NMEA GPS parser
- `PCF8574 library` — I/O expander (L298N direction pins)

Board: `upesy_wroom` (ESP32-WROOM-32).
