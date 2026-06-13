# Warehouse Robot Firmware

This is the firmware for a small warehouse robot we built around an ESP32. It
reads a handful of sensors, watches its IMU for things like bumps and
free-falls, drives a two-wheel base, swings a little four-joint arm that can
pick things up, and reports everything it sees to an MQTT broker. You drive it
and command the arm over MQTT too.

It's a student/hobby project, so treat it as one. Plenty of it is calibrated by
hand and tuned by trial and error. Wherever you see a "CALIBRATE on the robot"
note in the code, it means exactly that. The single source of truth for pins,
channels and tuning values is [include/defines.h](include/defines.h); if this
README ever disagrees with that file, believe the file.

---

## What's on the robot

### Sensors

| Sensor | What it gives us | How it's wired |
| --- | --- | --- |
| DHT11 | temperature + humidity | 1-Wire on GPIO 15 |
| MPU6050 | accel + gyro + die temp | I2C (SDA 21, SCL 22) |
| HC-SR04 | distance to whatever's in front | TRIG 12, ECHO 23 |
| MQ-135 | a rough air-quality / CO₂ number | ADC on GPIO 35 (needs a 2-minute warm-up) |
| GPS | location, over NMEA | UART2, RX 16 / TX 17 |
| TCS34725 | RGB colour of whatever's under it | I2C `0x29`, optional |

The colour sensor is the one part the robot genuinely doesn't need. We probe for
it once at boot, and if it isn't there, the colour task just deletes itself and
everything else carries on. So you can run the whole thing without it.

### The wheels (L298N)

Early on the motor wiring went through a PCF8574 I/O expander and the PCA9685.
That's gone now. Every motor line wires straight to an ESP32 GPIO, which is
simpler and gives us real speed control:

| Signal | GPIO | Job |
| --- | --- | --- |
| IN1 / IN2 | 4 / 5 | Motor A direction |
| IN3 / IN4 | 33 / 32 | Motor B direction |
| ENA | 25 | Motor A speed (LEDC PWM) |
| ENB | 27 | Motor B speed (LEDC PWM) |

IN1 to IN4 are plain digital direction lines. ENA/ENB run on native LEDC PWM at
~20 kHz (above hearing, so no whine), which means actual proportional speed
instead of the on/off-only behaviour the old PCA9685 wiring was stuck with.

The speed isn't slammed on. A separate task ramps it up and down so the robot
doesn't lurch. More on that under [the drive command](#robotcmddrive).

### The arm (PCA9685)

The PCA9685 PWM driver lives at `0x40` on the same I2C bus and now does nothing
but servos. Four joints:

| Channel | Joint |
| --- | --- |
| 0 | base (yaw) |
| 1 | elbow |
| 2 | shoulder |
| 3 | gripper |

One thing worth knowing if you go poking at servo code: hobby servos want a
1.0 to 2.0 ms pulse inside the 20 ms frame, **not** the full PWM range. At 50 Hz
on the PCA9685 that works out to roughly 205 to 410 counts. Drive the full 0 to
4095 range and the pulse falls outside the servo's valid window, so it just
jitters or stalls. We map angles into that 205 to 410 band and leave it there.

The shoulder and elbow are weak MG90S servos and they buzz and overheat if you
ask them to *hold* a pose against gravity. So after every move we cut the PWM and
let gearbox friction hold position. You get a little droop; you don't get a
cooked servo. The gripper opens at 0° and clamps at ~40° (yes, that feels
backwards, it's just what the physical servo does).

### Buzzer and LEDs

There's a buzzer on GPIO 26 and three LEDs (2, 32, 33). The buzzer is driven
with PWM at a low duty so it's a soft chirp rather than the ear-splitting thing
it used to be. It's the robot's only way of talking to you when there's no
serial monitor attached, so it beeps for most events: connection, motion,
obstacles, errors. The patterns are all in the code if you want to decode them.

---

## How it actually behaves

Right now **line following is turned off** (`LINE_FOLLOWING_ENABLED 0` in
defines.h), so here's what the robot does out of the box:

- It sits still until you tell it to move. Movement is MQTT-only.
- You send it a drive command; it drives that way for about a second, then stops.
- The whole time, it's watching the ultrasonic sensor. If something gets within
  50 cm, the obstacle behaviour takes over no matter what. That's a safety thing,
  and it overrides your command.

When it hits an obstacle, it stops and looks down with the colour sensor. If
what's in front is **red**, it treats it as a thing to pick up and runs the
arm's pick-and-place. If it's anything else, it turns away: a ~90° pivot to the
**right**, then it re-checks. Still blocked? It does a U-turn (keeps turning
right until it's ~180° around, so now it faces left of where it started) to
escape a corner instead of turning straight back into the same wall. Once the
path is clear it picks up whatever you'd last told it to do.

If you flip `LINE_FOLLOWING_ENABLED` back to `1`, the two IR sensors on GPIO 34
and 39 steer it along a line, and it patrols forward by default. One warning if
you go editing that flag: it feeds a `#if`, and it **must** be `1` or `0`. We
once set it to `TRUE`, the preprocessor read the unknown `TRUE` as `0`, and the
whole feature silently switched off with no error. That cost an afternoon.

---

## Talking to it over MQTT

Everything the robot publishes is rooted at `robot/devices/<DEVICE_ID>/`, and
out of the box `DEVICE_ID` is `WRBT202642`. Commands go to a few `robot/cmd/...`
topics. The broker, credentials and device ID all live in
[defines.h](include/defines.h).

### What it publishes

| Topic | When | What |
| --- | --- | --- |
| `…/boot` | once, at startup | "I'm online", plus IP and uptime |
| `…/readings` | every 5 s | a full snapshot of every sensor |
| `…/heartbeat` | every 5 s | a tiny "still alive" ping |

There are also `…/events/motion`, `…/events/freefall` and `…/events/zero_motion`
topics wired up for the IMU. Fair warning: the register-level event detection on
the MPU6500 path is currently a stub, so those don't fire yet. The plumbing is
there for when someone finishes it.

### What you can send it

There are three command topics.

#### `robot/cmd/drive`

Move the wheels.

```json
{ "cmd": "forward" }
{ "cmd": "forward", "speed": 60 }
```

`cmd` is one of `forward`, `backward`, `left`, `right`, `stop`, `brake`. `speed`
is optional (0 to 100 %); leave it out and it uses the cruise default. `left` and
`right` are treated as a single ~90° pivot, not a continuous spin. A
forward/backward command drives for about a second and then stops on its own.
That's the open-loop "distance per command" knob (`DRIVE_RUN_MS`), since we have
no wheel encoders yet.

About that ramp: a task wakes every 12 ms and nudges the live PWM duty toward
the target a few percent at a time, instead of jumping straight there. That's
what stops the robot from lurching and spinning its wheels. There's also a
`MotorSetFeedback()` hook sitting empty, waiting for the day someone wires up
encoders and drops a PID loop in. The gains are already in defines.h.

#### `robot/cmd/arm`

Move one joint.

```json
{ "joint": "shoulder", "angle": 45 }
{ "joint": "gripper",  "angle": 40, "hold": true }
```

`joint` is `base`, `shoulder`, `elbow` or `gripper`; `angle` is 0 to 180. The
optional `hold` keeps the PWM on after the move, which is useful on the gripper
when it needs to keep clamping something. Shoulder and elbow ignore `hold` by
default and release anyway, for the overheating reason above. You can queue up to
four of these, so you can set a whole pose with four quick messages.

#### `robot/cmd/pose`

Give it a point in space and let it do the inverse kinematics.

```json
{ "x": 80, "y": 0, "z": 50 }
{ "x": 80, "y": 0, "z": 50, "gripper": 40 }
```

`x`, `y`, `z` are millimetres in the arm's base frame (z is height above the
shoulder pivot). The arm solves that into joint angles and moves all three joints
together, smoothly. Its reach is 100 + 100 = 200 mm; ask for something outside
that and it just refuses with two beeps. `gripper` is optional. You can queue two
poses so a follow-up is lined up while the first is still moving.

### The `…/readings` payload

This is the big one, a full snapshot every five seconds:

```json
{
  "device_id": "WRBT202642",
  "timestamp": 1712500030,
  "uptime_s":  30,
  "environment": { "temperature_c": 24.5, "humidity_pct": 61.2, "air_quality_ppm": 412.3, "air_quality_v": 1.24 },
  "imu":         { "accel_x": 0.12, "accel_y": -0.05, "accel_z": 9.79, "gyro_x": 0.001, "gyro_y": -0.003, "gyro_z": 0.0, "temperature_c": 28.4 },
  "proximity":   { "distance_cm": 42.5, "valid": true },
  "location":    { "lat": -1.286389, "lon": 36.817223, "altitude_m": 1650.1, "speed_kmph": 0.0, "satellites": 8, "valid": true },
  "color":       { "present": true, "valid": true, "r": 1240, "g": 980, "b": 512, "clear": 2730, "color_temp": 4200, "lux": 145, "dominant": "RED" },
  "system":      { "rssi": -52, "transport": "wifi" }
}
```

A couple of notes. `proximity.distance_cm` is `-1` when the ultrasonic got no
echo (out of range), not 0, so `valid` tells you whether to trust it. And when
the colour sensor isn't fitted, `color.present` is `false` and the rest of that
block is left out, so a consumer can tell "no sensor" apart from "it's just dark
in here".

---

## How it's all stitched together (FreeRTOS)

The robot runs a pile of small FreeRTOS tasks split across the ESP32's two cores.
The short version: sensor tasks read their sensor and drop the result into a
one-slot queue; a publish task gathers all those latest values every five seconds
and ships the JSON; command tasks block on their queue waiting for something from
MQTT. Networking lives on one core so a slow WiFi moment can't stall the sensors
on the other.

The one piece of plumbing worth calling out, because it bit us: the **patrol
task** (`line_follow_task`) is the *only* thing that drains the MQTT drive queue.
So even though line following is off, that task still has to run. If it ever
deleted itself, your drive commands would vanish into a queue nobody reads, and
the robot would look dead while happily logging "command received". It hands
every motor instruction to `motor_cmd_task`, which stays the single owner of the
motors so two bits of code never fight over them.

Here's the queue map if you need it:

| Queue | Depth | Carries | From | To |
| --- | --- | --- | --- | --- |
| `g_dht_queue` | 1 | DHT reading | dht_task | publish_task |
| `g_imu_queue` | 1 | IMU reading | imu_task | publish_task |
| `g_sonic_queue` | 1 | distance | ultrasonic_task | publish_task |
| `g_mq135_queue` | 1 | air quality | mq135_task | publish_task |
| `g_gps_queue` | 1 | location | gps_task | publish_task |
| `g_color_queue` | 1 | colour | color_task | publish_task |
| `g_event_queue` | 5 | IMU event | imu_task | event_task |
| `g_drive_cmd_queue` | 1 | drive command | MQTT callback | patrol task |
| `g_motor_cmd_queue` | 1 | motor command | patrol task | motor_cmd_task |
| `g_servo_cmd_queue` | 4 | joint command | MQTT callback | servo_cmd_task |
| `g_pose_cmd_queue` | 2 | pose target | MQTT callback | pose_cmd_task |

The single-slot queues use "latest wins" (`xQueueOverwrite`) on purpose. For a
sensor reading or a drive command, the newest value is the only one that matters,
and stale ones piling up would just add lag.

---

## Optional extras

Two integrations ship in the tree but stay off unless you switch them on with a
build flag in [platformio.ini](platformio.ini):

- `-D ENABLE_BLE` adds a BLE control channel ([src/ble.cpp](src/ble.cpp)), handy
  when WiFi isn't around.
- `-D ENABLE_NETWIZARD` adds captive-portal WiFi setup with a triple-reset
  credential wipe ([src/wifi_config.cpp](src/wifi_config.cpp)), instead of the
  hardcoded SSID and password.

Neither is on in the default build.

---

## Building it

It's a PlatformIO project, so:

```bash
pio run                          # compile
pio run -t upload                # flash the ESP32
pio device monitor -b 115200     # watch the serial log
```

PlatformIO pulls the libraries itself from [platformio.ini](platformio.ini): the
Adafruit PCA9685 / MPU6050 / TCS34725 drivers, the DHT library, PubSubClient for
MQTT, ArduinoJson v7, and TinyGPSPlus. The board is the `upesy_wroom`
(ESP32-WROOM-32).

On boot it runs a couple of self-tests before the tasks start. It scans the I2C
bus and prints what answers (if the PCA9685 doesn't show up, nothing will move,
and now you'll know why), drives the wheels forward and back for a second each,
and sweeps every arm joint through its range. Watch the serial log the first
time; it tells you what's actually working.

---

## Authors

- Sebastian Muchui
- Gloria Ngei
- Glen Okoth

## License

MIT
