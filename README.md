# Warehouse Robot Firmware

ESP32-based firmware for a warehouse robot. Reads five sensor types, detects
IMU events (collision, free-fall, stationary), and publishes all data to an
MQTT broker using a FreeRTOS queue-based architecture.

---

## Hardware

| Sensor | Interface | GPIO | Sample Rate |
| --- | --- | --- | --- |
| DHT11 (temp + humidity) | 1-Wire | 15 | 2 s |
| MPU6050 (accel + gyro + temp) | I2C (SDA 21, SCL 22) | 21 / 22 | 500 ms |
| HC-SR04 ultrasonic (proximity) | GPIO | TRIG 12 / ECHO 23 | 1 s |
| MQ-135 (air quality / CO₂) | ADC2 | 4 | 2 s |
| GPS (NMEA via UART2) | UART2 | RX 16 / TX 17 | 1 s |
| PCF8574 I/O expander | I2C | 0x20, INT 34 | — |

**Servos** (PWM): Base 5 · Shoulder 27 · Elbow 19 · Gripper 18
**LEDs**: GPIO 2 (built-in) · GPIO 32 · GPIO 33 · PCF8574 P0/P1
**Buzzer**: GPIO 25

---

## MQTT Topic Strategy

All topics are rooted under `robot/devices/<DEVICE_ID>/`.

| Topic | Direction | Trigger | Description |
| --- | --- | --- | --- |
| `robot/devices/{id}/boot` | Publish | Once at boot | Device came online |
| `robot/devices/{id}/readings` | Publish | Every 30 s | Full sensor snapshot |
| `robot/devices/{id}/heartbeat` | Publish | Every 30 s | Liveness ping |
| `robot/devices/{id}/events/motion` | Publish | Immediate | Collision / impact detected |
| `robot/devices/{id}/events/freefall` | Publish | Immediate | Robot dropped / launched |
| `robot/devices/{id}/events/zero_motion` | Publish | Immediate | Robot became stationary |
| `robot/cmd` | Subscribe | Cloud -> device | Command dispatch |

`DEVICE_ID` is `IRK17352YV2026` (defined in `include/defines.h`).

---

## JSON Payload Schemas

### `…/boot`

Sent once immediately after the first successful MQTT connection. Provides the
back-end with the exact boot time (NTP-synced Unix timestamp), firmware
version, and network details.

```json
{
  "device_id":  "IRK17352YV2026",
  "event":      "boot",
  "timestamp":  1712500000,
  "firmware":   "1.0.0",
  "transport":  "wifi",
  "ip":         "192.168.1.42",
  "rssi":       -52,
  "uptime_ms":  8340
}
```

| Field | Type | Notes |
|---|---|---|
| `timestamp` | integer | Unix epoch seconds (0 if NTP not yet synced) |
| `uptime_ms` | integer | Milliseconds since power-on at time of publish |

---

### `…/readings`

Published every 30 seconds. Contains the latest reading from every sensor.

```json
{
  "device_id": "IRK17352YV2026",
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

  "system": {
    "rssi":      -52,
    "transport": "wifi"
  }
}
```

| Field | Unit | Source |
|---|---|---|
| `environment.temperature_c` | °C | DHT11 |
| `environment.humidity_pct` | % RH | DHT11 |
| `environment.air_quality_ppm` | ppm CO2 (estimated) | MQ-135 |
| `environment.air_quality_v` | V | MQ-135 raw ADC |
| `imu.accel_*` | m/s² | MPU6050 (±8 g range) |
| `imu.gyro_*` | rad/s | MPU6050 (±500 °/s range) |
| `imu.temperature_c` | °C | MPU6050 die temperature |
| `proximity.distance_cm` | cm | HC-SR04 (-1 = no echo) |
| `location.*` | — | GPS (NMEA via TinyGPS++) |
| `system.rssi` | dBm | WiFi |

---

### `…/heartbeat`

Published alongside each `/readings` message. Lightweight liveness check.

```json
{
  "device_id": "IRK17352YV2026",
  "timestamp": 1712500030,
  "uptime_s":  30,
  "rssi":      -52
}
```

---

### `…/events/motion`

Published immediately when the MPU6050 INT_STATUS reports a **motion
interrupt** — indicates collision or sharp direction change.

```json
{
  "device_id": "IRK17352YV2026",
  "event":     "motion",
  "timestamp": 1712500045,
  "uptime_s":  45,
  "accel_x":   3.12,
  "accel_y":  -1.45,
  "accel_z":   9.81
}
```

---

### `…/events/freefall`

Published immediately when all three accelerometer axes simultaneously read
below the free-fall threshold for the configured duration — the robot has been
dropped or launched.

```json
{
  "device_id": "IRK17352YV2026",
  "event":     "freefall",
  "timestamp": 1712500060,
  "uptime_s":  60,
  "accel_x":   0.03,
  "accel_y":   0.02,
  "accel_z":   0.18
}
```

---

### `…/events/zero_motion`

Published immediately when the robot has become stationary (all axes below the
zero-motion threshold for the configured duration).

```json
{
  "device_id": "IRK17352YV2026",
  "event":     "zero_motion",
  "timestamp": 1712500120,
  "uptime_s":  120,
  "accel_x":   0.01,
  "accel_y":  -0.00,
  "accel_z":   9.80
}
```

---

### `robot/cmd` (inbound)

The firmware subscribes to this topic on every boot. Payload format is
application-defined. The `callback()` function in `src/mqtt.cpp` receives all
messages and prints them to Serial. Extend it to dispatch servo / wheel /
gripper commands.

---

## IMU Event Detection (MPU6050 Register Configuration)

The Adafruit MPU6050 library does not expose free-fall or zero-motion
detection, so the firmware writes directly to the relevant registers after
`mpu.begin()`. `ConfigureIMUEvents()` in `src/imu.cpp` sets:

| Register | Address | Value | Meaning |
| --- | --- | --- | --- |
| `MOT_THR` | 0x1F | 5 | Motion threshold: 5 x 32 mg = 160 mg |
| `MOT_DUR` | 0x20 | 1 | Must exceed threshold for 1 ms |
| `FF_THR` | 0x1D | 17 | Free-fall threshold: 17 mg |
| `FF_DUR` | 0x1E | 100 | Must be below threshold for 100 ms |
| `ZRMOT_THR` | 0x21 | 4 | Zero-motion threshold |
| `ZRMOT_DUR` | 0x22 | 4 | 4 consecutive samples stationary |
| `INT_PIN_CFG` | 0x37 | 0x00 | Active-high, push-pull, 50 us pulse |
| `INT_ENABLE` | 0x38 | 0xE0 | FF (bit7) + MOT (bit6) + ZMOT (bit5) |

`INT_STATUS` (0x3A) is **read-clear**: a single I2C read atomically returns and
clears all flags. `CheckIMUEvents()` is called after every IMU sample; detected
events are pushed to `g_event_queue` (depth 5) for immediate publishing by
`event_task` on CPU1.

Adjust thresholds in `src/imu.cpp:ConfigureIMUEvents()` to tune sensitivity for
your warehouse floor vibration and payload weight.

---

## FreeRTOS Architecture

```
CPU0 (sensor tasks)                CPU1 (network tasks)
───────────────────────────────    ────────────────────────────────────────
led_task         500 ms timer      network_init_task  (once, self-deletes)
dht_task        2000 ms timer        WiFi + NTP sync
imu_task         500 ms timer        MQTT connect + subscribe robot/cmd
ultrasonic_task 1000 ms timer        Publish boot notification -> .../boot
mq135_task      2000 ms timer        Start 30 s pub_timer
gps_task        1000 ms timer        Set g_net_events NET_READY_BIT
                100 ms serial poll
                                   publish_task  (waits on g_publish_sem)
                                     peek all queues -> build JSON
                                     -> .../readings + .../heartbeat
                                   event_task    (blocks on g_event_queue)
                                     -> .../events/motion
                                     -> .../events/freefall
                                     -> .../events/zero_motion
```

### Queue and Semaphore Map

| Handle | Type | Depth | Item | Producer | Consumer |
| --- | --- | --- | --- | --- | --- |
| `g_dht_queue` | Queue | 1 | `dht_data_t` | `dht_task` | `publish_task` |
| `g_imu_queue` | Queue | 1 | `imu_reading_t` | `imu_task` | `publish_task` |
| `g_sonic_queue` | Queue | 1 | `float` | `ultrasonic_task` | `publish_task` |
| `g_mq135_queue` | Queue | 1 | `mq135_data_t` | `mq135_task` | `publish_task` |
| `g_gps_queue` | Queue | 1 | `gps_data_t` | `gps_task` | `publish_task` |
| `g_event_queue` | Queue | 5 | `imu_event_t` | `imu_task` | `event_task` |
| `g_publish_sem` | Binary semaphore | — | — | `pub_timer` (30 s) | `publish_task` |
| `s_*_sem` | Binary semaphores | — | — | sensor timers | sensor tasks |
| `g_net_events` | Event group | — | Bit 0 = NET_READY | `network_init_task` | `publish_task`, `event_task` |

Sensor queues use `xQueueOverwrite()` (latest wins). `publish_task` uses
`xQueuePeek()` (non-destructive) so the last known value persists until the
sensor writes again.

---

## Build

```bash
pio run                          # compile
pio run -t upload                # flash to ESP32
pio device monitor -b 115200     # Serial output
```

Dependencies (auto-installed by PlatformIO via `platformio.ini`):

- `Adafruit MPU6050` — IMU driver
- `DHT sensor library` — DHT11 temperature/humidity
- `PubSubClient` — MQTT client
- `ArduinoJson` v7 — JSON serialization
- `TinyGPSPlus` — NMEA GPS parser
- `PCF8574 library` — I/O expander
- `ESP32Servo` — Servo PWM
