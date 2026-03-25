# Warehouse Robot — ESP32

An ESP32-based warehouse robot with environmental sensing, motion control, and remote command/telemetry over MQTT.

---

## Hardware

| Component | Interface | GPIO |
|---|---|---|
| DHT11 (temp/humidity) | Digital | 15 |
| MPU6050 (accel/gyro) | I2C | SDA 16 / SCL 17 |
| HC-SR04 (ultrasonic) | Digital | TRIG 12 / ECHO 23 |
| MQ-135 (air quality) | Analog | 4 |
| PCF8574 I/O expander | I2C (0x20) | SDA 16 / SCL 17 / INT 34 |
| Servo — Base | PWM | 14 |
| Servo — Shoulder | PWM | 27 |
| Servo — Elbow | PWM | 19 |
| Servo — Gripper | PWM | 21 |
| Buzzer | Digital | 13 |
| LED (status) | Digital | 2 (built-in) |
| LED (WiFi/MQTT) | PCF8574 | P0 |
| LED (error) | PCF8574 | P1 |

> HC-SR04 ECHO is 5V — use a voltage divider before connecting to the ESP32.

---

## Boot Sequence

```
Power on
   │
   ├─ initialize_pins()       GPIO modes, Serial at 115200
   ├─ InitializeExpander()    PCF8574 via I2C, P0/P1 as outputs
   ├─ Pulsate(BUZZER, 2)      Startup beep
   │
   ├─ MQTT_INITIALIZE()
   │     ├─ WiFi.begin()      Connects to AP, blinks P0 on success
   │     ├─ client.connect()  Connects to broker, blinks P0 on success
   │     │                    Blinks P1 rapidly on failure (retries)
   │     ├─ publish()         Sends "Boot Notification" to robot/data
   │     └─ subscribe()       Listens on robot/cmd
   │
   ├─ InitializeDHT()         Starts DHT11
   ├─ InitializeIMU()         Wire.begin(16,17), MPU6050 self-test
   │
   ├─ xTaskCreate: LED Task   Heartbeat blink on GPIO 2 (Core 0)
   └─ xTaskCreate: SensorTask Sensor read loop (Core 0, 4096B stack)
```

---

## Sensor Task Loop

Runs continuously on Core 0, alternating reads every 1.5 s:

```
SensorTask (FreeRTOS, Core 0)
   │
   ├─ ReadDHT()               Temperature (°C) + Humidity (%)
   ├─ vTaskDelay(1500ms)
   │
   ├─ ReadGyro()              Angular velocity X/Y/Z (rad/s)
   ├─ ReadAccel()             Linear acceleration X/Y/Z (m/s²)
   ├─ ReadUltrasonic()        Distance (cm), -1 on timeout >30ms (~5m)
   └─ vTaskDelay(1500ms)
```

All readings are printed over Serial. MQTT publish hooks can be added here.

---

## MQTT

| Topic | Direction | Purpose |
|---|---|---|
| `robot/data` | Publish | Telemetry and boot notifications |
| `robot/cmd` | Subscribe | Remote commands |

Incoming messages are handled in `callback()` → `printPayload()` (currently logs to Serial).

---

## I/O Expander (PCF8574)

The PCF8574 adds 8 GPIO pins over I2C, freeing up ESP32 pins for PWM/analog use.

```
P0 → LED: blinks on WiFi connect and MQTT events
P1 → LED: blinks on MQTT connection failure
P2–P7 → available for additional modules
INT → GPIO 34 (input-only, triggers keyPressInterruptHandler via IRAM_ATTR ISR)
```

Address pins A0/A1/A2 all tied to GND → I2C address `0x20`.

---

## File Structure

```
src/
├── main.cpp        Setup, FreeRTOS task creation
├── boot.cpp        Pin initialization
├── tasks.cpp       SensorTask loop
├── imu.cpp         MPU6050 (accel, gyro, temp)
├── dht.cpp         DHT11 (temperature, humidity)
├── ultrasonic.cpp  HC-SR04 (distance)
├── mq135.cpp       MQ-135 (air quality ppm)
├── expander.cpp    PCF8574 (InitializeExpander, PCF_Pulsate)
├── mqtt.cpp        WiFi + MQTT init, callback, RobotMQTT class
└── display.cpp     Pulsate() for buzzer/GPIO LEDs
include/
└── defines.h       Pin defines, PCF aliases, function declarations
```
