# drone_esp32 — actuator + IMU firmware (ESP32-S3 N8R2)

The "dumb" hardware node. The PC runs the PID (`drone-teleop`); this board just
applies commands and reports orientation over MQTT.

- **Subscribes** `drone/hw` — `{"throttle":0..1, "s1":deg, "s2":deg, "s3":deg, "s4":deg}`
  → per-pin trim calibration → clamp to **[40°, 160°]** → 4 servos + ESC.
- **Publishes** `drone/imu` — `{"t","yaw","pitch","roll","gz"}` (degrees) at 50 Hz.
- **Fail-safe**: no command for 400 ms → ESC idle + vanes centred.

## Wiring

| Function    | GPIO        |
|-------------|-------------|
| IMU SDA     | 5           |
| IMU SCL     | 4           |
| Servo 1     | 36          |
| Servo 2     | 37          |
| Servo 3     | 38          |
| Servo 4     | 39          |
| Brushless ESC | 14        |

> Servo pins 36/37/38/39 are free on the **N8R2** (quad PSRAM). On octal-PSRAM
> boards (…R8) GPIO35–37 are reserved for PSRAM — pick other pins there.

## Servo limits & trim

The physical write is hard-clamped to **40°–160°** (the vane rotation limit),
neutral 90° (logical). Each pin has a 2-point trim (`in→out`) in `CAL[]`:

| Servo | logical 0 → | logical 90 → |
|-------|-------------|--------------|
| 36    | 5°          | 100°         |
| 37    | 5°          | 100°         |
| 38    | 10°         | 105°         |
| 39    | 10°         | 100°         |

## Build

```bash
cp include/secrets.example.h include/secrets.h   # fill in Wi-Fi + broker
pio run -t upload
pio device monitor
```

The ESC arms at idle for 2 s on boot — keep props clear.
