# drone-nav — altitude-hold + vane control for a singlecopter

Controls a **singlecopter** (one top-mounted propeller + 4 independent steering
vanes). A PID **holds altitude** via throttle, while the **four vanes are
commanded independently and raw** for steering. Commands and telemetry are
exchanged with a Blender physics simulation over **MQTT**.

```
┌────────────────────┐   drone/cmd (JSON)    ┌──────────────────────────┐
│   drone_nav         │ ────────────────────► │  Blender sim bridge       │
│  (PID controller)   │                       │  blender_sim_mqtt.py      │
│                     │ ◄──────────────────── │  (physics + autopilot in) │
└────────────────────┘  drone/telemetry      └──────────────────────────┘
```

The physical drone model (mass, thrust, vane limits, momentum-theory force law)
is **mirrored** from `blender-navigatio.py`, so the controller's inverse model
matches the plant it flies.

## Layout

```
config/config.yaml              MQTT, drone params, altitude PID gains
drone_nav/
  config.py                     typed config + YAML loader
  telemetry.py                  Telemetry / Command dataclasses (the MQTT wire format)
  pid.py                        reusable PID (anti-windup, derivative-on-measurement)
  controller.py                 altitude-hold controller (throttle PID)
  mqtt_io.py                    paho-mqtt transport (telemetry + IMU + raw vanes + hw cmd)
  main.py                       entry point: altitude-hold + raw vanes (or --sim)
  vane_cmd.py                   `vane-cmd` CLI to publish raw vane angles
  gamepad.py                    PS4/DS4 reader (pygame) → normalised StickState
  manual_control.py             ManualPilot: sticks + telemetry → Command (PID)
  servo_map.py                  Command (rad) → servo degrees [40,160] + ESC
  teleop.py                     `drone-teleop`: PS4 manual flight (MQTT/hardware or --sim)
sim_bridge/blender_sim_mqtt.py  the sim + MQTT autopilot (4 independent vanes) + telemetry
tools/sim_stub.py               headless re-impl of the sim physics (for offline tests)
firmware/drone_esp32/           ESP32-S3 actuator + IMU node (PlatformIO/Arduino, C++)
tests/                          PID + altitude-hold + independent-vane physics tests
blender-navigatio.py            original simulation (untouched reference)
```

## Control design

- **Altitude** is held by a PID on throttle:
  ```
  altitude error ──P──► climb-rate setpoint ──PID──► vertical accel
       T = m·(g + a_z)   →   throttle = √(T / T_max)
  ```
  Gravity feed-forward falls out for free (hover ≈ 68 % throttle).
- **Steering** uses **four independent vanes**. Each vane moves to its own
  angle; the angles are supplied **raw** (no PID, no mixing) on the
  `drone/vanes` topic. Vanes 1 & 3 (X arm) push fore/aft; vanes 2 & 4 (Y arm)
  push laterally. Force per vane ≈ `0.5·T_prop·sin(angle)`.
- **Yaw** is not an actuator on this airframe — it drifts from reactive prop
  torque.

> Earlier this project used a cascaded position PID that also drove the vanes.
> That was replaced (by request) with **altitude-hold + raw independent vanes**,
> so steering is manual/external.

## Quick start

### 1. Install (uv)

```bash
uv sync --extra dev
```

### 2. Prove it works — offline, no broker, no Blender

The headless physics stub replays the sim's force model. Hold altitude and
deflect vanes by hand:

```bash
uv run drone-nav --sim --verbose --v1 0.2 --v3 0.2   # hold 2 m, push fore/aft
uv run pytest                                        # PID + vane-physics tests
```

### 3. Run against Blender over MQTT

1. Start an MQTT broker (mosquitto) on the machine both sides can reach, and
   point `mqtt.host` in `config/config.yaml` at it.
2. In `sim_bridge/blender_sim_mqtt.py`, set `BROKER_HOST` to that broker, and
   **set `VANE_OBJECTS` to your four vane object names**.
3. Install `paho-mqtt` into **Blender's bundled Python** (not your venv):
   ```bash
   /path/to/Blender/.../python/bin/python3.x -m pip install paho-mqtt
   ```
4. In Blender, run `sim_bridge/blender_sim_mqtt.py` (Text Editor → Run Script).
   The banner should show `MQTT <host>:1883`.
5. Start the navigator (holds altitude automatically):
   ```bash
   uv run drone-nav --verbose
   ```
6. Steer by publishing raw vane angles whenever you like:
   ```bash
   uv run vane-cmd --deg --v1 10 --v3 10     # tilt fore/aft pair by 10°
   uv run vane-cmd --zero                     # vanes back to neutral
   ```

### 4. Autonomous A → B (closed-loop)

Instead of manual steering, let a cascaded position PID fly the drone to a point
— it computes the throttle *and* the four vane angles (with yaw-hold so it flies
straight). A and B can be far apart and at different altitudes; it climbs,
translates, and holds hover on arrival.

```bash
# fly to a single far point B (x y z) — add --sim to try it offline first
uv run drone-nav --sim --goto 14 10 6 --verbose
uv run drone-nav --goto 14 10 6 --verbose

# or fly the waypoint sequence in config.yaml (mission.waypoints)
uv run drone-nav --sim --mission --verbose
uv run drone-nav --mission --verbose
```

Tune the loops under `goto:` in `config.yaml`; set the A→B sequence under
`mission.waypoints`.

### 5. Manual flight with a PS4 controller (real hardware)

A PS4 pad steers the singlecopter through an **ESP32-S3** that drives the four
vane servos + the brushless ESC and streams its IMU back over MQTT.

**Where does the pad connect? → the PC.** The PID stays in Python, so you keep
live tuning, logging, and the same `config.yaml`. The ESP32 is a dumb
actuator+sensor node, which matches your existing telemetry→command flow:

```
PS4 pad ─Bluetooth→ PC ── drone/hw (servo°+ESC) ──→ ESP32 ──→ 4 servos + ESC
   (gamepad.py → ManualPilot PID → servo_map)          │
        heading-hold ←── drone/imu (yaw) ──────────────┘ (MPU6050, SDA5/SCL4)
```

(The alternative — pad → ESP32 directly via Bluepad32 — would force the whole
PID to be rewritten in C++ and abandons the Python stack. Only worth it if the
drone must fly with the PC off.) Because control rides the Wi-Fi/MQTT link, the
ESP32 **fails safe** — ESC to idle, vanes centred — if commands stop for 400 ms.

```bash
# try the controller + PID offline against the physics stub first (no hardware):
uv sync --extra gamepad
uv run drone-teleop --sim --verbose

# discover your pad's axis/button indices if the mapping looks off:
uv run python -m drone_nav.gamepad

# fly the real drone (broker + flashed ESP32 running):
uv run drone-teleop --verbose
```

Controls (DS4 defaults): the **right stick steers the vanes/servos** — Y = pitch
(fore/aft), X = roll (lateral). The **left stick Y = throttle** (up ramps the
brushless up, centre holds the level). **L1/R1 = yaw** left/right. **Options** =
arm/disarm; **Circle** = altitude-hold; **PS** = kill. Vanes map to a logical
servo angle centred at 90° and hard-clamped
to the **[40°, 160°]** rotation limit; the ESP32 then applies per-pin trim. See
`firmware/drone_esp32/README.md` for wiring, pins, and flashing.

> **IMU caveat**: an MPU6050 measures *orientation*, not position. Heading-hold
> uses its yaw; horizontal/altitude position feedback isn't available from the
> IMU alone, so manual throttle is direct by default (altitude-hold needs a `z`
> source and is mainly useful in `--sim`).

## Configuration

Everything lives in `config/config.yaml`: broker address/topics, drone physical
constants (keep in sync with the sim), the altitude PID gains, the target
altitude, the **manual** gamepad mapping, and the **servo** output limits. See
the comments in that file.

## MQTT contract

- **`drone/telemetry`** (sim → nav): `{t,x,y,z,vx,vy,vz,yaw,prop_speed}`
- **`drone/cmd`** (nav → sim): `{throttle:[0..1], vane1,vane2,vane3,vane4}` (rad)
  — vanes 1 & 3 = fore/aft pair, vanes 2 & 4 = lateral pair
- **`drone/vanes`** (external → nav): raw vane angles, `{vane1,vane2,vane3,vane4}`
  (or `[v1,v2,v3,v4]`); merged with the altitude-hold throttle
- **`drone/imu`** (ESP32 → nav): `{t, yaw, pitch, roll, gz}` in **degrees** — folded
  into telemetry as yaw (radians) for heading-hold
- **`drone/hw`** (nav → ESP32): `{throttle:[0..1], s1,s2,s3,s4}` — logical servo
  angles in **degrees** (90 neutral, clamped [40,160]) + ESC throttle fraction
- **`drone/status`** (nav → world): status strings

## Live web app (3D viewport + component map + telemetry/PID charts)

`webapp/` is a React + Three.js front-end that mirrors the Blender viewport in
the browser as a **live digital twin**, alongside a **component map** of the
moving parts (propeller + vanes, with motion directions) and a **visualizations**
section (telemetry charts + per-axis PID profiling). It is fed by a WebSocket
bridge — browsers can't speak raw MQTT TCP, so `sim_bridge/web_bridge.py`
subscribes to the MQTT topics, merges the state, and rebroadcasts JSON at 50 Hz.

```bash
# no broker / Blender needed — runs the real controller against the headless physics:
uv sync --extra web
uv run web-bridge --demo                  # ws://localhost:8765

cd webapp && npm install && npm run dev    # http://localhost:5173

# against the real Blender sim instead:
uv run web-bridge --mqtt --host <broker-ip>
```

In `--demo` mode the bridge also streams a `pid` block (P/I/D terms of the three
velocity loops) for the profiling charts. See `webapp/README.md` for details.

### `/imu` — live attitude from a real flight controller

The dashboard has a second route, **`/imu`**, that visualises real IMU data. The
bridge subscribes to two hardware topics and forwards them to the app:

- **`drone/imu`**: `{t, yaw, pitch, roll, gz}` — Euler attitude (degrees) + gyro-Z
- **`drone/hw`**: `{throttle, s1, s2, s3, s4}` — throttle + four servo angles

The page rotates the 3D drone live by yaw/pitch/roll (with the vanes set from the
servo angles), shows an artificial-horizon instrument + numeric readouts, and
plots attitude / gyro-Z history.

```bash
# IMU on the same broker you point --host at (subscribes drone/imu + drone/hw):
uv run web-bridge --mqtt --host 10.243.245.93

# OR keep the main twin on one broker and read IMU from a separate flight controller:
uv run web-bridge --mqtt --host <sim-broker> --imu-host 10.243.245.93
```

Then open <http://localhost:5173/imu>. The **IMU** nav item shows a green dot when
attitude data is flowing.
