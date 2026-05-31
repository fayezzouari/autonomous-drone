# drone-nav — altitude-hold + independent-vane control for a singlecopter

Controls a **singlecopter** (one top-mounted propeller + 4 independent steering
vanes). A PID **holds altitude** via throttle, while the **four vanes are
commanded independently** for steering. The same controller code flies three
plants — a Blender physics sim, a headless physics stub, and a real ESP32-driven
airframe — all talking over **MQTT** with one shared JSON wire format.

```
        gamepad / vane-cmd / mission                 ┌──────────────────────────┐
                    │                          cmd   │  Blender sim bridge       │
                    ▼               drone/cmd  ──────►│  blender_sim_mqtt.py      │
            ┌───────────────┐                         │  (physics + 4 vanes)      │
            │   drone_nav    │◄──── drone/telemetry ──┤                           │
            │  (PID stack)   │                         └──────────────────────────┘
            └───────┬───────┘
        drone/hw    │   ▲ drone/imu          ┌──────────────────────────┐
       (servo°+ESC) ▼   │ (yaw/attitude)     │  web_bridge.py  ──WS──►   │  React + Three.js
            ┌───────────────┐                │  (MQTT → WebSocket @50Hz) │  digital twin + /imu
            │  ESP32-S3 node │                └──────────────────────────┘
            │  4 servos+ESC  │   phone-imu.py can stand in for the MPU6050 on drone/imu
            │  MPU6050 IMU   │
            └───────────────┘
```

The physical drone model (mass, thrust, vane limits, momentum-theory force law)
is **mirrored** across the Blender sim (`blender-navigatio.py`), the headless
stub (`tools/sim_stub.py`), and the controller's config — so the controller's
inverse model matches whatever plant it flies.

---

## Nodes & data flow

Every box is an independent process; every arrow is one MQTT topic (or, for the
web app, a WebSocket). All MQTT payloads are JSON and share field names defined
once in `drone_nav/telemetry.py`.

| Node | Process | Publishes | Subscribes |
|------|---------|-----------|------------|
| **Navigator** | `drone-nav` | `drone/cmd`, `drone/status` | `drone/telemetry`, `drone/vanes`, `drone/imu` |
| **Teleop** | `drone-teleop` | `drone/hw`, `drone/cmd` | `drone/imu` (yaw feedback) |
| **Blender sim** | `blender_sim_mqtt.py` | `drone/telemetry` | `drone/cmd` |
| **ESP32 node** | `firmware/drone_esp32` | `drone/imu`, `drone/status` | `drone/hw` |
| **Phone IMU** | `phone-imu` | `drone/imu` | — (reads phone over WebSocket) |
| **Vane CLI** | `vane-cmd` | `drone/vanes` | — |
| **Servo bench** | `tools/servo_test.py` | `drone/hw` | — |
| **Web bridge** | `web-bridge` | WebSocket (50 Hz) | `drone/telemetry`, `drone/cmd`, `drone/status`, `drone/imu`, `drone/hw` |

The navigator can be the **autopilot** (computes throttle + vanes for altitude
hold or A→B missions) or a **pass-through** (holds altitude, forwards raw vanes
from `drone/vanes`). `drone-teleop` replaces the navigator for manual PS4 flight
and speaks the hardware topics directly.

---

## Messaging contract (MQTT payloads)

Topic names live in `config/config.yaml` (`mqtt:` block) and are echoed in the
firmware. The defaults:

### `drone/telemetry` — measured state  ·  sim → navigator / web bridge

Published every physics tick. Positions in **m** (world frame), velocities in
**m/s**, `yaw` in **radians**, `prop_speed` in **deg/s**.

```json
{ "t": 12.34, "x": 0.0, "y": 0.0, "z": 2.01,
  "vx": 0.0, "vy": 0.0, "vz": 0.03, "yaw": 0.0, "prop_speed": 488.7 }
```

The Blender sim additionally appends its **actuator echo** so a remote viewer
mirrors the vanes/throttle under *any* control source (autopilot or manual):
extra keys `"throttle"`, `"v1"`, `"v2"`, `"v3"`, `"v4"`. The navigator's
`Telemetry` parser ignores unknown keys; the web bridge uses them.

### `drone/cmd` — autopilot command  ·  navigator → sim

`throttle` is a fraction **[0, 1]** of full prop speed; `vane1…vane4` are the
four **independent** vane angles in **radians** (applied raw, no mixing).

```json
{ "throttle": 0.68, "vane1": 0.05, "vane2": 0.0, "vane3": 0.05, "vane4": 0.0 }
```

Geometry: vanes **1 & 3** (X arm) → fore/aft (body-X); vanes **2 & 4** (Y arm) →
lateral (body-Y). The sim accepts `v1…v4` as aliases for `vane1…vane4`.

### `drone/vanes` — raw steering input  ·  external → navigator

Lets you steer by hand while the navigator holds altitude. Accepts an object
(`vane1…vane4`, or `v1…v4` aliases) **or** a 4-element list, in **radians**:

```json
{ "vane1": 0.15, "vane2": 0.0, "vane3": -0.15, "vane4": 0.0 }
[ 0.15, 0.0, -0.15, 0.0 ]
```

### `drone/imu` — orientation  ·  ESP32 / phone → navigator / web bridge

Euler attitude in **degrees** (drone body frame, aerospace ZYX: roll about X
forward, pitch about Y left, yaw about Z up); `gz` is the yaw rate in **deg/s**.
Published at ~50 Hz. The navigator folds `yaw` (converted to radians) into
telemetry for heading-hold; position/velocity are *not* observable from the IMU.

```json
{ "t": 12.34, "yaw": -3.2, "pitch": 0.8, "roll": -1.1, "gz": 0.4 }
```

### `drone/hw` — hardware actuator command  ·  navigator/teleop → ESP32

`throttle` is the ESC fraction **[0, 1]**; `s1…s4` are **logical servo angles in
degrees** (90 = neutral, hard-clamped to **[40, 160]**). The ESP32 applies its
own per-pin trim on top.

```json
{ "throttle": 0.0, "s1": 90, "s2": 90, "s3": 90, "s4": 90 }
```

### `drone/status` — status strings  ·  any node → world

Plain UTF-8 strings, not JSON. The ESP32 emits `esp32_online`, `link_restored`,
`failsafe_no_command`; the navigator emits `mission_complete`.

### WebSocket protocol (`web-bridge` → browser)

Newline-free JSON objects. One `meta` on connect, then `state` at 50 Hz:

```jsonc
// once, on connect — scene description
{ "type": "meta", "source": "demo"|"mqtt",
  "drone": { "mass": f, "gravity": f, "thrust_max": f,
             "prop_max_speed": f, "max_vane_deg": f, "rotor_radius": f },
  "hover_throttle": f, "ground_z": 0.0, "target_altitude": f }

// 50 Hz — merged live state
{ "type": "state",
  "telemetry": { t,x,y,z,vx,vy,vz,yaw,prop_speed },
  "command":   { throttle, vane1, vane2, vane3, vane4 },
  "status":    "…",
  "pid":  { "alt": { p,i,d,out,setpoint,measurement } } | null,  // demo mode only
  "imu":  { t,yaw,pitch,roll,gz } | null,                        // /imu route
  "hw":   { throttle,s1,s2,s3,s4 } | null }                      // /imu route
```

### Phone IMU bridge

`phone-imu` reads an Android phone's fused orientation via **SensorServer**
(WebSocket) and republishes it on `drone/imu` in the **exact** ESP32 format
above — so the rest of the stack can't tell phone from MPU6050.

---

## Modules

### `drone_nav/` — the Python control stack

| File | Role |
|------|------|
| `config.py` | Typed dataclass config (`DroneParams`, `ControlConfig`, `GotoConfig`, `MissionConfig`, `ManualConfig`, `ServoConfig`, `MQTTConfig`) + YAML loader. Drone defaults mirror the sim; exposes `hover_throttle`, `max_vane_rad`, `thrust_from_prop_speed`. |
| `telemetry.py` | The MQTT wire format: `Telemetry` (sim → nav) and `Command` (nav → sim) dataclasses, with tolerant `from_json` / `to_json`. One place both ends agree on field names. |
| `pid.py` | Reusable `PID` with output clamping, integral clamp + **conditional anti-windup**, and **derivative-on-measurement** (no setpoint kick). Records last P/I/D/out for profiling. |
| `controller.py` | `AltitudeController` (altitude → climb-rate → accel → throttle) and `GotoController` (cascaded per-axis position→velocity→accel, inverted through the vane model into pitch/roll, yaw-hold, mixed to 4 vanes). |
| `mission.py` | `Mission` — waypoint sequencer; a waypoint is "reached" only when inside `arrival_radius` **and** below `arrival_speed` for a sustained `hold_time`. |
| `mqtt_io.py` | `MqttLink` — paho-mqtt transport. Caches latest telemetry/vanes behind a lock; folds `drone/imu` into a telemetry snapshot; publishes `drone/cmd`, `drone/hw`, `drone/status`. |
| `main.py` | `drone-nav` entry point. Manual altitude-hold + raw vanes (default), or autonomous `--goto X Y Z` / `--mission`; `--sim` runs against the stub. |
| `vane_cmd.py` | `vane-cmd` CLI — publish raw vane angles to `drone/vanes` (`--deg`, `--zero`, `--v1…--v4`). |
| `gamepad.py` | `Gamepad` — pygame DS4/PS4 reader → normalised `StickState` (deadzone + expo, latched arm/alt-hold/kill, ramp-and-hold throttle). Run `python -m drone_nav.gamepad` to discover axis/button indices. |
| `manual_control.py` | `ManualPilot` — sticks + telemetry → `Command`. Sticks set *setpoints*; PIDs close yaw heading-hold and (optional) altitude-hold; mixes pitch/roll/yaw into the 4 vanes with the anti-torque couple. |
| `servo_map.py` | `ServoMapper` — `Command` (vane rad + throttle) → `{throttle, s1…s4}` logical degrees for `drone/hw`. Auto-fits the deg/rad gain to the travel limit; handles per-servo `reverse`. |
| `teleop.py` | `drone-teleop` entry point — PS4 manual flight. Wires gamepad → ManualPilot → ServoMapper → `drone/hw` (+ `drone/cmd` for the twin); `--sim` flies the stub. |
| `phone_imu.py` | `phone-imu` entry point — bridges an Android phone (SensorServer) onto `drone/imu`; quaternion → aerospace Euler, zeroes to startup pose. |

### `sim_bridge/` — simulation & visualisation bridges

| File | Role |
|------|------|
| `blender_sim_mqtt.py` | The full Blender sim: rigid-body-style translational physics (gravity, speed² thrust, momentum-theory vane force, drag, ground effect, reactive-torque yaw, ground bounce), gamepad/keyboard control, GPU HUD + 3D overlays. Publishes `drone/telemetry`, subscribes `drone/cmd` (first command latches autopilot mode on). Set `BROKER_HOST` + `VANE_OBJECTS` before running. |
| `web_bridge.py` | `web-bridge` — subscribes to the MQTT topics, merges state, rebroadcasts JSON over WebSocket at 50 Hz for the React app. `--mqtt` bridges a real broker; `--demo` runs the real controller over the headless stub (and streams a `pid` block). `--imu-host` can read IMU/HW from a separate broker. |

### `tools/` — offline & bench utilities

| File | Role |
|------|------|
| `sim_stub.py` | `SimStub` — headless re-implementation of the Blender sim's force model. Consumes `Command`, produces `Telemetry`; lets tests and `--sim`/`--demo` modes close the loop with no Blender. |
| `servo_test.py` | Bench tool — streams a fixed `drone/hw` command (default 20 Hz, to beat the 400 ms failsafe) so a vane holds. `--sweep` exercises each vane in turn; `--hop` does an open-loop throttle ramp. |

### `firmware/drone_esp32/` — the actuator + IMU node (C++ / PlatformIO)

Deliberately "dumb": the PC runs the PID. `src/main.cpp` subscribes `drone/hw`
(per-pin trim → clamp [40,160] → 4 servos on GPIO 36–39 + ESC on GPIO 14 via
explicit LEDC channels), reads an MPU6050 (SDA 5 / SCL 4), and **fails safe**
(ESC idle, vanes centred) if no command arrives for **400 ms**. Onboard IMU
publishing is currently disabled in favour of `phone-imu` on the same topic; see
`firmware/drone_esp32/README.md` for wiring, trim table, and flashing.

### `webapp/` — React + Three.js dashboard

Live digital twin fed by `web_bridge.py`. `src/types.ts` mirrors the WebSocket
wire format. Two routes (`react-router`):

- **`/`** — `SimulationViewport` (3D twin, trail, A→B waypoints, velocity/force/
  downwash overlays), `ComponentMap` (which parts move and where), and
  `Visualizations` (telemetry charts + per-axis PID profiling in `--demo`).
- **`/imu`** — `ImuView`: rotates the drone live by real `drone/imu` yaw/pitch/
  roll, sets vanes from `drone/hw`, shows an artificial horizon + attitude/gyro
  charts.

Hot 50 Hz data lives in a mutable store (`src/store.ts`) read each animation
frame, so React isn't re-rendered at stream rate. See `webapp/README.md`.

### `config/config.yaml`

Single source of truth for broker address/topics, drone physical constants (keep
in sync with the sim), the altitude + goto + manual PID gains, the mission
waypoints, the gamepad mapping, and the servo/ESC output limits.

### `tests/`

`pytest` suite: `test_pid.py` (anti-windup, derivative-on-measurement),
`test_controller.py` (altitude-hold convergence), `test_telemetry.py`
(wire-format round-trips) — all against `SimStub`, no broker needed.

---

## Control design

- **Altitude** is held by a PID on throttle:
  ```
  altitude error ──P──► climb-rate setpoint ──PID──► vertical accel
       T = m·(g + a_z)   →   throttle = √(T / T_max)
  ```
  Gravity feed-forward falls out for free (hover ≈ 68 % throttle).
- **Steering** uses **four independent vanes**. In pass-through mode the angles
  are supplied **raw** on `drone/vanes` (no PID, no mixing). In autonomous /
  manual modes, pitch/roll/yaw are mixed onto the four vanes:
  ```
  a1 = pitch + yaw   a3 = pitch − yaw     (X pair → fore/aft + yaw couple)
  a2 = roll  + yaw   a4 = roll  − yaw     (Y pair → lateral  + yaw couple)
  ```
  Force per vane ≈ `0.5·T_prop·sin(angle)`; the opposing yaw terms form the
  anti-torque couple that fights the prop's reaction torque.
- **Yaw** on the real airframe is a *swirl* of all four vanes (the differential
  above); with no swirl it drifts from reactive prop torque.

---

## Quick start

### 1. Install (uv)

```bash
uv sync --extra dev
```

### 2. Prove it works — offline, no broker, no Blender

The headless physics stub replays the sim's force model:

```bash
uv run drone-nav --sim --verbose --v1 0.2 --v3 0.2   # hold 2 m, push fore/aft
uv run pytest                                        # PID + controller + wire tests
```

### 3. Run against Blender over MQTT

1. Start an MQTT broker (mosquitto) reachable by both sides; point `mqtt.host`
   in `config/config.yaml` at it.
2. In `sim_bridge/blender_sim_mqtt.py`, set `BROKER_HOST` to that broker and
   **set `VANE_OBJECTS` to your four vane object names**.
3. Install `paho-mqtt` into **Blender's bundled Python** (not your venv):
   ```bash
   /path/to/Blender/.../python/bin/python3.x -m pip install paho-mqtt
   ```
4. In Blender, run the script (Text Editor → Run Script). The banner should
   show `MQTT <host>:1883`.
5. Start the navigator (holds altitude automatically):
   ```bash
   uv run drone-nav --verbose
   ```
6. Steer by publishing raw vane angles:
   ```bash
   uv run vane-cmd --deg --v1 10 --v3 10     # tilt fore/aft pair by 10°
   uv run vane-cmd --zero                     # vanes back to neutral
   ```

### 4. Autonomous A → B (closed-loop)

A cascaded position PID flies the drone to a point — computing throttle *and*
the four vane angles, with yaw-hold so it flies straight:

```bash
# fly to a single far point B (x y z) — add --sim to try it offline first
uv run drone-nav --sim --goto 14 10 6 --verbose
uv run drone-nav --goto 14 10 6 --verbose

# or fly the waypoint sequence in config.yaml (mission.waypoints)
uv run drone-nav --sim --mission --verbose
uv run drone-nav --mission --verbose
```

Tune the loops under `goto:` in `config.yaml`; set the sequence under
`mission.waypoints`.

### 5. Manual flight with a PS4 controller (real hardware)

A PS4 pad steers the singlecopter through an **ESP32-S3** that drives the four
vane servos + the brushless ESC and reports orientation over MQTT. **The pad
connects to the PC** — the PID stays in Python, so you keep live tuning and the
same `config.yaml`; the ESP32 is a dumb actuator+sensor node.

```
PS4 pad ─Bluetooth→ PC ── drone/hw (servo°+ESC) ──→ ESP32 ──→ 4 servos + ESC
   (gamepad.py → ManualPilot PID → servo_map)          │
        heading-hold ←── drone/imu (yaw) ──────────────┘ (MPU6050 / phone-imu)
```

Because control rides the Wi-Fi/MQTT link, the ESP32 **fails safe** — ESC idle,
vanes centred — if commands stop for 400 ms.

```bash
# try the controller + PID offline against the physics stub first (no hardware):
uv sync --extra gamepad
uv run drone-teleop --sim --verbose

# discover your pad's axis/button indices if the mapping looks off:
uv run python -m drone_nav.gamepad

# fly the real drone (broker + flashed ESP32 running):
uv run drone-teleop --verbose
```

Controls (DS4 defaults): **right stick** steers the vanes — Y = pitch (fore/aft),
X = roll (lateral). **Left stick Y = throttle** (up ramps, centre holds).
**L1/R1 = yaw**; **Options** = arm/disarm; **Circle** = altitude-hold; **PS** =
kill. Vanes map to a logical servo angle centred at 90° and hard-clamped to
**[40°, 160°]**; the ESP32 then applies per-pin trim.

> **IMU caveat**: an MPU6050 measures *orientation*, not position. Heading-hold
> uses its yaw; horizontal/altitude feedback isn't available from the IMU alone,
> so manual throttle is direct by default (altitude-hold needs a `z` source and
> is mainly useful in `--sim`).

### 6. Use a phone as the IMU (optional)

```bash
# install SensorServer on the phone, note its IP:port, same LAN as the broker
uv run phone-imu --phone 192.168.1.50:8080            # publishes drone/imu @50 Hz
uv run phone-imu --phone 192.168.1.50 --roll-sign -1  # flip an axis if mounted differently
```

---

## Live web app (3D viewport + component map + telemetry/PID charts)

`webapp/` is a React + Three.js front-end that mirrors the Blender viewport in
the browser as a **live digital twin**. Browsers can't speak raw MQTT TCP, so
`sim_bridge/web_bridge.py` subscribes to the topics, merges the state, and
rebroadcasts JSON at 50 Hz.

```bash
# no broker / Blender needed — runs the real controller against headless physics:
uv sync --extra web
uv run web-bridge --demo                  # ws://localhost:8765

cd webapp && npm install && npm run dev    # http://localhost:5173

# against the real Blender sim instead:
uv run web-bridge --mqtt --host <broker-ip>
```

In `--demo` mode the bridge also streams a `pid` block (P/I/D terms) for the
profiling charts.

### `/imu` — live attitude from a real flight controller

The `/imu` route visualises real IMU data. The bridge subscribes to `drone/imu`
(`{t,yaw,pitch,roll,gz}`) and `drone/hw` (`{throttle,s1,s2,s3,s4}`) and forwards
them; the page rotates the 3D drone live and shows an artificial horizon +
attitude/gyro history.

```bash
# IMU on the same broker you point --host at:
uv run web-bridge --mqtt --host 10.243.245.93

# OR keep the main twin on one broker and read IMU from a separate controller:
uv run web-bridge --mqtt --host <sim-broker> --imu-host 10.243.245.93
```

Then open <http://localhost:5173/imu>.

---

## Configuration

Everything lives in `config/config.yaml`: broker address/topics, drone physical
constants (keep in sync with the sim), the altitude/goto/manual PID gains, the
target altitude, the mission waypoints, the gamepad mapping, and the servo/ESC
output limits. See the comments in that file and the `Modules` table for the
dataclasses each block maps to.
