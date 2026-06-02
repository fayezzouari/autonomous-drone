# drone-nav — altitude-hold + independent-vane control for a singlecopter

Controls a **singlecopter** (one top-mounted propeller + 4 independent steering
vanes). A PID **holds altitude** via throttle, while the **four vanes are
commanded independently** for steering. On top of altitude-hold the stack can
fly **autonomous A→B missions** and **plan collision-free routes around
obstacles** (a Theta\* / A\* planner that inflates every box by the drone's own
size). The same controller code flies three plants — a Blender physics sim, a
headless physics stub, and a real ESP32-driven airframe — all talking over
**MQTT** with one shared JSON wire format.

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
| **Navigator** | `drone-nav` | `drone/cmd`, `drone/status`, `drone/path` | `drone/telemetry`, `drone/vanes`, `drone/imu`, `drone/obs`, `drone/goto` |
| **Teleop** | `drone-teleop` | `drone/hw`, `drone/cmd` | `drone/imu` (yaw feedback) |
| **Blender sim** | `blender_sim_mqtt.py` | `drone/telemetry`, `drone/obs` | `drone/cmd` |
| **ESP32 node** | `firmware/drone_esp32` | `drone/imu`, `drone/status` | `drone/hw` |
| **Phone IMU** | `phone-imu` | `drone/imu` | — (reads phone over WebSocket) |
| **Vane CLI** | `vane-cmd` | `drone/vanes` | — |
| **Servo bench** | `tools/servo_test.py` | `drone/hw` | — |
| **Web bridge** | `web-bridge` | WebSocket (50 Hz) | `drone/telemetry`, `drone/cmd`, `drone/status`, `drone/imu`, `drone/hw`, `drone/obs` |

The navigator can be the **autopilot** (computes throttle + vanes for altitude
hold, A→B missions, or obstacle-avoiding routes), a **go-to server** (flies to
each target streamed on `drone/goto`), or a **pass-through** (holds altitude,
forwards raw vanes from `drone/vanes`). In obstacle modes it subscribes to
`drone/obs`, plans a collision-free polyline, publishes it on `drone/path`, and
**replans live** as the obstacle set changes. `drone-teleop` replaces the
navigator for manual PS4 flight and speaks the hardware topics directly.

---

## Messaging contract (MQTT payloads)

Topic names live in `config/config.yaml` (`mqtt:` block) and are echoed in the
firmware. The defaults:

### `drone/telemetry` — measured state  ·  sim → navigator / web bridge

Published every physics tick. Positions in **m** (world frame), velocities in
**m/s**, `yaw` in **radians**, `gz` (yaw rate) in **rad/s**, `prop_speed` in
**deg/s**. `gz` feeds the heading-hold derivative term (yaw-rate damping).

```json
{ "t": 12.34, "x": 0.0, "y": 0.0, "z": 2.01,
  "vx": 0.0, "vy": 0.0, "vz": 0.03, "yaw": 0.0, "gz": 0.0, "prop_speed": 488.7 }
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

### `drone/obs` — obstacle field  ·  sim → navigator / web bridge

The world the planner must avoid: a list of **axis-aligned boxes** given by
centre + **half**-extents, in **m** (Z up). An optional `yaw` (radians about Z)
widens the box to the smallest AABB that still encloses the rotated footprint.

```json
{ "obstacles": [ { "cx": 6.0, "cy": 5.0, "cz": 4.0,
                   "hw": 1.0, "hd": 3.0, "hh": 4.0 } ] }
```

The older full-extent shape `{ "c":[x,y,z], "w":…, "h":…, "t":… }` is still
accepted (halved on parse). Which extent maps to which world axis is set by
`planner.obstacle_axes` (default `"wdh"` → X=width, Y=depth, Z=height, matching
the Blender publisher). The navigator **grows every box by `drone_radius +
safety_margin`** before planning, so the flown path keeps that clearance.

### `drone/goto` — live A→B target  ·  world → navigator (`--goto-topic`)

Stream a target and the go-to server replans and flies to it. Accepts a list, an
`{x,y,z}` (or `{tx,ty,tz}`) object, or a `{"goto":[…]}` / `{"target":[…]}` wrapper:

```json
[ 14.0, 10.0, 6.0 ]
{ "x": 14.0, "y": 10.0, "z": 6.0 }
```

### `drone/path` — planned route  ·  navigator → world / web bridge

The collision-free polyline the navigator is about to fly (A → … → B), published
on each (re)plan so a viewer can preview it before the drone sets off.

```json
{ "waypoints": [ [0,0,2], [4.2,2.1,4], [14,10,6] ] }
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
// once, on connect — scene description (obstacles seeded from config or live feed)
{ "type": "meta", "source": "demo"|"mqtt",
  "drone": { "mass": f, "gravity": f, "thrust_max": f,
             "prop_max_speed": f, "max_vane_deg": f, "rotor_radius": f },
  "hover_throttle": f, "ground_z": 0.0, "target_altitude": f,
  "obstacles": [ { cx,cy,cz, hx,hy,hz } ] }

// only when the obstacle set changes — world-AABB boxes the planner avoids
{ "type": "obstacles", "obstacles": [ { cx,cy,cz, hx,hy,hz } ] }

// 50 Hz — merged live state
{ "type": "state",
  "telemetry": { t,x,y,z,vx,vy,vz,yaw,gz,prop_speed },
  "command":   { throttle, vane1, vane2, vane3, vane4 },
  "status":    "…",
  "pid":  { "alt": { p,i,d,out,setpoint,measurement } } | null,  // demo mode only
  "imu":  { t,yaw,pitch,roll,gz } | null,                        // /position route
  "hw":   { throttle,s1,s2,s3,s4 } | null }                      // /position route
```

Obstacles change rarely, so the bridge sends them on their own `obstacles`
message (and once inside `meta` on connect) rather than bloating every 50 Hz
state frame. The boxes are the inflated-free world AABBs the navigator reasons
about, so the browser draws exactly what the planner avoids.

### Phone IMU bridge

`phone-imu` reads an Android phone's fused orientation via **SensorServer**
(WebSocket) and republishes it on `drone/imu` in the **exact** ESP32 format
above — so the rest of the stack can't tell phone from MPU6050.

---

## Modules

### `drone_nav/` — the Python control stack

| File | Role |
|------|------|
| `config.py` | Typed dataclass config (`DroneParams`, `ControlConfig`, `GotoConfig`, `MissionConfig`, `ManualConfig`, `ServoConfig`, `MQTTConfig`, `PlannerConfig`, static `obstacles`) + YAML loader. Drone defaults mirror the sim; exposes `hover_throttle`, `max_vane_rad`, `thrust_from_prop_speed`. |
| `telemetry.py` | The MQTT wire format: `Telemetry` (sim → nav) and `Command` (nav → sim) dataclasses, with tolerant `from_json` / `to_json`. Carries `gz` (yaw rate) for heading-hold damping. One place both ends agree on field names. |
| `pid.py` | Reusable `PID` with output clamping, integral clamp + **conditional anti-windup**, and **derivative-on-measurement** (no setpoint kick). Records last P/I/D/out for profiling. |
| `controller.py` | `AltitudeController` (altitude → climb-rate → accel → throttle) and `GotoController` (cascaded per-axis position→velocity→accel, inverted through the vane model into pitch/roll, yaw-hold, mixed to 4 vanes). `couple_climb` makes it ascend *along* the path instead of shooting straight up from a standstill. |
| `obstacles.py` | `Box` (AABB) + `ObstacleField` — parses the `drone/obs` payload (centre + half-extents, optional yaw → enclosing AABB), Minkowski-inflates every box by the drone's clearance radius, and answers point/segment collision queries for the planner. Pure-Python, no numpy. |
| `planner.py` | `PathPlanner` — **Theta\*** (any-angle, default) or **A\*** over a 26-connected 3-D lattice, with a fast straight-line check and a greedy string-pull shortcut → a handful of any-angle waypoints. `PathFollower` flows along the polyline (advances by proximity, no stop-at-every-vertex), `densify` re-samples it, `PlannerConfig` holds the tuning. |
| `mission.py` | `Mission` — waypoint sequencer; a waypoint is "reached" only when inside `arrival_radius` **and** below `arrival_speed` for a sustained `hold_time`. |
| `mqtt_io.py` | `MqttLink` — paho-mqtt transport. Caches latest telemetry/vanes/obstacles/goto behind a lock (with change-versions for replanning); folds `drone/imu` into a telemetry snapshot; publishes `drone/cmd`, `drone/hw`, `drone/status`, `drone/path`. |
| `main.py` | `drone-nav` entry point. Manual altitude-hold + raw vanes (default), autonomous `--goto X Y Z` / `--mission`, obstacle-avoiding `--avoid`, or a live `--goto-topic` server; `--sim` runs any mode against the stub. |
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
| `web_bridge.py` | `web-bridge` — subscribes to the MQTT topics, merges state, rebroadcasts JSON over WebSocket at 50 Hz for the React app. Forwards the `drone/obs` obstacle boxes (seeded from config, replaced by the live feed) on their own message so the browser draws what the planner avoids. `--mqtt` bridges a real broker; `--demo` runs the real controller over the headless stub (and streams a `pid` block). `--imu-host` can read IMU/HW from a separate broker. |

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
wire format. Three routes (`react-router`):

- **`/`** — `SimulationViewport` (3D twin, trail, A→B waypoints, **obstacle boxes**
  the planner avoids, world reference axes, velocity/force/downwash overlays),
  `ComponentMap` (which parts move and where), and `Visualizations` (telemetry
  charts + per-axis PID profiling in `--demo`).
- **`/position`** — `ImuView`: rotates the drone live by real `drone/imu` yaw/
  pitch/roll, sets vanes from `drone/hw`, shows an artificial horizon +
  attitude/gyro charts.
- **`/profiling`** — `Profiling`: **offline** analysis of a Blender flight-trail
  dump (`samples/*.json`). `src/profiling.ts` differentiates the recorded path
  into velocity/accel, splits it into legs against the mission waypoints, and
  *replays the goto controller's setpoint logic* over the states; `StaticChart`
  renders the scrubable per-axis traces alongside a 3-D path view.

Hot 50 Hz data lives in a mutable store (`src/store.ts`) read each animation
frame, so React isn't re-rendered at stream rate (obstacles change rarely, so a
version counter re-renders them only on change). See `webapp/README.md`.

### `config/config.yaml`

Single source of truth for broker address/topics, drone physical constants (keep
in sync with the sim), the altitude + goto + manual PID gains, the mission
waypoints, the **path-planner tuning** (`planner:`) and **static obstacles**
(`obstacles:`, used by `--sim --avoid`), the gamepad mapping, and the servo/ESC
output limits.

### `tests/`

`pytest` suite: `test_pid.py` (anti-windup, derivative-on-measurement),
`test_controller.py` (altitude-hold convergence), `test_telemetry.py`
(wire-format round-trips), `test_planner.py` (obstacle geometry, planner
collision-freedom, and an end-to-end flight of a planned route through the stub),
and `test_goto_topic.py` (live-target parsing) — all against `SimStub`, no broker
needed.

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
  above); with no swirl it drifts from reactive prop torque. Manual/heading-hold
  adds a constant **feedforward anti-torque swirl** (`manual.yaw_antitorque`,
  ramped in with throttle) so the airframe doesn't spin up; the heading-hold PID
  then trims the residual (`ki`) and damps the measured yaw rate `gz` (`kd`).
- **Obstacle avoidance** (`--avoid`) plans *before* it flies. Each box is
  Minkowski-inflated by the clearance radius (drone size + margin), so the drone
  is treated as a point; **Theta\*** searches a 3-D lattice for an any-angle
  polyline, a string-pull pass trims it, and a `PathFollower` flows along it
  (advancing by proximity, capped to `cruise_speed` so turns stay on the line).
  The climb rate is **coupled to forward progress** so it ascends along the path
  instead of shooting straight up. No safe path ⇒ hold, never fly blind.

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

### 5. Obstacle avoidance (`--avoid`)

Add `--avoid` to any `--goto`/`--mission` run and the navigator plans a
**collision-free route** instead of flying straight. Every obstacle box is grown
by `drone_radius + safety_margin` (so the drone is treated as a point), a Theta\*
search finds an any-angle polyline around them, and the route is published on
`drone/path` for preview before takeoff. If no safe path exists the drone
**holds position** rather than flying blind — and it **replans live** whenever
the obstacle set on `drone/obs` changes.

```bash
# offline: plan around the static `obstacles:` in config.yaml, fly it in the stub
uv run drone-nav --sim --avoid --goto 14 10 6 --verbose

# live: read obstacles from drone/obs (published by the Blender sim), replan on change
uv run drone-nav --avoid --goto 14 10 6 --verbose
uv run drone-nav --avoid --mission --verbose
```

Tune the planner under `planner:` in `config.yaml` (clearance, lattice
resolution, `algorithm: theta|astar`, cruise/climb speed caps, the axis mapping
for incoming boxes, and the pre-flight `preview_pause`).

### 6. Live go-to server (`--goto-topic`)

Run the navigator as a service that holds a hover until a target arrives on
`drone/goto`, flies there (avoiding `drone/obs`), then holds again — replanning
on a new target or a changed obstacle set:

```bash
uv run drone-nav --goto-topic --verbose
# from anywhere on the broker, send a target:
mosquitto_pub -h <broker-ip> -t drone/goto -m '[14, 10, 6]'
mosquitto_pub -h <broker-ip> -t drone/goto -m '{"x":2,"y":0,"z":3}'
```

### 7. Manual flight with a PS4 controller (real hardware)

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

### 8. Use a phone as the IMU (optional)

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

### `/position` — live attitude from a real flight controller

The `/position` route visualises real IMU data. The bridge subscribes to
`drone/imu` (`{t,yaw,pitch,roll,gz}`) and `drone/hw` (`{throttle,s1,s2,s3,s4}`)
and forwards them; the page rotates the 3D drone live and shows an artificial
horizon + attitude/gyro history.

```bash
# IMU on the same broker you point --host at:
uv run web-bridge --mqtt --host 10.243.245.93

# OR keep the main twin on one broker and read IMU from a separate controller:
uv run web-bridge --mqtt --host <sim-broker> --imu-host 10.243.245.93
```

Then open <http://localhost:5173/position>.

---

## Configuration

Everything lives in `config/config.yaml`: broker address/topics, drone physical
constants (keep in sync with the sim), the altitude/goto/manual PID gains, the
target altitude, the mission waypoints, the `planner:` block (clearance,
lattice resolution, algorithm, speed caps, obstacle axis mapping) and static
`obstacles:` for `--sim --avoid`, the gamepad mapping, and the servo/ESC output
limits. See the comments in that file and the `Modules` table for the
dataclasses each block maps to.
