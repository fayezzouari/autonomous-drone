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
  mqtt_io.py                    paho-mqtt transport (telemetry + raw vane input)
  main.py                       entry point: altitude-hold + raw vanes (or --sim)
  vane_cmd.py                   `vane-cmd` CLI to publish raw vane angles
sim_bridge/blender_sim_mqtt.py  the sim + MQTT autopilot (4 independent vanes) + telemetry
tools/sim_stub.py               headless re-impl of the sim physics (for offline tests)
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

## Configuration

Everything lives in `config/config.yaml`: broker address/topics, drone physical
constants (keep in sync with the sim), the altitude PID gains, and the target
altitude. See the comments in that file.

## MQTT contract

- **`drone/telemetry`** (sim → nav): `{t,x,y,z,vx,vy,vz,yaw,prop_speed}`
- **`drone/cmd`** (nav → sim): `{throttle:[0..1], vane1,vane2,vane3,vane4}` (rad)
  — vanes 1 & 3 = fore/aft pair, vanes 2 & 4 = lateral pair
- **`drone/vanes`** (external → nav): raw vane angles, `{vane1,vane2,vane3,vane4}`
  (or `[v1,v2,v3,v4]`); merged with the altitude-hold throttle
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
