# drone-nav — PID navigation for a singlecopter

Navigates a **singlecopter** (one top-mounted propeller + 4 steering vanes) from
point A to point B (and through a list of waypoints). A cascaded PID controller
computes autopilot commands from telemetry and exchanges them with a Blender
physics simulation over **MQTT**.

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
config/config.yaml              MQTT, drone params, PID gains, mission/waypoints
drone_nav/
  config.py                     typed config + YAML loader
  telemetry.py                  Telemetry / Command dataclasses (the MQTT wire format)
  pid.py                        reusable PID (anti-windup, derivative-on-measurement)
  controller.py                 cascaded position/velocity/altitude controller
  mission.py                    waypoint sequencing + arrival logic
  mqtt_io.py                    paho-mqtt transport
  main.py                       entry point (real MQTT, or --sim offline)
sim_bridge/blender_sim_mqtt.py  the sim + MQTT autopilot input + telemetry output
tools/sim_stub.py               headless re-impl of the sim physics (for offline tests)
tests/                          PID unit tests + closed-loop A→B convergence tests
blender-navigatio.py            original simulation (untouched reference)
```

## Control design

Per world axis, cascaded:

```
position error ──P──► velocity setpoint ──PID──► acceleration command
```

The acceleration command is mapped to actuators by inverting the sim's own
force model:

- **Horizontal** — desired world acceleration is rotated into the drone body
  frame by the measured `yaw`, then the vane angles come from
  `sin(angle) = m·a_body / T_prop` (momentum theory gives lateral force
  `F_lat ≈ T_prop`). Live thrust `T_prop` is estimated from telemetry prop speed.
- **Vertical** — altitude PID → desired vertical acceleration → required thrust
  `T = m(g + a_z)/(cosθ·cosφ)` → `throttle = √(T / T_max)`. Gravity
  feed-forward falls out for free (hover ≈ 68 % throttle).
- **Yaw** is not an actuator on this airframe (it drifts from reactive prop
  torque), so the controller stays world-frame and only *compensates* for the
  measured heading.

## Quick start

### 1. Install (uv)

```bash
uv sync --extra dev
```

### 2. Prove it flies — offline, no broker, no Blender

The headless physics stub replays the sim's force model so you can verify
convergence and tune gains instantly:

```bash
uv run drone-nav --sim --verbose      # flies the waypoints in config.yaml
uv run pytest                         # PID + closed-loop A→B tests
```

### 3. Run against Blender over MQTT

1. Start an MQTT broker, e.g. mosquitto:
   ```bash
   brew install mosquitto && brew services start mosquitto   # localhost:1883
   ```
2. Install `paho-mqtt` into **Blender's bundled Python** (not your venv):
   ```bash
   # find Blender's python, then:
   /path/to/Blender.app/Contents/Resources/<ver>/python/bin/python3.x -m pip install paho-mqtt
   ```
3. In Blender, open and run `sim_bridge/blender_sim_mqtt.py` from the Text
   Editor (it sets up the scene, starts physics, and connects to the broker).
   Set `BROKER_HOST`/`BROKER_PORT` at the top of that file if your broker isn't
   on `localhost:1883`.
4. Start the navigator:
   ```bash
   uv run drone-nav --verbose
   ```

The drone takes off, flies the configured waypoints, and holds hover when the
mission completes.

## Configuration

Everything lives in `config/config.yaml`: broker address/topics, drone physical
constants (keep in sync with the sim), PID gains/limits, and the waypoint list
with arrival tolerances. See the comments in that file.

## MQTT contract

- **`drone/telemetry`** (sim → nav): `{t,x,y,z,vx,vy,vz,yaw,prop_speed}`
- **`drone/cmd`** (nav → sim): `{throttle:[0..1], pitch:rad, roll:rad}`
  (`pitch` → Vanes 1-3, `roll` → Vanes 2-4)
- **`drone/status`** (nav → world): mission status strings
