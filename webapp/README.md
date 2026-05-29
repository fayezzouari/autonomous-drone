# Singlecopter web app — live viewport · component map · telemetry/PID profiling

A React + Three.js front-end that shows, in the browser, exactly what the
Blender singlecopter simulation is doing — plus a component map of the moving
parts and a full telemetry/PID visualization section.

```
Blender sim ──MQTT──> sim_bridge/web_bridge.py ──WebSocket(JSON)──> this app
 (or --demo physics)        (subscribe + merge,                ├─ Simulation Viewport (3D digital twin)
                             rebroadcast @50 Hz)               ├─ Component Map (prop + vanes, who moves & where)
                                                               └─ Visualizations (telemetry charts + PID profiling)
```

## Why a digital twin (and not a Blender screen capture)

Browsers can't speak raw MQTT TCP, and pixel-streaming the Blender viewport is
heavy and fragile inside Blender's bundled Python. Instead the bridge streams
the **simulation state** (the same telemetry the navigator consumes) and the app
reconstructs the scene live with Three.js. Because it's driven by the exact
per-tick telemetry Blender publishes, the viewport mirrors the Blender viewport
— and the structured state is also what the component map and PID charts need.
(See the bridge file header and the project README for the research behind this.)

## Run it

Two terminals. **You do not need Blender or a broker to see it work** — the
bridge has a `--demo` mode that runs the real PID controller against the
headless physics in-process.

```bash
# 1) data source (from the repo root)
uv sync --extra web            # once, installs the `websockets` dep
uv run web-bridge --demo       # streams a live flight on ws://localhost:8765

# 2) the web app (this folder)
npm install                    # once
npm run dev                    # http://localhost:5173
```

To drive it from the **real** Blender sim instead, run the broker + Blender +
navigator as in the project README, then:

```bash
uv run web-bridge --mqtt --host <broker-ip>
```

If the bridge is not on `localhost:8765`, point the app at it:

```bash
VITE_BRIDGE_URL=ws://10.0.0.5:8765 npm run dev
```

## What's on screen

- **Simulation Viewport** — orbitable Z-up scene (matches Blender's axes): the
  singlecopter (body, spinning propeller, 4 steering vanes), ground grid,
  waypoints A→B→C with the active target ringed, a flight trail, and the same
  velocity (green) / lateral-force (orange) / downwash (blue) overlays Blender
  draws. The airframe leans into its motion using the sim's own lean model.
- **Component Map** — an exploded, auto-rotating view that highlights which
  parts are active and the direction each pushes: propeller spin sense + RPM,
  Vanes 1-3 (pitch / fore-aft), Vanes 2-4 (roll / left-right), and the vertical
  thrust arrow. A live legend lists each part's value and active state.
- **Visualizations** —
  - *Telemetry*: altitude, velocity, speed, propeller RPM, throttle, vane
    deflection, and X/Y position.
  - *PID profiling*: the cascaded velocity loops (X/Y/Z) each broken into their
    **P / I / D** terms and summed **output**. These come from the controller
    running inside the bridge, so they require `--demo` mode (in `--mqtt` mode
    the navigator is a separate process and only telemetry/commands are on the
    wire — the app shows a note).

## Notes

- Hot data (50 Hz) is kept in a mutable store and read each animation frame by
  the 3D scene and the canvas charts, so React isn't re-rendered at stream rate
  — see `src/store.ts`.
- `npm run build` type-checks and produces a static bundle in `dist/`.
