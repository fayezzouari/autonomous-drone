"""WebSocket bridge: MQTT sim state → browser.

Browsers cannot speak raw MQTT TCP, so this process sits between the broker and
the React web app. It subscribes to the same topics the navigator uses
(``drone/telemetry``, ``drone/cmd``, ``drone/status``), keeps the latest merged
state, and rebroadcasts it as JSON to every connected browser at a fixed rate.

The web app reconstructs the simulation as a live 3-D "digital twin": because
it is driven by the exact telemetry the Blender sim publishes each physics tick,
what you see in the browser is precisely what is happening inside Blender.

Two modes:

  --mqtt   (default)  bridge a real broker — run this alongside Blender + the
                      navigator. Set --host / --port to your broker.

  --demo               no broker, no Blender: run the real headless physics
                      (tools/sim_stub.py) + PID controller + mission in-process
                      and stream that. Lets you see the web app fly instantly.

Wire protocol (server → client), newline-free JSON objects:

  {"type":"meta",  "drone":{...}, "waypoints":[[x,y,z],...],
                   "hover_throttle":f, "ground_z":f, "source":"demo"|"mqtt"}
  {"type":"state", "telemetry":{t,x,y,z,vx,vy,vz,yaw,prop_speed},
                   "command":{throttle,pitch,roll}, "status":str,
                   "target_index":int|null}

Run:
  uv run web-bridge --demo
  uv run web-bridge --mqtt --host 10.158.32.93
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import threading
from dataclasses import asdict
from pathlib import Path
from typing import Optional, Set

from drone_nav.config import Config, load_config
from drone_nav.telemetry import Command, Telemetry

try:
    import websockets
except ImportError:  # pragma: no cover - guidance only
    websockets = None  # type: ignore

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.yaml"
BROADCAST_HZ = 50.0


class SharedState:
    """Latest merged sim state, written by a producer, read by the broadcaster.

    Field assignment under the GIL is atomic enough for these plain floats; the
    lock just guarantees readers see a consistent telemetry/command pair.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.telemetry = Telemetry()
        self.command = Command()
        self.status = "idle"
        self.target_index: Optional[int] = None
        # PID profiling block (demo mode only; None when not available).
        self.pid: Optional[dict] = None
        self.setpoint: Optional[dict] = None

    def update(self, *, telemetry=None, command=None, status=None,
               target_index=..., pid=..., setpoint=...):
        with self._lock:
            if telemetry is not None:
                self.telemetry = telemetry
            if command is not None:
                self.command = command
            if status is not None:
                self.status = status
            if target_index is not ...:
                self.target_index = target_index
            if pid is not ...:
                self.pid = pid
            if setpoint is not ...:
                self.setpoint = setpoint

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "type": "state",
                "telemetry": asdict(self.telemetry),
                "command": asdict(self.command),
                "status": self.status,
                "target_index": self.target_index,
                "pid": self.pid,
                "setpoint": self.setpoint,
            }


def _meta_message(cfg: Config, source: str) -> dict:
    d = cfg.drone
    return {
        "type": "meta",
        "source": source,
        "drone": {
            "mass": d.mass,
            "gravity": d.gravity,
            "thrust_max": d.thrust_max,
            "prop_max_speed": d.prop_max_speed,
            "max_vane_deg": d.max_vane_deg,
            "rotor_radius": d.rotor_radius,
        },
        "hover_throttle": d.hover_throttle,
        "ground_z": 0.0,
        "waypoints": [[float(c) for c in wp] for wp in cfg.mission.waypoints],
    }


# ── MQTT producer ──────────────────────────────────────────────────────────────
def start_mqtt(cfg: Config, state: SharedState) -> "object":
    """Subscribe to telemetry/cmd/status and feed SharedState. Returns the client."""
    import paho.mqtt.client as mqtt

    waypoints = [tuple(float(c) for c in wp) for wp in cfg.mission.waypoints]

    def nearest_unreached(tlm: Telemetry) -> Optional[int]:
        if not waypoints:
            return None
        # Highlight the closest waypoint as the presumed active target.
        dists = [math.dist((tlm.x, tlm.y, tlm.z), wp) for wp in waypoints]
        return int(min(range(len(dists)), key=dists.__getitem__))

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="web-bridge")
    except (AttributeError, TypeError):  # paho 1.x
        client = mqtt.Client(client_id="web-bridge")

    if cfg.mqtt.username:
        client.username_pw_set(cfg.mqtt.username, cfg.mqtt.password)

    def on_connect(c, u, flags, rc, properties=None):
        c.subscribe(cfg.mqtt.topic_telemetry)
        c.subscribe(cfg.mqtt.topic_command)
        c.subscribe(cfg.mqtt.topic_status)
        print(f"[web-bridge] MQTT connected to {cfg.mqtt.host}:{cfg.mqtt.port}; "
              f"subscribed to telemetry/cmd/status")

    def on_message(c, u, msg):
        topic = msg.topic
        if topic == cfg.mqtt.topic_telemetry:
            try:
                tlm = Telemetry.from_json(msg.payload)
            except (ValueError, TypeError):
                return
            state.update(telemetry=tlm, target_index=nearest_unreached(tlm))
        elif topic == cfg.mqtt.topic_command:
            try:
                cmd = Command.from_json(msg.payload)
            except (ValueError, TypeError):
                return
            state.update(command=cmd)
        elif topic == cfg.mqtt.topic_status:
            try:
                state.update(status=msg.payload.decode("utf-8"))
            except UnicodeDecodeError:
                return

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(cfg.mqtt.host, cfg.mqtt.port, cfg.mqtt.keepalive)
    client.loop_start()
    return client


# ── Demo producer (in-process physics, no broker) ───────────────────────────────
def start_demo(cfg: Config, state: SharedState, loop: asyncio.AbstractEventLoop):
    """Fly the real controller against the headless stub, feeding SharedState."""
    from drone_nav.controller import NavigationController
    from drone_nav.mission import Mission
    from tools.sim_stub import SimStub

    controller = NavigationController(cfg.drone, cfg.control)
    mission = Mission(cfg.mission)
    sim = SimStub()
    dt = 1.0 / cfg.mission.loop_rate_hz

    def pid_block():
        """Snapshot the three velocity PIDs' P/I/D terms for the profiler."""
        def one(pid):
            return {
                "p": pid.last_p, "i": pid.last_i, "d": pid.last_d,
                "out": pid.last_output, "error": pid.last_error,
                "setpoint": pid.last_setpoint, "measurement": pid.last_measurement,
            }
        return {
            "vx": one(controller.pid_vx),
            "vy": one(controller.pid_vy),
            "vz": one(controller.pid_vz),
        }

    async def runner():
        tlm = sim.telemetry()
        hold = Command(throttle=cfg.drone.hover_throttle)
        while True:
            target = mission.update(tlm, dt)
            if target is None:
                # Mission complete — hold a gentle hover so the twin keeps living.
                cmd = hold
                status = "mission_complete"
                idx = None
                # Loop the demo so there is always something to watch.
                if mission.complete and cfg.mission.waypoints:
                    mission.index = 0
                    mission.complete = False
                    mission._dwell = 0.0
            else:
                cmd = controller.update(tlm, target, dt)
                status = f"flying → wp{mission.index}"
                idx = mission.index
            tlm = sim.step(cmd, dt)
            sp = {"vx": controller.pid_vx.last_setpoint,
                  "vy": controller.pid_vy.last_setpoint,
                  "vz": controller.pid_vz.last_setpoint}
            state.update(telemetry=tlm, command=cmd, status=status,
                         target_index=idx, pid=pid_block(), setpoint=sp)
            await asyncio.sleep(dt)

    return loop.create_task(runner())


# ── WebSocket server ─────────────────────────────────────────────────────────────
async def serve(host: str, port: int, cfg: Config, state: SharedState, source: str):
    clients: Set = set()
    meta = json.dumps(_meta_message(cfg, source))

    async def handler(ws):
        clients.add(ws)
        peer = getattr(ws, "remote_address", "?")
        print(f"[web-bridge] browser connected ({peer}); {len(clients)} client(s)")
        try:
            await ws.send(meta)          # one-shot scene description
            async for _ in ws:           # ignore inbound; keep the socket open
                pass
        except Exception:
            pass
        finally:
            clients.discard(ws)
            print(f"[web-bridge] browser disconnected; {len(clients)} client(s)")

    async def broadcaster():
        period = 1.0 / BROADCAST_HZ
        while True:
            if clients:
                payload = json.dumps(state.snapshot())
                # websockets.broadcast fans out without awaiting each send.
                websockets.broadcast(clients, payload)
            await asyncio.sleep(period)

    print(f"[web-bridge] WebSocket listening on ws://{host}:{port}  (source: {source})")
    async with websockets.serve(handler, host, port):
        await broadcaster()


def main(argv=None) -> int:
    if websockets is None:
        print("The 'websockets' package is required. Install with:\n"
              "    uv add websockets    (or: uv sync --extra web)")
        return 2

    ap = argparse.ArgumentParser(description="MQTT → WebSocket bridge for the web viewport")
    ap.add_argument("-c", "--config", default=str(DEFAULT_CONFIG))
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--mqtt", action="store_true",
                      help="bridge a real MQTT broker (default)")
    mode.add_argument("--demo", action="store_true",
                      help="run in-process physics, no broker/Blender needed")
    ap.add_argument("--host", default=None, help="MQTT broker host (overrides config)")
    ap.add_argument("--port", type=int, default=None, help="MQTT broker port")
    ap.add_argument("--ws-host", default="0.0.0.0", help="WebSocket bind host")
    ap.add_argument("--ws-port", type=int, default=8765, help="WebSocket bind port")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.host:
        cfg.mqtt.host = args.host
    if args.port:
        cfg.mqtt.port = args.port

    state = SharedState()
    source = "demo" if args.demo else "mqtt"

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    mqtt_client = None
    if args.demo:
        start_demo(cfg, state, loop)
    else:
        try:
            mqtt_client = start_mqtt(cfg, state)
        except OSError as exc:
            print(f"[web-bridge] could not reach broker {cfg.mqtt.host}:{cfg.mqtt.port} "
                  f"— {exc}\n   (try --demo to run without a broker)")
            return 1

    try:
        loop.run_until_complete(
            serve(args.ws_host, args.ws_port, cfg, state, source))
    except KeyboardInterrupt:
        print("\n[web-bridge] shutting down.")
    finally:
        if mqtt_client is not None:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
