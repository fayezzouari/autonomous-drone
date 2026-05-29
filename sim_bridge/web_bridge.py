"""WebSocket bridge: MQTT sim state → browser.

Browsers cannot speak raw MQTT TCP, so this process sits between the broker and
the React web app. It subscribes to the same topics the navigator uses
(``drone/telemetry``, ``drone/cmd``, ``drone/status``), keeps the latest merged
state, and rebroadcasts it as JSON to every connected browser at a fixed rate.

The web app reconstructs the simulation as a live 3-D "digital twin": because
it is driven by the exact telemetry the Blender sim publishes each physics tick,
what you see in the browser is precisely what is happening inside Blender.

The airframe is an altitude-hold singlecopter: the controller computes throttle
to hold a target height, and the four vanes are commanded independently and raw
(vanes 1 & 3 → fore/aft body-X force, vanes 2 & 4 → lateral body-Y force).

Two modes:

  --mqtt   (default)  bridge a real broker — run this alongside Blender + the
                      navigator. Set --host / --port to your broker.

  --demo               no broker, no Blender: run the real altitude controller
                      (drone_nav) over the headless physics (tools/sim_stub.py)
                      and fly an automated vane figure, so you can see the web
                      app move instantly.

Wire protocol (server → client), newline-free JSON objects:

  {"type":"meta",  "drone":{...}, "target_altitude":f,
                   "hover_throttle":f, "ground_z":f, "source":"demo"|"mqtt"}
  {"type":"state", "telemetry":{t,x,y,z,vx,vy,vz,yaw,prop_speed},
                   "command":{throttle,vane1,vane2,vane3,vane4},
                   "status":str, "pid":{"alt":{p,i,d,out,setpoint,measurement}}|null}

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
        self.pid: Optional[dict] = None  # {"alt": {p,i,d,out,setpoint,measurement}}

    def update(self, *, telemetry=None, command=None, status=None, pid=...):
        with self._lock:
            if telemetry is not None:
                self.telemetry = telemetry
            if command is not None:
                self.command = command
            if status is not None:
                self.status = status
            if pid is not ...:
                self.pid = pid

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "type": "state",
                "telemetry": asdict(self.telemetry),
                "command": asdict(self.command),
                "status": self.status,
                "pid": self.pid,
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
        "target_altitude": cfg.control.target_altitude,
    }


# ── MQTT producer ──────────────────────────────────────────────────────────────
def start_mqtt(cfg: Config, state: SharedState) -> "object":
    """Subscribe to telemetry/cmd/status and feed SharedState. Returns the client."""
    import paho.mqtt.client as mqtt

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
            # If the sim also echoes its actuator state (throttle + v1..v4) in
            # telemetry, mirror it as the command so the twin's vanes/throttle
            # track the real Blender viewport even when no autopilot is
            # publishing drone/cmd (e.g. manual gamepad flight).
            cmd = None
            try:
                d = json.loads(msg.payload)
                if any(k in d for k in ("v1", "v2", "v3", "v4", "throttle")):
                    cmd = Command(
                        throttle=float(d.get("throttle", 0.0)),
                        vane1=float(d.get("v1", 0.0)), vane2=float(d.get("v2", 0.0)),
                        vane3=float(d.get("v3", 0.0)), vane4=float(d.get("v4", 0.0)))
            except (ValueError, TypeError):
                cmd = None
            state.update(telemetry=tlm, command=cmd)
        elif topic == cfg.mqtt.topic_command:
            try:
                state.update(command=Command.from_json(msg.payload))
            except (ValueError, TypeError):
                return
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
    """Hold altitude with the real controller while flying an automated vane
    figure, feeding SharedState — so the web app has lively motion to show."""
    from drone_nav.controller import AltitudeController
    from tools.sim_stub import SimStub

    controller = AltitudeController(cfg.drone, cfg.control)
    sim = SimStub()
    dt = 1.0 / cfg.control.loop_rate_hz
    target_z = cfg.control.target_altitude
    amp = math.radians(min(20.0, cfg.drone.max_vane_deg * 0.7))
    period = 9.0  # s for one figure cycle

    def pid_block():
        p = controller.pid_vz
        return {"alt": {
            "p": p.last_p, "i": p.last_i, "d": p.last_d, "out": p.last_output,
            "setpoint": p.last_setpoint, "measurement": p.last_measurement,
        }}

    async def runner():
        tlm = sim.telemetry()
        while True:
            thr = controller.throttle(tlm, target_z, dt)
            # Automated figure: fore/aft pair (1&3) and lateral pair (2&4) driven
            # in quadrature → a gentle circular wander while holding altitude.
            ph = 2 * math.pi * tlm.t / period
            v13 = amp * math.sin(ph)
            v24 = amp * math.sin(ph + math.pi / 2)
            cmd = Command(throttle=thr, vane1=v13, vane2=v24, vane3=v13, vane4=v24)
            tlm = sim.step(cmd, dt)
            state.update(telemetry=tlm, command=cmd,
                         status="demo · altitude-hold + auto-vane figure",
                         pid=pid_block())
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
