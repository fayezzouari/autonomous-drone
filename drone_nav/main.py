"""Entry point: wire telemetry → controller → command, run the mission loop.

Two modes:
  (default)  real MQTT — talks to the Blender sim bridge over a broker.
  --sim      in-process headless physics (tools/sim_stub.py) — no broker, no
             Blender. Great for tuning gains and proving A→B convergence.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from .config import Config, load_config
from .controller import NavigationController
from .mission import Mission
from .telemetry import Command, Telemetry

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.yaml"


def _format(tlm: Telemetry, tgt, cmd: Command) -> str:
    tx, ty, tz = tgt if tgt else (float("nan"),) * 3
    return (f"pos=({tlm.x:+.2f},{tlm.y:+.2f},{tlm.z:+.2f}) "
            f"tgt=({tx:+.2f},{ty:+.2f},{tz:+.2f}) "
            f"spd={tlm.speed:.2f} yaw={tlm.yaw:+.2f} "
            f"thr={cmd.throttle:.2f} pit={cmd.pitch:+.2f} rol={cmd.roll:+.2f}")


def run_sim(cfg: Config, verbose: bool, max_seconds: float = 60.0) -> bool:
    """Closed-loop run against the in-process physics stub. Returns success."""
    from tools.sim_stub import SimStub

    controller = NavigationController(cfg.drone, cfg.control)
    mission = Mission(cfg.mission)
    sim = SimStub()
    dt = 1.0 / cfg.mission.loop_rate_hz
    tlm = sim.telemetry()

    steps = int(max_seconds / dt)
    for i in range(steps):
        target = mission.update(tlm, dt)
        if target is None:
            print(f"[sim] mission complete at t={tlm.t:.1f}s, "
                  f"pos=({tlm.x:+.2f},{tlm.y:+.2f},{tlm.z:+.2f})")
            return True
        cmd = controller.update(tlm, target, dt)
        tlm = sim.step(cmd, dt)
        if verbose and i % int(cfg.mission.loop_rate_hz / 5) == 0:
            print(f"t={tlm.t:5.1f} " + _format(tlm, target, cmd))

    print(f"[sim] timed out after {max_seconds:.0f}s without completing mission")
    return False


def run_mqtt(cfg: Config, verbose: bool) -> None:
    """Closed-loop run over a real MQTT broker against the Blender sim bridge."""
    from .mqtt_io import MqttLink

    controller = NavigationController(cfg.drone, cfg.control)
    mission = Mission(cfg.mission)
    link = MqttLink(cfg.mqtt)

    print(f"[mqtt] connecting to {cfg.mqtt.host}:{cfg.mqtt.port} ...")
    if not link.connect():
        print(f"[mqtt] could not reach the broker at "
              f"{cfg.mqtt.host}:{cfg.mqtt.port}"
              + (f" ({link.last_error})" if link.last_error else "") + ".")
        print("       Checklist:")
        print("       - is the broker (mosquitto) running on that machine?")
        print("       - is it listening on 0.0.0.0 (not just localhost)?")
        print("       - is port 1883 open in that machine's firewall?")
        print(f"       - quick test from here:  "
              f"mosquitto_sub -h {cfg.mqtt.host} -t 'drone/#' -v")
        return
    print("[mqtt] connected. Waiting for telemetry from the sim ...")

    dt = 1.0 / cfg.mission.loop_rate_hz
    last_log = 0.0
    try:
        while True:
            loop_start = time.perf_counter()
            tlm = link.latest_telemetry()
            if tlm is None:
                time.sleep(dt)
                continue

            target = mission.update(tlm, dt)
            if target is None:
                link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                link.publish_status("mission_complete")
                print("[mqtt] mission complete — holding hover.")
                break

            cmd = controller.update(tlm, target, dt)
            link.publish_command(cmd)

            if verbose and time.perf_counter() - last_log > 0.5:
                print(_format(tlm, target, cmd))
                last_log = time.perf_counter()

            # Maintain loop rate.
            elapsed = time.perf_counter() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        print("\n[mqtt] interrupted — sending idle command.")
        link.publish_command(Command(throttle=0.0))
    finally:
        link.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PID drone navigation controller")
    ap.add_argument("-c", "--config", default=str(DEFAULT_CONFIG),
                    help="path to config.yaml")
    ap.add_argument("--sim", action="store_true",
                    help="run against the in-process physics stub (no broker)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print telemetry/command trace")
    ap.add_argument("--max-seconds", type=float, default=60.0,
                    help="sim-mode time budget before giving up")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if not cfg.mission.waypoints:
        print("No waypoints configured. Add some under `mission.waypoints` "
              "in your config.")
        return 2

    if args.sim:
        ok = run_sim(cfg, args.verbose, args.max_seconds)
        return 0 if ok else 1
    run_mqtt(cfg, args.verbose)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
