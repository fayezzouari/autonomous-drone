"""Entry point. Two ways to fly the singlecopter:

  • MANUAL (default): a PID holds altitude; you steer with raw, independent vane
    angles published to ``drone/vanes`` (see the ``vane-cmd`` CLI).

  • AUTONOMOUS A→B (--goto / --mission): a cascaded position PID flies the drone
    to a world target (and through a waypoint sequence), computing BOTH the
    throttle and the four vane angles automatically, with yaw-hold.

Add ``--sim`` to any mode to run against the in-process physics stub (no broker,
no Blender).
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from .config import Config, load_config
from .controller import AltitudeController, GotoController
from .mission import Mission
from .telemetry import Command, Telemetry

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.yaml"


# ── formatting ──────────────────────────────────────────────────────────────────
def _fmt_manual(tlm: Telemetry, target_z: float, cmd: Command) -> str:
    return (f"pos=({tlm.x:+.2f},{tlm.y:+.2f},{tlm.z:+.2f}) "
            f"alt_tgt={target_z:.2f} vz={tlm.vz:+.2f} yaw={tlm.yaw:+.2f} "
            f"thr={cmd.throttle:.2f} "
            f"vanes=({cmd.vane1:+.2f},{cmd.vane2:+.2f},"
            f"{cmd.vane3:+.2f},{cmd.vane4:+.2f})")


def _fmt_goto(tlm: Telemetry, target, cmd: Command) -> str:
    import math
    tx, ty, tz = target
    dist = math.dist(tlm.pos, target)
    return (f"pos=({tlm.x:+.2f},{tlm.y:+.2f},{tlm.z:+.2f}) "
            f"B=({tx:+.1f},{ty:+.1f},{tz:+.1f}) dist={dist:5.2f} "
            f"spd={tlm.speed:.2f} yaw={tlm.yaw:+.2f} thr={cmd.throttle:.2f} "
            f"vanes=({cmd.vane1:+.2f},{cmd.vane2:+.2f},"
            f"{cmd.vane3:+.2f},{cmd.vane4:+.2f})")


def _vanes_cmd(throttle, vanes) -> Command:
    return Command(throttle=throttle, vane1=vanes[0], vane2=vanes[1],
                   vane3=vanes[2], vane4=vanes[3])


# ── MANUAL: altitude hold + raw vanes ───────────────────────────────────────────
def run_manual_sim(cfg: Config, verbose, vanes, seconds) -> bool:
    from tools.sim_stub import SimStub
    controller = AltitudeController(cfg.drone, cfg.control)
    sim = SimStub()
    dt = 1.0 / cfg.control.loop_rate_hz
    target_z = cfg.control.target_altitude
    tlm = sim.telemetry()
    for i in range(int(seconds / dt)):
        thr = controller.throttle(tlm, target_z, dt)
        cmd = _vanes_cmd(thr, vanes)
        tlm = sim.step(cmd, dt)
        if verbose and i % int(cfg.control.loop_rate_hz / 5) == 0:
            print(f"t={tlm.t:5.1f} " + _fmt_manual(tlm, target_z, cmd))
    print("[sim] final: " + _fmt_manual(tlm, target_z, cmd))
    return abs(tlm.z - target_z) < 0.5


def run_manual_mqtt(cfg: Config, verbose) -> None:
    from .mqtt_io import MqttLink
    controller = AltitudeController(cfg.drone, cfg.control)
    link = MqttLink(cfg.mqtt)
    if not _connect(link, cfg):
        return
    print(f"[mqtt] connected. Holding altitude {cfg.control.target_altitude:.2f} m; "
          f"vane angles come from '{cfg.mqtt.topic_vane_input}'.")
    dt = 1.0 / cfg.control.loop_rate_hz
    target_z = cfg.control.target_altitude
    last_log = 0.0
    try:
        while True:
            loop_start = time.perf_counter()
            tlm = link.latest_telemetry()
            if tlm is None:
                time.sleep(dt); continue
            thr = controller.throttle(tlm, target_z, dt)
            v = link.latest_vanes()
            cmd = _vanes_cmd(thr, v)
            link.publish_command(cmd)
            if verbose and time.perf_counter() - last_log > 0.5:
                print(_fmt_manual(tlm, target_z, cmd)); last_log = time.perf_counter()
            _sleep_rest(loop_start, dt)
    except KeyboardInterrupt:
        print("\n[mqtt] interrupted — idle."); link.publish_command(Command())
    finally:
        link.close()


# ── AUTONOMOUS: fly a waypoint mission (A → B → …) ───────────────────────────────
def run_mission_sim(cfg: Config, mission: Mission, verbose, seconds) -> bool:
    from tools.sim_stub import SimStub
    controller = GotoController(cfg.drone, cfg.goto)
    sim = SimStub()
    dt = 1.0 / cfg.control.loop_rate_hz
    tlm = sim.telemetry()
    for i in range(int(seconds / dt)):
        target = mission.update(tlm, dt)
        if target is None:
            print(f"[sim] mission complete at t={tlm.t:.1f}s, "
                  f"pos=({tlm.x:+.2f},{tlm.y:+.2f},{tlm.z:+.2f})")
            return True
        thr, vanes = controller.update(tlm, target, dt)
        cmd = _vanes_cmd(thr, vanes)
        tlm = sim.step(cmd, dt)
        if verbose and i % int(cfg.control.loop_rate_hz / 5) == 0:
            print(f"t={tlm.t:5.1f} " + _fmt_goto(tlm, target, cmd))
    print("[sim] timed out: " + _fmt_goto(tlm, mission.target or (0, 0, 0), cmd))
    return False


def run_mission_mqtt(cfg: Config, mission: Mission, verbose) -> None:
    from .mqtt_io import MqttLink
    controller = GotoController(cfg.drone, cfg.goto)
    link = MqttLink(cfg.mqtt)
    if not _connect(link, cfg):
        return
    print(f"[mqtt] connected. Flying {len(mission.waypoints)} waypoint(s) "
          f"autonomously. Waiting for telemetry ...")
    dt = 1.0 / cfg.control.loop_rate_hz
    last_log = 0.0
    try:
        while True:
            loop_start = time.perf_counter()
            tlm = link.latest_telemetry()
            if tlm is None:
                time.sleep(dt); continue
            target = mission.update(tlm, dt)
            if target is None:
                link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                link.publish_status("mission_complete")
                print("[mqtt] mission complete — holding hover."); break
            thr, vanes = controller.update(tlm, target, dt)
            cmd = _vanes_cmd(thr, vanes)
            link.publish_command(cmd)
            if verbose and time.perf_counter() - last_log > 0.5:
                print(_fmt_goto(tlm, target, cmd)); last_log = time.perf_counter()
            _sleep_rest(loop_start, dt)
    except KeyboardInterrupt:
        print("\n[mqtt] interrupted — idle."); link.publish_command(Command())
    finally:
        link.close()


# ── helpers ─────────────────────────────────────────────────────────────────────
def _connect(link, cfg) -> bool:
    print(f"[mqtt] connecting to {cfg.mqtt.host}:{cfg.mqtt.port} ...")
    if link.connect():
        return True
    print(f"[mqtt] could not reach the broker at {cfg.mqtt.host}:{cfg.mqtt.port}"
          + (f" ({link.last_error})" if link.last_error else "") + ".")
    print(f"       quick test:  mosquitto_sub -h {cfg.mqtt.host} -t 'drone/#' -v")
    return False


def _sleep_rest(loop_start, dt):
    elapsed = time.perf_counter() - loop_start
    if elapsed < dt:
        time.sleep(dt - elapsed)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Singlecopter controller")
    ap.add_argument("-c", "--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--sim", action="store_true",
                    help="run against the in-process physics stub (no broker)")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--seconds", type=float, default=30.0, help="sim-mode duration")
    # autonomous modes
    ap.add_argument("--goto", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="autonomously fly to a single world point B")
    ap.add_argument("--mission", action="store_true",
                    help="autonomously fly the waypoint sequence in config")
    # manual-mode fixed vanes (sim only)
    for i in range(1, 5):
        ap.add_argument(f"--v{i}", type=float, default=0.0,
                        help=f"sim manual-mode vane{i} angle (rad)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)

    # Build a mission if an autonomous mode was requested.
    mission = None
    if args.goto is not None:
        from .config import MissionConfig
        mc = MissionConfig(waypoints=[list(args.goto)])
        mc.arrival_radius = cfg.mission.arrival_radius
        mc.arrival_speed = cfg.mission.arrival_speed
        mc.hold_time = cfg.mission.hold_time
        mission = Mission(mc)
    elif args.mission:
        mission = Mission(cfg.mission)

    if mission is not None:
        if args.sim:
            ok = run_mission_sim(cfg, mission, args.verbose, args.seconds)
            return 0 if ok else 1
        run_mission_mqtt(cfg, mission, args.verbose)
        return 0

    # Manual altitude-hold + raw vanes.
    if args.sim:
        ok = run_manual_sim(cfg, args.verbose,
                            (args.v1, args.v2, args.v3, args.v4), args.seconds)
        return 0 if ok else 1
    run_manual_mqtt(cfg, args.verbose)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
