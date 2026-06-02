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
from .obstacles import ObstacleField
from .planner import PathFollower, PathPlanner, PathResult, densify
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


# ── path-planning helpers ─────────────────────────────────────────────────────
def _mission_for_route(cfg: Config, sub_waypoints) -> Mission:
    """Build a Mission that flies an ordered list of (x,y,z) sub-waypoints."""
    from .config import MissionConfig
    mc = MissionConfig(
        waypoints=[list(w) for w in sub_waypoints],
        arrival_radius=cfg.mission.arrival_radius,
        arrival_speed=cfg.mission.arrival_speed,
        hold_time=cfg.mission.hold_time,
    )
    return Mission(mc)


def _follower_for_route(cfg: Config, start, sub_waypoints) -> PathFollower:
    """Flow-follow a planned route: densify it (INCLUDING the current position so
    the first leg is stepped, not one big jump) and advance by proximity so the
    drone hugs the line instead of orbiting/overshooting far waypoints."""
    full = [tuple(start)] + [tuple(w) for w in sub_waypoints]
    dense = densify(full, cfg.planner.follow_spacing)
    return PathFollower(
        dense,
        switch_radius=cfg.planner.switch_radius,
        arrival_radius=cfg.mission.arrival_radius,
        arrival_speed=cfg.mission.arrival_speed,
        hold_time=cfg.mission.hold_time,
    )


def _avoid_controller(cfg: Config) -> GotoController:
    """GotoController for obstacle flights, with the horizontal speed capped to
    planner.cruise_speed so the drone tracks the planned line tightly on turns."""
    import dataclasses
    goto = cfg.goto
    changes = {}
    if cfg.planner.cruise_speed and cfg.planner.cruise_speed > 0:
        changes["v_max_xy"] = cfg.planner.cruise_speed
    if cfg.planner.climb_speed and cfg.planner.climb_speed > 0:
        changes["vz_max"] = cfg.planner.climb_speed
    if changes:
        goto = dataclasses.replace(goto, **changes)
    # couple_climb: ascend along the planned line rather than shooting up first
    # from a standstill (vertical responds much faster than horizontal at rest).
    return GotoController(cfg.drone, goto, couple_climb=True,
                          climb_floor=cfg.planner.climb_floor)


def _describe_plan(res: PathResult, goal) -> str:
    gx, gy, gz = goal
    if not res.found:
        return f"[plan] B=({gx:+.1f},{gy:+.1f},{gz:+.1f}) FAILED ({res.reason}) — holding position"
    kind = "direct" if res.direct else f"{len(res.waypoints) - 1} legs, {res.expanded} nodes"
    legs = " → ".join(f"({p[0]:+.1f},{p[1]:+.1f},{p[2]:+.1f})" for p in res.waypoints)
    return (f"[plan] B=({gx:+.1f},{gy:+.1f},{gz:+.1f}) {kind}, "
            f"len={res.length:.1f} m\n       {legs}")


def _plan_route(planner: PathPlanner, cfg: Config, start, goal, field: ObstacleField):
    """Plan start→goal; return (sub-waypoints, result), or (None, result) on failure.

    We deliberately do NOT fall back to a straight line on failure — that would
    fly the drone blind through the obstacles. Callers hold position instead.
    """
    res = planner.plan(start, goal, field)
    print(_describe_plan(res, goal))
    if not res.found:
        return None, res
    wps = res.waypoints[1:] if len(res.waypoints) > 1 else [tuple(goal)]
    return wps, res


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


# ── AUTONOMOUS + OBSTACLE AVOIDANCE (--avoid) ────────────────────────────────────
def run_planned_sim(cfg: Config, targets, verbose, seconds) -> bool:
    """Offline: plan around the static config obstacles, then fly the route."""
    from tools.sim_stub import SimStub
    controller = _avoid_controller(cfg)
    planner = PathPlanner(cfg.planner)
    field = ObstacleField.from_list(
        cfg.obstacles, cfg.planner.obstacle_axes, cfg.planner.obstacle_flip)
    sim = SimStub()
    dt = 1.0 / cfg.control.loop_rate_hz
    tlm = sim.telemetry()

    # Plan every A→B→… leg around the obstacles, concatenating sub-waypoints.
    print(f"[plan] {len(field)} obstacle(s); clearance "
          f"{cfg.planner.clearance:.2f} m (drone {cfg.planner.drone_radius:.2f} + "
          f"margin {cfg.planner.safety_margin:.2f})")
    origin = tlm.pos
    route = []
    start = origin
    for goal in targets:
        wps, _ = _plan_route(planner, cfg, start, goal, field)
        if wps is None:
            print(f"[sim] no safe path to ({goal[0]:+.1f},{goal[1]:+.1f},{goal[2]:+.1f}) "
                  f"— aborting (refusing to fly blind through obstacles).")
            return False
        route.extend(wps)
        start = goal
    mission = _follower_for_route(cfg, origin, route)

    for i in range(int(seconds / dt)):
        target = mission.update(tlm, dt)
        if target is None:
            print(f"[sim] route complete at t={tlm.t:.1f}s, "
                  f"pos=({tlm.x:+.2f},{tlm.y:+.2f},{tlm.z:+.2f})")
            return True
        thr, vanes = controller.update(tlm, target, dt)
        tlm = sim.step(_vanes_cmd(thr, vanes), dt)
        if verbose and i % int(cfg.control.loop_rate_hz / 5) == 0:
            print(f"t={tlm.t:5.1f} " + _fmt_goto(tlm, target, _vanes_cmd(thr, vanes)))
    print("[sim] route timed out.")
    return False


def run_planned_mqtt(cfg: Config, targets, verbose) -> None:
    """Live: subscribe to obstacles, plan A→B around them, replan on changes."""
    from .mqtt_io import MqttLink
    controller = _avoid_controller(cfg)
    planner = PathPlanner(cfg.planner)
    link = MqttLink(cfg.mqtt, obstacle_axes=cfg.planner.obstacle_axes,
                    obstacle_flip=cfg.planner.obstacle_flip)
    if not _connect(link, cfg):
        return
    print(f"[mqtt] connected. Obstacle-avoiding flight of {len(targets)} target(s); "
          f"clearance {cfg.planner.clearance:.2f} m. Listening on "
          f"'{cfg.mqtt.topic_obstacles}', publishing route to '{cfg.mqtt.topic_path}'.")
    dt = 1.0 / cfg.control.loop_rate_hz

    # Use the obstacles we've retrieved before the first plan: wait briefly for a
    # drone/obs message (and for telemetry) so the initial route avoids them
    # instead of planning blind and replanning a moment later.
    _await_first_obstacles(link, cfg)

    goal_idx = 0
    mission = None
    plan_key = None  # (goal_idx, obstacle_version) the current plan was made for
    previewed = False
    last_log = 0.0
    try:
        while True:
            loop_start = time.perf_counter()
            tlm = link.latest_telemetry()
            if tlm is None:
                time.sleep(dt); continue
            field, version = link.latest_obstacles()

            # (Re)plan when we have no plan for this goal, or the obstacle set
            # changed under us. plan_key dedupes so a *failed* plan doesn't respin
            # every tick — we just hold until the obstacles change.
            if goal_idx < len(targets) and (goal_idx, version) != plan_key:
                wps, _ = _plan_route(planner, cfg, tlm.pos, targets[goal_idx], field)
                plan_key = (goal_idx, version)
                if wps is None:
                    mission = None
                    link.publish_status(f"plan_failed:{goal_idx + 1}/{len(targets)}")
                else:
                    mission = _follower_for_route(cfg, tlm.pos, wps)
                    link.publish_path([tlm.pos] + wps)
                    link.publish_status(f"planning:{goal_idx + 1}/{len(targets)}")
                    # Pre-flight preview: hold and keep the route on drone/path so
                    # it can be inspected before the drone sets off (first plan only).
                    if not previewed and cfg.planner.preview_pause > 0:
                        previewed = True
                        print(f"[mqtt] route published to '{cfg.mqtt.topic_path}' — "
                              f"holding {cfg.planner.preview_pause:.0f}s for preview …")
                        link.publish_status("preview")
                        t_end = time.perf_counter() + cfg.planner.preview_pause
                        while time.perf_counter() < t_end:
                            link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                            link.publish_path([tlm.pos] + wps)
                            time.sleep(0.1)

            # No safe route (yet) → hold position. Never fly blind through boxes.
            if mission is None:
                link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                _sleep_rest(loop_start, dt)
                continue

            target = mission.update(tlm, dt)
            if target is None:
                goal_idx += 1
                if goal_idx >= len(targets):
                    link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                    link.publish_status("mission_complete")
                    print("[mqtt] all targets reached — holding hover."); break
                mission = None  # advance; next iteration replans for the new goal
                continue

            thr, vanes = controller.update(tlm, target, dt)
            link.publish_command(_vanes_cmd(thr, vanes))
            if verbose and time.perf_counter() - last_log > 0.5:
                print(_fmt_goto(tlm, target, _vanes_cmd(thr, vanes)))
                last_log = time.perf_counter()
            _sleep_rest(loop_start, dt)
    except KeyboardInterrupt:
        print("\n[mqtt] interrupted — idle."); link.publish_command(Command())
    finally:
        link.close()


# ── LIVE GO-TO SERVER: targets stream in on drone/goto ───────────────────────────
def run_goto_topic_mqtt(cfg: Config, verbose) -> None:
    """Subscribe to ``drone/goto`` and fly to each target as it arrives, planning
    around the live ``drone/obs`` obstacles. Holds a hover until the first target,
    replans on a new target or an obstacle change, then holds again on arrival."""
    from .mqtt_io import MqttLink
    controller = _avoid_controller(cfg)
    planner = PathPlanner(cfg.planner)
    link = MqttLink(cfg.mqtt, obstacle_axes=cfg.planner.obstacle_axes,
                    obstacle_flip=cfg.planner.obstacle_flip)
    if not _connect(link, cfg):
        return
    print(f"[mqtt] connected — go-to server. Send a target to "
          f"'{cfg.mqtt.topic_goto}' (e.g. [x,y,z] or {{\"x\":..,\"y\":..,\"z\":..}}); "
          f"avoiding '{cfg.mqtt.topic_obstacles}', route on '{cfg.mqtt.topic_path}'.")
    _await_first_obstacles(link, cfg)

    dt = 1.0 / cfg.control.loop_rate_hz
    mission = None
    plan_key = None      # (goto_version, obstacle_version) the plan was built for
    last_goto_v = 0
    last_log = 0.0
    try:
        while True:
            loop_start = time.perf_counter()
            tlm = link.latest_telemetry()
            if tlm is None:
                time.sleep(dt); continue
            field, oversion = link.latest_obstacles()
            goal, gversion = link.latest_goto()

            if goal is None:                      # no command yet → idle hover
                link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                _sleep_rest(loop_start, dt); continue

            # (Re)plan on a new target or a changed obstacle set.
            key = (gversion, oversion)
            if key != plan_key:
                if gversion != last_goto_v:
                    print(f"[mqtt] new target ({goal[0]:+.1f},{goal[1]:+.1f},{goal[2]:+.1f})")
                    last_goto_v = gversion
                    controller.reset()            # fresh PID state for the new leg
                wps, _ = _plan_route(planner, cfg, tlm.pos, goal, field)
                plan_key = key
                if wps is None:
                    mission = None
                    link.publish_status("plan_failed")
                else:
                    mission = _follower_for_route(cfg, tlm.pos, wps)
                    link.publish_path([tlm.pos] + wps)
                    link.publish_status("goto")

            if mission is None:                   # no safe route → hold, never fly blind
                link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                _sleep_rest(loop_start, dt); continue

            target = mission.update(tlm, dt)
            if target is None:                    # reached the target → hold until next
                link.publish_command(Command(throttle=cfg.drone.hover_throttle))
                link.publish_status("reached")
                mission = None                    # plan_key stays → won't replan same goal
                _sleep_rest(loop_start, dt); continue

            thr, vanes = controller.update(tlm, target, dt)
            link.publish_command(_vanes_cmd(thr, vanes))
            if verbose and time.perf_counter() - last_log > 0.5:
                print(_fmt_goto(tlm, target, _vanes_cmd(thr, vanes)))
                last_log = time.perf_counter()
            _sleep_rest(loop_start, dt)
    except KeyboardInterrupt:
        print("\n[mqtt] interrupted — idle."); link.publish_command(Command())
    finally:
        link.close()


# ── helpers ─────────────────────────────────────────────────────────────────────
def _await_first_obstacles(link, cfg) -> None:
    """Block up to ``planner.obstacle_wait`` seconds for the first obstacle set.

    Lets the initial plan use the retrieved obstacles rather than planning on an
    empty world and only correcting once the first ``drone/obs`` message lands.
    """
    wait = max(0.0, cfg.planner.obstacle_wait)
    if wait == 0.0:
        return
    print(f"[mqtt] waiting up to {wait:.1f}s for obstacles on "
          f"'{cfg.mqtt.topic_obstacles}' before planning ...")
    deadline = time.perf_counter() + wait
    while time.perf_counter() < deadline:
        field, version = link.latest_obstacles()
        if version > 0 and link.latest_telemetry() is not None:
            print(f"[mqtt] got {len(field)} obstacle(s); planning.")
            return
        time.sleep(0.05)
    field, version = link.latest_obstacles()
    if version == 0:
        print("[mqtt] no obstacles received yet — planning on a clear world "
              "(will replan when 'drone/obs' arrives).")


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
    ap.add_argument("--goto-topic", dest="goto_topic", action="store_true",
                    help="run as a live go-to server: subscribe to 'drone/goto' and "
                         "fly to each target as it arrives, avoiding 'drone/obs'.")
    ap.add_argument("--avoid", action="store_true",
                    help="plan a collision-free path around 'drone/obs' obstacles "
                         "(grows each box by the drone's size); replans live as "
                         "obstacles change. In --sim, uses the config 'obstacles'.")
    # manual-mode fixed vanes (sim only)
    for i in range(1, 5):
        ap.add_argument(f"--v{i}", type=float, default=0.0,
                        help=f"sim manual-mode vane{i} angle (rad)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)

    # Live go-to server: targets stream in on drone/goto (always obstacle-aware).
    if args.goto_topic:
        run_goto_topic_mqtt(cfg, args.verbose)
        return 0

    # Resolve the autonomous target list (a single B for --goto, the configured
    # sequence for --mission), shared by both the plain and --avoid paths.
    targets = None
    if args.goto is not None:
        targets = [tuple(args.goto)]
    elif args.mission:
        targets = [tuple(w) for w in cfg.mission.waypoints]

    if targets is not None:
        # Obstacle-avoiding flight: plan A→B around 'drone/obs' (or config) boxes.
        if args.avoid:
            if args.sim:
                ok = run_planned_sim(cfg, targets, args.verbose, args.seconds)
                return 0 if ok else 1
            run_planned_mqtt(cfg, targets, args.verbose)
            return 0

        # Straight autonomous flight (no obstacle awareness).
        mission = (Mission(cfg.mission) if args.mission
                   else _mission_for_route(cfg, targets))
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
