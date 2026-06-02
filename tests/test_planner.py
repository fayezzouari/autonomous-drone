"""Tests for obstacle geometry, the 3-D path planner, and an end-to-end flight.

Everything runs headless (no Blender, no MQTT): geometry/planner are pure, and
the integration test flies the planned route through the physics stub.
"""

import math

from drone_nav.config import Config, MissionConfig
from drone_nav.controller import GotoController
from drone_nav.mission import Mission
from drone_nav.obstacles import Box, ObstacleField
from drone_nav.planner import PathFollower, PathPlanner, PlannerConfig, densify
from drone_nav.telemetry import Command
from tools.sim_stub import SimStub


# ── obstacle geometry ─────────────────────────────────────────────────────────
def test_box_from_msg_half_extents():
    # Current convention: cx/cy/cz centre + hw/hd/hh half-extents. Default axes
    # "wdh" matches the Blender publisher: hw→X, hd→Y, hh→Z; default flip "".
    b = Box.from_msg({"cx": 1.0, "cy": 2.0, "cz": 3.0, "hw": 0.5, "hd": 1.5, "hh": 2.0})
    assert (b.cx, b.cy, b.cz) == (1.0, 2.0, 3.0)
    assert math.isclose(b.hx, 0.5) and math.isclose(b.hy, 1.5) and math.isclose(b.hz, 2.0)
    assert b.lo == (0.5, 0.5, 1.0)
    assert b.hi == (1.5, 3.5, 5.0)


def test_box_from_msg_legacy_full_extents():
    # Fallback shape {c, w, h, t}: full extents halved; default "wdh" → w→X.
    b = Box.from_msg({"c": [1.0, 2.0, 3.0], "w": 4.0, "h": 6.0, "t": 2.0})
    assert (b.cx, b.cy, b.cz) == (1.0, 2.0, 3.0)
    assert math.isclose(b.hx, 2.0) and math.isclose(b.hy, 1.0) and math.isclose(b.hz, 3.0)


def test_box_axes_override_swap():
    # axes "dwh" swaps to X=depth(hd), Y=width(hw), Z=height(hh).
    b = Box.from_msg({"cx": 0, "cy": 0, "cz": 0, "hw": 0.5, "hd": 1.5, "hh": 2.0}, axes="dwh")
    assert math.isclose(b.hx, 1.5) and math.isclose(b.hy, 0.5) and math.isclose(b.hz, 2.0)


def test_flip_is_opt_in():
    # Default keeps world-frame coords; flip negates the named centre axes.
    assert ObstacleField.from_list([{"cx": 7, "cy": 0, "cz": 2, "hw": 1, "hd": 1, "hh": 4}]).boxes[0].cx == 7.0
    flipped = ObstacleField.from_list(
        [{"cx": 7, "cy": 0, "cz": 2, "hw": 1, "hd": 1, "hh": 4}], flip="x")
    assert flipped.boxes[0].cx == -7.0


def test_box_axes_override():
    # explicit "wth" maps width→X, thickness→Y, height→Z.
    b = Box.from_msg({"c": [0, 0, 0], "w": 4.0, "t": 2.0, "h": 6.0}, axes="wth")
    assert math.isclose(b.hx, 2.0) and math.isclose(b.hy, 1.0) and math.isclose(b.hz, 3.0)


def test_box_yaw_widens_to_enclosing_aabb():
    # default "wdh": width→X, so a 4-wide / 0-deep wall is hx=2, hy=0; rotating
    # 90° about Z swaps them → hx=0, hy=2.
    b = Box.from_msg({"c": [0, 0, 0], "w": 4.0, "t": 0.0, "h": 2.0, "yaw": math.pi / 2})
    assert math.isclose(b.hx, 0.0, abs_tol=1e-9)
    assert math.isclose(b.hy, 2.0, abs_tol=1e-9)


def test_inflate_grows_every_axis():
    b = Box(0, 0, 0, 1, 1, 1).inflated(0.5)
    assert (b.hx, b.hy, b.hz) == (1.5, 1.5, 1.5)


def test_segment_intersection():
    field = ObstacleField([Box(0, 0, 0, 1, 1, 1)])
    assert field.segment_blocked((-3, 0, 0), (3, 0, 0))      # straight through
    assert not field.segment_blocked((-3, 5, 0), (3, 5, 0))  # misses in Y
    assert not field.segment_blocked((-3, 0, 0), (-2, 0, 0)) # stops short


def test_point_blocked_and_parse_list():
    field = ObstacleField.from_list([{"c": [0, 0, 0], "w": 2, "h": 2, "t": 2}])
    assert len(field) == 1
    assert field.point_blocked((0, 0, 0))
    assert not field.point_blocked((5, 5, 5))


def test_from_payload_accepts_json_and_wrapper():
    a = ObstacleField.from_payload('[{"c":[0,0,0],"w":1,"h":1,"t":1}]')
    b = ObstacleField.from_payload('{"obstacles":[{"c":[0,0,0],"w":1,"h":1,"t":1}]}')
    assert len(a) == len(b) == 1


# ── planner ─────────────────────────────────────────────────────────────────────
def _clearance_ok(field: ObstacleField, planner: PathPlanner) -> ObstacleField:
    return field.inflate(planner.cfg.clearance)


def test_direct_when_clear():
    planner = PathPlanner(PlannerConfig())
    res = planner.plan((0, 0, 2), (10, 0, 2), ObstacleField([]))
    assert res.found and res.direct
    assert res.waypoints == [(0, 0, 2), (10, 0, 2)]


def test_routes_around_a_wall():
    # A wall straddling the straight A→B line forces a detour.
    field = ObstacleField([Box(5, 0, 2, 0.5, 4.0, 4.0)])  # spans y∈[-4,4], x≈5
    planner = PathPlanner(PlannerConfig(resolution=0.5))
    res = planner.plan((0, 0, 2), (10, 0, 2), field)
    assert res.found and not res.direct
    # every leg of the returned path must clear the inflated obstacle
    infl = _clearance_ok(field, planner)
    for p, q in zip(res.waypoints, res.waypoints[1:]):
        assert not infl.segment_blocked(p, q), f"leg {p}->{q} clips an obstacle"
    # and it should be longer than the (blocked) straight shot
    assert res.length > math.dist((0, 0, 2), (10, 0, 2))


def test_respects_drone_size_narrow_gap():
    # Two walls leave only a 0.6 m gap. A point robot could thread it, but a
    # drone with clearance 0.5 (radius .3 + margin .2 ⇒ 1.0 m needed) cannot, so
    # the planner must go around rather than through the gap at y≈0.
    walls = ObstacleField([
        Box(5, 1.3, 2, 0.4, 1.0, 4.0),   # upper wall, inner face at y=0.3
        Box(5, -1.3, 2, 0.4, 1.0, 4.0),  # lower wall, inner face at y=-0.3
    ])
    planner = PathPlanner(PlannerConfig(resolution=0.25))
    res = planner.plan((0, 0, 2), (10, 0, 2), walls)
    assert res.found
    infl = walls.inflate(planner.cfg.clearance)
    for p, q in zip(res.waypoints, res.waypoints[1:]):
        assert not infl.segment_blocked(p, q)
    # the path must not squeeze through the central gap: at x≈5 it should be
    # well clear of y=0 (|y| larger than the wall inner edge).
    mid_ys = [p[1] for p in res.waypoints if 4.0 <= p[0] <= 6.0]
    assert mid_ys, "expected at least one waypoint near the walls"
    assert max(abs(y) for y in mid_ys) > 2.0


def test_wider_gap_is_used():
    # Same walls but a 3 m gap (> required clearance) — the planner may thread it
    # and stay near the centreline.
    walls = ObstacleField([
        Box(5, 2.5, 2, 0.4, 1.0, 4.0),
        Box(5, -2.5, 2, 0.4, 1.0, 4.0),
    ])
    planner = PathPlanner(PlannerConfig(resolution=0.25))
    res = planner.plan((0, 0, 2), (10, 0, 2), walls)
    assert res.found
    infl = walls.inflate(planner.cfg.clearance)
    for p, q in zip(res.waypoints, res.waypoints[1:]):
        assert not infl.segment_blocked(p, q)


def test_theta_is_default_and_routes_around_wall():
    field = ObstacleField([Box(5, 0, 2, 0.5, 4.0, 4.0)])
    planner = PathPlanner(PlannerConfig(resolution=0.5))
    assert planner.cfg.algorithm == "theta"
    res = planner.plan((0, 0, 2), (10, 0, 2), field)
    assert res.found and not res.direct and res.reason == "theta"
    infl = field.inflate(planner.cfg.clearance)
    for p, q in zip(res.waypoints, res.waypoints[1:]):
        assert not infl.segment_blocked(p, q), f"theta leg {p}->{q} clips an obstacle"


def test_theta_no_longer_than_astar():
    # Any-angle Theta* should never produce a longer route than grid A*+shortcut.
    field = ObstacleField([Box(5, 0, 2, 0.5, 4.0, 4.0)])
    A, B = (0, 0, 2), (10, 0, 2)
    theta = PathPlanner(PlannerConfig(resolution=0.5, algorithm="theta")).plan(A, B, field)
    astar = PathPlanner(PlannerConfig(resolution=0.5, algorithm="astar")).plan(A, B, field)
    assert theta.found and astar.found
    assert theta.length <= astar.length + 1e-6


def test_theta_end_to_end_flies_clean():
    field = ObstacleField([Box(7, 5, 4, 1.0, 3.0, 4.0)])
    res = PathPlanner(PlannerConfig(resolution=0.5, algorithm="theta")).plan((0, 0, 3), (14, 10, 6), field)
    assert res.found
    done, sim, min_clear = _fly_route(res.waypoints, field, seconds=80.0)
    assert done and math.dist((sim.x, sim.y, sim.z), (14, 10, 6)) < 1.2
    assert min_clear > 0.0


def test_densify_caps_spacing():
    pts = densify([(0, 0, 0), (3, 0, 0)], spacing=0.8)
    assert pts[0] == (0, 0, 0) and pts[-1] == (3, 0, 0)
    gaps = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    assert max(gaps) <= 0.8 + 1e-9


class _FakeTlm:
    def __init__(self, pos, speed=0.0):
        self.pos = pos
        self.speed = speed


def test_path_follower_advances_by_proximity_without_stopping():
    # Intermediate vertices advance on proximity alone (no slow/dwell needed) —
    # this is what stops the orbiting that flung the drone into walls.
    f = PathFollower([(0, 0, 0), (1, 0, 0), (2, 0, 0)],
                     switch_radius=0.5, arrival_radius=0.3, arrival_speed=0.4, hold_time=0.5)
    assert f.target == (0, 0, 0)
    f.update(_FakeTlm((0.7, 0, 0), speed=3.0), 0.02)   # d to vtx0 = 0.7 > 0.5 → hold
    assert f.index == 0
    f.update(_FakeTlm((0.4, 0, 0), speed=3.0), 0.02)   # within 0.5 of vtx0 → advance (fast OK)
    assert f.index == 1 and f.target == (1, 0, 0)
    f.update(_FakeTlm((1.3, 0, 0), speed=3.0), 0.02)   # within 0.5 of vtx1 → advance to last
    assert f.index == 2 and not f.complete
    # final vertex needs the strict arrival (slow + dwell)
    f.update(_FakeTlm((2.0, 0, 0), speed=3.0), 0.02)   # at vtx2 but too fast → no complete
    assert not f.complete
    f.update(_FakeTlm((2.0, 0, 0), speed=0.1), 1.0)    # slow + dwell satisfied
    assert f.complete


def test_goal_inside_obstacle_relocates_or_fails_gracefully():
    field = ObstacleField([Box(10, 0, 2, 3, 3, 3)])  # goal buried inside
    planner = PathPlanner(PlannerConfig())
    res = planner.plan((0, 0, 2), (10, 0, 2), field)
    # Either it relocates the goal to a free, reachable cell, or reports failure —
    # never returns a path that ends inside the obstacle.
    if res.found:
        infl = field.inflate(planner.cfg.clearance)
        assert not infl.point_blocked(res.waypoints[-1])


# ── end-to-end: plan around an obstacle, then fly it ─────────────────────────────
def _fly_route(waypoints, obstacles, seconds=60.0, rate=50.0):
    cfg = Config()
    controller = GotoController(cfg.drone, cfg.goto)
    mission = Mission(MissionConfig(waypoints=[list(w) for w in waypoints],
                                    arrival_radius=0.6, arrival_speed=0.5,
                                    hold_time=0.4))
    sim = SimStub()
    dt = 1.0 / rate
    tlm = sim.telemetry()
    min_clear = math.inf
    for _ in range(int(seconds / dt)):
        target = mission.update(tlm, dt)
        if target is None:
            return True, sim, min_clear
        thr, vanes = controller.update(tlm, target, dt)
        tlm = sim.step(Command(throttle=thr, vane1=vanes[0], vane2=vanes[1],
                               vane3=vanes[2], vane4=vanes[3]), dt)
        # track closest approach to any *real* (un-inflated) obstacle
        for b in obstacles.boxes:
            dx = max(abs(tlm.x - b.cx) - b.hx, 0.0)
            dy = max(abs(tlm.y - b.cy) - b.hy, 0.0)
            dz = max(abs(tlm.z - b.cz) - b.hz, 0.0)
            min_clear = min(min_clear, math.hypot(dx, dy, dz))
    return mission.complete, sim, min_clear


def test_end_to_end_plan_and_fly_around_obstacle():
    field = ObstacleField([Box(7, 5, 4, 1.0, 3.0, 4.0)])
    planner = PathPlanner(PlannerConfig(resolution=0.5))
    res = planner.plan((0, 0, 3), (14, 10, 6), field)
    assert res.found

    done, sim, min_clear = _fly_route(res.waypoints, field, seconds=80.0)
    assert done, "drone failed to fly the planned route to B"
    assert math.dist((sim.x, sim.y, sim.z), (14, 10, 6)) < 1.2
    # It actually avoided the box: never penetrated it (clearance > 0).
    assert min_clear > 0.0, f"flew into the obstacle (min clearance {min_clear:.2f} m)"
