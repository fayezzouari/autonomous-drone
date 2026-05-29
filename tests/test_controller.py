"""Closed-loop tests: the controller must actually fly the stub to its targets.

These exercise the full stack (mission + cascaded PID + inverse force model)
against the headless physics port — no Blender, no MQTT.
"""

import math

from drone_nav.config import Config, MissionConfig
from drone_nav.controller import NavigationController
from drone_nav.mission import Mission
from tools.sim_stub import SimStub


def _fly(waypoints, max_seconds=40.0, rate=50.0):
    cfg = Config()
    cfg.mission = MissionConfig(
        waypoints=waypoints,
        arrival_radius=0.30,
        arrival_speed=0.30,
        hold_time=0.5,
        loop_rate_hz=rate,
    )
    controller = NavigationController(cfg.drone, cfg.control)
    mission = Mission(cfg.mission)
    sim = SimStub()
    dt = 1.0 / rate
    tlm = sim.telemetry()

    for _ in range(int(max_seconds / dt)):
        target = mission.update(tlm, dt)
        if target is None:
            return True, sim
        cmd = controller.update(tlm, target, dt)
        tlm = sim.step(cmd, dt)
    return mission.complete, sim


def test_takeoff_and_hover():
    done, sim = _fly([[0.0, 0.0, 2.0]])
    assert done, "drone failed to reach the hover waypoint"
    assert abs(sim.z - 2.0) < 0.4


def test_point_a_to_b():
    done, sim = _fly([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]])
    assert done, "drone failed to fly A -> B"
    assert math.dist((sim.x, sim.y, sim.z), (5.0, 0.0, 2.0)) < 0.5


def test_multi_waypoint_path():
    done, sim = _fly(
        [[0.0, 0.0, 2.0], [4.0, 0.0, 2.0], [4.0, 4.0, 3.0]],
        max_seconds=60.0,
    )
    assert done, "drone failed to complete the multi-waypoint path"
    assert math.dist((sim.x, sim.y, sim.z), (4.0, 4.0, 3.0)) < 0.5


def test_does_not_crash_into_ground():
    # Throughout an A->B flight the drone should stay airborne after takeoff.
    cfg = Config()
    cfg.mission = MissionConfig(
        waypoints=[[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]],
        hold_time=0.5, loop_rate_hz=50.0,
    )
    controller = NavigationController(cfg.drone, cfg.control)
    mission = Mission(cfg.mission)
    sim = SimStub()
    dt = 0.02
    tlm = sim.telemetry()
    min_alt_after_takeoff = math.inf
    airborne = False
    for _ in range(int(40 / dt)):
        target = mission.update(tlm, dt)
        if target is None:
            break
        cmd = controller.update(tlm, target, dt)
        tlm = sim.step(cmd, dt)
        if tlm.z > 1.5:
            airborne = True
        if airborne:
            min_alt_after_takeoff = min(min_alt_after_takeoff, tlm.z)
    assert airborne
    assert min_alt_after_takeoff > 1.0, "drone dipped dangerously low mid-flight"
