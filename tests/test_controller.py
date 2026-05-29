"""Tests for altitude hold and the 4-independent-vane physics.

Run against the headless stub — no Blender, no MQTT.
"""

import math

from drone_nav.config import Config, MissionConfig
from drone_nav.controller import AltitudeController, GotoController
from drone_nav.mission import Mission
from drone_nav.telemetry import Command
from tools.sim_stub import SimStub


def _run(vanes=(0.0, 0.0, 0.0, 0.0), seconds=12.0, rate=50.0):
    cfg = Config()
    controller = AltitudeController(cfg.drone, cfg.control)
    sim = SimStub()
    dt = 1.0 / rate
    tlm = sim.telemetry()
    for _ in range(int(seconds / dt)):
        thr = controller.throttle(tlm, cfg.control.target_altitude, dt)
        cmd = Command(throttle=thr, vane1=vanes[0], vane2=vanes[1],
                      vane3=vanes[2], vane4=vanes[3])
        tlm = sim.step(cmd, dt)
    return tlm


def test_altitude_hold_reaches_target():
    tlm = _run()
    assert abs(tlm.z - 2.0) < 0.3, f"altitude {tlm.z} did not settle near 2 m"


def test_altitude_hold_with_vane_deflection():
    # Even while steering, the altitude PID should keep it near target.
    tlm = _run(vanes=(0.2, 0.0, 0.2, 0.0))
    assert abs(tlm.z - 2.0) < 0.5


def test_pitch_pair_moves_forward_back():
    # vanes 1 & 3 positive → body -X force → drone moves to -X.
    tlm = _run(vanes=(0.25, 0.0, 0.25, 0.0))
    assert tlm.x < -0.2, f"expected -X motion, got x={tlm.x}"
    # opposite sign → +X
    tlm2 = _run(vanes=(-0.25, 0.0, -0.25, 0.0))
    assert tlm2.x > 0.2, f"expected +X motion, got x={tlm2.x}"


def test_roll_pair_moves_laterally():
    # vanes 2 & 4 positive → body +Y force.
    tlm = _run(vanes=(0.0, 0.25, 0.0, 0.25))
    assert tlm.y > 0.2, f"expected +Y motion, got y={tlm.y}"


def test_vanes_are_independent():
    # Deflecting only vane1 produces about half the lateral force of deflecting
    # the full 1+3 pair, proving the vanes act independently. Use a short window
    # (a single vane also induces yaw, which curves the path over time).
    only1 = _run(vanes=(0.25, 0.0, 0.0, 0.0), seconds=1.5)
    both = _run(vanes=(0.25, 0.0, 0.25, 0.0), seconds=1.5)
    d_only1 = math.hypot(only1.x, only1.y)
    d_both = math.hypot(both.x, both.y)
    assert d_only1 > 0.01                      # vane1 alone still moves the drone
    assert d_only1 < d_both                     # but less than the full pair


def _fly_mission(waypoints, seconds=40.0, rate=50.0):
    cfg = Config()
    controller = GotoController(cfg.drone, cfg.goto)
    mission = Mission(MissionConfig(waypoints=waypoints, arrival_radius=0.5,
                                    arrival_speed=0.4, hold_time=0.5))
    sim = SimStub()
    dt = 1.0 / rate
    tlm = sim.telemetry()
    for _ in range(int(seconds / dt)):
        target = mission.update(tlm, dt)
        if target is None:
            return True, sim
        thr, vanes = controller.update(tlm, target, dt)
        tlm = sim.step(Command(throttle=thr, vane1=vanes[0], vane2=vanes[1],
                               vane3=vanes[2], vane4=vanes[3]), dt)
    return mission.complete, sim


def test_autonomous_point_a_to_b_far_and_higher():
    # A on the ground, B far away horizontally AND at a different altitude.
    done, sim = _fly_mission([[0.0, 0.0, 3.0], [14.0, 10.0, 6.0]], seconds=60.0)
    assert done, "drone failed to reach B"
    assert math.dist((sim.x, sim.y, sim.z), (14.0, 10.0, 6.0)) < 1.0


def test_autonomous_descends_to_lower_target():
    # B is lower than A — exercises descent, not just climb.
    done, sim = _fly_mission([[0.0, 0.0, 6.0], [8.0, -6.0, 2.0]], seconds=60.0)
    assert done
    assert math.dist((sim.x, sim.y, sim.z), (8.0, -6.0, 2.0)) < 1.0


def test_yaw_swirl_rotates_the_drone():
    # A swirl pattern (a1>0, a3<0, a2>0, a4<0) makes a net yaw torque — this is
    # the new independent-vane yaw authority. Use a strong, brief deflection.
    ay = 0.35
    cw = _run(vanes=(ay, ay, -ay, -ay), seconds=3.0)
    ccw = _run(vanes=(-ay, -ay, ay, ay), seconds=3.0)
    assert cw.yaw > 0.1, f"swirl did not yaw the drone (yaw={cw.yaw})"
    assert ccw.yaw < -0.1
    # ...and a pure swirl shouldn't translate much (forces cancel).
    assert abs(cw.x) < abs(_run(vanes=(ay, 0, ay, 0), seconds=3.0).x)
