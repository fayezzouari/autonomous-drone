"""Altitude controller: hold a target altitude by computing throttle.

Steering is no longer done here — the four vanes are commanded independently
and raw (supplied externally). This controller only keeps the drone at a target
height, so the operator can fly it around manually with the vanes.

    altitude error ──P──► climb-rate setpoint ──PID──► vertical acceleration
                                                              │
                                              T = m·(g + a_z) │ → throttle = √(T/T_max)

Gravity feed-forward falls out for free: when the desired vertical acceleration
is zero, the required thrust equals the weight → hover throttle (≈ 0.68).
Vane deflections steal a little vertical thrust; the PID's integral term absorbs
that sag rather than the controller modelling it explicitly (it doesn't see the
raw vane angles).
"""

from __future__ import annotations

import math

from .config import ControlConfig, DroneParams, GotoConfig
from .pid import PID
from .telemetry import Telemetry


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _wrap_pi(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AltitudeController:
    def __init__(self, drone: DroneParams, control: ControlConfig):
        self.drone = drone
        self.cfg = control
        self.pid_vz = PID(control.vel_z)

    def reset(self) -> None:
        self.pid_vz.reset()

    def throttle(self, tlm: Telemetry, target_z: float, dt: float) -> float:
        """Throttle fraction [0, 1] to drive the drone toward ``target_z``."""
        vsp_z = _clamp(self.cfg.pos_z_p * (target_z - tlm.z),
                       -self.cfg.vz_max, self.cfg.vz_max)
        az = self.pid_vz.update(vsp_z, tlm.vz, dt)
        t_des = self.drone.mass * (self.drone.gravity + az)
        return math.sqrt(_clamp(t_des / self.drone.thrust_max, 0.0, 1.0))


class GotoController:
    """Autonomous point-to-point: world target (x, y, z) → throttle + 4 vanes.

    Cascaded per world axis:  position ─P─► velocity ─PID─► acceleration.
    The horizontal acceleration is rotated into the body frame by the measured
    yaw and inverted through the sim's vane model (F_lat ≈ T_prop) to get the
    pitch/roll deflections; a yaw PID holds heading. Finally pitch/roll/yaw are
    mixed into the four independent vane angles exactly as the sim expects:

        a1 = pitch + yaw   a3 = pitch − yaw      (N/S pair → fore/aft + swirl)
        a2 = roll  + yaw   a4 = roll  − yaw      (E/W pair → lateral  + swirl)
    """

    def __init__(self, drone: DroneParams, cfg: GotoConfig):
        self.drone = drone
        self.cfg = cfg
        self.pid_vx = PID(cfg.vel_xy)
        self.pid_vy = PID(cfg.vel_xy)
        self.pid_vz = PID(cfg.vel_z)
        self.pid_yaw = PID(cfg.yaw)
        self._min_thrust = max(0.1, 0.25 * drone.mass * drone.gravity)
        self._max_vane = drone.max_vane_rad

    def reset(self) -> None:
        for p in (self.pid_vx, self.pid_vy, self.pid_vz, self.pid_yaw):
            p.reset()

    def update(self, tlm: Telemetry, target, dt: float, target_yaw=None):
        """Return (throttle, [a1, a2, a3, a4]) to drive the drone toward target."""
        tx, ty, tz = target
        if target_yaw is None:
            target_yaw = self.cfg.target_yaw

        # ── Outer position → bounded velocity setpoint ────────────────────────
        vmax = self.cfg.v_max_xy
        vsp_x = _clamp(self.cfg.pos_xy_p * (tx - tlm.x), -vmax, vmax)
        vsp_y = _clamp(self.cfg.pos_xy_p * (ty - tlm.y), -vmax, vmax)
        vsp_z = _clamp(self.cfg.pos_z_p * (tz - tlm.z),
                       -self.cfg.vz_max, self.cfg.vz_max)

        # ── Inner velocity → acceleration ─────────────────────────────────────
        ax = self.pid_vx.update(vsp_x, tlm.vx, dt)
        ay = self.pid_vy.update(vsp_y, tlm.vy, dt)
        az = self.pid_vz.update(vsp_z, tlm.vz, dt)

        # ── Live thrust estimate ──────────────────────────────────────────────
        t_prop = max(self.drone.thrust_from_prop_speed(tlm.prop_speed),
                     self._min_thrust)
        m = self.drone.mass

        # ── Horizontal accel → body frame → pitch/roll vane angles ────────────
        fx_world, fy_world = m * ax, m * ay
        cy, sy = math.cos(tlm.yaw), math.sin(tlm.yaw)
        fx_body = fx_world * cy + fy_world * sy
        fy_body = -fx_world * sy + fy_world * cy
        sin_max = math.sin(self._max_vane)
        pitch = math.asin(_clamp(-fx_body / t_prop, -sin_max, sin_max))
        roll = math.asin(_clamp(fy_body / t_prop, -sin_max, sin_max))

        # ── Yaw hold → swirl deflection ───────────────────────────────────────
        yaw_err = _wrap_pi(target_yaw - tlm.yaw)
        yaw_cmd = self.pid_yaw.update(yaw_err, 0.0, dt)

        # ── Vertical accel → throttle (gravity feed-forward built in) ─────────
        tilt = max(0.2, math.cos(pitch) * math.cos(roll))
        t_des = m * (self.drone.gravity + az) / tilt
        throttle = math.sqrt(_clamp(t_des / self.drone.thrust_max, 0.0, 1.0))

        # ── Mix pitch/roll/yaw into the four independent vanes ────────────────
        mv = self._max_vane
        a1 = _clamp(pitch + yaw_cmd, -mv, mv)
        a3 = _clamp(pitch - yaw_cmd, -mv, mv)
        a2 = _clamp(roll + yaw_cmd, -mv, mv)
        a4 = _clamp(roll - yaw_cmd, -mv, mv)
        return throttle, [a1, a2, a3, a4]
