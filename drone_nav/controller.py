"""Cascaded flight controller: world target (x, y, z) → autopilot Command.

The control structure, per axis:

    position error ──P──► velocity setpoint ──PID──► acceleration command

The acceleration command is then mapped to physical actuator inputs by
*inverting the simulation's own force model*:

  Horizontal — the sim produces lateral force in the BODY frame:
        Fx_body = -F_lat·sin(pitch_vane)
        Fy_body =  F_lat·sin(roll_vane)
    and momentum theory gives F_lat ≈ T_prop. So to realise a desired world
    acceleration we rotate it into the body frame by the measured yaw, then
    solve for the vane angles:
        sin(pitch) = -m·ax_body / T_prop
        sin(roll)  =  m·ay_body / T_prop

  Vertical — Fz = T_prop·cos(pitch)·cos(roll) - m·g. Solving for the thrust
    that yields the desired vertical acceleration:
        T_des = m·(g + az) / (cos(pitch)·cos(roll))
        throttle = √(T_des / T_max)
    Gravity feed-forward falls out for free: az = 0 → throttle = hover.

Yaw is not an actuator in this airframe (it drifts from reactive prop torque),
so the controller works purely in the world frame and merely *compensates* for
the measured heading when rotating into the body frame.
"""

from __future__ import annotations

import math

from .config import ControlConfig, DroneParams
from .pid import PID
from .telemetry import Command, Telemetry


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class NavigationController:
    def __init__(self, drone: DroneParams, control: ControlConfig):
        self.drone = drone
        self.cfg = control

        self.pid_vx = PID(control.vel_xy)
        self.pid_vy = PID(control.vel_xy)
        self.pid_vz = PID(control.vel_z)

        # Minimum thrust used in the horizontal inverse model. Guards against a
        # divide-by-tiny when the prop is barely spinning; also caps achievable
        # tilt so we never ask for asin() out of domain.
        self._min_thrust = max(0.1, 0.25 * drone.mass * drone.gravity)

    def reset(self) -> None:
        self.pid_vx.reset()
        self.pid_vy.reset()
        self.pid_vz.reset()

    def update(self, tlm: Telemetry, target, dt: float) -> Command:
        """Compute the autopilot command to drive the drone toward ``target``.

        ``target`` is a world-frame (x, y, z) in metres.
        """
        tx, ty, tz = target

        # ── Outer loop: position error → bounded velocity setpoint ────────────
        vmax = self.cfg.v_max_xy
        vsp_x = _clamp(self.cfg.pos_xy_p * (tx - tlm.x), -vmax, vmax)
        vsp_y = _clamp(self.cfg.pos_xy_p * (ty - tlm.y), -vmax, vmax)
        vsp_z = _clamp(self.cfg.pos_z_p * (tz - tlm.z),
                       -self.cfg.vz_max, self.cfg.vz_max)

        # ── Inner loop: velocity error → acceleration command ─────────────────
        ax = self.pid_vx.update(vsp_x, tlm.vx, dt)
        ay = self.pid_vy.update(vsp_y, tlm.vy, dt)
        az = self.pid_vz.update(vsp_z, tlm.vz, dt)

        # ── Live thrust estimate from telemetry (mirrors the sim) ─────────────
        t_prop = self.drone.thrust_from_prop_speed(tlm.prop_speed)
        t_prop = max(t_prop, self._min_thrust)

        # ── Horizontal: world accel → body frame → vane angles ────────────────
        m = self.drone.mass
        fx_world = m * ax
        fy_world = m * ay
        cy = math.cos(tlm.yaw)
        sy = math.sin(tlm.yaw)
        fx_body = fx_world * cy + fy_world * sy
        fy_body = -fx_world * sy + fy_world * cy

        sin_max = math.sin(self.drone.max_vane_rad)
        pitch = math.asin(_clamp(-fx_body / t_prop, -sin_max, sin_max))
        roll = math.asin(_clamp(fy_body / t_prop, -sin_max, sin_max))

        # ── Vertical: desired accel → thrust → throttle ───────────────────────
        tilt_factor = max(0.2, math.cos(pitch) * math.cos(roll))
        t_des = m * (self.drone.gravity + az) / tilt_factor
        throttle = math.sqrt(_clamp(t_des / self.drone.thrust_max, 0.0, 1.0))

        return Command(throttle=throttle, pitch=pitch, roll=roll)
