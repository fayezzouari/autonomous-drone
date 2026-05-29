"""Headless re-implementation of the Blender sim's translational physics.

This is NOT the renderer — it is a faithful port of the force model in
``blender-navigatio.py`` (gravity, speed² thrust, vane lateral force, drag,
ground effect, reactive-torque yaw, ground bounce). It lets us close the loop
and verify the controller flies A→B *without* launching Blender.

It consumes a ``Command`` and produces ``Telemetry`` — exactly the contract the
MQTT bridge implements — so the same controller code drives both.
"""

from __future__ import annotations

import math

from drone_nav.telemetry import Command, Telemetry

# Constants copied from blender-navigatio.py
MASS = 0.70
GRAVITY = 9.81
THRUST_MAX = 15.0
PROP_MAX_SPEED = 720.0
MAX_DEG = 28.0
MAX_RAD = math.radians(MAX_DEG)
PROP_ACCEL = 1500.0
PROP_DECEL = 120.0
RHO_AIR = 1.225
ROTOR_RADIUS = 0.15
ROTOR_AREA = math.pi * ROTOR_RADIUS ** 2
VANE_COEFF = 2.0 * RHO_AIR * ROTOR_AREA
DRAG_LIN = 0.15
DRAG_QUAD = 0.037
GE_GAIN = 0.25
GE_HEIGHT_SCALE = 0.30
PROP_TORQUE_K = 0.004
YAW_INERTIA = 0.04
YAW_DRAG_K = 0.50


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SimStub:
    """Minimal copy of the sim's physics tick, driven by autopilot commands."""

    def __init__(self, start=(0.0, 0.0, 0.0), ground_z=0.0, coll_offset=0.05):
        self.x, self.y, self.z = start
        self.vx = self.vy = self.vz = 0.0
        self.yaw = 0.0
        self.yaw_vel = 0.0
        self.prop_speed = 0.0
        self.ground_z = ground_z
        self.coll_offset = coll_offset
        self.t = 0.0
        # Park on the ground at start, like the sim does.
        self.z = max(self.z, ground_z + coll_offset)

    def step(self, cmd: Command, dt: float) -> Telemetry:
        # ── Vane angles (autopilot sets them directly, clamped) ───────────────
        ap = _clamp(cmd.pitch, -MAX_RAD, MAX_RAD)
        ar = _clamp(cmd.roll, -MAX_RAD, MAX_RAD)

        # ── Propeller: ramp toward the commanded throttle fraction ────────────
        target_speed = _clamp(cmd.throttle, 0.0, 1.0) * PROP_MAX_SPEED
        if self.prop_speed < target_speed:
            self.prop_speed = min(target_speed, self.prop_speed + PROP_ACCEL * dt)
        else:
            self.prop_speed = max(target_speed, self.prop_speed - PROP_DECEL * dt)

        # ── Aerodynamics (mirror of the sim) ──────────────────────────────────
        h_agl = max(0.0, self.z - (self.ground_z + self.coll_offset))
        k_ge = 1.0 + GE_GAIN * math.exp(-h_agl / GE_HEIGHT_SCALE)
        t_prop = THRUST_MAX * (self.prop_speed / PROP_MAX_SPEED) ** 2 * k_ge

        v_ind = math.sqrt(t_prop / VANE_COEFF) if t_prop > 0 else 0.0
        v_desc = max(0.0, -self.vz)
        v_eff_sq = v_ind ** 2 + v_desc ** 2

        f_lat = VANE_COEFF * v_eff_sq
        fx_body = -f_lat * math.sin(ap)
        fy_body = f_lat * math.sin(ar)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        fx = fx_body * cy - fy_body * sy
        fy = fx_body * sy + fy_body * cy

        fz = t_prop * math.cos(ap) * math.cos(ar) - MASS * GRAVITY

        fx -= (DRAG_LIN + DRAG_QUAD * abs(self.vx)) * self.vx
        fy -= (DRAG_LIN + DRAG_QUAD * abs(self.vy)) * self.vy
        fz -= (DRAG_LIN + DRAG_QUAD * abs(self.vz)) * self.vz

        # ── Semi-implicit Euler integration ───────────────────────────────────
        self.vx += (fx / MASS) * dt
        self.vy += (fy / MASS) * dt
        self.vz += (fz / MASS) * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # ── Reactive-torque yaw ───────────────────────────────────────────────
        q_react = -PROP_TORQUE_K * t_prop
        q_damp = -YAW_DRAG_K * self.yaw_vel
        self.yaw_vel += (q_react + q_damp) / YAW_INERTIA * dt
        self.yaw += self.yaw_vel * dt

        # ── Ground collision ──────────────────────────────────────────────────
        floor = self.ground_z + self.coll_offset
        if self.z <= floor:
            self.z = floor
            if self.vz < 0:
                impact = abs(self.vz)
                rest = min(0.50, 0.12 + impact * 0.07)
                self.vz = impact * rest
            self.vx *= 0.82
            self.vy *= 0.82
            self.yaw_vel = 0.0

        self.t += dt
        return self.telemetry()

    def telemetry(self) -> Telemetry:
        return Telemetry(
            t=self.t, x=self.x, y=self.y, z=self.z,
            vx=self.vx, vy=self.vy, vz=self.vz,
            yaw=self.yaw, prop_speed=self.prop_speed,
        )
