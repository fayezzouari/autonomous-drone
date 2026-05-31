"""Manual (gamepad) pilot: turn pilot stick intent into a :class:`Command`.

This is the "PID using the controller" stage. The sticks set *setpoints*, not
raw outputs, and PIDs close the loops that the pilot cannot comfortably hold by
hand:

  * pitch / roll  : right stick → vane deflection directly (rate/acro feel).
  * yaw           : left-stick X commands a yaw *rate*; when it is centred a yaw
                    PID holds the heading captured at release (so the nose stops
                    drifting from reactive prop torque). The PID output is a
                    swirl deflection mixed differentially into the four vanes.
  * throttle      : left-stick Y. In the default mode it maps directly around
                    the hover point. With altitude-hold on (Circle), the stick
                    commands a climb rate and a PID on vertical velocity holds
                    height — identical math to :class:`AltitudeController`.

The four vanes are mixed so each pair combines translation (pitch/roll) with an
OPPOSING yaw term — the opposition is the anti-torque couple that holds heading
against the prop's reaction torque. With the mirror servo mounting
(servo.reverse), pitch/roll tilt a pair TOGETHER (translate) while yaw makes the
pair OPPOSE (swirl)::

    a1 = pitch + yaw   a3 = pitch − yaw      (X pair → fore/aft + yaw couple)
    a2 = roll  + yaw   a4 = roll  − yaw      (Y pair → lateral  + yaw couple)
"""

from __future__ import annotations

import math

from .config import ControlConfig, DroneParams, ManualConfig
from .pid import PID
from .telemetry import Command, Telemetry


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ManualPilot:
    def __init__(self, drone: DroneParams, manual: ManualConfig,
                 control: ControlConfig):
        self.drone = drone
        self.cfg = manual
        self.control = control
        self.pid_yaw = PID(manual.yaw)
        self.pid_vz = PID(control.vel_z)
        self._mv = drone.max_vane_rad

        self.yaw_setpoint = 0.0       # heading held while the yaw stick is centred
        self._yaw_captured = False
        self._yaw_i = 0.0             # anti-torque trim integrator (rad of swirl)
        self.alt_setpoint = control.target_altitude
        self._alt_captured = False
        self.alt_hold = manual.altitude_hold_default

    def reset(self) -> None:
        self.pid_yaw.reset()
        self.pid_vz.reset()
        self._yaw_captured = False
        self._yaw_i = 0.0
        self._alt_captured = False

    # ── throttle ───────────────────────────────────────────────────────────────────
    def _throttle(self, sticks, tlm: Telemetry, dt: float) -> float:
        if not self.alt_hold:
            # Direct: stick position maps linearly to throttle. Full down (−1) =
            # motor off, full up (+1) = max thrust, centre = 50%.
            self._alt_captured = False
            return _clamp((sticks.throttle + 1.0) * 0.5, 0.0, 1.0)

        # Altitude-hold: stick commands a climb rate; integrate into a setpoint.
        if not self._alt_captured:
            self.alt_setpoint = tlm.z
            self._alt_captured = True
        self.alt_setpoint += sticks.throttle * self.cfg.climb_rate_max * dt
        vsp_z = _clamp(self.control.pos_z_p * (self.alt_setpoint - tlm.z),
                       -self.control.vz_max, self.control.vz_max)
        az = self.pid_vz.update(vsp_z, tlm.vz, dt)
        t_des = self.drone.mass * (self.drone.gravity + az)
        return math.sqrt(_clamp(t_des / self.drone.thrust_max, 0.0, 1.0))

    # ── yaw ──────────────────────────────────────────────────────────────────────
    def _antitorque_ff(self, throttle: float) -> float:
        """Constant anti-torque swirl bias, ramped in over the first sliver of
        throttle (no prop wash → no vane authority, so none commanded near idle).
        """
        ramp = _clamp(throttle / 0.1, 0.0, 1.0)
        return self.cfg.yaw_antitorque * ramp

    def _yaw_swirl(self, sticks, tlm: Telemetry, throttle: float, dt: float) -> float:
        """Anti-torque swirl = feedforward + integral trim + rate damping.

        Three layers cancel the prop's reaction torque efficiently:
          * feedforward  — a constant swirl gets ~most of the way instantly;
          * integral trim — absorbs whatever the feedforward misses (battery
            sag, air density, prop wear), so the *steady-state* spin → 0 even
            when ``yaw_antitorque`` is imperfect;
          * rate damping  — ``-kd·gz`` brakes transients (the gyro yaw rate).
        Everything is summed before the single ``max_swirl`` clamp, and the
        integrator uses conditional anti-windup against *that* limit.
        """
        g = self.pid_yaw.gains
        max_swirl = self.cfg.yaw_swirl_frac * self._mv
        ff = self._antitorque_ff(throttle)
        damp = -g.kd * tlm.gz

        if abs(sticks.yaw) > 1e-3:
            # Active steering: stick commands swirl directly. Hold the trim
            # integrator frozen (don't wind during the manoeuvre) but keep the
            # feedforward + damping live, and recapture heading continuously.
            self.yaw_setpoint = tlm.yaw
            self._yaw_captured = True
            return _clamp(sticks.yaw * max_swirl + ff + damp, -max_swirl, max_swirl)

        if not self._yaw_captured:
            self.yaw_setpoint = tlm.yaw
            self._yaw_captured = True

        err = _wrap_pi(self.yaw_setpoint - tlm.yaw)
        i_tent = self._yaw_i + g.ki * err * dt
        if g.i_limit is not None:
            i_tent = _clamp(i_tent, -g.i_limit, g.i_limit)
        raw = g.kp * err + i_tent + damp + ff
        out = _clamp(raw, -max_swirl, max_swirl)
        # Conditional anti-windup against the real swirl limit: only keep the new
        # integral if it didn't push an already-saturated output deeper in.
        if not ((raw > max_swirl and err > 0.0) or (raw < -max_swirl and err < 0.0)):
            self._yaw_i = i_tent
        return out

    # ── full update ────────────────────────────────────────────────────────────────
    def update(self, sticks, tlm: Telemetry, dt: float) -> Command:
        self.alt_hold = sticks.alt_hold

        if sticks.kill or not sticks.armed:
            # Disarmed / killed → motor off, vanes neutral, controllers reset.
            self.reset()
            return Command(throttle=0.0)

        tilt = self.cfg.tilt_frac * self._mv
        pitch = _clamp(sticks.pitch * tilt, -self._mv, self._mv)
        roll = _clamp(sticks.roll * tilt, -self._mv, self._mv)
        throttle = self._throttle(sticks, tlm, dt)
        yaw = self._yaw_swirl(sticks, tlm, throttle, dt)   # FF needs throttle

        # Singlecopter vane mix: each pair sums translation (pitch/roll) with an
        # OPPOSING yaw term. That opposition is what forms the yaw/anti-torque
        # couple that fights the prop's reaction torque (without it the airframe
        # just spins). Combined with the mirror servo mounting (servo.reverse
        # [F,F,T,T]): pitch/roll tilt a pair TOGETHER (translate), while yaw makes
        # the pair OPPOSE (swirl / hold heading).
        mv = self._mv
        a1 = _clamp(pitch + yaw, -mv, mv)
        a3 = _clamp(pitch - yaw, -mv, mv)
        a2 = _clamp(roll + yaw, -mv, mv)
        a4 = _clamp(roll - yaw, -mv, mv)
        return Command(throttle=throttle, vane1=a1, vane2=a2, vane3=a3, vane4=a4)
