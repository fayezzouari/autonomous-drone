"""A single, reusable PID controller.

Features that matter for flight control:
  - output clamping (saturation limits)
  - integral clamping + conditional anti-windup (don't accumulate while the
    output is saturated and the error pushes it further into saturation)
  - derivative-on-measurement (avoids the derivative "kick" when the setpoint
    changes suddenly, e.g. switching waypoints)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    out_min: float = -math.inf
    out_max: float = math.inf
    i_limit: Optional[float] = None  # symmetric clamp on the integral term


class PID:
    """Standard PID with anti-windup and derivative-on-measurement."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_measurement: Optional[float] = None
        # Last-update telemetry, exposed for profiling/visualisation. These
        # mirror the terms summed in update(); they never feed back into it.
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_error = 0.0
        self.last_setpoint = 0.0
        self.last_measurement = 0.0
        self.last_output = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0.0:
            out = self._clamp(self.gains.kp * (setpoint - measurement))
            self._record(self.gains.kp * (setpoint - measurement), self._integral,
                         0.0, setpoint - measurement, setpoint, measurement, out)
            return out

        g = self.gains
        error = setpoint - measurement

        proportional = g.kp * error

        # Derivative on measurement (negated) to avoid setpoint-change kick.
        if self._prev_measurement is None:
            derivative = 0.0
        else:
            derivative = -g.kd * (measurement - self._prev_measurement) / dt
        self._prev_measurement = measurement

        # Tentative integral, then clamp.
        integral = self._integral + g.ki * error * dt
        if g.i_limit is not None:
            integral = _clamp(integral, -g.i_limit, g.i_limit)

        raw = proportional + integral + derivative
        out = self._clamp(raw)

        # Conditional anti-windup: only keep the new integral if it didn't push
        # an already-saturated output deeper into saturation.
        saturated_high = raw > g.out_max and error > 0.0
        saturated_low = raw < g.out_min and error < 0.0
        if not (saturated_high or saturated_low):
            self._integral = integral

        self._record(proportional, integral, derivative, error,
                     setpoint, measurement, out)
        return out

    def _record(self, p, i, d, error, setpoint, measurement, out) -> None:
        self.last_p = p
        self.last_i = i
        self.last_d = d
        self.last_error = error
        self.last_setpoint = setpoint
        self.last_measurement = measurement
        self.last_output = out

    def _clamp(self, value: float) -> float:
        return _clamp(value, self.gains.out_min, self.gains.out_max)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
