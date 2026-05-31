"""Map a controller ``Command`` (vane deflections in rad + throttle) to the
hardware command the ESP32 expects: four *logical* servo angles in degrees and
an ESC throttle fraction.

Coordinate split (kept deliberately simple so the ESP32 stays dumb):

  * here (PC)  : vane deflection [rad] ── neutral 0 ──► logical servo angle [deg]
                 centred on ``neutral_deg`` (90°) and hard-clamped to the
                 ``[min_deg, max_deg]`` rotation limit (default 40°–160°).
  * ESP32      : applies its own per-pin trim calibration on top of the logical
                 angle, then re-clamps to [min_deg, max_deg] for safety.

The deflection→degree gain is auto-fitted by default so a full ``max_vane_rad``
deflection lands on the nearer travel limit; override ``gain_deg_per_rad`` in the
config to harden it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from .config import ServoConfig
from .telemetry import Command


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class ServoMapper:
    cfg: ServoConfig
    max_vane_rad: float

    @property
    def gain(self) -> float:
        """Logical degrees of servo travel per radian of vane deflection."""
        if self.cfg.gain_deg_per_rad is not None:
            return self.cfg.gain_deg_per_rad
        span = min(self.cfg.neutral_deg - self.cfg.min_deg,
                   self.cfg.max_deg - self.cfg.neutral_deg)
        return span / self.max_vane_rad if self.max_vane_rad else 0.0

    def vane_deg(self, deflection_rad: float, reverse: bool = False) -> float:
        """One vane deflection [rad] → logical servo angle [deg], clamped to limit.

        ``reverse`` flips the rotation direction for a mirror-mounted servo.
        """
        d = -deflection_rad if reverse else deflection_rad
        angle = self.cfg.neutral_deg + d * self.gain
        return _clamp(angle, self.cfg.min_deg, self.cfg.max_deg)

    def esc(self, throttle: float) -> float:
        """Throttle fraction → ESC fraction with the configured idle floor."""
        t = _clamp(throttle, 0.0, 1.0)
        floor = self.cfg.esc_min_throttle
        return floor + (1.0 - floor) * t if t > 0.0 else 0.0

    def to_hw(self, cmd: Command) -> Dict[str, float]:
        """Full hardware command payload: ESC fraction + 4 logical servo degrees."""
        s = self.servo_degs(cmd)
        return {"throttle": round(self.esc(cmd.throttle), 4),
                "s1": s[0], "s2": s[1], "s3": s[2], "s4": s[3]}

    def servo_degs(self, cmd: Command) -> List[float]:
        rev = self.cfg.reverse
        return [round(self.vane_deg(v, rev[i] if i < len(rev) else False), 2)
                for i, v in enumerate(cmd.vanes)]
