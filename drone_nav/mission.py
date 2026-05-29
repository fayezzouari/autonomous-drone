"""Waypoint sequencing: the A→B (→C…) mission layer above the controller.

The mission holds an ordered list of world-frame waypoints and decides which
one is the *active target*. A waypoint counts as reached once the drone is
within ``arrival_radius`` AND slower than ``arrival_speed`` for a sustained
``hold_time`` — the dwell prevents declaring arrival while merely passing
through at speed.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .config import MissionConfig
from .telemetry import Telemetry


class Mission:
    def __init__(self, cfg: MissionConfig):
        self.cfg = cfg
        self.waypoints: List[Tuple[float, float, float]] = [
            tuple(float(c) for c in wp) for wp in cfg.waypoints
        ]
        self.index = 0
        self._dwell = 0.0
        self.complete = len(self.waypoints) == 0

    @property
    def target(self) -> Optional[Tuple[float, float, float]]:
        """The active waypoint, or None when the mission is complete."""
        if self.complete or self.index >= len(self.waypoints):
            return None
        return self.waypoints[self.index]

    def distance_to_target(self, tlm: Telemetry) -> float:
        tgt = self.target
        if tgt is None:
            return 0.0
        return math.dist(tlm.pos, tgt)

    def update(self, tlm: Telemetry, dt: float) -> Optional[Tuple[float, float, float]]:
        """Advance the mission given new telemetry; return the active target."""
        tgt = self.target
        if tgt is None:
            return None

        within = math.dist(tlm.pos, tgt) <= self.cfg.arrival_radius
        slow = tlm.speed <= self.cfg.arrival_speed
        if within and slow:
            self._dwell += dt
        else:
            self._dwell = 0.0

        if self._dwell >= self.cfg.hold_time:
            self._advance()

        return self.target

    def _advance(self) -> None:
        self._dwell = 0.0
        self.index += 1
        if self.index >= len(self.waypoints):
            if self.cfg.loop and self.waypoints:
                self.index = 0
            else:
                self.index = len(self.waypoints)
                self.complete = True
