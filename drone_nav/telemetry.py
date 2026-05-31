"""Data exchanged with the simulation over MQTT.

Two small, JSON-serialisable dataclasses define the wire format:

  Telemetry  — sim → nav : the drone's measured state every tick
  Command    — nav → sim : the autopilot inputs the controller produces

Keeping these in one module guarantees both ends agree on field names.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass


@dataclass
class Telemetry:
    """Measured drone state, published by the sim bridge each physics tick.

    Positions are in metres (world frame), velocities in m/s, ``yaw`` in
    radians (heading about world +Z), ``prop_speed`` in deg/s.
    """

    t: float = 0.0           # sim time / timestamp (seconds)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    gz: float = 0.0          # rad/s — yaw rate (body Z), for heading-hold damping
    prop_speed: float = 0.0  # deg/s — lets the controller estimate live thrust

    @property
    def pos(self) -> tuple:
        return (self.x, self.y, self.z)

    @property
    def vel(self) -> tuple:
        return (self.vx, self.vy, self.vz)

    @property
    def speed(self) -> float:
        return math.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)

    @property
    def horizontal_speed(self) -> float:
        return math.sqrt(self.vx ** 2 + self.vy ** 2)

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, payload) -> "Telemetry":
        if isinstance(payload, (bytes, bytearray)):
            payload = payload.decode("utf-8")
        data = json.loads(payload)
        # Tolerate extra/missing keys so the wire format can evolve safely.
        fields = cls.__dataclass_fields__
        return cls(**{k: float(v) for k, v in data.items() if k in fields})


@dataclass
class Command:
    """Autopilot inputs consumed by the sim bridge.

    ``throttle`` is a fraction [0, 1] of full prop speed. ``vane1``…``vane4``
    are the four *independent* vane deflection angles in radians — each vane
    moves to its own angle (the sim clamps them to its ±MAX_DEG limit). No
    mixing is applied: whatever you put here is applied vane-for-vane.

    Geometry (mirrors the sim's force convention):
        vane1, vane3  → on the X arm → fore/aft (body-X) force
        vane2, vane4  → on the Y arm → lateral (body-Y) force
    """

    throttle: float = 0.0
    vane1: float = 0.0   # radians
    vane2: float = 0.0
    vane3: float = 0.0
    vane4: float = 0.0

    @property
    def vanes(self) -> tuple:
        return (self.vane1, self.vane2, self.vane3, self.vane4)

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, payload) -> "Command":
        if isinstance(payload, (bytes, bytearray)):
            payload = payload.decode("utf-8")
        data = json.loads(payload)
        fields = cls.__dataclass_fields__
        return cls(**{k: float(v) for k, v in data.items() if k in fields})
