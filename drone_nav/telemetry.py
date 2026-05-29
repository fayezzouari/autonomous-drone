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

    ``throttle`` is a fraction [0, 1] of full prop speed; ``pitch`` and
    ``roll`` are vane deflection *angles in radians* (the sim clamps them to
    its own ±MAX_DEG limit). ``pitch`` drives Vanes 1-3, ``roll`` Vanes 2-4 —
    matching the sim's body-frame force convention.
    """

    throttle: float = 0.0
    pitch: float = 0.0   # radians, body-frame fore/aft vane
    roll: float = 0.0    # radians, body-frame lateral vane

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, payload) -> "Command":
        if isinstance(payload, (bytes, bytearray)):
            payload = payload.decode("utf-8")
        data = json.loads(payload)
        fields = cls.__dataclass_fields__
        return cls(**{k: float(v) for k, v in data.items() if k in fields})
