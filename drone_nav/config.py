"""Typed configuration + YAML loader.

The ``DroneParams`` defaults are copied verbatim from the Blender simulation
(``blender-navigatio.py``) so the controller's inverse model matches the plant
it is flying. Everything is overridable from ``config/config.yaml``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

from .pid import PIDGains


# ── Drone physical parameters (mirror the sim's constants) ─────────────────────
@dataclass
class DroneParams:
    mass: float = 0.70           # kg            (sim MASS)
    gravity: float = 9.81        # m/s²          (sim GRAVITY)
    thrust_max: float = 15.0     # N             (sim THRUST_MAX)
    prop_max_speed: float = 720.0  # deg/s       (sim PROP_MAX_SPEED)
    max_vane_deg: float = 28.0   # deg           (sim MAX_DEG)
    # Momentum-theory coefficient: F_lat ≈ T_prop, derived in the sim as
    # VANE_COEFF = 2·ρ·A. We only need the relationship F_lat ≈ T_prop, so the
    # raw value is informational here.
    rho_air: float = 1.225
    rotor_radius: float = 0.15

    @property
    def max_vane_rad(self) -> float:
        return math.radians(self.max_vane_deg)

    @property
    def hover_throttle(self) -> float:
        """Throttle fraction that produces thrust == weight (≈ 0.68)."""
        return math.sqrt(self.mass * self.gravity / self.thrust_max)

    def thrust_from_prop_speed(self, prop_speed: float) -> float:
        """Estimate current prop thrust [N] from measured prop speed [deg/s].

        Mirrors the sim: T = THRUST_MAX·(prop_speed / PROP_MAX_SPEED)²
        (ground effect omitted — the altitude PID absorbs the residual).
        """
        frac = prop_speed / self.prop_max_speed if self.prop_max_speed else 0.0
        return self.thrust_max * frac * frac


# ── Cascaded control gains + limits ────────────────────────────────────────────
@dataclass
class ControlConfig:
    # Outer position→velocity proportional gains (1/s).
    pos_xy_p: float = 1.2
    pos_z_p: float = 1.5
    # Inner velocity→acceleration PID gains.
    vel_xy: PIDGains = field(default_factory=lambda: PIDGains(
        kp=3.0, ki=0.8, kd=0.15, out_min=-6.0, out_max=6.0, i_limit=4.0))
    vel_z: PIDGains = field(default_factory=lambda: PIDGains(
        kp=4.0, ki=2.0, kd=0.2, out_min=-8.0, out_max=8.0, i_limit=6.0))
    # Kinematic limits.
    v_max_xy: float = 4.0    # m/s   max commanded horizontal speed
    vz_max: float = 2.5      # m/s   max commanded climb/descent speed


# ── MQTT transport ─────────────────────────────────────────────────────────────
@dataclass
class MQTTConfig:
    host: str = "localhost"
    port: int = 1883
    username: Optional[str] = None
    password: Optional[str] = None
    client_id: str = "drone-nav"
    keepalive: int = 30
    topic_telemetry: str = "drone/telemetry"
    topic_command: str = "drone/cmd"
    topic_status: str = "drone/status"


# ── Mission / waypoints ─────────────────────────────────────────────────────────
@dataclass
class MissionConfig:
    waypoints: List[List[float]] = field(default_factory=list)  # [[x,y,z], ...]
    arrival_radius: float = 0.30   # m    within this of a waypoint = "reached"
    arrival_speed: float = 0.30    # m/s  and slower than this
    hold_time: float = 1.0         # s    sustained before advancing
    loop_rate_hz: float = 50.0     # controller update rate
    loop: bool = False             # cycle back to the first waypoint at the end


@dataclass
class Config:
    drone: DroneParams = field(default_factory=DroneParams)
    control: ControlConfig = field(default_factory=ControlConfig)
    mqtt: MQTTConfig = field(default_factory=MQTTConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)


# ── Loading ─────────────────────────────────────────────────────────────────────
def _pid_from_dict(d: dict, default: PIDGains) -> PIDGains:
    base = vars(default).copy()
    base.update({k: v for k, v in d.items() if k in base})
    return PIDGains(**base)


def load_config(path) -> Config:
    """Load a YAML config, falling back to dataclass defaults for any omission."""
    path = Path(path)
    cfg = Config()
    if not path.exists():
        return cfg

    with open(path, "r") as fh:
        raw = yaml.safe_load(fh) or {}

    if "drone" in raw:
        d = raw["drone"]
        cfg.drone = DroneParams(**{k: v for k, v in d.items()
                                   if k in vars(DroneParams())})
    if "control" in raw:
        c = raw["control"]
        cfg.control = ControlConfig(
            pos_xy_p=c.get("pos_xy_p", cfg.control.pos_xy_p),
            pos_z_p=c.get("pos_z_p", cfg.control.pos_z_p),
            vel_xy=_pid_from_dict(c.get("vel_xy", {}), cfg.control.vel_xy),
            vel_z=_pid_from_dict(c.get("vel_z", {}), cfg.control.vel_z),
            v_max_xy=c.get("v_max_xy", cfg.control.v_max_xy),
            vz_max=c.get("vz_max", cfg.control.vz_max),
        )
    if "mqtt" in raw:
        m = raw["mqtt"]
        cfg.mqtt = MQTTConfig(**{k: v for k, v in m.items()
                                 if k in vars(MQTTConfig())})
    if "mission" in raw:
        ms = raw["mission"]
        cfg.mission = MissionConfig(**{k: v for k, v in ms.items()
                                       if k in vars(MissionConfig())})
    return cfg
