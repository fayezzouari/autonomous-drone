"""Typed configuration + YAML loader.

The ``DroneParams`` defaults are copied verbatim from the Blender simulation
(``blender-navigatio.py``) so the controller's thrust model matches the plant.
Everything is overridable from ``config/config.yaml``.

Control model (current): the controller holds a target ALTITUDE via a PID on
throttle. Steering is done with four INDEPENDENT vane angles supplied
externally (raw) — see ``Command`` in telemetry.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

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
        """Estimate current prop thrust [N] from measured prop speed [deg/s]."""
        frac = prop_speed / self.prop_max_speed if self.prop_max_speed else 0.0
        return self.thrust_max * frac * frac


# ── Altitude control ────────────────────────────────────────────────────────────
@dataclass
class ControlConfig:
    target_altitude: float = 2.0   # m    altitude the throttle PID holds
    pos_z_p: float = 1.5           # 1/s  altitude error → climb-rate setpoint
    vz_max: float = 2.5            # m/s  climb/descent cap
    loop_rate_hz: float = 50.0     # controller update rate
    # Inner climb-rate → vertical-acceleration PID.
    vel_z: PIDGains = field(default_factory=lambda: PIDGains(
        kp=4.0, ki=2.0, kd=0.2, out_min=-8.0, out_max=8.0, i_limit=6.0))


# ── Autonomous point-to-point (A → B) control ──────────────────────────────────
@dataclass
class GotoConfig:
    """Cascaded position controller that drives throttle AND the 4 vanes to fly
    the drone to a world target. Horizontal accel is inverted through the vane
    model into pitch/roll; a yaw PID holds heading; pitch/roll/yaw are mixed into
    the four independent vane angles."""
    pos_xy_p: float = 1.2          # 1/s  horizontal position → velocity
    pos_z_p: float = 1.5           # 1/s  altitude position → velocity
    v_max_xy: float = 4.0          # m/s
    vz_max: float = 2.5            # m/s
    vel_xy: PIDGains = field(default_factory=lambda: PIDGains(
        kp=3.0, ki=0.8, kd=0.15, out_min=-6.0, out_max=6.0, i_limit=4.0))
    vel_z: PIDGains = field(default_factory=lambda: PIDGains(
        kp=4.0, ki=2.0, kd=0.2, out_min=-8.0, out_max=8.0, i_limit=6.0))
    yaw: PIDGains = field(default_factory=lambda: PIDGains(
        kp=1.5, ki=0.0, kd=0.1, out_min=-0.384, out_max=0.384))  # ±22°
    target_yaw: float = 0.0        # heading to hold while travelling (rad)


@dataclass
class MissionConfig:
    """An ordered A → B (→ C …) sequence of world waypoints."""
    waypoints: List[List[float]] = field(default_factory=lambda: [
        [0.0, 0.0, 3.0],     # A — climb off the ground to 3 m
        [14.0, 10.0, 6.0],   # B — far away (≈17 m) and higher (6 m)
    ])
    arrival_radius: float = 0.5    # m
    arrival_speed: float = 0.4     # m/s
    hold_time: float = 1.0         # s sustained inside the arrival window
    loop: bool = False


# ── MQTT transport ─────────────────────────────────────────────────────────────
@dataclass
class MQTTConfig:
    host: str = "localhost"
    port: int = 1883
    username: Optional[str] = None
    password: Optional[str] = None
    client_id: str = "drone-nav"
    keepalive: int = 30
    topic_telemetry: str = "drone/telemetry"     # sim → nav
    topic_command: str = "drone/cmd"             # nav → sim  (throttle + 4 vanes)
    topic_vane_input: str = "drone/vanes"        # external → nav (raw vane angles)
    topic_status: str = "drone/status"           # nav → world


@dataclass
class Config:
    drone: DroneParams = field(default_factory=DroneParams)
    control: ControlConfig = field(default_factory=ControlConfig)
    goto: GotoConfig = field(default_factory=GotoConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    mqtt: MQTTConfig = field(default_factory=MQTTConfig)


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
            target_altitude=c.get("target_altitude", cfg.control.target_altitude),
            pos_z_p=c.get("pos_z_p", cfg.control.pos_z_p),
            vz_max=c.get("vz_max", cfg.control.vz_max),
            loop_rate_hz=c.get("loop_rate_hz", cfg.control.loop_rate_hz),
            vel_z=_pid_from_dict(c.get("vel_z", {}), cfg.control.vel_z),
        )
    if "goto" in raw:
        g = raw["goto"]
        cfg.goto = GotoConfig(
            pos_xy_p=g.get("pos_xy_p", cfg.goto.pos_xy_p),
            pos_z_p=g.get("pos_z_p", cfg.goto.pos_z_p),
            v_max_xy=g.get("v_max_xy", cfg.goto.v_max_xy),
            vz_max=g.get("vz_max", cfg.goto.vz_max),
            vel_xy=_pid_from_dict(g.get("vel_xy", {}), cfg.goto.vel_xy),
            vel_z=_pid_from_dict(g.get("vel_z", {}), cfg.goto.vel_z),
            yaw=_pid_from_dict(g.get("yaw", {}), cfg.goto.yaw),
            target_yaw=g.get("target_yaw", cfg.goto.target_yaw),
        )
    if "mission" in raw:
        ms = raw["mission"]
        cfg.mission = MissionConfig(**{k: v for k, v in ms.items()
                                       if k in vars(MissionConfig())})
    if "mqtt" in raw:
        m = raw["mqtt"]
        cfg.mqtt = MQTTConfig(**{k: v for k, v in m.items()
                                 if k in vars(MQTTConfig())})
    return cfg
