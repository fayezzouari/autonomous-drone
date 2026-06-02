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
from .planner import PlannerConfig


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


# ── Manual gamepad (teleop) control ─────────────────────────────────────────────
@dataclass
class ManualConfig:
    """Maps PS4 stick inputs to vane/throttle commands.

    Right stick → pitch/roll (translation), left stick X → yaw rate, left stick Y
    → throttle (direct) or climb rate (when altitude-hold is on). A yaw PID holds
    the last heading whenever the yaw stick is centred.
    """
    deadzone: float = 0.08          # stick deadzone (fraction of full travel)
    expo: float = 0.30              # 0 = linear, →1 = softer around centre
    throttle_rate: float = 0.6      # throttle units/s while Triangle/Cross held
    tilt_frac: float = 1.0          # fraction of max_vane_rad a full stick commands
    yaw_rate_max: float = 1.2       # rad/s commanded by a full yaw stick
    yaw_swirl_frac: float = 0.6     # fraction of max_vane_rad used for yaw swirl
    climb_rate_max: float = 2.0     # m/s commanded by a full throttle stick (alt-hold)
    altitude_hold_default: bool = False
    # Feedforward anti-torque: constant swirl (rad) that cancels the prop's
    # reaction torque. Both the torque and the vanes' authority scale with prop
    # wash, so the required swirl ANGLE is ~constant while the prop spins — hence
    # a fixed bias (ramped in with throttle), not a throttle-proportional one.
    # MUST be bench-tuned: sign depends on prop rotation direction. 0 disables.
    yaw_antitorque: float = 0.008   # rad of swirl bias (≈sim model; tune on hw)
    # kp/ki act on heading error (ki = anti-torque trim, nulls residual spin),
    # kd damps the measured yaw rate gz, i_limit caps the trim swirl.
    yaw: PIDGains = field(default_factory=lambda: PIDGains(
        kp=1.5, ki=0.4, kd=0.1, out_min=-0.384, out_max=0.384, i_limit=0.2))


# ── Servo / ESC output mapping (PC → ESP32) ─────────────────────────────────────
@dataclass
class ServoConfig:
    """Maps a vane deflection (rad, neutral 0) to a *logical* servo angle (deg).

    Neutral deflection → ``neutral_deg``; the angle is hard-clamped to
    ``[min_deg, max_deg]`` (the vane rotation limit). The ESP32 then applies its
    own per-pin trim calibration on top of this logical angle. ``gain_deg_per_rad``
    is auto-fitted (so a full ``max_vane_rad`` deflection reaches the nearer limit)
    when left as ``None``.
    """
    neutral_deg: float = 90.0
    min_deg: float = 40.0
    max_deg: float = 160.0
    gain_deg_per_rad: Optional[float] = None
    esc_min_throttle: float = 0.0   # ESC idle floor (fraction) sent at zero stick
    # Per-servo direction. The two servos on each arm are mirror-mounted, so one
    # of each pair must be reversed. Order [v1, v2, v3, v4]; default flips v3 & v4.
    reverse: List[bool] = field(default_factory=lambda: [False, False, True, True])


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
    topic_command: str = "drone/cmd"             # nav → sim  (throttle + 4 vanes, rad)
    topic_vane_input: str = "drone/vanes"        # external → nav (raw vane angles)
    topic_status: str = "drone/status"           # nav → world
    topic_imu: str = "drone/imu"                 # ESP32 IMU → nav (orientation)
    topic_hw_cmd: str = "drone/hw"               # nav → ESP32 (servo deg + ESC)
    topic_obstacles: str = "drone/obs"           # sim → nav (obstacle box list)
    topic_path: str = "drone/path"               # nav → world (planned waypoints)
    topic_goto: str = "drone/goto"               # world → nav (live A→B target)


@dataclass
class Config:
    drone: DroneParams = field(default_factory=DroneParams)
    control: ControlConfig = field(default_factory=ControlConfig)
    goto: GotoConfig = field(default_factory=GotoConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    manual: ManualConfig = field(default_factory=ManualConfig)
    servo: ServoConfig = field(default_factory=ServoConfig)
    mqtt: MQTTConfig = field(default_factory=MQTTConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    # Static obstacle boxes for offline (--sim) demos/tests; live runs read them
    # from the MQTT ``drone/obs`` topic instead. Each item is a {c,w,h,t} dict.
    obstacles: List[dict] = field(default_factory=list)


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
    if "manual" in raw:
        mn = raw["manual"]
        cfg.manual = ManualConfig(
            deadzone=mn.get("deadzone", cfg.manual.deadzone),
            expo=mn.get("expo", cfg.manual.expo),
            throttle_rate=mn.get("throttle_rate", cfg.manual.throttle_rate),
            tilt_frac=mn.get("tilt_frac", cfg.manual.tilt_frac),
            yaw_rate_max=mn.get("yaw_rate_max", cfg.manual.yaw_rate_max),
            yaw_swirl_frac=mn.get("yaw_swirl_frac", cfg.manual.yaw_swirl_frac),
            climb_rate_max=mn.get("climb_rate_max", cfg.manual.climb_rate_max),
            altitude_hold_default=mn.get("altitude_hold_default",
                                         cfg.manual.altitude_hold_default),
            yaw_antitorque=mn.get("yaw_antitorque", cfg.manual.yaw_antitorque),
            yaw=_pid_from_dict(mn.get("yaw", {}), cfg.manual.yaw),
        )
    if "servo" in raw:
        cfg.servo = ServoConfig(**{k: v for k, v in raw["servo"].items()
                                   if k in vars(ServoConfig())})
    if "mqtt" in raw:
        m = raw["mqtt"]
        cfg.mqtt = MQTTConfig(**{k: v for k, v in m.items()
                                 if k in vars(MQTTConfig())})
    if "planner" in raw:
        p = raw["planner"]
        cfg.planner = PlannerConfig(**{k: v for k, v in p.items()
                                       if k in vars(PlannerConfig())})
    if "obstacles" in raw and isinstance(raw["obstacles"], list):
        cfg.obstacles = list(raw["obstacles"])
    return cfg
