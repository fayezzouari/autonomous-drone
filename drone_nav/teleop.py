"""``drone-teleop`` — fly the singlecopter manually with a PS4 controller.

Pipeline (all on the PC):

    PS4 pad ──► Gamepad ──► StickState ──► ManualPilot (PID) ──► Command
                                                                   │
                       ServoMapper ──► {throttle, s1..s4 deg} ─────┤──► MQTT drone/hw ──► ESP32
                                                Command (rad) ─────┘──► MQTT drone/cmd (sim/webapp)

    ESP32 IMU ──► MQTT drone/imu ──► yaw feedback (heading hold)

Modes:
  • default          : publish over MQTT to the ESP32 (and drone/cmd for the
                       webapp / Blender). Heading-hold uses IMU yaw feedback.
  • --sim            : fly the in-process physics stub, no broker / no hardware.

Controls (DualShock 4 defaults — see ``python -m drone_nav.gamepad`` to verify):
  right stick Y  THROTTLE (up ramps up, centre holds)
  left stick  Y  pitch (fwd/back)       left stick X   roll (left/right)
  L1 / R1        yaw left / right        Options        arm / disarm
  Circle         toggle altitude-hold    PS             kill (disarm now)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from .config import Config, load_config
from .manual_control import ManualPilot
from .servo_map import ServoMapper
from .telemetry import Command, Telemetry

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.yaml"


def _fmt(tlm: Telemetry, sticks, cmd: Command, hw) -> str:
    mode = "ALT" if sticks.alt_hold else "DIR"
    arm = "ARMED" if sticks.armed else "disarm"
    return (f"[{arm}/{mode}] yaw={tlm.yaw:+.2f} "
            f"thr={cmd.throttle:.2f} "
            f"vanes=({cmd.vane1:+.2f},{cmd.vane2:+.2f},{cmd.vane3:+.2f},{cmd.vane4:+.2f}) "
            f"servo=({hw['s1']:.0f},{hw['s2']:.0f},{hw['s3']:.0f},{hw['s4']:.0f})")


def _make_pad(cfg: Config):
    from .gamepad import Gamepad
    return Gamepad(deadzone=cfg.manual.deadzone, expo=cfg.manual.expo,
                   throttle_rate=cfg.manual.throttle_rate)


# ── SIM: drive the in-process physics stub ────────────────────────────────────────
def run_sim(cfg: Config, verbose: bool) -> int:
    from tools.sim_stub import SimStub

    pad = _make_pad(cfg)
    pilot = ManualPilot(cfg.drone, cfg.manual, cfg.control)
    mapper = ServoMapper(cfg.servo, cfg.drone.max_vane_rad)
    sim = SimStub()
    dt = 1.0 / cfg.control.loop_rate_hz
    tlm = sim.telemetry()
    print(f"[sim] teleop on '{pad.name}'. Arm with Options, then fly. Ctrl-C to quit.")
    last_log = 0.0
    try:
        while True:
            loop_start = time.perf_counter()
            sticks = pad.read()
            cmd = pilot.update(sticks, tlm, dt)
            hw = mapper.to_hw(cmd)
            tlm = sim.step(cmd, dt)
            pad.clear_kill()
            if verbose and time.perf_counter() - last_log > 0.2:
                print(f"t={tlm.t:6.1f} z={tlm.z:+.2f} " + _fmt(tlm, sticks, cmd, hw))
                last_log = time.perf_counter()
            _sleep_rest(loop_start, dt)
    except KeyboardInterrupt:
        print("\n[sim] stopped.")
    finally:
        pad.close()
    return 0


# ── HARDWARE / MQTT: publish to the ESP32 ─────────────────────────────────────────
def run_mqtt(cfg: Config, verbose: bool) -> int:
    from .mqtt_io import MqttLink

    pad = _make_pad(cfg)
    pilot = ManualPilot(cfg.drone, cfg.manual, cfg.control)
    mapper = ServoMapper(cfg.servo, cfg.drone.max_vane_rad)
    link = MqttLink(cfg.mqtt)
    print(f"[mqtt] connecting to {cfg.mqtt.host}:{cfg.mqtt.port} ...")
    if not link.connect():
        print(f"[mqtt] could not reach the broker"
              + (f" ({link.last_error})" if link.last_error else "") + ".")
        pad.close()
        return 1
    print(f"[mqtt] connected. Teleop on '{pad.name}'. Servo+ESC → "
          f"'{cfg.mqtt.topic_hw_cmd}', yaw feedback ← '{cfg.mqtt.topic_imu}'.")
    print("       Arm with Options. The ESP32 fails safe if commands stop.")
    dt = 1.0 / cfg.control.loop_rate_hz
    last_log = 0.0
    try:
        while True:
            loop_start = time.perf_counter()
            sticks = pad.read()
            # Use IMU yaw if it has arrived; otherwise an empty snapshot (yaw 0).
            tlm = link.latest_telemetry() or Telemetry()
            cmd = pilot.update(sticks, tlm, dt)
            hw = mapper.to_hw(cmd)
            link.publish_hw_command(hw)
            link.publish_command(cmd)   # for the webapp / Blender / logging
            pad.clear_kill()
            if verbose and time.perf_counter() - last_log > 0.2:
                print(_fmt(tlm, sticks, cmd, hw))
                last_log = time.perf_counter()
            _sleep_rest(loop_start, dt)
    except KeyboardInterrupt:
        print("\n[mqtt] interrupted — sending neutral/disarm.")
        link.publish_hw_command(mapper.to_hw(Command(throttle=0.0)))
        link.publish_command(Command())
    finally:
        link.close()
        pad.close()
    return 0


def _sleep_rest(loop_start: float, dt: float) -> None:
    elapsed = time.perf_counter() - loop_start
    if elapsed < dt:
        time.sleep(dt - elapsed)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PS4 manual teleop for the singlecopter")
    ap.add_argument("-c", "--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--sim", action="store_true",
                    help="fly the in-process physics stub (no broker/hardware)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    return run_sim(cfg, args.verbose) if args.sim else run_mqtt(cfg, args.verbose)


if __name__ == "__main__":
    raise SystemExit(main())
