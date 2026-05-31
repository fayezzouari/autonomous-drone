"""Controller bridge: read a PS4 (DualShock 4) pad on the PC and turn it into a
normalised :class:`StickState`.

This is the *transport* layer for manual flight — it knows nothing about the
drone. It reads raw axes/buttons via pygame, applies a deadzone + expo curve,
and exposes four normalised stick axes plus a few latched buttons. The control
math (sticks → vanes/throttle) lives in :mod:`drone_nav.manual_control`.

Left stick = brushless throttle; right stick = vanes (servos); yaw on L1/R1. The
throttle stick *ramps* the level while deflected and *holds* it when centred
(like a collective lever), so a self-centring stick still parks the motor at a
chosen power. Axis/button indices follow the common SDL DualShock 4 layout but
are overridable (they vary a little by OS / pygame-SDL version). Defaults:

    axis 1    left stick  Y   → THROTTLE (brushless; up ramps up, centre holds)
    axis 3    right stick Y   → pitch (vanes: fore/aft; forward = +, so we negate)
    axis 2    right stick X   → roll  (vanes: lateral)
    (left stick X is unused)

    button 9  L1        → yaw left
    button 10 R1        → yaw right
    button 1  Circle     → toggle altitude-hold
    button 6  Options    → toggle arm
    button 5  PS          → kill (disarm immediately)

Throttle starts at minimum and is pinned there while disarmed, so arming never
spins the motor on its own — push the right stick up to bring it up.

Run ``python -m drone_nav.gamepad`` to print the live mapping and discover the
indices for your specific pad.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class AxisMap:
    # DualShock 4, SDL2 standard layout. Verify with `python -m drone_nav.gamepad`.
    throttle: int = 1     # left stick Y  → brushless throttle (ramp/hold)
    pitch: int = 3        # right stick Y → fore/aft vanes
    roll: int = 2         # right stick X → lateral vanes
    # Sticks whose "positive" hardware direction is the wrong way for us.
    invert: Dict[str, bool] = field(default_factory=lambda: {
        "throttle": True,   # pushing the stick up reads negative
        "pitch": True,      # pushing the stick forward reads negative
    })


@dataclass
class ButtonMap:
    # SDL2 standard DualShock 4 layout (Options=6 confirmed on this pad).
    yaw_left: int = 9       # L1
    yaw_right: int = 10     # R1
    alt_hold: int = 1       # Circle
    arm: int = 6            # Options
    kill: int = 5           # PS


@dataclass
class StickState:
    """Normalised pilot intent. All axes in [-1, 1]; buttons latched booleans."""
    throttle: float = 0.0   # +1 = climb / more throttle
    pitch: float = 0.0      # +1 = nose forward (+body X)
    roll: float = 0.0       # +1 = right (+body Y)
    yaw: float = 0.0        # +1 = yaw right
    armed: bool = False
    alt_hold: bool = False
    kill: bool = False


def _shape(v: float, deadzone: float, expo: float) -> float:
    """Apply a deadzone then an expo curve, preserving sign and full-scale ends."""
    if abs(v) <= deadzone:
        return 0.0
    # rescale so the edge of the deadzone maps to 0 and |v|=1 stays 1
    s = (abs(v) - deadzone) / (1.0 - deadzone)
    s = (1.0 - expo) * s + expo * s ** 3
    return s if v > 0 else -s


class Gamepad:
    """Thin pygame-joystick reader producing a :class:`StickState`."""

    def __init__(self, deadzone: float = 0.08, expo: float = 0.30,
                 throttle_rate: float = 0.6, index: int = 0,
                 axes: Optional[AxisMap] = None,
                 buttons: Optional[ButtonMap] = None):
        try:
            import pygame  # noqa: F401
        except ImportError as exc:  # pragma: no cover - env dependent
            raise RuntimeError(
                "pygame is required for the gamepad bridge. Install it with:\n"
                "    uv sync --extra gamepad"
            ) from exc
        import pygame

        self._pygame = pygame
        self.deadzone = deadzone
        self.expo = expo
        self.throttle_rate = throttle_rate   # throttle units per second held
        self.axes = axes or AxisMap()
        self.buttons = buttons or ButtonMap()

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= index:
            raise RuntimeError(
                "No gamepad detected. Pair the PS4 controller over Bluetooth "
                "(or plug it in) and try again."
            )
        self.js = pygame.joystick.Joystick(index)
        self.js.init()
        self.name = self.js.get_name()

        # Latched/toggled state (edge-detected against the previous frame).
        self._armed = False
        self._alt_hold = False
        self._kill = False
        self._prev_buttons: Dict[int, bool] = {}

        # Throttle level [-1, 1] ramped by the Triangle/Cross buttons. Starts at
        # minimum; -1 maps to motor-off in direct mode.
        self._throttle = -1.0
        self._last_read = None   # monotonic timestamp for a framerate-free ramp

    # ── reading ──────────────────────────────────────────────────────────────────
    def _axis(self, idx: int, name: str) -> float:
        raw = self.js.get_axis(idx)
        if self.axes.invert.get(name):
            raw = -raw
        return _shape(raw, self.deadzone, self.expo)

    def _held(self, idx: int) -> bool:
        """True while a button is held down."""
        try:
            return bool(self.js.get_button(idx))
        except Exception:
            return False

    def _pressed_edge(self, idx: int) -> bool:
        """True only on the frame a button transitions up→down."""
        try:
            now = bool(self.js.get_button(idx))
        except Exception:
            return False
        was = self._prev_buttons.get(idx, False)
        self._prev_buttons[idx] = now
        return now and not was

    def read(self) -> StickState:
        self._pygame.event.pump()
        now = time.monotonic()
        dt = 0.0 if self._last_read is None else now - self._last_read
        self._last_read = now

        if self._pressed_edge(self.buttons.arm):
            self._armed = not self._armed
        if self._pressed_edge(self.buttons.alt_hold):
            self._alt_hold = not self._alt_hold
        if self._pressed_edge(self.buttons.kill):
            self._kill = True
            self._armed = False

        # Throttle ramp: the left stick raises/lowers the level while deflected
        # and holds it when centred. Pinned to minimum while disarmed so arming
        # never spins the motor on its own.
        if not self._armed:
            self._throttle = -1.0
        else:
            rate = self._axis(self.axes.throttle, "throttle") * self.throttle_rate
            self._throttle = max(-1.0, min(1.0, self._throttle + rate * dt))

        # Yaw is a digital command from the L1/R1 shoulder buttons.
        yaw = float(self._held(self.buttons.yaw_right) -
                    self._held(self.buttons.yaw_left))

        return StickState(
            throttle=self._throttle,
            pitch=self._axis(self.axes.pitch, "pitch"),
            roll=self._axis(self.axes.roll, "roll"),
            yaw=yaw,
            armed=self._armed,
            alt_hold=self._alt_hold,
            kill=self._kill,
        )

    def clear_kill(self) -> None:
        self._kill = False

    def close(self) -> None:
        try:
            self.js.quit()
            self._pygame.joystick.quit()
            self._pygame.quit()
        except Exception:
            pass


def _main() -> int:  # pragma: no cover - manual discovery helper
    import time

    pad = Gamepad()
    print(f"Reading '{pad.name}'. Move sticks / press buttons (Ctrl-C to stop).")
    print("Use this to confirm axis/button indices for your pad.")
    try:
        while True:
            s = pad.read()
            pressed = [i for i in range(pad.js.get_numbuttons())
                       if pad.js.get_button(i)]
            print(f"thr={s.throttle:+.2f} pitch={s.pitch:+.2f} roll={s.roll:+.2f} "
                  f"yaw={s.yaw:+.2f} armed={s.armed} alt_hold={s.alt_hold} "
                  f"kill={s.kill}  buttons_down={pressed}  axes="
                  f"{[round(pad.js.get_axis(i), 2) for i in range(pad.js.get_numaxes())]}"
                  "      ",
                  end="\r")
            pad.clear_kill()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        pad.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
