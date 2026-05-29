import math

from drone_nav.pid import PID, PIDGains


def test_proportional_drives_toward_setpoint():
    pid = PID(PIDGains(kp=2.0))
    out = pid.update(setpoint=1.0, measurement=0.0, dt=0.02)
    assert out == 2.0  # 2 * (1 - 0)


def test_output_is_clamped():
    pid = PID(PIDGains(kp=100.0, out_min=-1.0, out_max=1.0))
    assert pid.update(10.0, 0.0, 0.02) == 1.0
    assert pid.update(-10.0, 0.0, 0.02) == -1.0


def test_integral_accumulates_and_is_limited():
    pid = PID(PIDGains(ki=1.0, i_limit=0.5))
    for _ in range(100):
        out = pid.update(1.0, 0.0, 0.1)
    assert math.isclose(out, 0.5, rel_tol=1e-6)


def test_anti_windup_stops_integral_growth_when_saturated():
    pid = PID(PIDGains(kp=1.0, ki=10.0, out_max=1.0, out_min=-1.0))
    # Large persistent error keeps the output saturated.
    for _ in range(50):
        pid.update(5.0, 0.0, 0.1)
    saturated_integral = pid._integral
    # The integral must not have run away unboundedly.
    assert saturated_integral < 5.0


def test_derivative_opposes_motion_not_setpoint_jump():
    pid = PID(PIDGains(kd=1.0))
    pid.update(0.0, 0.0, 0.1)          # establish baseline
    out = pid.update(0.0, 1.0, 0.1)    # measurement moved +1 over dt=0.1
    # derivative-on-measurement => -kd * (1-0)/0.1 = -10
    assert math.isclose(out, -10.0, rel_tol=1e-6)
