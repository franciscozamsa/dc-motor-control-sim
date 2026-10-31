import numpy as np

from dcmotor.model import Motor
from dcmotor.pid import PID, simulate
from dcmotor.response import load_step_speed, metrics, step_speed


def test_step_final_value_matches_dc_gain():
    m = Motor()
    t, y = step_speed(m, volts=12.0, t_end=1.0)
    assert abs(y[-1] - 12.0 * m.dc_gain()) < 0.5


def test_poles_are_stable_and_real():
    p = Motor().poles()
    assert np.all(p.real < 0)
    assert np.all(np.abs(p.imag) < 1e-9)


def test_load_torque_lowers_steady_speed():
    m = Motor()
    _, free = step_speed(m, volts=12.0, t_end=1.0)
    _, loaded = load_step_speed(m, volts=12.0, torque=0.02, t_end=1.0)
    assert loaded[-1] < free[-1]


def test_closed_loop_removes_steady_state_error_and_rejects_load():
    m = Motor()
    pi = PID(kp=0.1, ki=4.0)
    t, w, _ = simulate(m, pi, 200.0, t_end=1.0, load=lambda t: 0.02 if t > 0.4 else 0.0)
    assert abs(w[399] - 200.0) < 1.0
    assert abs(w[-1] - 200.0) < 0.5


def test_voltage_is_always_within_limits():
    m = Motor()
    c = PID(kp=0.5, ki=40.0)
    _, _, u = simulate(m, c, 350.0, t_end=0.6)
    assert u.max() <= 12.0 + 1e-9 and u.min() >= -12.0 - 1e-9


def test_anti_windup_reduces_overshoot_without_diverging():
    m = Motor()
    with_aw = PID(kp=0.5, ki=40.0, anti_windup=True)
    without = PID(kp=0.5, ki=40.0, anti_windup=False)
    t, w1, _ = simulate(m, with_aw, 350.0, t_end=0.8)
    _, w2, _ = simulate(m, without, 350.0, t_end=0.8)
    o1 = metrics(t, w1, 350.0)["overshoot_pct"]
    o2 = metrics(t, w2, 350.0)["overshoot_pct"]
    assert o1 < o2
    assert np.all(np.isfinite(w1))


def test_position_loop_settles_on_setpoint():
    m = Motor()
    c = PID(kp=0.5, ki=0.0, kd=0.01)
    t, th, _ = simulate(m, c, 1.0, t_end=1.0, mode="position")
    assert abs(th[-1] - 1.0) < 0.01
