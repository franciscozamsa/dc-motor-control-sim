import numpy as np

from dcmotor.cascade import default_cascade, simulate
from dcmotor.demo import contact_load, grip_reference
from dcmotor.model import Motor
from dcmotor.pid import PID
from dcmotor.response import open_loop


def test_open_loop_steady_state_matches_the_model():
    m = Motor()
    t, y = open_loop(m, volts=12.0, t_end=10.0)
    assert abs(y["speed"][-1] - 12.0 * m.dc_gain()) < 1e-4
    assert abs(y["torque"][-1] - m.Km * y["current"][-1]) < 1e-12


def test_table_1_motor_gives_about_12_mnm_at_12_v():
    _, y = open_loop(Motor(), volts=12.0, t_end=10.0)
    assert abs(y["torque"][-1] - 0.012) < 1e-5


def test_open_loop_angle_has_no_upper_bound():
    t, y = open_loop(Motor(), volts=12.0, t_end=10.0)
    slope = (y["angle"][-1] - y["angle"][-201]) / (t[-1] - t[-201])
    assert abs(slope - y["speed"][-1]) < 1e-4
    assert y["angle"][-1] > y["angle"][-201] > y["angle"][1000]


def test_poles_are_stable_and_real():
    p = Motor().poles()
    assert np.all(p.real < 0)
    assert np.all(np.abs(p.imag) < 1e-9)


def test_load_torque_lowers_steady_speed():
    m = Motor()
    _, free = open_loop(m, volts=12.0, t_end=10.0)
    _, loaded = open_loop(m, volts=12.0, t_end=10.0, load=0.005)
    assert loaded["speed"][-1] < free["speed"][-1]


def test_cascade_reaches_the_grip_angle_within_voltage_limits():
    m = Motor()
    t, o = simulate(m, default_cascade(m), grip_reference, t_end=10.0, load=contact_load)
    k = int(3.9 / 1e-3)
    assert abs(o["angle"][k] - 0.25) < 0.005
    assert np.abs(o["voltage"]).max() <= 12.0 + 1e-9


def test_cascade_holds_against_contact_load():
    m = Motor()
    t, o = simulate(m, default_cascade(m), grip_reference, t_end=10.0, load=contact_load)
    window = slice(int(4.5 / 1e-3), int(6.0 / 1e-3))
    assert np.abs(o["angle"][window] - 0.25).max() < 0.005
    assert abs(o["torque"][int(5.9 / 1e-3)] - 0.008) < 5e-4


def test_current_changes_polarity_to_open_the_gripper():
    m = Motor()
    t, o = simulate(m, default_cascade(m), grip_reference, t_end=10.0, load=contact_load)
    assert o["current"][int(3.0 / 1e-3)] > 1.0
    assert o["current"][int(9.0 / 1e-3)] < -1.0
    assert abs(o["angle"][-1]) < 0.005


def test_anti_windup_bounds_the_integral_under_saturation():
    with_aw = PID(kp=1.0, ki=10.0, u_min=-1.0, u_max=1.0, anti_windup=True)
    without = PID(kp=1.0, ki=10.0, u_min=-1.0, u_max=1.0, anti_windup=False)
    for _ in range(1000):
        with_aw.step(5.0, 0.0)
        without.step(5.0, 0.0)
    assert with_aw.integral < without.integral
    assert with_aw.integral < 1.0
