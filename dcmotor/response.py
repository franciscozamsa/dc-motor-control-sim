import numpy as np
from scipy import signal


def step_speed(motor, volts=1.0, t_end=0.5, n=2000):
    t = np.linspace(0.0, t_end, n)
    t, y = signal.step(motor.speed_tf(), T=t)
    return t, volts * y


def load_step_speed(motor, volts, torque, t_end=0.5, n=2000):
    """Speed after a voltage step with a constant load torque applied at t = 0."""
    A, B, C, D = motor.state_space()
    sys = signal.StateSpace(A, B, C, D)
    t = np.linspace(0.0, t_end, n)
    u = np.tile([volts, torque], (n, 1))
    t, y, _ = signal.lsim(sys, U=u, T=t)
    return t, y[:, 0]


def metrics(t, y, setpoint):
    """Rise time (10 to 90 %), overshoot (%), 2 % settling time and steady-state error."""
    y = np.asarray(y, dtype=float)
    final = y[-1]
    lo, hi = 0.1 * setpoint, 0.9 * setpoint
    t_lo = t[np.argmax(y >= lo)] if np.any(y >= lo) else np.nan
    t_hi = t[np.argmax(y >= hi)] if np.any(y >= hi) else np.nan
    rise = t_hi - t_lo
    overshoot = max(0.0, (y.max() - setpoint) / setpoint * 100.0)
    outside = np.where(np.abs(y - setpoint) > 0.02 * abs(setpoint))[0]
    settling = t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(t) else (0.0 if not len(outside) else np.nan)
    return {"rise_time_s": float(rise), "overshoot_pct": float(overshoot),
            "settling_time_s": float(settling), "steady_state_error": float(setpoint - final)}
