from dataclasses import dataclass

import numpy as np
from scipy import signal

from .pid import PID


@dataclass
class Cascade:
    """Three nested loops: position -> speed -> current -> voltage.

    Each controller's output is the next one's setpoint, as in the paper's
    Figure 9. The output torque is Km times the current, so the inner loop
    is also the torque loop.
    """
    position: PID
    speed: PID
    current: PID

    def reset(self, ts):
        for c in (self.position, self.speed, self.current):
            c.ts = ts
            c.reset()

    def step(self, theta_ref, theta, w, i):
        w_ref = self.position.step(theta_ref, theta)
        i_ref = self.speed.step(w_ref, w)
        return self.current.step(i_ref, i)


def default_cascade(motor, v_max=12.0):
    """Gains tuned by hand for the paper's motor; limits follow from v_max."""
    i_max = v_max / motor.R
    w_max = motor.Km * v_max / (motor.B * motor.R + motor.Km * motor.Ka)
    return Cascade(
        position=PID(kp=5.0, ki=0.0, u_min=-w_max, u_max=w_max),
        speed=PID(kp=20.0, ki=100.0, u_min=-i_max, u_max=i_max),
        current=PID(kp=5.0, ki=2000.0, u_min=-v_max, u_max=v_max),
    )


def simulate(motor, cascade, theta_ref, t_end=10.0, ts=1e-3, load=None):
    """Closed loop on a zero-order-hold plant.

    `theta_ref` and `load` are functions of time (a number is a constant).
    Returns t and a dict with the same signals as `response.open_loop`, plus
    the commanded voltage.
    """
    ref = theta_ref if callable(theta_ref) else (lambda t, v=theta_ref: v)
    A, B, C, D = motor.state_space()
    Ad, Bd, _, _, _ = signal.cont2discrete((A, B, C, D), ts, method="zoh")
    n = int(round(t_end / ts))
    t = np.arange(n) * ts
    x = np.zeros(3)
    out = {k: np.zeros(n) for k in ("current", "speed", "angle", "torque", "voltage")}
    cascade.reset(ts)
    for k in range(n):
        i, w, theta = x
        v = cascade.step(ref(t[k]), theta, w, i)
        tl = load(t[k]) if load else 0.0
        out["current"][k], out["speed"][k], out["angle"][k] = i, w, theta
        out["torque"][k] = motor.Km * i
        out["voltage"][k] = v
        x = Ad @ x + Bd @ np.array([v, tl])
    return t, out
