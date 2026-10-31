from dataclasses import dataclass

import numpy as np
from scipy import signal


@dataclass
class PID:
    kp: float
    ki: float
    kd: float = 0.0
    ts: float = 1e-3
    u_min: float = -12.0
    u_max: float = 12.0
    anti_windup: bool = True
    kb: float = 0.0
    d_filter: float = 0.01

    def __post_init__(self):
        self.integral = 0.0
        self._prev_meas = None
        self._d_state = 0.0
        if self.kb == 0.0:
            self.kb = self.ki / max(self.kp, 1e-12)

    def reset(self):
        self.integral = 0.0
        self._prev_meas = None
        self._d_state = 0.0

    def step(self, setpoint, measurement):
        error = setpoint - measurement
        if self._prev_meas is None:
            self._prev_meas = measurement
        raw_d = -(measurement - self._prev_meas) / self.ts
        alpha = self.ts / (self.d_filter + self.ts)
        self._d_state += alpha * (raw_d - self._d_state)
        self._prev_meas = measurement

        u_unsat = self.kp * error + self.integral + self.kd * self._d_state
        u = float(np.clip(u_unsat, self.u_min, self.u_max))
        self.integral += self.ki * error * self.ts
        if self.anti_windup:
            self.integral += self.kb * (u - u_unsat) * self.ts
        return u


def simulate(motor, controller, setpoint, t_end=0.5, ts=1e-3, mode="speed",
             load=None, setpoint_fn=None):
    """Closed loop with a zero-order-hold plant. `mode` is "speed" or "position".
    `load` is a function t -> load torque. Returns t, output, voltage."""
    A, B, C, D = motor.state_space()
    Ad, Bd, _, _, _ = signal.cont2discrete((A, B, C, D), ts, method="zoh")
    n = int(round(t_end / ts))
    t = np.arange(n) * ts
    x = np.zeros(3)
    out = np.zeros(n)
    volts = np.zeros(n)
    idx = 0 if mode == "speed" else 1
    controller.ts = ts
    controller.reset()
    for k in range(n):
        y = C[idx] @ x
        out[k] = y
        sp = setpoint_fn(t[k]) if setpoint_fn else setpoint
        u = controller.step(sp, y)
        volts[k] = u
        tl = load(t[k]) if load else 0.0
        x = Ad @ x + Bd @ np.array([u, tl])
    return t, out, volts
