from dataclasses import dataclass

import numpy as np
from scipy import signal


@dataclass(frozen=True)
class Motor:
    R: float = 1.2
    L: float = 0.002
    J: float = 2e-5
    b: float = 1e-5
    Kt: float = 0.03
    Ke: float = 0.03

    def state_space(self):
        """States [i, w, theta]; inputs [V, T_load]; outputs [w, theta]."""
        A = np.array([[-self.R / self.L, -self.Ke / self.L, 0.0],
                      [self.Kt / self.J, -self.b / self.J, 0.0],
                      [0.0, 1.0, 0.0]])
        B = np.array([[1.0 / self.L, 0.0],
                      [0.0, -1.0 / self.J],
                      [0.0, 0.0]])
        C = np.array([[0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]])
        D = np.zeros((2, 2))
        return A, B, C, D

    def speed_tf(self):
        """Voltage to angular speed: Kt / ((J s + b)(L s + R) + Kt Ke)."""
        den = np.polymul([self.J, self.b], [self.L, self.R])
        den = np.polyadd(den, [self.Kt * self.Ke])
        return signal.TransferFunction([self.Kt], den)

    def dc_gain(self):
        return self.Kt / (self.b * self.R + self.Kt * self.Ke)

    def poles(self):
        return np.roots(self.speed_tf().den)
