from dataclasses import dataclass

import numpy as np
from scipy import signal


@dataclass(frozen=True)
class Motor:
    """Defaults are Table 1 of the paper (R, L, B, J, Km, Ka)."""
    R: float = 10.0
    L: float = 1e-3
    J: float = 0.01
    B: float = 0.1
    Km: float = 0.01
    Ka: float = 0.01

    def state_space(self):
        """States [i, w, theta]; inputs [v, T_load]; outputs [i, w, theta, T].

        Equations (1) to (4) of the paper, plus a load torque on the mechanical
        side that the paper leaves out of its open-loop model.
        """
        A = np.array([[-self.R / self.L, -self.Ka / self.L, 0.0],
                      [self.Km / self.J, -self.B / self.J, 0.0],
                      [0.0, 1.0, 0.0]])
        Bm = np.array([[1.0 / self.L, 0.0],
                       [0.0, -1.0 / self.J],
                       [0.0, 0.0]])
        C = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0],
                      [self.Km, 0.0, 0.0]])
        D = np.zeros((4, 2))
        return A, Bm, C, D

    def speed_tf(self):
        """Voltage to angular speed: Km / ((J s + B)(L s + R) + Km Ka)."""
        den = np.polymul([self.J, self.B], [self.L, self.R])
        den = np.polyadd(den, [self.Km * self.Ka])
        return signal.TransferFunction([self.Km], den)

    def dc_gain(self):
        return self.Km / (self.B * self.R + self.Km * self.Ka)

    def poles(self):
        return np.roots(self.speed_tf().den)
