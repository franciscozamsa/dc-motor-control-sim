import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .cascade import default_cascade, simulate
from .model import Motor
from .response import metrics, open_loop

DOCS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs")

CLOSE_AT, OPEN_AT, GRIP_ANGLE = 1.0, 7.0, 0.25
LOAD_FROM, LOAD_TO, LOAD_TORQUE = 4.0, 6.0, 0.008


def grip_reference(t):
    return GRIP_ANGLE if CLOSE_AT <= t < OPEN_AT else 0.0


def contact_load(t):
    return LOAD_TORQUE if LOAD_FROM <= t < LOAD_TO else 0.0


def main():
    os.makedirs(DOCS, exist_ok=True)
    motor = Motor()

    t, y = open_loop(motor, volts=12.0, t_end=10.0)
    fig, axes = plt.subplots(2, 2, figsize=(9, 5.4), sharex=True)
    panels = (("current", "current (A)"), ("speed", "angular speed (rad/s)"),
              ("angle", "angular position (rad)"), ("torque", "output torque (N m)"))
    for ax, (key, label) in zip(axes.ravel(), panels):
        ax.plot(t, y[key])
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].axhline(0.012, color="gray", ls="--", lw=0.8)
    axes[1, 1].text(0.3, 0.0106, "12 mN m", fontsize=8, color="gray")
    fig.suptitle("Open loop, 12 V step (Table 1 parameters)")
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "open_loop.png"), dpi=130)
    plt.close(fig)

    t, o = simulate(motor, default_cascade(motor), grip_reference, t_end=10.0, load=contact_load)
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t, o["angle"], label="angle")
    axes[0].plot(t, [grip_reference(x) for x in t], color="gray", ls="--", lw=0.8, label="setpoint")
    axes[0].set_ylabel("angle (rad)")
    axes[0].legend(loc="center right")
    axes[1].plot(t, o["torque"] * 1e3, label="output torque")
    axes[1].plot(t, [contact_load(x) * 1e3 for x in t], color="gray", ls="--", lw=0.8, label="load torque")
    axes[1].set_ylabel("torque (mN m)")
    axes[1].legend(loc="upper right")
    axes[2].plot(t, o["voltage"])
    axes[2].set_ylabel("voltage (V)")
    axes[2].set_xlabel("time (s)")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title("Cascade control: close, hold against a contact load, open")
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "cascade.png"), dpi=130)
    plt.close(fig)

    print(f"DC gain {motor.dc_gain():.4f} rad/s per V, poles {[round(float(p.real), 1) for p in motor.poles()]}")
    print(f"Open loop, 12 V: torque {y['torque'][-1] * 1e3:.2f} mN m, speed {y['speed'][-1]:.4f} rad/s, "
          f"angle after 10 s {y['angle'][-1]:.3f} rad (no upper bound)")

    i0, i1 = int(CLOSE_AT / 1e-3), int(LOAD_FROM / 1e-3)
    m = metrics(t[i0:i1] - CLOSE_AT, o["angle"][i0:i1], GRIP_ANGLE)
    print(f"Cascade, close to {GRIP_ANGLE} rad: rise {m['rise_time_s']:.2f} s, overshoot {m['overshoot_pct']:.1f} %, "
          f"settling {m['settling_time_s']:.2f} s, error {m['steady_state_error'] * 1e3:.2f} mrad")
    j0, j1 = int(LOAD_FROM / 1e-3), int(LOAD_TO / 1e-3)
    print(f"Cascade, {LOAD_TORQUE * 1e3:.0f} mN m load: angle dip {(GRIP_ANGLE - o['angle'][j0:j1].min()) * 1e3:.1f} mrad, "
          f"holding torque {o['torque'][j1 - 1] * 1e3:.2f} mN m")
    print(f"Cascade, opening: current reaches {o['current'].min():.2f} A (polarity reversal), "
          f"final angle {o['angle'][-1] * 1e3:.1f} mrad")


if __name__ == "__main__":
    main()
