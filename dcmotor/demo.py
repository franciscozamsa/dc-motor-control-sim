import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .model import Motor
from .pid import PID, simulate
from .response import load_step_speed, metrics, step_speed

DOCS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs")


def main():
    os.makedirs(DOCS, exist_ok=True)
    motor = Motor()

    t, y = step_speed(motor, volts=12.0, t_end=0.4)
    tl, yl = load_step_speed(motor, volts=12.0, torque=0.02, t_end=0.4)
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(t, y, label="12 V step, no load")
    ax.plot(tl, yl, label="12 V step, 0.02 N m load")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (rad/s)")
    ax.set_title("Open loop: speed response of the example motor")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "step.png"), dpi=130)
    plt.close(fig)

    rows = []
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))

    pi = PID(kp=0.1, ki=4.0)
    t, w, u = simulate(motor, pi, 200.0, t_end=0.4)
    axes[0].plot(t, w)
    axes[0].axhline(200, color="gray", ls="--", lw=0.8)
    axes[0].set_title("Speed, PI (setpoint 200 rad/s)")
    rows.append(("Speed PI, 200 rad/s", metrics(t, w, 200.0)))

    for aw, label in ((True, "anti-windup"), (False, "no anti-windup")):
        c = PID(kp=0.5, ki=40.0, anti_windup=aw)
        t, w, u = simulate(motor, c, 350.0, t_end=0.6)
        axes[1].plot(t, w, label=label)
        rows.append((f"Speed PI saturated, 350 rad/s, {label}", metrics(t, w, 350.0)))
    axes[1].axhline(350, color="gray", ls="--", lw=0.8)
    axes[1].set_title("Voltage limit 12 V: windup")
    axes[1].legend()

    pd = PID(kp=0.5, ki=0.0, kd=0.01)
    t, th, u = simulate(motor, pd, 1.0, t_end=1.0, mode="position")
    axes[2].plot(t, th)
    axes[2].axhline(1.0, color="gray", ls="--", lw=0.8)
    axes[2].set_title("Position, PD (setpoint 1 rad)")
    rows.append(("Position PD, 1 rad", metrics(t, th, 1.0)))

    for ax, yl in zip(axes, ("speed (rad/s)", "speed (rad/s)", "angle (rad)")):
        ax.set_xlabel("time (s)")
        ax.set_ylabel(yl)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "pid.png"), dpi=130)
    plt.close(fig)

    print(f"DC gain {motor.dc_gain():.2f} rad/s per V, poles {[round(float(p.real), 1) for p in motor.poles()]}")
    print(f"{'case':46s} {'rise (s)':>9s} {'overshoot %':>12s} {'settling (s)':>13s} {'ss error':>9s}")
    for name, m in rows:
        print(f"{name:46s} {m['rise_time_s']:9.3f} {m['overshoot_pct']:12.2f} {m['settling_time_s']:13.3f} {m['steady_state_error']:9.3f}")


if __name__ == "__main__":
    main()
