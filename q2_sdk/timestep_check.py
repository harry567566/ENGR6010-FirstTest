"""Why the simulated Go2 slid away while "standing" in unitree_mujoco.

With the simulator's default 5 ms physics step, the stand example's PD gains
(kp = 60, kd = 5) make the dog creep backwards about 1 m/s even though every
joint target is fixed. This script repeats the stand example's joint targets
directly in MuJoCo (no DDS, no threads) with the same robot model, so the
physics step is the only thing that changes.

    python timestep_check.py        -> results/q2_timestep.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "third_party", "unitree_mujoco", "unitree_robots", "go2", "scene.xml")
# the four target poses of go2_stand_example.py
T1 = np.array([0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65])
T2 = np.array([0.0, 0.67, -1.3] * 4)
T3 = np.array([-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65])


def run(dt, secs=6.0, kp=60.0, kd=5.0):
    m = mujoco.MjModel.from_xml_path(SCENE)
    m.opt.timestep = dt
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    nu = m.nu
    start = d.sensordata[:nu].copy()
    ts, xs, zs = [], [], []
    for k in range(int(secs / dt)):
        t = k * dt
        if t < 1:
            q = (1 - t) * start + t * T1
        elif t < 2:
            q = (2 - t) * T1 + (t - 1) * T2
        elif t < 4:
            q = T2
        else:
            s = min((t - 4) / 1.8, 1)
            q = (1 - s) * T2 + s * T3
        d.ctrl[:] = kp * (q - d.sensordata[:nu]) - kd * d.sensordata[nu:2 * nu]
        mujoco.mj_step(m, d)
        if k % max(1, int(0.02 / dt)) == 0:
            ts.append(t)
            xs.append(d.qpos[0])
            zs.append(d.qpos[2])
    return np.array(ts), np.array(xs), np.array(zs)


def main():
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.6))
    for dt, col in [(0.005, "#b03030"), (0.002, "#3070b0"), (0.001, "#3b9668")]:
        t, x, z = run(dt)
        ax[0].plot(t, x, color=col, label=f"step {dt * 1000:.0f} ms")
        ax[1].plot(t, z, color=col, label=f"step {dt * 1000:.0f} ms")
        print(f"step {dt * 1000:.0f} ms: moved {x[-1] - x[0]:+.2f} m in x, "
              f"highest body {z.max():.3f} m")
    ax[0].set_ylabel("body x (m)")
    ax[0].set_title("where the body is (all joint targets are fixed)")
    ax[1].set_ylabel("body height (m)")
    ax[1].set_title("stand example: up, hold, then the last low pose")
    for a in ax:
        a.set_xlabel("time (s)")
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    out = os.path.join(HERE, "results", "q2_timestep.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print("figure ->", os.path.relpath(out, HERE))


if __name__ == "__main__":
    main()
