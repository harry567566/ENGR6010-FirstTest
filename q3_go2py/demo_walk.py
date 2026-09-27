"""Question 3: Go2Py's walk-these-ways RL controller driving a simulated Go2.

Go2Py (machines-in-motion/Go2Py) is a Python interface for the Unitree Go2
with a MuJoCo simulator and pre-trained walking policies. Of the five demo repos
in the test it is the only one that runs without the real robot. Two things had
to be fixed to run it on a Mac, both without editing Go2Py's files:

  1. its walk-these-ways checkpoint was pickled with CUDA tensors, which cannot
     be loaded without an NVIDIA GPU (go2py_compat.py maps them to the CPU)
  2. its "front_camera" actually looks out of the dog's left side
     (q4_sim/scene.py re-points it; this demo does not need the camera)

    python demo_walk.py              record a video and a tracking plot
    mjpython demo_walk.py --view     watch it live in the MuJoCo viewer (macOS)
"""
import argparse
import os
import time

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import go2py_compat  # noqa: F401,E402
from Go2Py.sim.mujoco import Go2Sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
# (start s, vx, vy, yaw rate, label)
SCRIPT = [(0.0, 0.0, 0.0, 0.0, "stand"),
          (1.0, 0.6, 0.0, 0.0, "forward"),
          (6.0, 0.4, 0.0, 0.8, "walk + turn"),
          (10.0, 0.0, 0.3, 0.0, "sideways"),
          (13.0, -0.3, 0.0, 0.0, "backward"),
          (16.0, 0.0, 0.0, 0.0, "stop")]
DURATION = 17.0


def cmd_at(t):
    cur = SCRIPT[0]
    for row in SCRIPT:
        if t >= row[0]:
            cur = row
    return cur[1:4]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    a = ap.parse_args()
    sim = Go2Sim(mode="highlevel", render=a.view)
    m, d = sim.model, sim.data
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    writer = rend = None
    if not a.view:
        m.vis.global_.offwidth, m.vis.global_.offheight = 960, 540
        rend = mujoco.Renderer(m, 540, 960)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = m.body("base").id
        cam.distance, cam.azimuth, cam.elevation = 2.2, 140, -20
        writer = cv2.VideoWriter(os.path.join(HERE, "results", "q3_go2py_walk.mp4"),
                                 cv2.VideoWriter_fourcc(*"mp4v"), 30, (960, 540))
    log, stills = [], {}
    n = int(DURATION / sim.dt)
    t0 = time.time()
    for k in range(n):
        t = k * sim.dt
        vx, vy, wz = cmd_at(t)
        sim.step(vx, vy, wz)
        if k % 10 == 0:
            v = sim.getLinVel().ravel()
            pos, _ = sim.getPose()
            log.append((t, vx, vy, wz, v[0], v[1], d.qvel[5], pos[0], pos[1], pos[2]))
        if writer is not None and k % int(1 / (30 * sim.dt)) == 0:
            rend.update_scene(d, cam)
            frame = rend.render()
            writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            for tt in (0.5, 4.0, 8.0, 11.5, 14.5):
                if abs(t - tt) < 0.02 and tt not in stills:
                    stills[tt] = frame.copy()
        if a.view:
            if not sim.viewer.is_running():
                break
            time.sleep(max(0.0, sim.dt - 1e-4))
    if writer is None:
        return
    writer.release()
    L = np.array(log)
    print(f"simulated {DURATION:.0f} s in {time.time() - t0:.1f} s")
    for i, row in enumerate(SCRIPT):
        s0 = row[0] + 1.0
        s1 = SCRIPT[i + 1][0] if i + 1 < len(SCRIPT) else DURATION
        sel = (L[:, 0] >= s0) & (L[:, 0] < s1)
        if sel.any():
            e = L[sel]
            print(f"  {row[4]:<12} command ({row[1]:+.1f}, {row[2]:+.1f}, {row[3]:+.1f})   "
                  f"actual mean ({e[:, 4].mean():+.2f}, {e[:, 5].mean():+.2f}, {e[:, 6].mean():+.2f})"
                  f"   lowest body {e[:, 9].min():.3f} m")

    fig = plt.figure(figsize=(16, 6.4))
    gs = fig.add_gridspec(2, 5, height_ratios=[1, 1.1])
    for c, (tt, img) in enumerate(sorted(stills.items())):
        ax = fig.add_subplot(gs[0, c])
        ax.imshow(img)
        ax.set_title(f"t = {tt:.1f} s")
        ax.axis("off")
    for c, (idx, cmd_i, title) in enumerate([(4, 1, "forward speed (m/s)"),
                                             (5, 2, "sideways speed (m/s)"),
                                             (6, 3, "yaw rate (rad/s)")]):
        ax = fig.add_subplot(gs[1, c + (1 if c else 0)] if False else gs[1, c])
        ax.plot(L[:, 0], L[:, idx], color="#3070b0", label="actual")
        ax.plot(L[:, 0], L[:, cmd_i], "k--", lw=1, label="command")
        ax.set_title(title)
        ax.set_xlabel("time (s)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    ax = fig.add_subplot(gs[1, 3:])
    ax.plot(L[:, 7], L[:, 8], color="#3b9668")
    ax.set_aspect("equal")
    ax.set_title("path seen from above (m)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(HERE, "results", "q3_go2py_walk.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("figure ->", os.path.relpath(out, HERE))


if __name__ == "__main__":
    main()
