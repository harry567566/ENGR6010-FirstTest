"""Run the unitree_rl_gym Go2 walking policy in MuJoCo (sim-to-sim) and grade it.

The official unitree_rl_gym only ships humanoid policies. A pull request
(unitree_rl_gym #62, fork UESTC-REVERIE/unitree_rl_gym_go2) adds a Go2 policy
trained in Isaac Gym and deploy_go2.py, which runs it in MuJoCo with a viewer.
Training needs Isaac Gym (Linux + NVIDIA GPU); running the trained policy does
not, so this works on a Mac.

This script uses the same config, policy, robot model, PD control and
observation code as deploy_go2.py, but without a window, so a whole run can be
recorded and measured. It also tests one detail I noticed in deploy_go2.py:
it gives the policy the body's linear velocity in WORLD coordinates
(d.qvel[:3]), while Isaac Gym trains on velocity in the robot's own BODY frame.
The two agree only while the robot faces +x.

    python rl_gym_go2_sim.py                  both versions, video + figure
    mjpython rl_gym_go2_sim.py --view         watch it live (body-frame version)
"""
import argparse
import os
import sys
import time

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "third_party", "unitree_rl_gym_go2"))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "deploy", "deploy_mujoco"))
from deploy_go2 import get_gravity_orientation, pd_control  # noqa: E402  (the fork's own code)

# (start s, vx m/s, vy m/s, yaw rate rad/s, label)
SCRIPT = [(0.0, 0.0, 0.0, 0.0, "stand"),
          (2.0, 0.5, 0.0, 0.0, "forward"),
          (7.0, 0.5, 0.0, 0.6, "walk + turn left"),
          (13.0, 0.0, 0.3, 0.0, "sideways"),
          (17.0, 0.0, 0.0, 0.0, "stop")]
DURATION = 19.0


def load_cfg():
    with open(os.path.join(ROOT, "deploy", "deploy_mujoco", "configs", "go2.yaml")) as f:
        c = yaml.load(f, Loader=yaml.FullLoader)
    for k in ("policy_path", "xml_path"):
        c[k] = c[k].replace("{LEGGED_GYM_ROOT_DIR}", ROOT)
    return c


def cmd_at(t):
    cur = SCRIPT[0]
    for row in SCRIPT:
        if t >= row[0]:
            cur = row
    return np.array(cur[1:4], np.float32), cur[4]


def run(frame="body", video=None, view=False):
    c = load_cfg()
    kps, kds = np.array(c["kps"], np.float32), np.array(c["kds"], np.float32)
    default = np.array(c["default_angles"], np.float32)
    cmd_scale = np.array(c["cmd_scale"], np.float32)
    m = mujoco.MjModel.from_xml_path(c["xml_path"])
    m.opt.timestep = c["simulation_dt"]
    d = mujoco.MjData(m)
    policy = torch.jit.load(c["policy_path"], map_location="cpu")
    action = np.zeros(c["num_actions"], np.float32)
    target = default.copy()
    obs = np.zeros(c["num_obs"], np.float32)

    writer, rend, cam = None, None, None
    if video:
        m.vis.global_.offwidth, m.vis.global_.offheight = 720, 480
        rend = mujoco.Renderer(m, 480, 720)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = 1
        cam.distance, cam.azimuth, cam.elevation = 2.0, 135, -20
        writer = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 30, (720, 480))
    viewer = mujoco.viewer.launch_passive(m, d) if view else None

    log = []
    n = int(DURATION / m.opt.timestep)
    for k in range(n):
        t = k * m.opt.timestep
        cmd, _ = cmd_at(t)
        d.ctrl[:] = pd_control(target, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
        mujoco.mj_step(m, d)
        if (k + 1) % c["control_decimation"] == 0:
            quat = d.qpos[3:7]
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, quat)
            R = R.reshape(3, 3)                       # body -> world
            lin_world = d.qvel[:3].copy()
            lin = R.T @ lin_world if frame == "body" else lin_world
            obs[:3] = lin * c["lin_vel_scale"]
            obs[3:6] = d.qvel[3:6] * c["ang_vel_scale"]   # free-joint angular velocity is already in the body frame
            obs[6:9] = get_gravity_orientation(quat)
            obs[9:12] = cmd * cmd_scale
            obs[12:24] = (d.qpos[7:] - default) * c["dof_pos_scale"]
            obs[24:36] = d.qvel[6:] * c["dof_vel_scale"]
            obs[36:48] = action
            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
            target = action * c["action_scale"] + default
            vb = R.T @ lin_world
            log.append((t, *cmd, vb[0], vb[1], d.qvel[5], d.qpos[2], d.qpos[0], d.qpos[1]))
        if writer is not None and k % int(1 / (30 * m.opt.timestep)) == 0:
            rend.update_scene(d, cam)
            writer.write(cv2.cvtColor(rend.render(), cv2.COLOR_RGB2BGR))
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            time.sleep(max(0.0, m.opt.timestep - 1e-4))
    if writer is not None:
        writer.release()
    return np.array(log)


def score(log):
    """RMS tracking error per phase, skipping the first 1 s after each command change."""
    t = log[:, 0]
    out = {}
    for i, row in enumerate(SCRIPT):
        t0 = row[0] + 1.0
        t1 = SCRIPT[i + 1][0] if i + 1 < len(SCRIPT) else DURATION
        sel = (t >= t0) & (t < t1)
        if not sel.any():
            continue
        e = log[sel]
        out[row[4]] = {"vx_rms": float(np.sqrt(np.mean((e[:, 4] - e[:, 1]) ** 2))),
                       "vy_rms": float(np.sqrt(np.mean((e[:, 5] - e[:, 2]) ** 2))),
                       "yaw_rate_rms": float(np.sqrt(np.mean((e[:, 6] - e[:, 3]) ** 2))),
                       "min_height": float(e[:, 7].min())}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="live window (run with mjpython on macOS)")
    a = ap.parse_args()
    if a.view:
        import mujoco.viewer  # noqa: F401
        run("body", view=True)
        return
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    logs = {}
    for frame in ("world", "body"):
        vid = os.path.join(HERE, "results", f"rl_gym_go2_{frame}.mp4")
        t0 = time.time()
        logs[frame] = run(frame, video=vid)
        s = score(logs[frame])
        print(f"\nvelocity given to the policy in the {frame.upper()} frame "
              f"({'as in deploy_go2.py' if frame == 'world' else 'as in Isaac Gym training'}), "
              f"{time.time() - t0:.0f}s:")
        for ph, v in s.items():
            print(f"  {ph:<18} vx err {v['vx_rms']:.3f}  vy err {v['vy_rms']:.3f}  "
                  f"yaw-rate err {v['yaw_rate_rms']:.3f}  lowest body {v['min_height']:.3f} m")
        np.save(os.path.join(HERE, "results", f"rl_gym_go2_{frame}.npy"), logs[frame])

    fig, ax = plt.subplots(1, 3, figsize=(17, 3.8))
    for frame, col in [("world", "#b03030"), ("body", "#3070b0")]:
        L = logs[frame]
        ax[0].plot(L[:, 0], L[:, 4], color=col, label=f"actual, {frame}-frame input")
        ax[1].plot(L[:, 0], L[:, 6], color=col, label=f"actual, {frame}-frame input")
        ax[2].plot(L[:, 8], L[:, 9], color=col, label=f"{frame}-frame input")
    L = logs["body"]
    ax[0].plot(L[:, 0], L[:, 1], "k--", lw=1, label="command")
    ax[1].plot(L[:, 0], L[:, 3], "k--", lw=1, label="command")
    ax[0].set_title("forward speed in the body frame (m/s)")
    ax[1].set_title("yaw rate (rad/s)")
    ax[2].set_title("path seen from above (m)")
    ax[2].set_aspect("equal")
    for a_ in ax[:2]:
        a_.set_xlabel("time (s)")
        for row in SCRIPT[1:]:
            a_.axvline(row[0], color="gray", lw=0.6, ls=":")
    for a_ in ax:
        a_.grid(alpha=0.3)
        a_.legend(fontsize=7)
    out = os.path.join(HERE, "results", "q4_rl_gym_go2.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print("\nfigure ->", os.path.relpath(out, HERE))


if __name__ == "__main__":
    main()
