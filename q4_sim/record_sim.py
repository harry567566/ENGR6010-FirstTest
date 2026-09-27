"""Record synthetic data from a simulated Unitree Go2 walking through the test scene.

The dog is driven by Go2Py's pre-trained walk-these-ways RL policy. It follows
a fixed script of velocity commands, and every 0.1 s the front camera saves:

    rgb_XXXX.png     what the camera sees
    depth_XXXX.png   distance along the camera axis, in millimeters (uint16)
    seg_XXXX.png     which object is under each pixel (label ids in scene.LABELS)

meta.npz holds, for every frame, the camera pose, the intrinsics, the command,
the body velocity and the 12 joint angles. Because the scene is static and the
depth and camera poses are exact, the true optical flow between two frames can
be computed (see flow_gt.py). That is the ground truth used to grade the flow
methods in vision_pipeline.py.

A third-person video of the run is written as well.

    python record_sim.py                 record data/sim_run (about 1 minute)
    python record_sim.py --fps 10 --out data/sim_run
"""
import argparse
import os
import sys
import time

import cv2
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "q3_go2py"))
import go2py_compat  # noqa: F401,E402  (must come before Go2Py)
from Go2Py.sim.mujoco import Go2Sim  # noqa: E402

import scene  # noqa: E402

# (start time s, vx m/s, vy m/s, yaw rate rad/s, label)
SCRIPT = [
    (0.0, 0.0, 0.0, 0.0, "stand"),
    (1.0, 0.4, 0.0, 0.0, "forward"),        # objects grow as the dog walks at them
    (6.0, 0.0, 0.0, 0.8, "turn_left"),      # fast rotation: large image motion
    (8.0, 0.0, 0.0, -0.8, "turn_right"),    # and back again
    (10.0, 0.0, -0.2, 0.0, "sidestep"),     # sideways: near things move more than far things
    (13.0, 0.3, 0.0, 0.0, "forward"),
    (16.0, 0.0, 0.0, 0.0, "stop"),
]
DURATION = 17.0


def command_at(t):
    cur = SCRIPT[0]
    for row in SCRIPT:
        if t >= row[0]:
            cur = row
    return cur[1:4]


def intrinsics(model, cam_id, w, h):
    fovy = np.deg2rad(model.cam_fovy[cam_id])
    f = (h / 2) / np.tan(fovy / 2)
    return np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=float, default=10.0, help="front camera frames per second")
    ap.add_argument("--video-fps", type=float, default=30.0)
    ap.add_argument("--out", default=os.path.join(HERE, "data", "sim_run"))
    a = ap.parse_args()

    xml = scene.build_scene()
    sim = Go2Sim(mode="highlevel", render=False, xml_path=xml)
    m, d = sim.model, sim.data
    lut = scene.label_lookup(m)
    cam_id = m.camera("front_camera").id
    W, H = 640, 480
    K = intrinsics(m, cam_id, W, H)

    front = mujoco.Renderer(m, H, W)
    third = mujoco.Renderer(m, 720, 1280)
    tcam = mujoco.MjvCamera()
    tcam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    tcam.trackbodyid = m.body("base").id
    tcam.distance, tcam.azimuth, tcam.elevation = 2.4, 200, -22

    os.makedirs(a.out, exist_ok=True)
    vid_third = cv2.VideoWriter(os.path.join(a.out, "third_person.mp4"),
                                cv2.VideoWriter_fourcc(*"mp4v"), a.video_fps, (1280, 720))
    vid_front = cv2.VideoWriter(os.path.join(a.out, "front_camera.mp4"),
                                cv2.VideoWriter_fourcc(*"mp4v"), a.video_fps, (W, H))

    rec = {k: [] for k in ["t", "cmd", "label", "cam_pos", "cam_R", "base_pos", "base_quat",
                           "body_vel", "yaw_rate", "q", "dq"]}
    step_frame = int(round(1 / (a.fps * sim.dt)))
    step_video = int(round(1 / (a.video_fps * sim.dt)))
    n_steps = int(DURATION / sim.dt)
    t0, fi = time.time(), 0
    for k in range(n_steps):
        t = k * sim.dt
        vx, vy, wz = command_at(t)
        sim.step(vx, vy, wz)

        if k % step_video == 0:
            front.update_scene(d, camera=cam_id)
            vid_front.write(cv2.cvtColor(front.render(), cv2.COLOR_RGB2BGR))
            third.update_scene(d, tcam)
            vid_third.write(cv2.cvtColor(third.render(), cv2.COLOR_RGB2BGR))

        if k % step_frame == 0:
            front.update_scene(d, camera=cam_id)
            rgb = front.render()
            front.enable_depth_rendering()
            depth = front.render().copy()
            front.disable_depth_rendering()
            front.enable_segmentation_rendering()
            seg = front.render()[:, :, 0].copy()
            front.disable_segmentation_rendering()
            lab = np.where(seg >= 0, lut[np.clip(seg, 0, None)], scene.LABELS["background"])

            cv2.imwrite(os.path.join(a.out, f"rgb_{fi:04d}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(os.path.join(a.out, f"depth_{fi:04d}.png"),
                        np.clip(depth * 1000, 0, 65535).astype(np.uint16))
            cv2.imwrite(os.path.join(a.out, f"seg_{fi:04d}.png"), lab.astype(np.uint8))

            pos, quat = sim.getPose()
            rec["t"].append(t)
            rec["cmd"].append((vx, vy, wz))
            rec["label"].append([r[4] for r in SCRIPT if t >= r[0]][-1])
            rec["cam_pos"].append(d.cam_xpos[cam_id].copy())
            rec["cam_R"].append(d.cam_xmat[cam_id].reshape(3, 3).copy())
            rec["base_pos"].append(pos.copy())
            rec["base_quat"].append(quat.copy())
            rec["body_vel"].append(sim.getLinVel().ravel().copy())
            rec["yaw_rate"].append(float(d.qvel[5]))
            js = sim.getJointStates()
            rec["q"].append(js["q"].copy())
            rec["dq"].append(js["dq"].copy())
            fi += 1

    vid_third.release()
    vid_front.release()
    np.savez(os.path.join(a.out, "meta.npz"), K=K, width=W, height=H, fps=a.fps,
             labels=np.array(list(scene.LABELS.keys())),
             **{k: np.array(v) for k, v in rec.items()})
    bp = np.array(rec["base_pos"])
    print(f"{fi} frames in {time.time() - t0:.0f}s -> {a.out}")
    print(f"dog moved from {np.round(bp[0], 2)} to {np.round(bp[-1], 2)}, "
          f"lowest body height {bp[:, 2].min():.3f} m")


if __name__ == "__main__":
    main()
