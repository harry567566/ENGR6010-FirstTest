"""Collect real data from a Unitree Go2: front-camera frames plus robot state.

READ-ONLY. This script never sends a command, so the robot will not move.
It is the real-robot version of q4_sim/record_sim.py and saves the same kind
of record (without depth or segmentation, which a real camera does not give):

    frame_XXXX.jpg     front camera image (from VideoClient.GetImageSample)
    frames.csv         capture time of every frame
    state.csv          rt/lowstate at full rate: 12 joint angles and speeds,
                       foot forces, IMU, battery; plus the body velocity and
                       height estimated by the robot (rt/sportmodestate)

Run it in the Ubuntu VM with the robot connected over Ethernet:
    python3 collect_real.py enp0s2 --secs 30 --fps 5
The first argument is the VM network interface that is bridged to the robot.
While it records, someone can walk the dog with the remote control.
"""
import argparse
import csv
import os
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

JOINTS = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("iface", help="network interface bridged to the robot, e.g. enp0s2")
    ap.add_argument("--secs", type=float, default=30.0)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join("data", time.strftime("real_run_%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)

    ChannelFactoryInitialize(0, a.iface)
    lock = threading.Lock()
    rows, high = [], {"vel": [float("nan")] * 3, "pos": [float("nan")] * 3}

    def on_high(msg: SportModeState_):
        with lock:
            high["vel"] = [float(v) for v in msg.velocity]
            high["pos"] = [float(p) for p in msg.position]

    def on_low(msg: LowState_):
        t = time.time()
        q = [msg.motor_state[i].q for i in range(12)]
        dq = [msg.motor_state[i].dq for i in range(12)]
        with lock:
            rows.append([f"{t:.4f}"] + q + dq + list(msg.foot_force)[:4]
                        + list(msg.imu_state.rpy) + list(msg.imu_state.gyroscope)
                        + [msg.power_v] + high["vel"] + high["pos"])

    hs = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    hs.Init(on_high, 10)
    ls = ChannelSubscriber("rt/lowstate", LowState_)
    ls.Init(on_low, 10)

    cam = VideoClient()
    cam.SetTimeout(3.0)
    cam.Init()

    frames, t_end, k = [], time.time() + a.secs, 0
    print(f"recording {a.secs:.0f} s into {out} (the robot will not be commanded)")
    while time.time() < t_end:
        t0 = time.time()
        code, data = cam.GetImageSample()
        if code == 0:
            name = f"frame_{k:04d}.jpg"
            with open(os.path.join(out, name), "wb") as f:
                f.write(bytes(data))
            frames.append((name, f"{t0:.4f}"))
            k += 1
        else:
            print("camera error code", code)
        time.sleep(max(0.0, 1.0 / a.fps - (time.time() - t0)))

    with open(os.path.join(out, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "time"])
        w.writerows(frames)
    with lock:
        state = list(rows)
    with open(os.path.join(out, "state.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time"] + [f"q_{j}" for j in JOINTS] + [f"dq_{j}" for j in JOINTS]
                   + ["foot_FR", "foot_FL", "foot_RR", "foot_RL", "roll", "pitch", "yaw",
                      "gyro_x", "gyro_y", "gyro_z", "battery_v",
                      "body_vx", "body_vy", "body_vz", "body_x", "body_y", "body_z"])
        w.writerows(state)
    dur = float(state[-1][0]) - float(state[0][0]) if len(state) > 1 else 0
    print(f"{len(frames)} camera frames, {len(state)} state messages "
          f"({len(state) / max(dur, 1e-6):.0f} Hz) -> {out}")


if __name__ == "__main__":
    main()
