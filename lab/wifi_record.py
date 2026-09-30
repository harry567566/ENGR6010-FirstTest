"""Record front-camera frames plus robot state from a REAL Go2 over Wi-Fi. READ-ONLY.
The Wi-Fi twin of collect_real.py, and it writes the same files with the same columns:

    frame_XXXX.jpg     front camera image
    frames.csv         capture time of every frame
    state.csv          rt/lf/lowstate (12 joint angles and speeds, foot forces, IMU, battery)
                       plus the body velocity and position estimated by the robot (rt/lf/sportmodestate)

    .venv_wifi/bin/python lab/wifi_record.py ap --secs 20 --fps 5 --out lab/data/wifi_run
While it records, someone can walk the dog with the remote control.
"""
import argparse
import asyncio
import csv
import os
import time

from wifi_common import connect, disconnect, watch_for_stop

JOINTS = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="'ap' (the dog's own Wi-Fi) or the dog's IP on a shared router")
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join("data", time.strftime("wifi_run_%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)

    rows, frames = [], []
    high = {"vel": [float("nan")] * 3, "pos": [float("nan")] * 3}
    recording = {"on": True}

    def on_sport(message):
        d = message.get("data", {})
        high["vel"] = [float(v) for v in d.get("velocity", [float("nan")] * 3)]
        high["pos"] = [float(p) for p in d.get("position", [float("nan")] * 3)]

    def on_low(message):
        d = message.get("data", {})
        motors = (d.get("motor_state") or [])[:12]
        if len(motors) < 12:
            return
        imu = d.get("imu_state", {})
        rows.append([f"{time.time():.4f}"] + [m.get("q", 0.0) for m in motors] + [m.get("dq", 0.0) for m in motors]
                    + list(d.get("foot_force", [0, 0, 0, 0]))[:4] + list(imu.get("rpy", [0, 0, 0]))
                    + list(imu.get("gyroscope", [0, 0, 0])) + [d.get("power_v", 0.0)] + high["vel"] + high["pos"])

    async def on_track(track):
        last = 0.0
        while recording["on"]:
            frame = await track.recv()
            t = time.time()
            if t - last >= 1.0 / a.fps:
                last = t
                name = f"frame_{len(frames):04d}.jpg"
                frame.to_image().save(os.path.join(out, name), "JPEG", quality=92)
                frames.append((name, f"{t:.4f}"))

    stop_event = asyncio.Event()
    watch_for_stop(stop_event)
    conn = await connect(a.target)
    try:
        conn.datachannel.pub_sub.subscribe("rt/lf/sportmodestate", on_sport)
        conn.datachannel.pub_sub.subscribe("rt/lf/lowstate", on_low)
        conn.video.add_track_callback(on_track)
        conn.video.switchVideoChannel(True)
        print(f"recording {a.secs:.0f} s into {out} (the robot will not be commanded)", flush=True)
        try:  # a time limit or STOP ends the recording early; what was recorded is still saved
            await asyncio.wait_for(stop_event.wait(), a.secs)
            print("stopped early; saving what was recorded")
        except asyncio.TimeoutError:
            pass
        recording["on"] = False
        conn.video.switchVideoChannel(False)
    finally:
        await disconnect(conn)

    with open(os.path.join(out, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "time"])
        w.writerows(frames)
    with open(os.path.join(out, "state.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time"] + [f"q_{j}" for j in JOINTS] + [f"dq_{j}" for j in JOINTS]
                   + ["foot_FR", "foot_FL", "foot_RR", "foot_RL", "roll", "pitch", "yaw",
                      "gyro_x", "gyro_y", "gyro_z", "battery_v",
                      "body_vx", "body_vy", "body_vz", "body_x", "body_y", "body_z"])
        w.writerows(rows)
    dur = float(rows[-1][0]) - float(rows[0][0]) if len(rows) > 1 else 0
    print(f"{len(frames)} camera frames, {len(rows)} state messages ({len(rows) / max(dur, 1e-6):.0f} Hz) -> {out}")


if __name__ == "__main__":
    asyncio.run(main())
