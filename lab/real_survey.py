"""Walk the building and record everything the robot will give us. READ-ONLY.

    .venv_robot/bin/python lab/real_survey.py en7 --secs 180 --name floor2
    python3 real_survey.py enp0s2 --secs 300 --name corridor --fps 4

Someone drives the dog with the remote; this listens and writes. It never
sends a command, so the robot only does what the handler tells it to.

What it records, with one clock for all of it:

    state      rt/lowstate and rt/sportmodestate at full rate - IMU, joints,
               foot forces, battery, and the robot's own position estimate
    cloud      rt/utlidar/cloud (or cloud_deskewed) if this robot publishes
               one, saved as raw bytes plus the field layout, so nothing is
               lost by decoding it wrongly here
    odometry   on-board pose, if the firmware publishes it
    camera     front-camera JPEGs at --fps, through VideoClient

The point of recording raw and recording everything is that the robot stays
in the lab. Whatever is missing from this file cannot be gone back for.

Output: lab/data/survey_<name>_<time>/
    meta.json        what was recorded, rates, what was silent and why
    state.jsonl      one line per state sample
    odom.jsonl       one line per pose, if any
    clouds/NNNNN.npz raw point cloud frames
    frames/NNNNN.jpg camera
"""
import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np

DOMAIN_ID = 0
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "data")

# Order matters: the first one that exists is the one worth having.
# cloud_deskewed is motion-compensated and already in the odom frame, so it
# needs no deskewing from us; uslam/cloud_map, if the dog runs its own SLAM,
# is the finished map.
CLOUD_TOPICS = ["rt/uslam/cloud_map",
                "rt/utlidar/cloud_deskewed",
                "rt/utlidar/cloud"]
ODOM_TOPICS_ODOM = ["rt/uslam/localization/odom",
                    "rt/utlidar/robot_odom",
                    "rt/lio_sam_ros2/mapping/odometry"]
ODOM_TOPICS_POSE = ["rt/utlidar/robot_pose"]


def _idl(mod, cls):
    paths = {"unitree_go": "unitree_sdk2py.idl.unitree_go.msg.dds_",
             "sensor_msgs": "unitree_sdk2py.idl.sensor_msgs.msg.dds_",
             "geometry_msgs": "unitree_sdk2py.idl.geometry_msgs.msg.dds_",
             "nav_msgs": "unitree_sdk2py.idl.nav_msgs.msg.dds_"}
    try:
        m = __import__(paths[mod], fromlist=[cls])
        return getattr(m, cls)
    except Exception:
        return None


class Survey:
    def __init__(self, root, fps):
        self.root = root
        self.fps = fps
        self.t0 = time.time()
        self.lock = threading.Lock()
        self.n = {"state": 0, "cloud": 0, "odom": 0, "frame": 0,
                  "height": 0}
        self.cloud_dir = os.path.join(root, "clouds")
        self.frame_dir = os.path.join(root, "frames")
        os.makedirs(self.cloud_dir, exist_ok=True)
        os.makedirs(self.frame_dir, exist_ok=True)
        os.makedirs(os.path.join(root, "height"), exist_ok=True)
        self.state_f = open(os.path.join(root, "state.jsonl"), "w")
        self.odom_f = open(os.path.join(root, "odom.jsonl"), "w")
        self.cloud_meta = {}
        self.n_by_topic = {}
        self.last_state_write = 0.0

    def t(self):
        return round(time.time() - self.t0, 4)

    # ---------------------------------------------------------------- state
    def on_low(self, msg):
        now = time.time()
        # lowstate arrives at several hundred Hz; 50 Hz is plenty and keeps
        # the file readable
        if now - self.last_state_write < 0.02:
            return
        self.last_state_write = now
        try:
            row = {"t": self.t(), "src": "lowstate",
                   "rpy": [float(x) for x in msg.imu_state.rpy],
                   "gyro": [float(x) for x in msg.imu_state.gyroscope],
                   "acc": [float(x) for x in msg.imu_state.accelerometer],
                   "q": [float(m.q) for m in msg.motor_state[:12]],
                   "dq": [float(m.dq) for m in msg.motor_state[:12]],
                   "tau": [float(m.tau_est) for m in msg.motor_state[:12]],
                   "foot_force": [int(x) for x in msg.foot_force],
                   "soc": int(msg.bms_state.soc),
                   "power_v": float(msg.power_v)}
        except Exception as e:
            row = {"t": self.t(), "src": "lowstate", "error": str(e)}
        with self.lock:
            self.state_f.write(json.dumps(row) + "\n")
            self.n["state"] += 1

    def on_sport(self, msg):
        try:
            row = {"t": self.t(), "src": "sport", "mode": int(msg.mode),
                   "body_height": float(msg.body_height),
                   "position": [float(x) for x in msg.position],
                   "velocity": [float(x) for x in msg.velocity],
                   "yaw_speed": float(msg.yaw_speed)}
        except Exception as e:
            row = {"t": self.t(), "src": "sport", "error": str(e)}
        with self.lock:
            self.state_f.write(json.dumps(row) + "\n")

    # ---------------------------------------------------------------- cloud
    def on_cloud(self, msg, topic):
        """One callback per topic, and the topic is recorded with the data.

        These topics are NOT interchangeable. rt/utlidar/cloud is in the
        sensor frame, which on a Go2 is rotated about 165 degrees from the
        body; rt/utlidar/cloud_deskewed is motion-compensated and already in
        the odom frame. Writing both into one stream without saying which is
        which silently mixes two coordinate systems, and the mistake is
        invisible until the map comes out as a spiral.
        """
        tag = topic.rsplit("/", 1)[-1]
        with self.lock:
            i = self.n["cloud"]
            self.n["cloud"] += 1
            self.n_by_topic[topic] = self.n_by_topic.get(topic, 0) + 1
        if topic not in self.cloud_meta:
            try:
                self.cloud_meta[topic] = {
                    "fields": [{"name": f.name, "offset": int(f.offset),
                                "datatype": int(f.datatype),
                                "count": int(f.count)} for f in msg.fields],
                    "point_step": int(msg.point_step),
                    "row_step": int(msg.row_step),
                    "is_bigendian": bool(msg.is_bigendian),
                    "frame_id": str(msg.header.frame_id)}
            except Exception as e:
                self.cloud_meta[topic] = {"error": str(e)}
        try:
            # Keep the bytes exactly as they arrived. Decoding here, on the
            # day, with a layout we have not checked, is how a survey comes
            # home useless.
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            np.savez_compressed(
                os.path.join(self.cloud_dir, f"{i:05d}_{tag}.npz"),
                data=raw, width=int(msg.width), height=int(msg.height),
                point_step=int(msg.point_step), t=self.t(),
                topic=topic)
        except Exception:
            pass

    def on_height(self, msg):
        """The dog's own traversability grid, if it publishes one. Saved
        whole; it is small and it is most of what a navigator needs."""
        try:
            import numpy as _np
            with self.lock:
                i = self.n.setdefault("height", 0)
                self.n["height"] = i + 1
            if i % 10:                      # it repeats; every tenth is plenty
                return
            _np.savez_compressed(
                os.path.join(self.root, "height", f"{i:05d}.npz"),
                data=_np.asarray(msg.data, _np.float32),
                width=int(msg.width), height=int(msg.height),
                resolution=float(msg.resolution),
                origin=_np.asarray(msg.origin, _np.float32), t=self.t())
        except Exception:
            pass

    def on_odom(self, msg):
        try:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            row = {"t": self.t(), "p": [float(p.x), float(p.y), float(p.z)],
                   "q": [float(o.w), float(o.x), float(o.y), float(o.z)]}
        except Exception:
            try:
                p, o = msg.pose.position, msg.pose.orientation
                row = {"t": self.t(), "p": [float(p.x), float(p.y), float(p.z)],
                       "q": [float(o.w), float(o.x), float(o.y), float(o.z)]}
            except Exception as e:
                row = {"t": self.t(), "error": str(e)}
        with self.lock:
            self.odom_f.write(json.dumps(row) + "\n")
            self.n["odom"] += 1

    def close(self):
        self.state_f.close()
        self.odom_f.close()


def camera_loop(sv, stop, fps):
    """Front camera through VideoClient, in its own thread."""
    try:
        from unitree_sdk2py.go2.video.video_client import VideoClient
    except Exception as e:
        print(f"  camera unavailable: {e}")
        return
    vc = VideoClient()
    vc.SetTimeout(3.0)
    vc.Init()
    period = 1.0 / max(fps, 0.1)
    nxt = time.time()
    while not stop.is_set():
        now = time.time()
        if now < nxt:
            time.sleep(min(0.05, nxt - now))
            continue
        nxt = now + period
        try:
            code, data = vc.GetImageSample()
            if code == 0 and data:
                i = sv.n["frame"]
                with open(os.path.join(sv.frame_dir, f"{i:05d}.jpg"), "wb") as f:
                    f.write(bytes(data))
                with sv.lock:
                    sv.state_f.write(json.dumps(
                        {"t": sv.t(), "src": "frame", "i": i}) + "\n")
                    sv.n["frame"] += 1
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("iface")
    ap.add_argument("--secs", type=float, default=180.0)
    ap.add_argument("--fps", type=float, default=4.0)
    ap.add_argument("--name", default="run")
    ap.add_argument("--no-camera", action="store_true")
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                             ChannelSubscriber)

    root = os.path.join(OUT, f"survey_{a.name}_{datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(root, exist_ok=True)
    print(f"recording to {root}")
    print(f"READ-ONLY: this never commands the robot. Drive it with the "
          f"remote.\n")

    ChannelFactoryInitialize(DOMAIN_ID, a.iface)
    sv = Survey(root, a.fps)
    subs, silent = [], []

    def sub(topic, klass, cb):
        if klass is None:
            silent.append((topic, "no IDL in this SDK"))
            return False
        try:
            s = ChannelSubscriber(topic, klass)
            s.Init(cb, 10)
            subs.append((topic, s))
            return True
        except Exception as e:
            silent.append((topic, f"{type(e).__name__}: {e}"))
            return False

    sub("rt/lowstate", _idl("unitree_go", "LowState_"), sv.on_low)
    sub("rt/sportmodestate", _idl("unitree_go", "SportModeState_"), sv.on_sport)
    for t in CLOUD_TOPICS:
        # bind the topic into the callback so every sweep knows where it
        # came from
        sub(t, _idl("sensor_msgs", "PointCloud2_"),
            (lambda tp: (lambda m: sv.on_cloud(m, tp)))(t))
    for t in ODOM_TOPICS_ODOM:
        sub(t, _idl("nav_msgs", "Odometry_"), sv.on_odom)
    for t in ODOM_TOPICS_POSE:
        sub(t, _idl("geometry_msgs", "PoseStamped_"), sv.on_odom)
    sub("rt/utlidar/height_map_array", _idl("unitree_go", "HeightMap_"),
        sv.on_height)

    stop = threading.Event()
    cam = None
    if not a.no_camera:
        cam = threading.Thread(target=camera_loop, args=(sv, stop, a.fps),
                               daemon=True)
        cam.start()

    t0 = time.time()
    try:
        while time.time() - t0 < a.secs:
            time.sleep(1.0)
            el = time.time() - t0
            print(f"\r  {el:5.0f}/{a.secs:.0f}s   state {sv.n['state']:6d}   "
                  f"cloud {sv.n['cloud']:5d}   odom {sv.n['odom']:5d}   "
                  f"frames {sv.n['frame']:4d}  ", end="", flush=True)
    except KeyboardInterrupt:
        print("\n  stopped early")
    stop.set()
    if cam:
        cam.join(timeout=3.0)
    el = time.time() - t0
    sv.close()

    meta = {"name": a.name, "when": datetime.now().isoformat(timespec="seconds"),
            "seconds": round(el, 1), "counts": sv.n,
            "rates_hz": {k: round(v / el, 2) for k, v in sv.n.items()},
            "cloud_layout_by_topic": sv.cloud_meta,
            "cloud_count_by_topic": sv.n_by_topic,
            "silent": silent, "interface": a.iface}
    with open(os.path.join(root, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1, default=str)

    print(f"\n\n  {el:.0f} s recorded")
    for k, v in sv.n.items():
        print(f"    {k:<8}{v:>8}   {v/el:>7.1f} Hz")
    if sv.n["cloud"] == 0:
        print("\n  NO POINT CLOUD. Run real_probe.py to find out why before "
              "leaving.")
    if sv.n_by_topic:
        print("\n  point clouds by topic:")
        for t, c in sorted(sv.n_by_topic.items(), key=lambda x: -x[1]):
            fid = (sv.cloud_meta.get(t) or {}).get("frame_id", "?")
            print(f"    {t:<34}{c:>6}  frame_id={fid}")
        if len(sv.n_by_topic) > 1:
            print("    (more than one is live - they are in DIFFERENT frames;")
            print("     the mapper picks one, it does not merge them)")
    print(f"\nwrote {root}")


if __name__ == "__main__":
    main()
