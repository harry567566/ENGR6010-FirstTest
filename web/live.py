"""Live telemetry from the robot, for the browser.

Imported by web/server.py. Runs in its own thread, subscribes to the robot
the same way lab/real_state.py does, and keeps only the latest of everything
so the browser can poll or stream without the robot's rate becoming the
browser's rate.

What it exposes:

    state()   the newest IMU, joints, battery, mode, velocity, foot forces
    cloud()   the newest LiDAR sweep, already decoded to xyz float32
    frame()   the newest camera JPEG

Decoding the point cloud here, rather than in the browser, is deliberate:
the field layout is in the message, Python has numpy, and JavaScript would
have to re-derive the same offsets from the same header. The browser gets
plain xyz floats.

READ-ONLY. This module never writes a topic and never commands the robot.
"""
import struct
import threading
import time

import numpy as np

DOMAIN_ID = 0

# PointField datatype codes from the ROS PointCloud2 definition.
_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
       5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}

# In preference order. These are NOT the same thing: cloud_deskewed is
# motion-compensated and already in the odom frame, while cloud is in the
# sensor frame, which on a Go2 is rotated about 165 degrees from the body.
# The viewer locks onto the first one that actually publishes and ignores
# the rest, because drawing both into one buffer mixes two frames and the
# result looks like a smeared double exposure.
CLOUD_TOPICS = ["rt/utlidar/cloud_deskewed", "rt/utlidar/cloud"]


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


def decode_cloud(msg, max_points=25000):
    """PointCloud2 -> (N,3) float32, without assuming the field layout.

    The offsets come out of the message itself. Guessing them is how a
    point cloud ends up looking like noise."""
    try:
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")):
            return None
        step = int(msg.point_step)
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        n = len(raw) // step
        if n == 0:
            return None
        raw = raw[:n * step].reshape(n, step)
        out = np.empty((n, 3), np.float32)
        for i, k in enumerate(("x", "y", "z")):
            f = fields[k]
            dt = _NP.get(int(f.datatype), np.float32)
            off = int(f.offset)
            w = np.dtype(dt).itemsize
            out[:, i] = raw[:, off:off + w].copy().view(dt).ravel()[:n]
        good = np.isfinite(out).all(axis=1)
        out = out[good]
        if len(out) > max_points:            # keep the browser responsive
            out = out[np.random.default_rng(0).choice(
                len(out), max_points, replace=False)]
        return out
    except Exception:
        return None


class Live:
    def __init__(self):
        self.lock = threading.Lock()
        self.started = False
        self.iface = None
        self._state = {}
        self._cloud = None
        self._cloud_n = 0
        self._cloud_t = 0.0
        self._cloud_topic = None            # locked on first arrival
        self._cloud_frame = ""
        self._frame = None
        self._frame_t = 0.0
        self.counts = {"low": 0, "sport": 0, "cloud": 0, "frame": 0}
        self._stop = threading.Event()

    # ------------------------------------------------------------ control
    def start(self, iface):
        """Subscribe once. Safe to call again; it only acts the first time."""
        with self.lock:
            if self.started:
                return True, "already running"
            if not iface:
                return False, "no 192.168.123.x interface"
        try:
            from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                                     ChannelSubscriber)
            ChannelFactoryInitialize(DOMAIN_ID, iface)
        except Exception as e:
            return False, f"DDS init failed: {e}"

        def sub(topic, klass, cb):
            if klass is None:
                return
            try:
                s = ChannelSubscriber(topic, klass)
                s.Init(cb, 10)
            except Exception:
                pass

        sub("rt/lowstate", _idl("unitree_go", "LowState_"), self._on_low)
        sub("rt/sportmodestate", _idl("unitree_go", "SportModeState_"),
            self._on_sport)
        for t in CLOUD_TOPICS:
            sub(t, _idl("sensor_msgs", "PointCloud2_"),
                (lambda tp: (lambda m: self._on_cloud(m, tp)))(t))

        threading.Thread(target=self._camera, args=(), daemon=True).start()
        with self.lock:
            self.started = True
            self.iface = iface
        return True, f"live telemetry on {iface}"

    # ------------------------------------------------------------ callbacks
    def _on_low(self, msg):
        try:
            d = {
                "rpy": [round(float(x), 4) for x in msg.imu_state.rpy],
                "gyro": [round(float(x), 3) for x in msg.imu_state.gyroscope],
                "acc": [round(float(x), 2)
                        for x in msg.imu_state.accelerometer],
                "q": [round(float(m.q), 3) for m in msg.motor_state[:12]],
                "dq": [round(float(m.dq), 2) for m in msg.motor_state[:12]],
                "tau": [round(float(m.tau_est), 2)
                        for m in msg.motor_state[:12]],
                "temp": [int(m.temperature) for m in msg.motor_state[:12]],
                "foot_force": [int(x) for x in msg.foot_force],
                "soc": int(msg.bms_state.soc),
                "power_v": round(float(msg.power_v), 2),
                "power_a": round(float(msg.power_a), 2),
            }
        except Exception:
            return
        with self.lock:
            self._state.update(d)
            self.counts["low"] += 1

    def _on_sport(self, msg):
        try:
            d = {"mode": int(msg.mode),
                 "body_height": round(float(msg.body_height), 3),
                 "position": [round(float(x), 3) for x in msg.position],
                 "velocity": [round(float(x), 3) for x in msg.velocity],
                 "yaw_speed": round(float(msg.yaw_speed), 3),
                 "foot_raise_height": round(float(msg.foot_raise_height), 3)}
        except Exception:
            return
        with self.lock:
            self._state.update(d)
            self.counts["sport"] += 1

    def _on_cloud(self, msg, topic):
        with self.lock:
            if self._cloud_topic is None:
                self._cloud_topic = topic            # first one wins
                try:
                    self._cloud_frame = str(msg.header.frame_id)
                except Exception:
                    pass
            elif topic != self._cloud_topic:
                return                               # different frame, drop
        pts = decode_cloud(msg)
        if pts is None:
            return
        with self.lock:
            self._cloud = pts
            self._cloud_n = len(pts)
            self._cloud_t = time.time()
            self.counts["cloud"] += 1

    def _camera(self):
        try:
            from unitree_sdk2py.go2.video.video_client import VideoClient
        except Exception:
            return
        try:
            vc = VideoClient()
            vc.SetTimeout(3.0)
            vc.Init()
        except Exception:
            return
        while not self._stop.is_set():
            try:
                code, data = vc.GetImageSample()
                if code == 0 and data:
                    with self.lock:
                        self._frame = bytes(data)
                        self._frame_t = time.time()
                        self.counts["frame"] += 1
            except Exception:
                pass
            time.sleep(0.12)                 # ~8 fps is plenty for a browser

    # ---------------------------------------------------------------- read
    def state(self):
        with self.lock:
            s = dict(self._state)
            s["counts"] = dict(self.counts)
            s["cloud_points"] = self._cloud_n
            s["cloud_topic"] = self._cloud_topic
            s["cloud_frame"] = self._cloud_frame
            s["cloud_age"] = (round(time.time() - self._cloud_t, 2)
                              if self._cloud_t else None)
            s["frame_age"] = (round(time.time() - self._frame_t, 2)
                              if self._frame_t else None)
            s["live"] = self.started
            s["iface"] = self.iface
        return s

    def cloud_bytes(self):
        """The newest sweep as raw little-endian float32 xyz, for the GPU."""
        with self.lock:
            c = self._cloud
        if c is None:
            return None
        return np.ascontiguousarray(c, dtype="<f4").tobytes()

    def frame(self):
        with self.lock:
            return self._frame


LIVE = Live()
