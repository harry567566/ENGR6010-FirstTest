"""Live view and driving over Wi-Fi: camera, LiDAR map, and the controls.

    .venv_wifi/bin/python lab/wifi_live.py [ap | <dog ip>]

ONE WebRTC connection, held open, carrying everything:

    rt/lf/lowstate                  battery, attitude, joints, foot forces
    rt/lf/sportmodestate            mode, body height, velocity
    rt/utlidar/voxel_map_compressed the point cloud, decoded by the library
    rt/utlidar/robot_pose           the dog's own LiDAR odometry
    the video track                 the front camera

Why one: every other Wi-Fi button opens its own session, does one thing and
hangs up. That costs three to five seconds a press, and the dog answers
`429 Too Many Requests` when two land close together - which is what killed
1-5 in the lab. So stand / balance / sit / stop live in this window too, on
the same connection. While this window is open, leave the panel's other
Wi-Fi buttons alone: they would be a second client, and the dog takes one.

The map is built, not splattered. One sweep is a thin shell of points and
looks like noise from above. Every sweep is folded into a single occupancy
grid in the odometry frame, and the floor is dropped - floor points
outnumber wall points several to one and wash the structure out entirely.
What is left is walls, which is what a floor plan is made of.

MOTION IS OFF UNTIL THE BOX IS TICKED. W A S D drive, Q E turn, the
spacebar stops, and so do closing the window and Ctrl+C. The sliders set
the speed; the remote's L2 + B is still faster than any of this.
"""
import asyncio
import json
import math
import os
import random
import sys
import threading
import time
from collections import deque
import tkinter as tk
from tkinter import ttk

import numpy as np
from PIL import Image, ImageDraw, ImageTk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SPORT_TOPIC = "rt/api/sport/request"
AVOID_TOPIC = "rt/api/obstacles_avoid/request"
# The dog's own obstacle avoidance. MOVE here is the same velocity command
# as the sport one, except the robot is allowed to refuse the parts of it
# that would walk into something. Switching it on and driving through this
# topic is the whole of "don't hit the wall" - none of it is ours to write.
AVOID = {"on": 1001, "get": 1002, "move": 1003, "remote": 1004}

# The robot's own SLAM. rt/uslam/client_command takes a PLAIN STRING -
# "mapping/start", "navigation/set_goal_pose/1.250/-0.400/1.571" - not the
# api_id envelope every other topic uses, and it must go out through
# publish_without_callback: publish() waits on a future keyed to a uuid dug
# out of the payload, and a bare string has none, so it never returns.
USLAM_CMD = "rt/uslam/client_command"
USLAM_LOG = "rt/uslam/server_log"
USLAM_CLOUD = "rt/uslam/frontend/cloud_world_ds"
USLAM_ODOM = "rt/uslam/frontend/odom"
USLAM_LOC_ODOM = "rt/uslam/localization/odom"
USLAM_PATH = "rt/uslam/navigation/global_path"

# Go2 front camera, from Go2Py's calibration note (640 x 480). The WebRTC
# stream is 1280 x 720, a different aspect, so this is scaled by width and
# the vertical centre is taken from the frame - good enough to see whether
# the mount is right, not good enough to publish. The overlay view exists
# to judge exactly this.
FX, FY, CX, CY = 379.7099, 379.3696, 320.3064, 243.1175
LOWSTATE = "rt/lf/lowstate"
SPORTSTATE = "rt/lf/sportmodestate"
CLOUD = "rt/utlidar/voxel_map_compressed"
POSE = "rt/utlidar/robot_pose"

# The slider ceiling depends on the gait, because the gait is what decides
# whether the dog can take the speed. These are what the gait is asked for,
# not what it will necessarily do - the dog's own measured velocity is on
# screen, and that is the number to believe.
GAITS = {
    "walk":  {"api": 1061, "name": "慢走",  "vx": 0.6, "vyaw": 0.8},
    "trot":  {"api": 1019, "name": "普通",  "vx": 1.5, "vyaw": 1.6},
    "run":   {"api": 1062, "name": "快跑",  "vx": 3.0, "vyaw": 2.0},
}
MAX_VY = 0.8
MOVE_HZ = 20.0
STANDING_LOCKED = (0, 2, 6, 8)      # Move does little here until BalanceStand
API = {"damp": 1001, "balance": 1002, "stop": 1003, "stand": 1004,
       "sit": 1005, "recover": 1006, "move": 1008,
       "walk": 1061, "trot": 1019, "run": 1062, "speed": 1015,
       "follow": 2056,            # LeadFollow - the dog's own person-follow
       "freeavoid": 2048}         # FreeAvoid  - walk and dodge by itself

CAM_W, CAM_H = 460, 345
MAP_PX = 560                        # the map panel is square; metres vary

# 6 cm cells over 80 m: a teaching building with room to spare, and fine
# enough that a wall is two or three cells thick.
CELL = 0.06
EXTENT = 80.0
GRID_N = int(EXTENT / CELL)

FLOOR_BAND = (0.25, 2.20)           # metres above the floor that count as
#                                     structure: below is floor, above is
#                                     ceiling, and neither draws a wall

# How much of the past to keep. Accumulating for ever turns the view into
# a solid block in which nothing can be read and which never forgets a
# person who has walked away; keeping only the newest sweep is a thin
# shell that shows nothing. Seconds is the knob that matters, so it is the
# one on screen.
KEEP_CHOICES = [3, 10, 30, 90, 0]       # 0 = keep everything
VOX3 = 0.06
MAX3 = 600_000


class Shared:
    """What the robot thread writes and the window draws.

    Only the newest of everything is kept, except the map, which is the
    whole point and accumulates."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = {}
        self.frame = None
        self.raw = None              # the camera frame as an array, for the
        self.raw_t = 0.0             # overlay; the PIL one is for the label
        self.cloud = None            # the newest sweep, un-accumulated
        self.cloud_t = 0.0
        self.pose = (0.0, 0.0, 0.0)
        self.trail = []
        self.counts = {"low": 0, "sport": 0, "cloud": 0, "frame": 0}
        self.note = "connecting ..."
        self.zrange = None
        self.grid = np.zeros((GRID_N, GRID_N), np.uint16)
        self.cells = 0
        self.sweeps = deque()        # (t, (N,3)) - the retained window
        self.keep = 30.0             # seconds; 0 means keep everything
        self.pts3 = np.zeros((0, 3), np.float32)   # the retained cloud, flat
        self.pts3_t = 0.0
        self.slam = None             # the dog's own SLAM map, (N, 3)
        self.slam_n = 0
        self.path = None             # the planner's route, (N, 2)
        self.goal = None
        self.slog = []               # what the SLAM server says

    def put(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def snap(self):
        with self.lock:
            return dict(state=dict(self.state), frame=self.frame,
                        cloud_t=self.cloud_t, pose=self.pose,
                        trail=list(self.trail), counts=dict(self.counts),
                        note=self.note, zrange=self.zrange, cells=self.cells,
                        pts3=self.pts3, cloud=self.cloud, raw=self.raw,
                        slam=self.slam, slam_n=self.slam_n, path=self.path,
                        goal=self.goal, slog=list(self.slog))

    def clear_map(self):
        with self.lock:
            self.grid[:] = 0
            self.cells = 0
            self.trail = []
            self.sweeps.clear()
            self.pts3 = np.zeros((0, 3), np.float32)


SH = Shared()
DRIVE = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0, "allowed": False, "cmd": None,
         "avoid": False}
drive_lock = threading.Lock()


# --------------------------------------------------------------- the robot

def decode_cloud(data):
    """Whatever the decoder produced -> (N, 3) float32 metres.

    There are two decoders in the library and they do NOT return the same
    thing, which is the bug that made the cloud useless:

        native   {"points": (N,3)}   already metres - the last line of
                 bits_to_points is `points * resolution + origin`
        libvoxel {"positions": ...}  VOXEL INDICES, not metres. They have
                 to be multiplied by the message's resolution and offset by
                 its origin, or the cloud comes out twenty times too big
                 (resolution is usually 0.05) and in the wrong place.

    The panel asks for the native decoder, so the first branch is the one
    that runs; the second is here because the default is libvoxel and a
    silent twenty-fold error is not worth risking again."""
    try:
        pts = data.get("points")
        if pts is not None:
            xyz = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
            return np.ascontiguousarray(xyz[np.isfinite(xyz).all(axis=1)])

        pos = data.get("positions")
        n = int(data.get("face_count", 0))
        if pos is None or n <= 0:
            return None
        xyz = np.frombuffer(bytes(pos), dtype=np.float32)[:n * 3]
        xyz = xyz.reshape(-1, 3).astype(np.float32)
        res = float(data.get("resolution", 0.05))
        org = np.asarray(data.get("origin", (0.0, 0.0, 0.0)), np.float32)
        xyz = xyz * res + org
        return np.ascontiguousarray(xyz[np.isfinite(xyz).all(axis=1)])
    except Exception:
        return None


def fold_into_map(pts):
    """Add one sweep to the occupancy grid, floor and ceiling removed.

    The floor height is taken from the sweep rather than assumed: the
    cloud's frame and its zero are the robot's business, and a fixed
    threshold would keep the floor on one dog and delete the walls on
    another. The 5th percentile is the floor under any sweep not pointed at
    the sky."""
    if pts is None or len(pts) < 50:
        return
    z = pts[:, 2]
    floor = float(np.percentile(z, 5))
    keep = (z > floor + FLOOR_BAND[0]) & (z < floor + FLOOR_BAND[1])
    SH.put(zrange=(round(float(z.min()), 2), round(float(z.max()), 2),
                   round(floor, 2), int(keep.sum()), len(pts)))
    wall = pts[keep]
    if not len(wall):
        return
    ix = (wall[:, 0] / CELL + GRID_N / 2).astype(np.int32)
    iy = (wall[:, 1] / CELL + GRID_N / 2).astype(np.int32)
    ok = (ix >= 0) & (ix < GRID_N) & (iy >= 0) & (iy < GRID_N)
    ix, iy = ix[ok], iy[ok]
    with SH.lock:
        g = SH.grid
        cur = g[iy, ix]
        # saturating: a cell seen a thousand times is no more a wall than
        # one seen two hundred times, and uint16 would wrap
        g[iy, ix] = np.where(cur < 60000, cur + 1, cur)
        SH.cells = int(np.count_nonzero(g))


def add_points3(pts):
    """Remember this sweep, drop the ones that have aged out, rebuild.

    The window is in seconds rather than in sweeps: the LiDAR's rate
    changes with what the dog is doing - it drops to a third of normal
    while the dog is lying down - so a count of sweeps is a different
    length of time from one minute to the next, and seconds is what
    anyone looking at the screen is actually thinking in."""
    if pts is None or len(pts) < 10:
        return
    now = time.time()
    with SH.lock:
        SH.sweeps.append((now, pts))
        keep = SH.keep
        if keep:
            while SH.sweeps and now - SH.sweeps[0][0] > keep:
                SH.sweeps.popleft()
        while len(SH.sweeps) > 900:              # a hard backstop
            SH.sweeps.popleft()
        chunks = [p for _t, p in SH.sweeps]

    if not chunks:
        return
    allp = np.concatenate(chunks)
    # thin to one point per voxel so the renderer is not asked to draw
    # fifty points that land on the same pixel
    v = np.floor(allp / VOX3).astype(np.int64)
    k = ((v[:, 0] + 4000) * 8000 + (v[:, 1] + 4000)) * 4000 + (v[:, 2] + 1000)
    _u, first = np.unique(k, return_index=True)
    out = allp[first]
    if len(out) > MAX3:
        out = out[np.random.default_rng(0).choice(len(out), MAX3, False)]
    SH.put(pts3=np.ascontiguousarray(out), pts3_t=now)


async def dds_loop(iface):
    """The same window, driven over DDS instead of WebRTC.

    This is the route the simulator speaks. unitree_mujoco publishes the
    same rt/lowstate and rt/sportmodestate a real Go2 does, on the same
    wired network, so a window that can read DDS reads the simulated dog
    and the real one through one set of code - which is the point of
    rehearsing in simulation at all. It is also the only route to
    rt/lowcmd, so it is the one anything trained in simulation comes back
    through.

    No camera and no WebRTC cloud here: over the cable the point cloud is
    rt/utlidar/cloud_deskewed, a PointCloud2, read with the same parser the
    navigation path uses."""
    from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                             ChannelSubscriber)

    def idl(mod, cls):
        m = __import__(f"unitree_sdk2py.idl.{mod}.msg.dds_", fromlist=[cls])
        return getattr(m, cls)

    ChannelFactoryInitialize(0, iface)
    SH.put(note=f"DDS on {iface}")

    def on_low(msg):
        try:
            SH.put(state={**SH.state,
                          "soc": int(msg.bms_state.soc),
                          "volt": round(float(msg.power_v), 2),
                          "rpy": [round(float(x), 3)
                                  for x in msg.imu_state.rpy],
                          "foot": [int(x) for x in msg.foot_force]})
            SH.counts["low"] += 1
        except Exception:
            pass

    def on_sport(msg):
        try:
            SH.put(state={**SH.state, "mode": int(msg.mode),
                          "height": round(float(msg.body_height), 3),
                          "vel": [round(float(x), 2) for x in msg.velocity]})
            SH.counts["sport"] += 1
        except Exception:
            pass

    def on_cloud(msg):
        pts = parse_pointcloud2_obj(msg)
        if pts is None or not len(pts):
            return
        SH.put(cloud=pts, cloud_t=time.time())
        SH.counts["cloud"] += 1
        fold_into_map(pts)
        add_points3(pts)

    subs = []
    for topic, mod, cls, cb in (
            ("rt/lowstate", "unitree_go", "LowState_", on_low),
            ("rt/sportmodestate", "unitree_go", "SportModeState_", on_sport),
            ("rt/utlidar/cloud_deskewed", "sensor_msgs", "PointCloud2_",
             on_cloud)):
        try:
            sub = ChannelSubscriber(topic, idl(mod, cls))
            sub.Init(cb, 10)
            subs.append(sub)
        except Exception as e:
            SH.put(note=f"{topic}: {e}")

    SH.put(note="connected")
    try:
        while True:
            await asyncio.sleep(0.5)        # read-only: no commands on DDS
    except asyncio.CancelledError:
        pass


def parse_pointcloud2_obj(msg):
    """A PointCloud2 as an IDL object rather than a dict."""
    try:
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")):
            return None
        step = int(msg.point_step)
        raw = np.frombuffer(bytes(msg.data), np.uint8)
        n = len(raw) // step
        if n == 0:
            return None
        raw = raw[:n * step].reshape(n, step)
        out = np.empty((n, 3), np.float32)
        for i, k in enumerate(("x", "y", "z")):
            off = int(fields[k].offset)
            out[:, i] = raw[:, off:off + 4].copy().view(np.float32).ravel()
        return np.ascontiguousarray(out[np.isfinite(out).all(axis=1)])
    except Exception:
        return None


async def robot_loop(target):
    from wifi_common import connect, disconnect
    conn = await connect(target)
    slam_dec = None
    try:
        from slam_decoder import SlamDecoder
        slam_dec = SlamDecoder()
    except Exception as e:
        SH.put(note=f"slam decoder: {e}")

    def on_low(msg):
        try:
            d = msg["data"]
            SH.put(state={**SH.state,
                          "soc": int(d["bms_state"]["soc"]),
                          "volt": round(float(d["power_v"]), 2),
                          "rpy": [round(float(x), 3)
                                  for x in d["imu_state"]["rpy"]],
                          "foot": [int(x) for x in d["foot_force"]]})
            SH.counts["low"] += 1
        except Exception:
            pass

    def on_sport(msg):
        try:
            d = msg["data"]
            SH.put(state={**SH.state,
                          "mode": int(d["mode"]),
                          "height": round(float(d["body_height"]), 3),
                          "vel": [round(float(x), 2) for x in d["velocity"]]})
            SH.counts["sport"] += 1
        except Exception:
            pass

    def on_cloud(msg):
        pts = decode_cloud(msg.get("data", {}).get("data", {}))
        if pts is None or not len(pts):
            return
        SH.put(cloud=pts, cloud_t=time.time())
        SH.counts["cloud"] += 1
        fold_into_map(pts)
        add_points3(pts)

    def on_pose(msg):
        try:
            d = msg["data"]
            p = d["pose"]["position"]
            q = d["pose"]["orientation"]
            yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                             1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
            pose = (float(p["x"]), float(p["y"]), yaw)
            with SH.lock:
                SH.pose = pose
                if not SH.trail or math.dist(SH.trail[-1], pose[:2]) > 0.08:
                    SH.trail.append(pose[:2])
                    del SH.trail[:-4000]
        except Exception:
            pass

    async def on_video(track):
        while True:
            try:
                frame = await track.recv()
            except Exception:
                return
            try:
                img = Image.fromarray(frame.to_ndarray(format="rgb24"))
                img.thumbnail((CAM_W, CAM_H))
                box = Image.new("RGB", (CAM_W, CAM_H), (7, 9, 13))
                box.paste(img, ((CAM_W - img.width) // 2,
                                (CAM_H - img.height) // 2))
                SH.put(frame=box, raw=frame.to_ndarray(format="rgb24"),
                       raw_t=time.time())
                SH.counts["frame"] += 1
            except Exception:
                pass            # a dropped frame is not worth stopping for

    def slog(text):
        with SH.lock:
            SH.slog.append(f"{time.strftime('%H:%M:%S')}  {text}")
            del SH.slog[:-80]

    def on_slog(msg):
        try:
            txt = str(msg["data"]["data"] if isinstance(msg["data"], dict)
                      else msg["data"])
        except Exception:
            return
        slog("← " + txt)
        SH.put(note=txt[:60])

    def on_slam_cloud(msg):
        if slam_dec is None:
            return
        d = msg.get("data", {})
        whole, _frame = slam_dec.decode(d.get("data", b""), d)
        if whole is None or not len(whole):
            return
        SH.put(slam=whole, slam_n=len(whole))

    def on_path(msg):
        pts = parse_pointcloud2(msg.get("data", {}))
        if pts is not None and len(pts):
            SH.put(path=pts[:, :2])

    ps = conn.datachannel.pub_sub
    ps.subscribe(USLAM_LOG, on_slog)
    ps.subscribe(USLAM_CLOUD, on_slam_cloud)
    ps.subscribe(USLAM_PATH, on_path)
    ps.subscribe(LOWSTATE, on_low)
    ps.subscribe(SPORTSTATE, on_sport)
    ps.subscribe(POSE, on_pose)

    # Without this the dog throttles the cloud away, and a plain subscribe
    # looks exactly like a LiDAR that is not running.
    try:
        # native, not the default libvoxel: it returns points in metres
        # instead of voxel indices, so there is no scale to get wrong
        conn.datachannel.set_decoder("native")
    except Exception as e:
        SH.put(note=f"decoder: {e}")
    try:
        await conn.datachannel.disableTrafficSaving(True)
    except Exception as e:
        SH.put(note=f"traffic saving: {e}")
    ps.subscribe(CLOUD, on_cloud)

    try:
        conn.video.add_track_callback(on_video)
        conn.video.switchVideoChannel(True)
    except Exception as e:
        SH.put(note=f"camera unavailable: {e}")

    # The LiDAR has to be running before any of the SLAM commands do
    # anything: the mapping frontend's inputs are the lidar topics, and a
    # mapping/start with the LiDAR off succeeds and then publishes nothing.
    try:
        ps.publish_without_callback("rt/utlidar/switch", "ON")
        slog("→ rt/utlidar/switch ON")
    except Exception as e:
        slog(f"lidar switch: {e}")

    SH.put(note="connected")

    last_sent, moving = 0.0, False
    period = 1.0 / MOVE_HZ
    try:
        while True:
            await asyncio.sleep(period)
            with drive_lock:
                vx, vy, vyaw = DRIVE["vx"], DRIVE["vy"], DRIVE["vyaw"]
                allowed, cmd = DRIVE["allowed"], DRIVE["cmd"]
                DRIVE["cmd"] = None

            if isinstance(cmd, str) and cmd.startswith("uslam:"):
                # a plain string on its own topic, no envelope, no reply
                text = cmd[6:]
                ps.publish_without_callback(USLAM_CMD, text)
                slog("→ " + text)
                # Ask straight back. If uslam is not running nothing will
                # answer, and silence then means something rather than
                # being indistinguishable from a button that did nothing.
                ns = text.split("/")[0]
                if ns in ("mapping", "localization", "navigation", "patrol"):
                    await asyncio.sleep(0.4)
                    ps.publish_without_callback(USLAM_CMD,
                                                f"{ns}/get_status")
                continue

            if cmd == "lidar_on":
                ps.publish_without_callback("rt/utlidar/switch", "ON")
                slog("→ rt/utlidar/switch ON")
                continue

            if cmd == "avoid_on" or cmd == "avoid_off":
                want = cmd == "avoid_on"
                try:
                    await asyncio.wait_for(ps.publish_request_new(
                        AVOID_TOPIC, {"api_id": AVOID["on"],
                                      "parameter": json.dumps(
                                          {"enable": want})}), 4.0)
                    slog(f"← 避障 {'开' if want else '关'}")
                except Exception as e:
                    slog(f"避障失败: {e}")
                continue

            if cmd:                                 # a button in the window
                name, param = (cmd if isinstance(cmd, tuple) else (cmd, None))
                code = await request(conn, API[name], param)
                slog(f"← {name}: {'OK' if code == 0 else code}")
                if name in ("stop", "sit", "damp"):
                    moving = False
                continue

            if not allowed:
                vx = vy = vyaw = 0.0

            if vx or vy or vyaw:
                if SH.state.get("mode") in STANDING_LOCKED and not moving:
                    # Move does almost nothing from idle stand, so switch
                    # first - the same thing 1-7 does
                    await request(conn, API["balance"])
                    SH.put(note="balance stand")
                send_move(conn, vx, vy, vyaw, avoid=DRIVE["avoid"])
                moving, last_sent = True, time.time()
            elif moving and time.time() - last_sent > 0.12:
                await request(conn, API["stop"])
                moving = False
    except asyncio.CancelledError:
        pass
    finally:
        try:
            if moving:
                await request(conn, API["stop"])
        except Exception:
            pass
        await disconnect(conn)


async def request(conn, api_id, parameter=None, timeout=4.0):
    payload = {"api_id": api_id}
    if parameter is not None:
        payload["parameter"] = parameter
    try:
        answer = await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(
                SPORT_TOPIC, payload), timeout)
        return answer.get("data", {}).get("header", {}) \
                     .get("status", {}).get("code", -1)
    except Exception:
        return -1


def parse_pointcloud2(d):
    """sensor_msgs/PointCloud2 -> (N, 3) float32, offsets read from the msg.

    The navigation path and the localisation cloud arrive as plain
    PointCloud2 rather than anything compressed."""
    try:
        fields = {f["name"]: f for f in d["fields"]}
        if not all(k in fields for k in ("x", "y", "z")):
            return None
        step = int(d["point_step"])
        raw = np.frombuffer(bytes(d["data"]), np.uint8)
        n = len(raw) // step
        if n == 0:
            return None
        raw = raw[:n * step].reshape(n, step)
        out = np.empty((n, 3), np.float32)
        for i, k in enumerate(("x", "y", "z")):
            off = int(fields[k]["offset"])
            out[:, i] = raw[:, off:off + 4].copy().view(np.float32).ravel()
        return out[np.isfinite(out).all(axis=1)]
    except Exception:
        return None


def send_move(conn, vx, vy, vyaw, avoid=False):
    """Fire and forget, the shape wifi_sport.py sends.

    With `avoid` the same velocity goes to the obstacle-avoidance service
    instead, which is free to refuse the part of it that would walk into
    something. That refusal is the whole of "do not hit the wall", and it
    is the robot's, not ours."""
    if avoid:
        conn.datachannel.pub_sub.publish_without_callback(AVOID_TOPIC, {
            "header": {"identity": {"id": int(time.time() * 1000) % 2147483648
                                          + random.randint(0, 1000),
                                    "api_id": AVOID["move"]},
                       "policy": {"priority": 0, "noreply": True}},
            "parameter": json.dumps({"x": vx, "y": vy, "yaw": vyaw,
                                     "mode": 0}),
            "binary": [],
        })
        return
    conn.datachannel.pub_sub.publish_without_callback(SPORT_TOPIC, {
        "header": {"identity": {"id": int(time.time() * 1000) % 2147483648
                                      + random.randint(0, 1000),
                                "api_id": API["move"]},
                   "policy": {"priority": 0, "noreply": True}},
        "parameter": json.dumps({"x": vx, "y": vy, "z": vyaw}),
        "binary": [],
    })


# ----------------------------------------------------------------- the map

def render_map(snap, metres, follow=True, n=None):
    """The accumulated grid as a floor plan, with the dog and its trail."""
    n = int(n or MAP_PX)
    img = np.zeros((n, n, 3), np.uint8)
    img[:, :] = (10, 13, 18)

    ppm = n / metres                                   # pixels per metre
    cx, cy = snap["pose"][:2] if follow else (0.0, 0.0)
    yaw = snap["pose"][2]

    step = ppm                                         # one-metre grid
    if step >= 9:
        off = (math.ceil(cx - metres / 2) - (cx - metres / 2)) * ppm
        xs = np.arange(off, n, step).astype(int)
        img[:, xs[(xs >= 0) & (xs < n)]] = (25, 31, 41)
        off = (math.ceil(cy - metres / 2) - (cy - metres / 2)) * ppm
        ys = (n - 1 - np.arange(off, n, step)).astype(int)
        img[ys[(ys >= 0) & (ys < n)], :] = (25, 31, 41)

    # the slice of grid on screen, nearest-neighbour scaled to the panel
    half_cells = int(metres / CELL / 2)
    ci = int(cx / CELL + GRID_N / 2)
    cj = int(cy / CELL + GRID_N / 2)
    i0, i1 = max(0, ci - half_cells), min(GRID_N, ci + half_cells)
    j0, j1 = max(0, cj - half_cells), min(GRID_N, cj + half_cells)
    with SH.lock:
        sub = SH.grid[j0:j1, i0:i1].astype(np.float32)

    if sub.size:
        # Hits per cell span orders of magnitude - a wall walked past twice
        # against one glimpsed once - so brightness is logarithmic, or
        # everything except the nearest wall comes out black.
        # log(8) rather than something larger: a wall seen two or three
        # times should already read as a wall, not as a smudge
        v = np.clip(np.log1p(sub) / math.log(8.0), 0, 1)
        src_j = ((np.arange(n) / ppm + (cy - metres / 2) - (j0 * CELL
                 - EXTENT / 2)) / CELL).astype(np.int32)
        src_i = ((np.arange(n) / ppm + (cx - metres / 2) - (i0 * CELL
                 - EXTENT / 2)) / CELL).astype(np.int32)
        okj = (src_j >= 0) & (src_j < v.shape[0])
        oki = (src_i >= 0) & (src_i < v.shape[1])
        if okj.any() and oki.any():
            big = v[np.clip(src_j, 0, v.shape[0] - 1)][:,
                    np.clip(src_i, 0, v.shape[1] - 1)]
            big[~okj, :] = 0
            big[:, ~oki] = 0
            big = np.flipud(big)                       # +y is up on screen
            m = big > 0.02
            img[..., 0] = np.where(m, 40 + 215 * big, img[..., 0])
            img[..., 1] = np.where(m, 95 + 160 * big, img[..., 1])
            img[..., 2] = np.where(m, 125 + 55 * big, img[..., 2])

    def to_px(x, y):
        return (int((x - cx) * ppm + n / 2), int(n / 2 - (y - cy) * ppm))

    # The dog's own SLAM map, when it is running: this is what the phone
    # app draws, and it is better than anything folded together out here.
    sl = snap.get("slam")
    if sl is not None and len(sl):
        lo = float(np.percentile(sl[:, 2], 3))
        keep = (sl[:, 2] > lo + FLOOR_BAND[0]) & (sl[:, 2] < lo + FLOOR_BAND[1])
        w2 = sl[keep]
        if len(w2):
            sxp = ((w2[:, 0] - cx) * ppm + n / 2).astype(np.int32)
            syp = (n / 2 - (w2[:, 1] - cy) * ppm).astype(np.int32)
            v = (sxp >= 0) & (sxp < n) & (syp >= 0) & (syp < n)
            img[syp[v], sxp[v]] = (120, 240, 190)

    for i in range(1, len(snap.get("path") or [])):     # the planner's route
        x0, y0 = to_px(*snap["path"][i - 1])
        x1, y1 = to_px(*snap["path"][i])
        for t in np.linspace(0, 1, 8):
            qx, qy = int(x0 + (x1 - x0) * t), int(y0 + (y1 - y0) * t)
            if 0 <= qx < n - 1 and 0 <= qy < n - 1:
                img[qy:qy + 2, qx:qx + 2] = (255, 215, 90)

    if snap.get("goal"):                                # where it was sent
        gx, gy = to_px(*snap["goal"])
        if 2 <= gx < n - 3 and 2 <= gy < n - 3:
            img[gy - 3:gy + 4, gx - 1:gx + 2] = (90, 255, 140)
            img[gy - 1:gy + 2, gx - 3:gx + 4] = (90, 255, 140)

    trail = snap["trail"]
    for i in range(1, len(trail)):                     # where it has been
        x0, y0 = to_px(*trail[i - 1])
        x1, y1 = to_px(*trail[i])
        if abs(x1 - x0) + abs(y1 - y0) > 60:
            continue
        for t in np.linspace(0, 1, 7):
            px, py = int(x0 + (x1 - x0) * t), int(y0 + (y1 - y0) * t)
            if 0 <= px < n and 0 <= py < n:
                img[py, px] = (150, 105, 100)

    px, py = to_px(snap["pose"][0], snap["pose"][1])
    if 0 <= px < n and 0 <= py < n:                    # the dog, nose first
        img[max(0, py - 5):py + 6, max(0, px - 5):px + 6] = (245, 248, 255)
        for d in range(6, 28):
            ax, ay = int(px + d * math.cos(-yaw)), int(py + d * math.sin(-yaw))
            if 0 <= ax < n - 1 and 0 <= ay < n - 1:
                img[ay:ay + 2, ax:ax + 2] = (255, 95, 86)

    bar = max(int(ppm), 8)                             # a one-metre scale bar
    img[n - 20:n - 16, 16:16 + bar] = (205, 215, 230)
    return Image.fromarray(img)


def render_3d(snap, az, el, dist, follow=True, drop_floor=True, n=None):
    """The retained cloud from any angle, coloured by height.

    Perspective, drawn with numpy straight into the pixels: four hundred
    thousand points is nothing as arrays and impossible as canvas items.
    Painted far-to-near so the nearer surface wins the pixel."""
    n = int(n or MAP_PX)
    img = np.zeros((n, n, 3), np.uint8)
    img[:, :] = (8, 10, 15)

    pts = snap["pts3"]
    cx, cy, yaw = snap["pose"]
    if len(pts) and drop_floor:
        # The floor is most of the cloud and it reads as a blue carpet with
        # the room hidden under it. Taking it out leaves the structure,
        # which is the part worth looking at from an angle.
        floor = float(np.percentile(pts[:, 2], 3))
        pts = pts[pts[:, 2] > floor + 0.18]
    if not len(pts):
        _draw_hud_3d(img, n, az, el)
        return Image.fromarray(img)

    centre = np.array([cx, cy, 0.6], np.float32) if follow \
        else pts.mean(axis=0)
    p = pts - centre

    ca, sa = math.cos(az), math.sin(az)                 # spin about z
    x = p[:, 0] * ca + p[:, 1] * sa
    y = -p[:, 0] * sa + p[:, 1] * ca
    z = p[:, 2]
    ce, se = math.cos(el), math.sin(el)                 # then tilt
    y2 = y * ce - z * se
    z2 = y * se + z * ce

    depth = y2 + dist                                   # camera down -y
    ok = depth > 0.35
    if not ok.any():
        _draw_hud_3d(img, n, az, el)
        return Image.fromarray(img)
    x, z2, depth, hgt = x[ok], z2[ok], depth[ok], z[ok]

    f = n * 0.62
    sx = (x * f / depth + n / 2).astype(np.int32)
    sy = (n / 2 - z2 * f / depth).astype(np.int32)
    vis = (sx >= 0) & (sx < n) & (sy >= 0) & (sy < n)
    sx, sy, depth, hgt = sx[vis], sy[vis], depth[vis], hgt[vis]
    if not len(sx):
        _draw_hud_3d(img, n, az, el)
        return Image.fromarray(img)

    order = np.argsort(-depth)                          # far first
    sx, sy, depth, hgt = sx[order], sy[order], depth[order], hgt[order]

    lo, hi = float(hgt.min()), float(hgt.max())
    u = np.clip((hgt - lo) / max(hi - lo, 0.5), 0, 1)
    fade = np.clip(1.25 - depth / 26.0, 0.25, 1.0)      # distance haze
    img[sy, sx, 0] = (fade * (45 + 210 * u)).astype(np.uint8)
    img[sy, sx, 1] = (fade * (105 + 150 * np.sin(u * math.pi))).astype(np.uint8)
    img[sy, sx, 2] = (fade * (255 - 150 * u)).astype(np.uint8)

    # the dog, drawn at the middle since the view is centred on it
    if follow:
        c = n // 2
        img[c - 2:c + 3, c - 4:c + 5] = (250, 250, 250)

    _draw_hud_3d(img, n, az, el)
    return Image.fromarray(img)


def _draw_hud_3d(img, n, az, el):
    img[8:11, 8:8 + 60] = (40, 48, 60)                  # a quiet corner mark
    img[8:8 + 60, 8:11] = (40, 48, 60)


def render_overlay(snap, n=None, dx=0.0, dy=0.0, fov=1.0):
    """The current sweep drawn on the camera picture, coloured by range.

    This is the fusion of the two sensors, and it is also the only honest
    way to judge the mount: if the points land on the walls and the floor
    in the picture, the extrinsics are right; if they sit an arm's length
    off, they are not, and no amount of colouring a cloud will fix it.

    The cloud's frame is worked out from the data rather than assumed - a
    sweep centred on the robot's odometry position is in the odom frame, a
    sweep centred on nothing in particular is already in the body frame."""
    n = int(n or MAP_PX)
    raw, pts = snap["raw"], snap["cloud"]
    if raw is None:
        img = np.zeros((n, n, 3), np.uint8)
        img[:, :] = (7, 9, 13)
        return Image.fromarray(img), "no-camera"
    img = raw.copy()
    h, w = img.shape[:2]
    if pts is None or not len(pts):
        return Image.fromarray(img), "no-cloud"

    cx_o, cy_o, yaw = snap["pose"]
    mid = pts[:, :2].mean(axis=0)
    in_odom = math.dist(mid, (cx_o, cy_o)) < math.hypot(*mid) * 0.6 + 1.5
    if in_odom:
        d = pts[:, :2] - np.array([cx_o, cy_o], np.float32)
        c, sn = math.cos(-yaw), math.sin(-yaw)
        body = np.stack([d[:, 0] * c - d[:, 1] * sn,
                         d[:, 0] * sn + d[:, 1] * c,
                         pts[:, 2]], 1)
    else:
        body = pts

    # camera axes: +z forward, +x right, +y down; lidar: +x fwd, +y left, +z up
    X, Y, Z = -body[:, 1], -body[:, 2], body[:, 0]
    front = Z > 0.25
    if not front.any():
        return Image.fromarray(img), "behind"
    X, Y, Z, rng = X[front], Y[front], Z[front], Z[front]
    # The calibration is for 640x480 and the stream is 1280x720 - a
    # different aspect, so a plain scale is only an estimate. fov, dx and
    # dy are the three knobs that make it land, and they are on screen
    # because guessing them from a spec sheet has not worked.
    sc = (w / 640.0) * fov
    u = (FX * sc * X / Z + w / 2 + dx).astype(np.int32)
    v = (FY * sc * Y / Z + h / 2 + dy).astype(np.int32)
    ok = (u >= 1) & (u < w - 1) & (v >= 1) & (v < h - 1)
    u, v, rng = u[ok], v[ok], rng[ok]
    if not len(u):
        return Image.fromarray(img), "off-frame"

    t = np.clip(rng / 8.0, 0, 1)                    # near red, far blue
    r = (255 * (1 - t)).astype(np.uint8)
    g = (90 + 110 * np.sin(t * math.pi)).astype(np.uint8)
    b = (60 + 195 * t).astype(np.uint8)
    for dy in (-1, 0, 1):                           # 3x3 so they read
        for dx in (-1, 0, 1):
            img[v + dy, u + dx, 0] = r
            img[v + dy, u + dx, 1] = g
            img[v + dy, u + dx, 2] = b

    out = Image.fromarray(img)
    out.thumbnail((n, n))
    pad = Image.new("RGB", (n, n), (8, 10, 15))
    pad.paste(out, (0, max(0, (n - out.height) // 2)))
    return pad, ("hit", len(u), "odom" if in_odom else "body")


# ---------------------------------------------------------------- the view

DARK, PANEL, INK, DIM = "#0d0f14", "#161a22", "#e8ebf0", "#8891a3"
LINE, ACCENT, BAD, GOOD = "#2a313d", "#4da3ff", "#ff5f56", "#46c06c"

# Every word on screen, in both languages. The panel next door learned this
# the hard way: translating only the buttons leaves most of the window in
# one language whichever is picked.
T = {
    "title":    ("Go2 实时", "Go2 live"),
    "camera":   ("前置摄像头", "Front camera"),
    "lidar":    ("激光雷达", "LiDAR"),
    "v_map":    ("俯视图", "Floor plan"),
    "v_3d":     ("3D", "3D"),
    "v_front":  ("前视", "Front"),
    "v_fuse":   ("相机+雷达", "Camera+LiDAR"),
    "floor":    ("含地面", "with floor"),
    "followdog": ("跟随狗", "follow dog"),
    "clear":    ("清空", "Clear"),
    "keep":     ("保留", "Keep"),
    "keep_all": ("全部", "all"),
    "sec":      ("秒", "s"),
    "gait":     ("步态", "Gait"),
    "g_walk":   ("慢走", "Walk"),
    "g_trot":   ("普通", "Trot"),
    "g_run":    ("快跑", "Run"),
    "spd":      ("走路速度", "Speed"),
    "turn":     ("转向速度", "Turn"),
    "allow":    ("允许运动 —— W A S D 走 · Q E 转 · 空格停",
                 "Allow motion — W A S D drive · Q E turn · space stops"),
    "stand":    ("站起来", "Stand up"),
    "balance":  ("平衡站立", "Balance"),
    "sit":      ("趴下", "Lie down"),
    "stop":     ("■ 停止", "■ STOP"),
    "own":      ("狗自己的本事", "What the dog can do by itself"),
    "avoid":    ("避障(走路时自己绕开)", "Avoid obstacles while driving"),
    "lidar_on": ("开雷达", "LiDAR on"),
    "freewalk": ("自己走", "Walk itself"),
    "follow":   ("跟着我", "Follow me"),
    "nav_head": ("建图与导航 —— 按顺序来", "Mapping and navigation — in order"),
    "s1":       ("① 开雷达", "① LiDAR on"),
    "s2":       ("② 开始建图", "② Start mapping"),
    "s3":       ("③ 遛一圈", "③ Walk it around"),
    "s4":       ("④ 结束建图", "④ Stop mapping"),
    "s5":       ("⑤ 定位", "⑤ Localise"),
    "s6":       ("⑥ 导航", "⑥ Navigation"),
    "s7":       ("⑦ 双击地图设目标", "⑦ Double-click the map"),
    "s3hint":   ("用 WASD 慢慢走一圈,地图会长出来", "Drive it slowly with WASD; the map grows"),
    "conn":     ("已连接", "connected"),
    "waitcam":  ("等待摄像头", "waiting for the camera"),
    "waitcloud": ("等待点云", "waiting for a sweep"),
    "behind":   ("点都在身后", "every point is behind the camera"),
    "offframe": ("点云不在画面里", "the cloud misses the frame"),
    "fuse_ok":  ("{0} 点落在画面上 · 坐标系 {1} · 近红远蓝 · 下面三个滑块调对齐",
                 "{0} points on the picture · frame {1} · near red, far blue · "
                 "the three sliders below align it"),
    "fov":      ("视角", "zoom"),
    "ofsx":     ("左右", "x"),
    "ofsy":     ("上下", "y"),
    "view_m":   ("视野 {0} m", "{0} m across"),
    "orbit":    ("距离 {0} m · 拖动旋转 · {1} 点", "{0} m away · drag to orbit · {1} points"),
    "slam_pts": ("狗的SLAM {0} 点", "dog's SLAM {0} pts"),
    "grid_c":   ("地图 {0} 格", "map {0} cells"),
    "cloud_age": ("点云 {0} s 前", "sweep {0} s ago"),
    "nocloud":  ("等待点云", "no sweep yet"),
    "zline":    ("本帧 {0}/{1} 点算作墙 · 高度 {2} ~ {3} m(地面 {4})",
                 "{0}/{1} points counted as wall · heights {2} to {3} m (floor {4})"),
    "nozline":  ("高度范围:还没有点云", "heights: no sweep yet"),
    "telem":    ("前进 {0} m/s  侧移 {1} m/s  脚底 {2}\n收到  状态 {3} · 运动 {4} · 点云 {5} · 画面 {6}",
                 "fwd {0} m/s  side {1} m/s  feet {2}\nreceived  state {3} · motion {4} · cloud {5} · frames {6}"),
    "hdr":      ("{0}   电池 {1}%  {2} V   模式 {3}   身高 {4} m   朝向 {5}°",
                 "{0}   battery {1}%  {2} V   mode {3}   height {4} m   heading {5}°"),
}
LANG = {"i": 0}


def t(key, *a):
    pair = T.get(key)
    if not pair:
        return key
    out = pair[LANG["i"]]
    for i, v in enumerate(a):
        out = out.replace("{%d}" % i, str(v))
    return out


class Btn(tk.Label):
    """A button that is actually the colour it is told to be.

    On macOS tk.Button is drawn by the system: bg and fg are ignored, so
    every button came out white-on-white and unreadable. A Label does
    honour them, and a Label with <Button-1> bound is a button."""

    def __init__(self, parent, text, command, bg=PANEL, fg=INK, bold=False,
                 pad=(14, 9), **kw):
        super().__init__(parent, text=text, bg=bg, fg=fg,
                         font=("Helvetica", 12, "bold" if bold else "normal"),
                         padx=pad[0], pady=pad[1], cursor="pointinghand",
                         highlightthickness=1, highlightbackground=LINE, **kw)
        self._bg, self._cmd, self._key = bg, command, None
        self.bind("<Button-1>", self._press)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Enter>", lambda _e: self.configure(highlightbackground=ACCENT))
        self.bind("<Leave>", lambda _e: self.configure(bg=self._bg,
                                                       highlightbackground=LINE))

    def _press(self, _e):
        self.configure(bg=_lighten(self._bg))

    def _release(self, _e):
        self.configure(bg=self._bg)
        self._cmd()

    def paint(self, on, bg=None):
        self._bg = (ACCENT if on else (bg or PANEL))
        self.configure(bg=self._bg, fg=("#07090d" if on else INK))


def _lighten(hexcol, by=28):
    r, g, b = (int(hexcol[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % (min(r + by, 255), min(g + by, 255),
                              min(b + by, 255))

class App:
    """One window, everything in it, and it resizes.

    The panels are drawn at whatever size the window gives them rather than
    at a fixed one, so full screen means a bigger map instead of a bigger
    grey border. The order top to bottom is the order anyone works in:
    what the dog sees, how fast it may go, what it can do by itself, and
    then the mapping sequence - numbered, because "navigation" is four
    steps that have to happen first and nothing on screen said so."""

    def __init__(self, root, target):
        self.root, self.target = root, target
        self.metres = 16.0
        self.az, self.el, self.dist = -0.6, 0.45, 9.0
        self.fov, self.ofsx, self.ofsy = 1.0, 0.0, 0.0
        self._drag_from = None
        self.keys = set()
        self.cam_size = (CAM_W, CAM_H)
        self.map_size = MAP_PX
        self._logn = -1
        root.configure(bg=DARK)
        root.minsize(900, 620)

        root.rowconfigure(1, weight=1)
        root.columnconfigure(0, weight=3, uniform="col")
        root.columnconfigure(1, weight=4, uniform="col")

        # ---- header
        top = tk.Frame(root, bg=PANEL)
        top.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.hdr = tk.Label(top, text="", fg=INK, bg=PANEL, anchor="w",
                            font=("Menlo", 12), padx=10, pady=6)
        self.hdr.pack(side="left", fill="x", expand=True)
        self.lang_btn = Btn(top, "EN", self.toggle_lang, pad=(10, 4))
        self.lang_btn.pack(side="right", padx=8, pady=4)

        # ---- camera, left
        left = tk.Frame(root, bg=DARK)
        left.grid(row=1, column=0, sticky="nsew", padx=(8, 4), pady=6)
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)
        self.cam_lbl = tk.Label(left, text="", fg=INK, bg=DARK, anchor="w",
                                font=("Helvetica", 12, "bold"))
        self.cam_lbl.grid(row=0, column=0, sticky="w", pady=(0, 3))
        self.cam = tk.Label(left, bg="#07090d")
        self.cam.grid(row=1, column=0, sticky="nsew")
        self.cam.bind("<Configure>",
                      lambda e: setattr(self, "cam_size", (e.width, e.height)))
        self.tele = tk.Label(left, text="", fg=DIM, bg=DARK, justify="left",
                             anchor="w", font=("Menlo", 10))
        self.tele.grid(row=2, column=0, sticky="ew", pady=(5, 0))
        self.build_controls(left, row=3)

        # ---- lidar, right
        right = tk.Frame(root, bg=DARK)
        right.grid(row=1, column=1, sticky="nsew", padx=(4, 8), pady=6)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        head = tk.Frame(right, bg=DARK)
        head.grid(row=0, column=0, sticky="ew", pady=(0, 3))
        self.lidar_lbl = tk.Label(head, text="", fg=INK, bg=DARK,
                                  font=("Helvetica", 12, "bold"))
        self.lidar_lbl.pack(side="left", padx=(0, 8))
        self.view = "map"
        self._vbtn = {}
        for key in ("map", "3d", "front", "fuse"):
            b = Btn(head, "", lambda k=key: self.set_view(k), pad=(9, 3))
            b.pack(side="left", padx=2)
            self._vbtn[key] = b
        self.clear_btn = Btn(head, "", self.clear, pad=(10, 3))
        self.clear_btn.pack(side="right", padx=2)
        for txt, fn in (("+", self.zoom_in), ("−", self.zoom_out)):
            Btn(head, txt, fn, pad=(10, 3)).pack(side="right", padx=2)
        self.follow = tk.BooleanVar(value=True)
        self.follow_cb = ttk.Checkbutton(head, text="", variable=self.follow)
        self.follow_cb.pack(side="right", padx=6)
        self.floor = tk.BooleanVar(value=False)
        self.floor_cb = ttk.Checkbutton(head, text="", variable=self.floor)
        self.floor_cb.pack(side="right", padx=6)

        self.map = tk.Label(right, bg="#0a0d12")
        self.map.grid(row=1, column=0, sticky="nsew")
        self.map.bind("<Configure>", self._map_resize)
        self.map.bind("<Button-1>", self._drag_start)
        self.map.bind("<B1-Motion>", self._drag)
        self.map.bind("<Double-Button-1>", self.click_map)

        # how much of the past to keep, and the fusion knobs
        self.keepv = tk.IntVar(value=30)
        krow = tk.Frame(right, bg=DARK)
        krow.grid(row=2, column=0, sticky="ew", pady=(5, 0))
        self.keep_lbl = tk.Label(krow, text="", fg=DIM, bg=DARK,
                                 font=("Helvetica", 11))
        self.keep_lbl.pack(side="left", padx=(0, 6))
        self._kbtn = {}
        for secs in KEEP_CHOICES:
            b = Btn(krow, "", lambda x=secs: self.set_keep(x), pad=(9, 3))
            b.pack(side="left", padx=2)
            self._kbtn[secs] = b

        self.fuse_row = tk.Frame(right, bg=DARK)
        self.fuse_vars = {}
        for i, (name, lo, hi, init) in enumerate((
                ("fov", 0.4, 2.5, 1.0), ("ofsx", -300, 300, 0),
                ("ofsy", -300, 300, 0))):
            v = tk.DoubleVar(value=init)
            lab = tk.Label(self.fuse_row, text="", fg=DIM, bg=DARK,
                           font=("Helvetica", 10))
            lab.grid(row=0, column=i * 2, sticky="e", padx=(6, 2))
            ttk.Scale(self.fuse_row, from_=lo, to=hi, variable=v, length=120,
                      command=lambda *_: None).grid(row=0, column=i * 2 + 1)
            self.fuse_vars[name] = (v, lab)
        self.fuse_row.grid(row=3, column=0, sticky="ew", pady=(4, 0))
        self.fuse_row.grid_remove()

        self.mapinfo = tk.Label(right, text="", fg=DIM, bg=DARK, anchor="w",
                                justify="left", font=("Menlo", 10))
        self.mapinfo.grid(row=4, column=0, sticky="ew", pady=(5, 2))
        self.log = tk.Text(right, height=6, bg="#07090d", fg="#b9c1cf",
                           font=("Menlo", 10), relief="flat", wrap="none",
                           highlightthickness=1, highlightbackground=LINE)
        self.log.grid(row=5, column=0, sticky="ew")
        self.log.configure(state="disabled")

        self.set_view("map")
        self.set_gait("trot")
        self.set_keep(30)
        self.retext()
        root.bind("<KeyPress>", self.down)
        root.bind("<KeyRelease>", self.up)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.focus_set()
        self.tick()

    # ------------------------------------------------------------ controls
    def build_controls(self, parent, row):
        box = tk.Frame(parent, bg=DARK)
        box.grid(row=row, column=0, sticky="ew", pady=(8, 0))
        box.columnconfigure(0, weight=1)

        gait = tk.Frame(box, bg=DARK)
        gait.grid(row=0, column=0, sticky="ew")
        self.gait_lbl = tk.Label(gait, text="", fg=DIM, bg=DARK,
                                 font=("Helvetica", 11))
        self.gait_lbl.pack(side="left", padx=(0, 6))
        self.gait = "trot"
        self._gbtn = {}
        for key in GAITS:
            b = Btn(gait, "", lambda k=key: self.set_gait(k), pad=(8, 5))
            b.pack(side="left", padx=2)
            self._gbtn[key] = b

        sl = tk.Frame(box, bg=DARK)
        sl.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        sl.columnconfigure(1, weight=1)
        self.vsp, self.wsp = tk.DoubleVar(value=0.30), tk.DoubleVar(value=0.50)
        self._labs = []
        g = GAITS[self.gait]
        for i, (var, lo, hi, unit, key) in enumerate((
                (self.vsp, 0.05, g["vx"], "m/s", "spd"),
                (self.wsp, 0.10, g["vyaw"], "rad/s", "turn"))):
            name = tk.Label(sl, text="", fg=DIM, bg=DARK,
                            font=("Helvetica", 11))
            name.grid(row=i, column=0, sticky="w", pady=2)
            sc = ttk.Scale(sl, from_=lo, to=hi, variable=var,
                           command=lambda *_: self.apply())
            sc.grid(row=i, column=1, sticky="ew", padx=6)
            val = tk.Label(sl, text="", fg=INK, bg=DARK, font=("Menlo", 11),
                           width=10, anchor="w")
            val.grid(row=i, column=2, sticky="w")
            setattr(self, "_sc%d" % i, sc)
            self._labs.append((name, val, var, unit, key))

        self.allow = tk.BooleanVar(value=False)
        self.allow_cb = ttk.Checkbutton(box, text="", variable=self.allow,
                                        command=self.on_allow)
        self.allow_cb.grid(row=2, column=0, sticky="ew", pady=(8, 4))

        btns = tk.Frame(box, bg=DARK)
        btns.grid(row=3, column=0, sticky="ew")
        self._abtn = {}
        for cmd, colour in (("stand", PANEL), ("balance", PANEL),
                            ("sit", PANEL), ("stop", BAD)):
            b = Btn(btns, "", lambda c=cmd: self.send(c), bg=colour,
                    fg="#ffffff" if colour != PANEL else INK,
                    bold=(cmd == "stop"), pad=(6, 9))
            b.pack(side="left", expand=True, fill="x", padx=2)
            self._abtn[cmd] = b

        own = tk.Frame(box, bg=DARK)
        own.grid(row=4, column=0, sticky="ew", pady=(10, 2))
        self.own_lbl = tk.Label(own, text="", fg=DIM, bg=DARK,
                                font=("Helvetica", 11))
        self.own_lbl.pack(anchor="w")
        r1 = tk.Frame(own, bg=DARK)
        r1.pack(fill="x", pady=2)
        self.avoid = tk.BooleanVar(value=False)
        self.avoid_cb = ttk.Checkbutton(r1, text="", variable=self.avoid,
                                        command=self.on_avoid)
        self.avoid_cb.pack(side="left")
        self._obtn = {}
        for cmd in ("follow", "freewalk"):
            b = Btn(r1, "", lambda c=cmd: self.send(
                "follow" if c == "follow" else "freeavoid"), pad=(10, 4))
            b.pack(side="right", padx=2)
            self._obtn[cmd] = b

        nav = tk.Frame(box, bg=DARK)
        nav.grid(row=5, column=0, sticky="ew", pady=(10, 2))
        self.nav_lbl = tk.Label(nav, text="", fg=DIM, bg=DARK,
                                font=("Helvetica", 11))
        self.nav_lbl.pack(anchor="w")
        g1 = tk.Frame(nav, bg=DARK)
        g1.pack(fill="x", pady=2)
        self._sbtn = {}
        for key, cmd in (("s1", "lidar"), ("s2", "mapping/start"),
                         ("s4", "mapping/stop")):
            b = Btn(g1, "", lambda c=cmd: self.step(c), pad=(6, 5))
            b.pack(side="left", expand=True, fill="x", padx=1)
            self._sbtn[key] = b
        g2 = tk.Frame(nav, bg=DARK)
        g2.pack(fill="x", pady=2)
        for key, cmd in (("s5", "localization/start"),
                         ("s6", "navigation/start")):
            b = Btn(g2, "", lambda c=cmd: self.step(c), pad=(6, 5))
            b.pack(side="left", expand=True, fill="x", padx=1)
            self._sbtn[key] = b
        self.step_hint = tk.Label(nav, text="", fg=ACCENT, bg=DARK,
                                  anchor="w", justify="left",
                                  font=("Helvetica", 11))
        self.step_hint.pack(fill="x", pady=(3, 0))

    # --------------------------------------------------------------- text
    def toggle_lang(self):
        LANG["i"] = 1 - LANG["i"]
        self.retext()

    def retext(self):
        z = LANG["i"]
        self.root.title(t("title") + " · " + self.target)
        self.lang_btn.configure(text="中文" if z == 0 else "EN")
        self.cam_lbl.configure(text=t("camera"))
        self.lidar_lbl.configure(text=t("lidar"))
        for k, b in self._vbtn.items():
            b.configure(text=t("v_" + k))
        self.clear_btn.configure(text=t("clear"))
        self.follow_cb.configure(text=t("followdog"))
        self.floor_cb.configure(text=t("floor"))
        self.keep_lbl.configure(text=t("keep"))
        for secs, b in self._kbtn.items():
            b.configure(text=(t("keep_all") if secs == 0
                              else f"{secs}{t('sec')}"))
        for name, (_v, lab) in self.fuse_vars.items():
            lab.configure(text=t(name))
        self.gait_lbl.configure(text=t("gait"))
        for k, b in self._gbtn.items():
            b.configure(text=f"{t('g_' + k)} {GAITS[k]['vx']:.1f}")
        for name, _val, _var, _u, key in self._labs:
            name.configure(text=t(key))
        self.allow_cb.configure(text=t("allow"))
        for k, b in self._abtn.items():
            b.configure(text=t(k))
        self.own_lbl.configure(text=t("own"))
        self.avoid_cb.configure(text=t("avoid"))
        for k, b in self._obtn.items():
            b.configure(text=t(k))
        self.nav_lbl.configure(text=t("nav_head"))
        for k, b in self._sbtn.items():
            b.configure(text=t(k))
        self.set_view(self.view)
        self.set_gait(self.gait)
        self.set_keep(self.keepv.get())
        self._logn = -1

    # ------------------------------------------------------------- actions
    def send(self, cmd):
        if cmd == "stop":
            self.stop()
        with drive_lock:
            DRIVE["cmd"] = cmd

    def step(self, cmd):
        if cmd == "lidar":
            return self.send("lidar_on")
        self.uslam(cmd)

    def uslam(self, text):
        with drive_lock:
            DRIVE["cmd"] = "uslam:" + text

    def on_avoid(self):
        on = bool(self.avoid.get())
        with drive_lock:
            DRIVE["avoid"] = on
            DRIVE["cmd"] = "avoid_on" if on else "avoid_off"

    def set_keep(self, secs):
        self.keepv.set(secs)
        with SH.lock:
            SH.keep = float(secs)
        for k, b in self._kbtn.items():
            b.paint(k == secs)

    def set_gait(self, key):
        self.gait = key
        g = GAITS[key]
        self._sc0.configure(to=g["vx"])
        self._sc1.configure(to=g["vyaw"])
        if self.vsp.get() > g["vx"]:
            self.vsp.set(g["vx"])
        if self.wsp.get() > g["vyaw"]:
            self.wsp.set(g["vyaw"])
        for k, b in self._gbtn.items():
            b.paint(k == key)
        with drive_lock:
            DRIVE["cmd"] = key
        self.apply()

    def set_view(self, key):
        self.view = key
        if key == "front":
            self.az, self.el = 0.0, 0.08
        for k, b in self._vbtn.items():
            b.paint(k == key)
        if key == "fuse":
            self.fuse_row.grid()
        else:
            self.fuse_row.grid_remove()

    def _map_resize(self, e):
        self.map_size = max(160, min(e.width, e.height))

    def _drag_start(self, e):
        self._drag_from = (e.x, e.y, self.az, self.el)

    def _drag(self, e):
        if self.view in ("map", "fuse") or not self._drag_from:
            return
        x0, y0, az0, el0 = self._drag_from
        self.az = az0 - (e.x - x0) * 0.008
        self.el = max(-0.25, min(1.45, el0 + (e.y - y0) * 0.008))

    def zoom_in(self):
        if self.view == "map":
            self.metres = max(4.0, self.metres / 1.5)
        else:
            self.dist = max(1.5, self.dist / 1.4)

    def zoom_out(self):
        if self.view == "map":
            self.metres = min(70.0, self.metres * 1.5)
        else:
            self.dist = min(80.0, self.dist * 1.4)

    def clear(self):
        SH.clear_map()

    def click_map(self, e):
        """Double-click the floor plan to send the dog there."""
        if self.view != "map" or not self.allow.get():
            return
        s = SH.snap()
        cx, cy = s["pose"][:2] if self.follow.get() else (0.0, 0.0)
        ppm = self.map_size / self.metres
        gx = cx + (e.x - self.map_size / 2) / ppm
        gy = cy - (e.y - self.map_size / 2) / ppm
        SH.put(goal=(gx, gy))
        self.uslam(f"navigation/set_goal_pose/{gx:.3f}/{gy:.3f}/0.000")

    # ---------------------------------------------------------------- keys
    def on_allow(self):
        with drive_lock:
            DRIVE["allowed"] = bool(self.allow.get())
        if not self.allow.get():
            self.stop()

    def stop(self):
        self.keys.clear()
        self.allow.set(False)
        with drive_lock:
            DRIVE.update(vx=0.0, vy=0.0, vyaw=0.0, allowed=False)

    def down(self, e):
        k = (e.keysym or "").lower()
        if k in ("space", "escape"):
            return self.stop()
        if k in ("plus", "equal"):
            return self.zoom_in()
        if k == "minus":
            return self.zoom_out()
        if k in "wasdqe":
            self.keys.add(k)
            self.apply()

    def up(self, e):
        k = (e.keysym or "").lower()
        if k in self.keys:
            self.keys.discard(k)
            self.apply()

    def apply(self):
        k, v, w = self.keys, self.vsp.get(), self.wsp.get()
        with drive_lock:
            DRIVE.update(vx=v * (("w" in k) - ("s" in k)),
                         vy=min(v, MAX_VY) * (("a" in k) - ("d" in k)),
                         vyaw=w * (("q" in k) - ("e" in k)))

    def close(self):
        self.stop()
        self.root.after(300, self.root.destroy)

    # --------------------------------------------------------------- paint
    def nav_state(self, slog):
        """Which step the mapping sequence is on, from what the dog said."""
        txt = " ".join(slog[-25:])
        if "navigation/start/success" in txt:
            return "s7", t("s7")
        if "localization" in txt and "succeed" in txt.lower():
            return "s6", t("s6")
        if "mapping/stop/success" in txt:
            return "s5", t("s5")
        if "mapping/start/success" in txt:
            return "s3", t("s3") + " — " + t("s3hint")
        if "utlidar/switch" in txt:
            return "s2", t("s2")
        return "s1", t("s1")

    def tick(self):
        s = SH.snap()
        st, c = s["state"], s["counts"]

        cw, ch = self.cam_size
        if s["frame"] is not None and cw > 20 and ch > 20:
            im = s["frame"].copy()
            im.thumbnail((cw, ch))
            self._cam = ImageTk.PhotoImage(im)
            self.cam.configure(image=self._cam)

        n = self.map_size
        extra = None
        if self.view == "map":
            pic = render_map(s, self.metres, self.follow.get(), n=n)
        elif self.view == "fuse":
            fv = {k: v[0].get() for k, v in self.fuse_vars.items()}
            pic, extra = render_overlay(s, n=n, dx=fv["ofsx"], dy=fv["ofsy"],
                                        fov=fv["fov"])
            for name, (v, lab) in self.fuse_vars.items():
                lab.configure(text=f"{t(name)} {v.get():.0f}"
                              if name != "fov" else f"{t(name)} {v.get():.2f}")
        else:
            az = self.az + (s["pose"][2] if self.view == "front" else 0.0)
            pic = render_3d(s, az, self.el, self.dist, self.follow.get(),
                            drop_floor=not self.floor.get(), n=n)
        self._map = ImageTk.PhotoImage(pic)
        self.map.configure(image=self._map)

        for _nm, val, var, unit, _k in self._labs:
            val.configure(text=f"{var.get():.2f} {unit}")

        rpy = st.get("rpy") or [0, 0, 0]
        dot = "●" if s["note"] == "connected" else "○"
        note = t("conn") if s["note"] == "connected" else s["note"]
        self.hdr.configure(text=t("hdr", f"{dot} {note}", st.get("soc", "–"),
                                 st.get("volt", "–"), st.get("mode", "–"),
                                 st.get("height", "–"),
                                 f"{rpy[2] * 57.3:.1f}"))

        vel = st.get("vel") or [0, 0, 0]
        ft = st.get("foot") or [0, 0, 0, 0]
        self.tele.configure(text=t("telem", f"{vel[0]:+.2f}", f"{vel[1]:+.2f}",
                                  " ".join("%3d" % x for x in ft),
                                  c["low"], c["sport"], c["cloud"],
                                  c["frame"]))

        if self.view == "map":
            where = t("view_m", f"{self.metres:.0f}")
        elif self.view == "fuse":
            if extra and extra[0] == "hit":
                where = t("fuse_ok", f"{extra[1]:,}", extra[2])
            else:
                where = {"no-camera": t("waitcam"), "no-cloud": t("waitcloud"),
                         "behind": t("behind"),
                         "off-frame": t("offframe")}.get(extra, "")
        else:
            where = t("orbit", f"{self.dist:.0f}", f"{len(s['pts3']):,}")
        age = time.time() - s["cloud_t"] if s["cloud_t"] else None
        zr = s["zrange"]
        bits = [where]
        if s.get("slam_n"):
            bits.append(t("slam_pts", f"{s['slam_n']:,}"))
        bits.append(t("grid_c", f"{s['cells']:,}"))
        bits.append(t("cloud_age", f"{age:.1f}") if age is not None
                    else t("nocloud"))
        line2 = (t("zline", f"{zr[3]:,}", f"{zr[4]:,}", f"{zr[0]:+.2f}",
                   f"{zr[1]:+.2f}", f"{zr[2]:+.2f}") if zr else t("nozline"))
        self.mapinfo.configure(text="   ".join(bits) + "\n" + line2)

        slog = s.get("slog") or []
        step, hint = self.nav_state(slog)
        for k, b in self._sbtn.items():
            b.paint(k == step, bg=PANEL)
        self.step_hint.configure(text="→ " + hint)

        if len(slog) != self._logn:
            self._logn = len(slog)
            self.log.configure(state="normal")
            self.log.delete("1.0", "end")
            self.log.insert("1.0", "\n".join(slog[-6:]))
            self.log.configure(state="disabled")

        self.root.after(60, self.tick)

def set_icon(root):
    """A dog on the Dock and in the title bar, drawn rather than shipped.

    A file would be one more thing to keep next to the script and one more
    thing to lose; sixty lines of rectangles cost nothing and cannot go
    missing."""
    try:
        n = 128
        im = Image.new("RGBA", (n, n), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        d.rounded_rectangle([10, 10, 118, 118], 26, fill=(22, 26, 34, 255),
                            outline=(77, 163, 255, 255), width=3)
        body = (210, 218, 230, 255)
        d.rounded_rectangle([30, 52, 86, 76], 7, fill=body)       # torso
        d.rounded_rectangle([80, 42, 100, 62], 6, fill=body)      # head
        d.ellipse([86, 34, 98, 46], fill=(77, 163, 255, 255))     # lidar
        for x in (34, 52, 66, 80):                                # legs
            d.rounded_rectangle([x, 74, x + 6, 96], 3, fill=body)
        d.line([(100, 52), (116, 52)], fill=(255, 95, 86, 255), width=4)
        for r, a in ((16, 70), (26, 46), (36, 26)):               # sweep
            d.arc([96 - r, 52 - r, 96 + r, 52 + r], -40, 40,
                  fill=(77, 163, 255, a), width=3)
        root._icon = ImageTk.PhotoImage(im)
        root.iconphoto(True, root._icon)
    except Exception:
        pass


def wifi_target():
    """'ap', or the dog's IP on a router - the file the panel also reads."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        ".go2_wifi_ip")
    try:
        return open(path).read().strip() or "ap"
    except OSError:
        return "ap"


def main():
    # --dds <iface> drives the window over DDS: the simulator in the VM, or
    # the real dog on the cable. Without it, WebRTC over the dog's Wi-Fi.
    args = sys.argv[1:]
    dds_iface = None
    if "--dds" in args:
        i = args.index("--dds")
        dds_iface = args[i + 1] if i + 1 < len(args) else "en0"
        args = args[:i] + args[i + 2:]
    target = args[0] if args else wifi_target()

    loop = asyncio.new_event_loop()
    task = {}

    def run():
        asyncio.set_event_loop(loop)
        task["t"] = loop.create_task(
            dds_loop(dds_iface) if dds_iface else robot_loop(target))
        try:
            loop.run_until_complete(task["t"])
        except SystemExit:
            SH.put(note="could not connect - see the terminal")
        except Exception as e:
            SH.put(note=f"disconnected: {e}")

    threading.Thread(target=run, daemon=True).start()

    root = tk.Tk()
    set_icon(root)
    App(root, "DDS " + dds_iface if dds_iface else target)
    try:
        root.mainloop()
    finally:
        # let the robot loop send StopMove and hang up: a closed window must
        # not leave the dog walking
        try:
            loop.call_soon_threadsafe(task["t"].cancel)
        except Exception:
            pass
        time.sleep(1.0)


if __name__ == "__main__":
    main()
