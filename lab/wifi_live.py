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
import tkinter as tk
from tkinter import ttk

import numpy as np
from PIL import Image, ImageTk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wifi_common import connect, disconnect                 # noqa: E402

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

# The 3D view keeps its own copy of the cloud, thinned to one point per 8 cm
# box. Without thinning a few minutes of sweeps is tens of millions of
# points, almost all of them landing on a pixel another point already owns.
VOX3 = 0.08
MAX3 = 400_000


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
        self.pts3 = np.zeros((0, 3), np.float32)    # the 3D view's cloud
        self.keys3 = np.zeros(0, np.int64)          # its voxel keys, sorted
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
            self.pts3 = np.zeros((0, 3), np.float32)
            self.keys3 = np.zeros(0, np.int64)


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
    """Keep one point per 8 cm box, for the 3D view.

    Deduplicating with a sorted key array and searchsorted rather than a
    Python set: twenty thousand points a sweep, fifteen sweeps a second,
    and a set would spend more time hashing than the renderer spends
    drawing."""
    if pts is None or len(pts) < 10:
        return
    v = np.floor(pts / VOX3).astype(np.int64)
    # pack into one integer; the offsets keep every real coordinate positive
    k = ((v[:, 0] + 2000) * 4000 + (v[:, 1] + 2000)) * 2000 + (v[:, 2] + 500)
    k, first = np.unique(k, return_index=True)
    with SH.lock:
        have = SH.keys3
        if len(have):
            pos = np.searchsorted(have, k)
            pos = np.clip(pos, 0, len(have) - 1)
            fresh = have[pos] != k
        else:
            fresh = np.ones(len(k), bool)
        if not fresh.any():
            return
        new_pts = pts[first[fresh]]
        SH.pts3 = np.concatenate([SH.pts3, new_pts])[-MAX3:]
        SH.keys3 = np.union1d(have, k[fresh])


async def robot_loop(target):
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

def render_map(snap, metres, follow=True):
    """The accumulated grid as a floor plan, with the dog and its trail."""
    n = MAP_PX
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


def render_3d(snap, az, el, dist, follow=True, drop_floor=True):
    """The accumulated cloud from any angle, coloured by height.

    Perspective, drawn with numpy straight into the pixels: four hundred
    thousand points is nothing as arrays and impossible as canvas items.
    Painted far-to-near so the nearer surface wins the pixel."""
    n = MAP_PX
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


def render_overlay(snap):
    """The current sweep drawn on the camera picture, coloured by range.

    This is the fusion of the two sensors, and it is also the only honest
    way to judge the mount: if the points land on the walls and the floor
    in the picture, the extrinsics are right; if they sit an arm's length
    off, they are not, and no amount of colouring a cloud will fix it.

    The cloud's frame is worked out from the data rather than assumed - a
    sweep centred on the robot's odometry position is in the odom frame, a
    sweep centred on nothing in particular is already in the body frame."""
    raw, pts = snap["raw"], snap["cloud"]
    if raw is None:
        img = np.zeros((CAM_H, CAM_W, 3), np.uint8)
        img[:, :] = (7, 9, 13)
        return Image.fromarray(img), "等待摄像头"
    img = raw.copy()
    h, w = img.shape[:2]
    if pts is None or not len(pts):
        return Image.fromarray(img), "等待点云"

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
        return Image.fromarray(img), "点都在身后"
    X, Y, Z, rng = X[front], Y[front], Z[front], Z[front]
    sc = w / 640.0
    u = (FX * sc * X / Z + w / 2).astype(np.int32)
    v = (FY * sc * Y / Z + h / 2).astype(np.int32)
    ok = (u >= 1) & (u < w - 1) & (v >= 1) & (v < h - 1)
    u, v, rng = u[ok], v[ok], rng[ok]
    if not len(u):
        return Image.fromarray(img), "点云不在画面里"

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
    out.thumbnail((MAP_PX, MAP_PX))
    pad = Image.new("RGB", (MAP_PX, MAP_PX), (8, 10, 15))
    pad.paste(out, (0, (MAP_PX - out.height) // 2))
    return pad, (f"{len(u):,} 点落在画面上 · "
                 f"坐标系 {'odom(已转到机身)' if in_odom else '机身'} · 近红远蓝")


# ---------------------------------------------------------------- the view

DARK, PANEL, INK, DIM = "#0d0f14", "#161a22", "#e8ebf0", "#8891a3"
LINE, ACCENT, BAD = "#2a313d", "#4da3ff", "#ff5f56"


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
        self._bg, self._cmd = bg, command
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


def _lighten(hexcol, by=28):
    r, g, b = (int(hexcol[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % (min(r + by, 255), min(g + by, 255),
                              min(b + by, 255))


class App:
    def __init__(self, root, target):
        self.root = root
        self.metres = 16.0
        self.az, self.el, self.dist = -0.6, 0.45, 9.0
        self._drag_from = None
        self.keys = set()               # before anything that calls apply()
        self.follow = tk.BooleanVar(value=True)
        root.title(f"Go2 · {target}")
        root.configure(bg=DARK)

        top = tk.Frame(root, bg=PANEL)
        top.pack(fill="x")
        self.hdr = tk.Label(top, text="", fg=INK, bg=PANEL, anchor="w",
                            font=("Menlo", 12), padx=10, pady=6)
        self.hdr.pack(fill="x")

        body = tk.Frame(root, bg=DARK)
        body.pack(padx=8, pady=6)

        left = tk.Frame(body, bg=DARK)
        left.grid(row=0, column=0, sticky="n", padx=(0, 8))
        tk.Label(left, text="前置摄像头", fg=INK, bg=DARK, anchor="w",
                 font=("Helvetica", 12, "bold")).pack(fill="x", pady=(0, 3))
        self._blank = ImageTk.PhotoImage(
            Image.new("RGB", (CAM_W, CAM_H), (7, 9, 13)))
        self.cam = tk.Label(left, bg="#07090d", image=self._blank)
        self.cam.pack()
        self.tele = tk.Label(left, text="", fg=DIM, bg=DARK, justify="left",
                             anchor="w", font=("Menlo", 10))
        self.tele.pack(fill="x", pady=(5, 0))
        self.build_controls(left)

        right = tk.Frame(body, bg=DARK)
        right.grid(row=0, column=1, sticky="n")
        head = tk.Frame(right, bg=DARK)
        head.pack(fill="x", pady=(0, 3))
        tk.Label(head, text="激光雷达", fg=INK, bg=DARK,
                 font=("Helvetica", 12, "bold")).pack(side="left", padx=(0, 8))
        self.view = "map"
        self._vbtn = {}
        for key, name in (("map", "俯视图"), ("3d", "3D"), ("front", "前视"),
                          ("fuse", "相机+雷达")):
            b = Btn(head, name, lambda k=key: self.set_view(k), pad=(9, 3))
            b.pack(side="left", padx=2)
            self._vbtn[key] = b
        self.floor = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="含地面",
                        variable=self.floor).pack(side="left", padx=8)
        for txt, fn in (("清空", self.clear), ("+", self.zoom_in),
                        ("−", self.zoom_out)):
            Btn(head, txt, fn, pad=(10, 3)).pack(side="right", padx=2)
        ttk.Checkbutton(head, text="跟随狗",
                        variable=self.follow).pack(side="right", padx=8)
        self._mblank = ImageTk.PhotoImage(
            Image.new("RGB", (MAP_PX, MAP_PX), (10, 13, 18)))
        self.map = tk.Label(right, bg="#0a0d12", image=self._mblank)
        self.map.pack()
        # drag to orbit in 3D; the map view ignores it
        self.map.bind("<Button-1>", self._drag_start)
        self.map.bind("<B1-Motion>", self._drag)
        self.map.bind("<Double-Button-1>", self.click_map)
        self.mapinfo = tk.Label(right, text="", fg=DIM, bg=DARK, anchor="w",
                                justify="left", font=("Menlo", 10))
        self.mapinfo.pack(fill="x", pady=(5, 2))

        # Every command and every reply, so a button that appears to do
        # nothing can be told apart from one whose reply nobody showed.
        self.log = tk.Text(right, height=7, bg="#07090d", fg="#b9c1cf",
                           font=("Menlo", 10), relief="flat", wrap="none",
                           highlightthickness=1, highlightbackground=LINE)
        self.log.pack(fill="x")
        self.log.configure(state="disabled")
        self._logn = 0

        self.set_view("map")
        self.set_gait("trot")
        root.bind("<KeyPress>", self.down)
        root.bind("<KeyRelease>", self.up)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.focus_set()
        self.tick()

    # ------------------------------------------------------------ controls
    def build_controls(self, parent):
        gait = tk.Frame(parent, bg=DARK)
        gait.pack(fill="x", pady=(10, 2))
        tk.Label(gait, text="步态", fg=DIM, bg=DARK,
                 font=("Helvetica", 11)).pack(side="left", padx=(0, 6))
        self.gait = "trot"
        self._gbtn = {}
        for key, g in GAITS.items():
            b = Btn(gait, f"{g['name']}  {g['vx']:.1f}", 
                    lambda k=key: self.set_gait(k), pad=(8, 5))
            b.pack(side="left", padx=2)
            self._gbtn[key] = b

        box = tk.Frame(parent, bg=DARK)
        box.pack(fill="x", pady=(6, 0))
        self.vsp = tk.DoubleVar(value=0.30)
        self.wsp = tk.DoubleVar(value=0.50)
        self._labs = []
        g = GAITS[self.gait]
        for row, (var, lo, hi, unit, name) in enumerate((
                (self.vsp, 0.05, g["vx"], "m/s", "走路速度"),
                (self.wsp, 0.10, g["vyaw"], "rad/s", "转向速度"))):
            tk.Label(box, text=name, fg=DIM, bg=DARK,
                     font=("Helvetica", 11)).grid(row=row, column=0,
                                                  sticky="w", pady=2)
            sc = ttk.Scale(box, from_=lo, to=hi, variable=var, length=200,
                           command=lambda *_: self.apply())
            sc.grid(row=row, column=1, padx=6)
            setattr(self, "_sc%d" % row, sc)
            lab = tk.Label(box, text="", fg=INK, bg=DARK,
                           font=("Menlo", 11), width=10, anchor="w")
            lab.grid(row=row, column=2, sticky="w")
            self._labs.append((lab, var, unit))

        self.allow = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent,
                        text="允许运动  ——  W A S D 走 · Q E 转 · 空格停",
                        variable=self.allow,
                        command=self.on_allow).pack(fill="x", pady=(10, 5))

        smart = tk.Frame(parent, bg=DARK)
        smart.pack(fill="x", pady=(10, 4))
        tk.Label(smart, text="狗自己的本事", fg=DIM, bg=DARK,
                 font=("Helvetica", 11)).pack(anchor="w")
        row1 = tk.Frame(smart, bg=DARK); row1.pack(fill="x", pady=2)
        self.avoid = tk.BooleanVar(value=False)
        ttk.Checkbutton(row1, text="避障(走路时自己绕开)",
                        variable=self.avoid,
                        command=self.on_avoid).pack(side="left")
        Btn(row1, "跟着我", lambda: self.send("follow"),
            pad=(10, 4)).pack(side="right", padx=2)
        Btn(row1, "自己走", lambda: self.send("freeavoid"),
            pad=(10, 4)).pack(side="right", padx=2)
        Btn(row1, "开雷达", lambda: self.send("lidar_on"),
            pad=(10, 4)).pack(side="right", padx=2)

        row2 = tk.Frame(smart, bg=DARK); row2.pack(fill="x", pady=2)
        for text, cmd in (("开始建图", "mapping/start"),
                          ("结束建图", "mapping/stop"),
                          ("定位", "localization/start"),
                          ("导航", "navigation/start")):
            Btn(row2, text, lambda c=cmd: self.uslam(c), pad=(7, 4)).pack(
                side="left", expand=True, fill="x", padx=1)

        btns = tk.Frame(parent, bg=DARK)
        btns.pack(fill="x", pady=(6, 0))
        for text, cmd, colour in (("站起来", "stand", PANEL),
                                  ("平衡站立", "balance", PANEL),
                                  ("趴下", "sit", PANEL),
                                  ("■ 停止", "stop", BAD)):
            Btn(btns, text, lambda c=cmd: self.send(c), bg=colour,
                fg="#ffffff" if colour != PANEL else INK,
                bold=(cmd == "stop"), pad=(6, 9)).pack(
                    side="left", expand=True, fill="x", padx=2)

    def send(self, cmd):
        if cmd == "stop":
            self.stop()
        with drive_lock:
            DRIVE["cmd"] = cmd

    def uslam(self, text):
        """One of the robot's own SLAM commands, as a plain string."""
        with drive_lock:
            DRIVE["cmd"] = "uslam:" + text

    def on_avoid(self):
        on = bool(self.avoid.get())
        with drive_lock:
            DRIVE["avoid"] = on
            DRIVE["cmd"] = "avoid_on" if on else "avoid_off"

    def click_map(self, e):
        """Click the floor plan to send the dog there.

        Only in the map view, and only with motion allowed - a click is
        very easy to make by accident, and this one walks a robot across a
        building."""
        if self.view != "map" or not self.allow.get():
            return
        s = SH.snap()
        cx, cy = s["pose"][:2] if self.follow.get() else (0.0, 0.0)
        ppm = MAP_PX / self.metres
        gx = cx + (e.x - MAP_PX / 2) / ppm
        gy = cy - (e.y - MAP_PX / 2) / ppm
        SH.put(goal=(gx, gy))
        self.uslam(f"navigation/set_goal_pose/{gx:.3f}/{gy:.3f}/0.000")

    def set_gait(self, key):
        """Pick the gait, and move the sliders' ceiling with it.

        Asking for 3 m/s in a walking gait does not produce 3 m/s; the gait
        is what decides. The dog's own measured speed is on screen, and
        that is the number to trust over anything asked for here."""
        self.gait = key
        g = GAITS[key]
        self._sc0.configure(to=g["vx"])
        self._sc1.configure(to=g["vyaw"])
        if self.vsp.get() > g["vx"]:
            self.vsp.set(g["vx"])
        if self.wsp.get() > g["vyaw"]:
            self.wsp.set(g["vyaw"])
        for k, b in self._gbtn.items():
            b.configure(bg=(ACCENT if k == key else PANEL),
                        fg=("#07090d" if k == key else INK))
            b._bg = ACCENT if k == key else PANEL
        with drive_lock:
            DRIVE["cmd"] = key
        self.apply()

    def set_view(self, key):
        self.view = key
        if key == "front":                 # look along the dog's nose
            self.az, self.el = 0.0, 0.08
        for k, b in self._vbtn.items():
            b.configure(bg=(ACCENT if k == key else PANEL),
                        fg=("#07090d" if k == key else INK))
            b._bg = ACCENT if k == key else PANEL

    def _drag_start(self, e):
        self._drag_from = (e.x, e.y, self.az, self.el)

    def _drag(self, e):
        if self.view == "map" or not self._drag_from:
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
    def tick(self):
        s = SH.snap()
        st, c = s["state"], s["counts"]

        if s["frame"] is not None:
            self._cam = ImageTk.PhotoImage(s["frame"])
            self.cam.configure(image=self._cam)
        extra = ""
        if self.view == "map":
            pic = render_map(s, self.metres, self.follow.get())
        elif self.view == "fuse":
            pic, extra = render_overlay(s)
        else:
            az = self.az + (s["pose"][2] if self.view == "front" else 0.0)
            pic = render_3d(s, az, self.el, self.dist, self.follow.get(),
                            drop_floor=not self.floor.get())
        self._map = ImageTk.PhotoImage(pic)
        self.map.configure(image=self._map)

        for lab, var, unit in self._labs:
            lab.configure(text=f"{var.get():.2f} {unit}")

        rpy = st.get("rpy") or [0, 0, 0]
        dot = "●" if s["note"] == "connected" else "○"
        self.hdr.configure(text=(
            f"{dot} {s['note']}    电池 {st.get('soc', '–')}%  "
            f"{st.get('volt', '–')} V    模式 {st.get('mode', '–')}    "
            f"身高 {st.get('height', '–')} m    朝向 {rpy[2] * 57.3:6.1f}°"))

        vel = st.get("vel") or [0, 0, 0]
        ft = st.get("foot") or [0, 0, 0, 0]
        self.tele.configure(text=(
            f"前进 {vel[0]:+.2f} m/s   侧移 {vel[1]:+.2f} m/s   "
            f"脚底 {ft[0]:3d} {ft[1]:3d} {ft[2]:3d} {ft[3]:3d}\n"
            f"收到  状态 {c['low']} · 运动 {c['sport']} · "
            f"点云 {c['cloud']} · 画面 {c['frame']}"))

        age = time.time() - s["cloud_t"] if s["cloud_t"] else None
        zr = s["zrange"]
        where = (f"视野 {self.metres:.0f} m" if self.view == "map"
                 else extra if self.view == "fuse"
                 else f"距离 {self.dist:.0f} m · 拖动旋转 · {len(s['pts3']):,} 点")
        slam_bit = (f"狗的SLAM {s['slam_n']:,} 点   " if s.get("slam_n")
                    else "")
        self.mapinfo.configure(text=(
            f"{where}   {slam_bit}地图 {s['cells']:,} 格   "
            + (f"点云 {age:.1f} s 前" if age is not None else "等待点云") + "\n"
            + (f"本帧 {zr[3]:,}/{zr[4]:,} 点算作墙   "
               f"高度 {zr[0]:+.2f} ~ {zr[1]:+.2f} m  (地面 {zr[2]:+.2f})"
               if zr else "高度范围:还没有点云")))
        lines = s.get("slog") or []
        if len(lines) != self._logn:
            self._logn = len(lines)
            self.log.configure(state="normal")
            self.log.delete("1.0", "end")
            self.log.insert("1.0", "\n".join(lines[-7:]))
            self.log.configure(state="disabled")

        self.root.after(60, self.tick)


def wifi_target():
    """'ap', or the dog's IP on a router - the file the panel also reads."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        ".go2_wifi_ip")
    try:
        return open(path).read().strip() or "ap"
    except OSError:
        return "ap"


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else wifi_target()
    loop = asyncio.new_event_loop()
    task = {}

    def run():
        asyncio.set_event_loop(loop)
        task["t"] = loop.create_task(robot_loop(target))
        try:
            loop.run_until_complete(task["t"])
        except SystemExit:
            SH.put(note="could not connect - see the terminal")
        except Exception as e:
            SH.put(note=f"disconnected: {e}")

    threading.Thread(target=run, daemon=True).start()

    root = tk.Tk()
    App(root, target)
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
