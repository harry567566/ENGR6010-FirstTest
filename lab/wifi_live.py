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
LOWSTATE = "rt/lf/lowstate"
SPORTSTATE = "rt/lf/sportmodestate"
CLOUD = "rt/utlidar/voxel_map_compressed"
POSE = "rt/utlidar/robot_pose"

# Ceilings, not speeds: the sliders choose anything up to these. A key held
# down is a command repeated twenty times a second, so this is what stands
# between a slip of the finger and a robot at a run.
MAX = {"vx": 0.5, "vy": 0.3, "vyaw": 0.8}
MOVE_HZ = 20.0
STANDING_LOCKED = (0, 2, 6, 8)      # Move does little here until BalanceStand
API = {"damp": 1001, "balance": 1002, "stop": 1003, "stand": 1004,
       "sit": 1005, "recover": 1006, "move": 1008}

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


class Shared:
    """What the robot thread writes and the window draws.

    Only the newest of everything is kept, except the map, which is the
    whole point and accumulates."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = {}
        self.frame = None
        self.cloud_t = 0.0
        self.pose = (0.0, 0.0, 0.0)
        self.trail = []
        self.counts = {"low": 0, "sport": 0, "cloud": 0, "frame": 0}
        self.note = "connecting ..."
        self.zrange = None
        self.grid = np.zeros((GRID_N, GRID_N), np.uint16)
        self.cells = 0

    def put(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def snap(self):
        with self.lock:
            return dict(state=dict(self.state), frame=self.frame,
                        cloud_t=self.cloud_t, pose=self.pose,
                        trail=list(self.trail), counts=dict(self.counts),
                        note=self.note, zrange=self.zrange, cells=self.cells)

    def clear_map(self):
        with self.lock:
            self.grid[:] = 0
            self.cells = 0
            self.trail = []


SH = Shared()
DRIVE = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0, "allowed": False, "cmd": None}
drive_lock = threading.Lock()


# --------------------------------------------------------------- the robot

def decode_cloud(data):
    """The voxel decoder's output -> (N, 3) float32 metres.

    `positions` is raw bytes: three float32 per point. The count that
    matters is face_count; point_count is voxels before expansion."""
    try:
        pos = data.get("positions")
        n = int(data.get("face_count", 0))
        if pos is None or n <= 0:
            return None
        xyz = np.frombuffer(bytes(pos), dtype=np.float32)[:n * 3]
        xyz = xyz.reshape(-1, 3)
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


async def robot_loop(target):
    conn = await connect(target)

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
        SH.put(cloud_t=time.time())
        SH.counts["cloud"] += 1
        fold_into_map(pts)

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
                SH.put(frame=box)
                SH.counts["frame"] += 1
            except Exception:
                pass            # a dropped frame is not worth stopping for

    ps = conn.datachannel.pub_sub
    ps.subscribe(LOWSTATE, on_low)
    ps.subscribe(SPORTSTATE, on_sport)
    ps.subscribe(POSE, on_pose)

    # Without this the dog throttles the cloud away, and a plain subscribe
    # looks exactly like a LiDAR that is not running.
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

            if cmd:                                 # a button in the window
                code = await request(conn, API[cmd])
                SH.put(note=f"{cmd}: {'OK' if code == 0 else code}")
                if cmd in ("stop", "sit", "damp"):
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
                send_move(conn, vx, vy, vyaw)
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


async def request(conn, api_id, timeout=4.0):
    try:
        answer = await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(
                SPORT_TOPIC, {"api_id": api_id}), timeout)
        return answer.get("data", {}).get("header", {}) \
                     .get("status", {}).get("code", -1)
    except Exception:
        return -1


def send_move(conn, vx, vy, vyaw):
    """Fire and forget, the shape wifi_sport.py sends."""
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


# ---------------------------------------------------------------- the view

DARK, PANEL, INK, DIM = "#0d0f14", "#161a22", "#e8ebf0", "#8891a3"


class App:
    def __init__(self, root, target):
        self.root = root
        self.metres = 16.0
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
        tk.Label(head, text="激光雷达地图 · 俯视", fg=INK, bg=DARK,
                 font=("Helvetica", 12, "bold")).pack(side="left")
        for txt, fn in (("清空", self.clear), ("+", self.zoom_in),
                        ("−", self.zoom_out)):
            tk.Button(head, text=txt, command=fn, bg=PANEL, fg=INK,
                      relief="flat", width=4,
                      font=("Helvetica", 11)).pack(side="right", padx=2)
        ttk.Checkbutton(head, text="跟随狗",
                        variable=self.follow).pack(side="right", padx=8)
        self._mblank = ImageTk.PhotoImage(
            Image.new("RGB", (MAP_PX, MAP_PX), (10, 13, 18)))
        self.map = tk.Label(right, bg="#0a0d12", image=self._mblank)
        self.map.pack()
        self.mapinfo = tk.Label(right, text="", fg=DIM, bg=DARK, anchor="w",
                                justify="left", font=("Menlo", 10))
        self.mapinfo.pack(fill="x", pady=(5, 0))

        self.keys = set()
        root.bind("<KeyPress>", self.down)
        root.bind("<KeyRelease>", self.up)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.focus_set()
        self.tick()

    # ------------------------------------------------------------ controls
    def build_controls(self, parent):
        box = tk.Frame(parent, bg=DARK)
        box.pack(fill="x", pady=(10, 0))
        self.vsp = tk.DoubleVar(value=0.30)
        self.wsp = tk.DoubleVar(value=0.50)
        self._labs = []
        for row, (var, lo, hi, unit, name) in enumerate((
                (self.vsp, 0.05, MAX["vx"], "m/s", "走路速度"),
                (self.wsp, 0.10, MAX["vyaw"], "rad/s", "转向速度"))):
            tk.Label(box, text=name, fg=DIM, bg=DARK,
                     font=("Helvetica", 11)).grid(row=row, column=0,
                                                  sticky="w", pady=2)
            ttk.Scale(box, from_=lo, to=hi, variable=var, length=200,
                      command=lambda *_: self.apply()).grid(row=row, column=1,
                                                            padx=6)
            lab = tk.Label(box, text="", fg=INK, bg=DARK,
                           font=("Menlo", 11), width=10, anchor="w")
            lab.grid(row=row, column=2, sticky="w")
            self._labs.append((lab, var, unit))

        self.allow = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent,
                        text="允许运动  ——  W A S D 走 · Q E 转 · 空格停",
                        variable=self.allow,
                        command=self.on_allow).pack(fill="x", pady=(10, 5))

        btns = tk.Frame(parent, bg=DARK)
        btns.pack(fill="x")
        for text, cmd, colour in (("站起来", "stand", PANEL),
                                  ("平衡站立", "balance", PANEL),
                                  ("趴下", "sit", PANEL),
                                  ("■ 停止", "stop", "#ff5f56")):
            tk.Button(btns, text=text, bg=colour, relief="flat",
                      fg="#ffffff" if colour != PANEL else INK,
                      font=("Helvetica", 12,
                            "bold" if cmd == "stop" else "normal"),
                      command=lambda c=cmd: self.send(c)).pack(
                          side="left", expand=True, fill="x", padx=2)

    def send(self, cmd):
        if cmd == "stop":
            self.stop()
        with drive_lock:
            DRIVE["cmd"] = cmd

    def zoom_in(self):
        self.metres = max(4.0, self.metres / 1.5)

    def zoom_out(self):
        self.metres = min(70.0, self.metres * 1.5)

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
                         vy=min(v, MAX["vy"]) * (("a" in k) - ("d" in k)),
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
        self._map = ImageTk.PhotoImage(
            render_map(s, self.metres, self.follow.get()))
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
        self.mapinfo.configure(text=(
            f"视野 {self.metres:.0f} m   地图 {s['cells']:,} 格   "
            + (f"点云 {age:.1f} s 前" if age is not None else "等待点云") + "\n"
            + (f"本帧 {zr[3]:,}/{zr[4]:,} 点算作墙   "
               f"高度 {zr[0]:+.2f} ~ {zr[1]:+.2f} m  (地面 {zr[2]:+.2f})"
               if zr else "高度范围:还没有点云")))
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
