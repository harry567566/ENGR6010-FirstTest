"""Live view and driving over Wi-Fi: camera, LiDAR and WASD in one window.

    .venv_wifi/bin/python lab/wifi_live.py ap

Every other Wi-Fi button opens its own WebRTC connection, does one thing and
hangs up. That costs three to five seconds a press, and the dog's signalling
endpoint answers `429 Too Many Requests` when two of them land close
together, which is exactly what happened in the lab. Driving with the
keyboard is impossible that way.

So this holds ONE connection open and does everything on it:

    rt/lf/lowstate                  battery, attitude, joints, foot forces
    rt/lf/sportmodestate            mode, body height, velocity, position
    rt/utlidar/voxel_map_compressed the point cloud, decoded by the library
    rt/utlidar/robot_pose           the dog's own LiDAR odometry
    the video track                 the front camera

The point cloud does arrive over Wi-Fi. It is not DDS - DDS really is
cable-only - it is the robot's own WebRTC channel, the one the phone app
draws its map from, and the driver in .venv_wifi already carries the voxel
decoder for it.

MOTION IS OFF UNTIL YOU TICK THE BOX. With it on, W A S D drive and Q E
turn, at the same limits as wifi_sport.py; releasing a key sends StopMove,
and so does the spacebar, closing the window, and Ctrl+C. The remote's
L2 + B is still faster than any of them.
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
from wifi_common import connect, disconnect, fail          # noqa: E402

SPORT_TOPIC = "rt/api/sport/request"
LOWSTATE = "rt/lf/lowstate"
SPORTSTATE = "rt/lf/sportmodestate"
CLOUD = "rt/utlidar/voxel_map_compressed"
POSE = "rt/utlidar/robot_pose"

# The same ceilings wifi_sport.py uses. A key held down is a command repeated
# twenty times a second, so this is the only thing between a slip of the
# finger and a robot at full speed.
LIMITS = {"vx": 0.3, "vy": 0.2, "vyaw": 0.5}
MOVE_HZ = 20.0
STANDING_LOCKED = (0, 2, 6, 8)      # Move does little here until BalanceStand

VIEW_W, VIEW_H = 480, 360           # both panels, so the window is predictable


class Shared:
    """What the asyncio thread writes and the tkinter thread draws.

    Only the newest of everything is kept: the robot's rate is not the
    screen's rate, and a backlog of stale frames helps nobody."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = {}
        self.frame = None            # PIL image from the camera
        self.cloud = None            # (N, 3) float32
        self.cloud_t = 0.0
        self.pose = None
        self.counts = {"low": 0, "sport": 0, "cloud": 0, "frame": 0}
        self.note = "connecting ..."
        self.cloud_note = ""

    def put(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def snap(self):
        with self.lock:
            return (dict(self.state), self.frame, self.cloud, self.pose,
                    dict(self.counts), self.note, self.cloud_note,
                    self.cloud_t)


SH = Shared()
DRIVE = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0, "allowed": False, "stop": False}
drive_lock = threading.Lock()


# --------------------------------------------------------------- the robot

def decode_cloud(data):
    """The voxel decoder's output -> (N, 3) float32 in metres.

    `positions` comes back as raw bytes: three float32 per point, packed by
    the wasm decoder. The count that matters is face_count, not point_count;
    point_count is the number of voxels before they are expanded."""
    try:
        pos = data.get("positions")
        n = int(data.get("face_count", 0))
        if pos is None or n <= 0:
            return None
        xyz = np.frombuffer(bytes(pos), dtype=np.float32)
        xyz = xyz[:n * 3].reshape(-1, 3)
        good = np.isfinite(xyz).all(axis=1)
        return np.ascontiguousarray(xyz[good])
    except Exception:
        return None


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
                          "foot": [int(x) for x in d["foot_force"]],
                          "tau": [round(float(m["tau_est"]), 1)
                                  for m in d["motor_state"][:12]]})
            SH.counts["low"] += 1
        except Exception:
            pass

    def on_sport(msg):
        try:
            d = msg["data"]
            SH.put(state={**SH.state,
                          "mode": int(d["mode"]),
                          "height": round(float(d["body_height"]), 3),
                          "vel": [round(float(x), 2) for x in d["velocity"]],
                          "pos": [round(float(x), 2) for x in d["position"]]})
            SH.counts["sport"] += 1
        except Exception:
            pass

    def on_cloud(msg):
        pts = decode_cloud(msg.get("data", {}).get("data", {}))
        if pts is None or not len(pts):
            return
        SH.put(cloud=pts, cloud_t=time.time(),
               cloud_note=f"{len(pts):,} points")
        SH.counts["cloud"] += 1

    def on_pose(msg):
        try:
            p = msg["data"]["pose"]["position"]
            SH.put(pose=(float(p["x"]), float(p["y"]), float(p["z"])))
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
                img.thumbnail((VIEW_W, VIEW_H))
                # pad to the panel size: a frame whose aspect differs would
                # otherwise resize the label and make the window jump
                box = Image.new("RGB", (VIEW_W, VIEW_H), (7, 9, 13))
                box.paste(img, ((VIEW_W - img.width) // 2,
                                (VIEW_H - img.height) // 2))
                SH.put(frame=box)
                SH.counts["frame"] += 1
            except Exception:
                pass            # a dropped frame is not worth stopping for

    ps = conn.datachannel.pub_sub
    ps.subscribe(LOWSTATE, on_low)
    ps.subscribe(SPORTSTATE, on_sport)
    ps.subscribe(POSE, on_pose)

    # The dog throttles the cloud away unless this is off, which is why a
    # plain subscribe to the LiDAR topic looks like the LiDAR is not running.
    try:
        await conn.datachannel.disableTrafficSaving(True)
    except Exception as e:
        SH.put(cloud_note=f"traffic saving: {e}")
    ps.subscribe(CLOUD, on_cloud)

    try:
        conn.video.add_track_callback(on_video)
        conn.video.switchVideoChannel(True)
    except Exception as e:
        SH.put(note=f"camera unavailable: {e}")

    SH.put(note="connected")

    # ------------------------------------------------------------ driving
    last_sent = 0.0
    moving = False
    period = 1.0 / MOVE_HZ
    try:
        while True:
            await asyncio.sleep(period)
            with drive_lock:
                vx, vy, vyaw = DRIVE["vx"], DRIVE["vy"], DRIVE["vyaw"]
                allowed, stop = DRIVE["allowed"], DRIVE["stop"]
                DRIVE["stop"] = False
            if stop or not allowed:
                vx = vy = vyaw = 0.0
            want = (vx or vy or vyaw)

            if want:
                mode = SH.state.get("mode")
                if mode in STANDING_LOCKED and not moving:
                    # Move does almost nothing from idle stand; the dog has
                    # to be in balance stand first, exactly as 1-7 does it
                    await request(conn, 1002)
                    SH.put(note="balance stand")
                send_move(conn, vx, vy, vyaw)
                moving = True
                last_sent = time.time()
            elif moving and time.time() - last_sent > 0.1:
                await request(conn, 1003)             # StopMove
                moving = False
                SH.put(note="stopped")
    except asyncio.CancelledError:
        pass
    finally:
        try:
            if moving:
                await request(conn, 1003)
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
    """Fire and forget, the same shape wifi_sport.py sends."""
    conn.datachannel.pub_sub.publish_without_callback(SPORT_TOPIC, {
        "header": {"identity": {"id": int(time.time() * 1000) % 2147483648
                                      + random.randint(0, 1000),
                                "api_id": 1008},
                   "policy": {"priority": 0, "noreply": True}},
        "parameter": json.dumps({"x": vx, "y": vy, "z": vyaw}),
        "binary": [],
    })


# ---------------------------------------------------------------- the view

def draw_cloud(pts, pose, size=(VIEW_W, VIEW_H)):
    """Top-down, coloured by height, with the dog at the centre.

    Drawn with numpy into an image rather than as canvas items: twenty
    thousand points is nothing as an array and far too much as widgets."""
    w, h = size
    img = np.zeros((h, w, 3), np.uint8)
    img[:, :] = (7, 9, 13)

    # Grid and robot are drawn whether or not a sweep has arrived: an empty
    # black rectangle reads as broken, a grid with the dog on it reads as
    # an instrument waiting for data.
    have = pts is not None and len(pts)
    span = (max(float(np.abs(pts[:, :2]).max()) * 1.1, 1.0) if have else 5.0)
    s = min(w, h) / (2 * span)

    g = max(int(round(s)), 1)                          # a one-metre grid
    if g >= 6:
        img[::g, :] = (29, 35, 45)
        img[:, ::g] = (29, 35, 45)

    if have:
        x = (pts[:, 0] * s + w / 2).astype(np.int32)
        y = (h / 2 - pts[:, 1] * s).astype(np.int32)
        keep = (x >= 0) & (x < w) & (y >= 0) & (y < h)
        x, y, z = x[keep], y[keep], pts[keep, 2]
        if len(z):
            lo, hi = float(z.min()), float(z.max())
            u = (z - lo) / max(hi - lo, 0.4)
            img[y, x, 0] = (56 + 199 * u).astype(np.uint8)
            img[y, x, 1] = (140 + 89 * np.sin(u * math.pi)).astype(np.uint8)
            img[y, x, 2] = (255 - 184 * u).astype(np.uint8)

    cx, cy = w // 2, h // 2                            # the dog, facing +x
    img[cy - 3:cy + 4, cx - 5:cx + 6] = (240, 240, 240)
    img[cy - 1:cy + 2, cx + 6:cx + 20] = (255, 95, 86)
    return Image.fromarray(img)


class App:
    def __init__(self, root, target):
        self.root = root
        root.title(f"Go2 实时画面 · {target}")
        root.configure(bg="#0d0f14")

        views = tk.Frame(root, bg="#0d0f14")
        views.pack(padx=8, pady=(8, 4))
        # A Label's width and height are counted in characters, not pixels,
        # so the panels are sized by the image they hold. Both start with a
        # blank one so the window opens at its real size instead of
        # collapsing and then jumping when the first frame lands.
        blank = ImageTk.PhotoImage(Image.new("RGB", (VIEW_W, VIEW_H),
                                             (7, 9, 13)))
        self._blank = blank
        self.cam = tk.Label(views, bg="#07090d", image=blank)
        self.cam.grid(row=1, column=0, padx=(0, 6))
        self.lid = tk.Label(views, bg="#07090d", image=blank)
        self.lid.grid(row=1, column=1)
        for col, text in ((0, "前置摄像头"), (1, "激光雷达 · 俯视")):
            tk.Label(views, text=text, fg="#e8ebf0", bg="#0d0f14",
                     font=("Helvetica", 12, "bold")).grid(row=0, column=col,
                                                          sticky="w", pady=2)

        self.info = tk.Label(root, text="", fg="#b9c1cf", bg="#0d0f14",
                             font=("Menlo", 11), justify="left", anchor="w")
        self.info.pack(fill="x", padx=10)

        bar = tk.Frame(root, bg="#0d0f14")
        bar.pack(fill="x", padx=10, pady=(4, 10))
        self.allow = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="允许运动 —— 勾上之后 W A S D 走、Q E 转、空格停",
                        variable=self.allow,
                        command=self.on_allow).pack(side="left")
        tk.Button(bar, text="■ 停止", command=self.stop, bg="#ff5f56",
                  fg="white", relief="flat",
                  font=("Helvetica", 12, "bold")).pack(side="right")

        self.keys = set()
        root.bind("<KeyPress>", self.down)
        root.bind("<KeyRelease>", self.up)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.focus_set()
        self.tick()

    # ------------------------------------------------------------- keys
    def on_allow(self):
        with drive_lock:
            DRIVE["allowed"] = bool(self.allow.get())
            if not DRIVE["allowed"]:
                DRIVE.update(vx=0.0, vy=0.0, vyaw=0.0, stop=True)

    def stop(self):
        self.keys.clear()
        self.allow.set(False)
        with drive_lock:
            DRIVE.update(vx=0.0, vy=0.0, vyaw=0.0, allowed=False, stop=True)

    def down(self, e):
        k = (e.keysym or "").lower()
        if k == "space" or k == "escape":
            return self.stop()
        if k in "wasdqe":
            self.keys.add(k)
            self.apply()

    def up(self, e):
        k = (e.keysym or "").lower()
        if k in self.keys:
            self.keys.discard(k)
            self.apply()

    def apply(self):
        k = self.keys
        vx = LIMITS["vx"] * (("w" in k) - ("s" in k))
        vy = LIMITS["vy"] * (("a" in k) - ("d" in k))
        vyaw = LIMITS["vyaw"] * (("q" in k) - ("e" in k))
        with drive_lock:
            DRIVE.update(vx=vx, vy=vy, vyaw=vyaw)

    def close(self):
        self.stop()
        self.root.after(250, self.root.destroy)

    # ------------------------------------------------------------- paint
    def tick(self):
        st, frame, cloud, pose, counts, note, cnote, ct = SH.snap()

        if frame is not None:
            self._cam_img = ImageTk.PhotoImage(frame)
            self.cam.configure(image=self._cam_img)
        self._lid_img = ImageTk.PhotoImage(draw_cloud(cloud, pose))
        self.lid.configure(image=self._lid_img)

        rpy = st.get("rpy") or [0, 0, 0]
        age = (time.time() - ct) if ct else None
        self.info.configure(text=(
            f"{note}   电池 {st.get('soc', '–')}%  {st.get('volt', '–')} V   "
            f"模式 {st.get('mode', '–')}   身高 {st.get('height', '–')} m   "
            f"朝向 {rpy[2] * 57.3:6.1f}°\n"
            f"点云 {cnote or '等待'}"
            f"{'' if age is None else f'  ({age:.1f} s 前)'}   "
            f"收到 低层 {counts['low']} · 运动 {counts['sport']} · "
            f"点云 {counts['cloud']} · 画面 {counts['frame']}"))
        self.root.after(50, self.tick)


def wifi_target():
    """'ap', or the dog's IP on a shared router, the same file the panel uses."""
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
        # give the robot loop a moment to send StopMove and hang up, so the
        # dog does not keep walking because a window was closed
        try:
            loop.call_soon_threadsafe(task["t"].cancel)
        except Exception:
            pass
        time.sleep(1.0)


if __name__ == "__main__":
    main()
