"""Give the map its colours: project the camera onto the LiDAR points.

    python3 mapping/colorize.py lab/data/survey_walk_XXXX
    python3 mapping/colorize.py <dir> --check        # one overlay, to judge
    python3 mapping/colorize.py <dir> --yaw 2 --pitch -1   # nudge the mount

survey_map.py builds the shape of the building. This adds what it looks
like, by taking every LiDAR point, working out where it lands in the camera
image recorded nearest in time, and reading the colour there.

Two things have to be right for that, and only one of them is published.

INTRINSICS - how the lens maps directions to pixels. Unitree does not
publish these, but Go2Py's lidar-to-camera calibration note gives a
measured set for the Go2 front camera, and they are the default here:

    fx 379.7099  fy 379.3696  cx 320.3064  cy 243.1175     (640 x 480)
    plumb_bob k1 -0.057967  k2 0.070432  p1 -0.000153
              p2 0.000606   k3 -0.022367

    source: third_party/Go2Py/docs/utlidar-camera-calib.md

EXTRINSICS - where the camera sits relative to the LiDAR. Go2Py's answer is
to run a whole calibration pipeline in Docker, which is not happening on a
Mac in a week. The default here is the nominal mount from the Go2's own
geometry: the camera sits below and ahead of the LiDAR, both looking
forward. That is close but not exact, so --yaw/--pitch/--roll and --dx/--dy
/--dz let you nudge it, and --check renders one overlay so you can see
whether the edges line up before colouring 300,000 points.

An uncalibrated colourisation looks convincing and is wrong: the colours
slide off the geometry near edges. Always run --check first.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

# Go2 front camera, from Go2Py's calibration note.
FX, FY, CX, CY = 379.7099304199219, 379.3695983886719, \
                 320.3064270019531, 243.11753845214844
DIST = np.array([-0.057967256754636765, 0.0704321563243866,
                 -0.00015285948757082224, 0.0006057045538909733,
                 -0.022366832941770554], float)

# Nominal LiDAR -> camera mount. The L1 sits on the head, the camera just
# below and in front of it, both pointing forward along +x.
NOMINAL_T = np.array([0.058, 0.0, -0.062], float)       # metres


def rot(yaw_deg, pitch_deg, roll_deg):
    y, p, r = np.radians([yaw_deg, pitch_deg, roll_deg])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    return Rz @ Ry @ Rx


def project(pts_lidar, R_lc, t_lc, w, h, use_dist=True):
    """LiDAR points -> pixel coordinates, with the plumb_bob distortion.

    Returns (u, v, keep) where keep marks points in front of the camera and
    inside the image."""
    P = pts_lidar @ R_lc.T + t_lc
    # Camera convention: +z forward, +x right, +y down. The LiDAR's is
    # +x forward, +y left, +z up, so the axes have to be swapped before
    # anything is divided by depth.
    X, Y, Z = -P[:, 1], -P[:, 2], P[:, 0]
    front = Z > 0.15
    Z = np.where(front, Z, 1.0)
    x, y = X / Z, Y / Z
    if use_dist:
        k1, k2, p1, p2, k3 = DIST
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = xd, yd
    u = FX * x + CX
    v = FY * y + CY
    keep = front & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    return u, v, keep


def load_frames(root):
    """Camera frames with the time they were taken, from state.jsonl."""
    times = {}
    p = os.path.join(root, "state.jsonl")
    if os.path.exists(p):
        for ln in open(p):
            try:
                r = json.loads(ln)
            except Exception:
                continue
            if r.get("src") == "frame":
                times[int(r["i"])] = float(r["t"])
    files = sorted(glob.glob(os.path.join(root, "frames", "*.jpg")))
    out = []
    for f in files:
        i = int(os.path.splitext(os.path.basename(f))[0])
        out.append({"i": i, "path": f, "t": times.get(i)})
    out = [o for o in out if o["t"] is not None]
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("survey")
    ap.add_argument("--check", action="store_true",
                    help="render one overlay and stop")
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--dx", type=float, default=NOMINAL_T[0])
    ap.add_argument("--dy", type=float, default=NOMINAL_T[1])
    ap.add_argument("--dz", type=float, default=NOMINAL_T[2])
    ap.add_argument("--max-dt", type=float, default=0.25,
                    help="a sweep is only coloured by a frame this close in time")
    a = ap.parse_args()

    import cv2
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from survey_map import load_survey, voxel, quat_to_R

    root = a.survey.rstrip("/")
    meta, sweeps, odom = load_survey(root)
    frames = load_frames(root)
    if not sweeps:
        sys.exit("no point clouds in this survey")
    if not frames:
        sys.exit("no camera frames in this survey - was --no-camera used?")
    print(f"  {len(sweeps)} sweeps, {len(frames)} camera frames")

    R_lc = rot(a.yaw, a.pitch, a.roll)
    t_lc = np.array([a.dx, a.dy, a.dz], float)
    ft = np.array([f["t"] for f in frames])

    # ------------------------------------------------------------- check
    if a.check:
        mid = sweeps[len(sweeps) // 2]
        j = int(np.argmin(np.abs(ft - mid["t"])))
        img = cv2.imread(frames[j]["path"])
        if img is None:
            sys.exit(f"could not read {frames[j]['path']}")
        h, w = img.shape[:2]
        pts = voxel(mid["pts"], 0.04)
        u, v, keep = project(pts, R_lc, t_lc, w, h)
        d = np.linalg.norm(pts, axis=1)
        over = img.copy()
        uu, vv, dd = u[keep].astype(int), v[keep].astype(int), d[keep]
        if len(dd):
            c = np.clip((dd - dd.min()) / max(dd.ptp(), 1e-6), 0, 1)
            for x, y, t in zip(uu, vv, c):
                cv2.circle(over, (x, y), 1,
                           (int(255 * (1 - t)), int(90 + 120 * t),
                            int(255 * t)), -1)
        out = np.hstack([img, cv2.addWeighted(over, 0.75, img, 0.25, 0)])
        p = os.path.join(root, "colorize_check.png")
        cv2.imwrite(p, out)
        print(f"  {keep.sum()} of {len(pts)} points land in the image "
              f"({100*keep.mean():.0f}%)")
        print(f"  dt to the nearest frame: {abs(ft[j]-mid['t'])*1000:.0f} ms")
        print(f"\nwrote {p}")
        print("  Left is the photo, right is the photo with LiDAR points on")
        print("  top, coloured near-to-far. If the points follow the edges in")
        print("  the photo the mount is right. If they sit beside them, nudge")
        print("  --yaw and --pitch by a degree at a time and run this again.")
        return

    # ----------------------------------------------------------- colourise
    cache = {}
    allpts, allcol = [], []
    used = 0
    for i, s in enumerate(sweeps):
        j = int(np.argmin(np.abs(ft - s["t"])))
        if abs(ft[j] - s["t"]) > a.max_dt:
            continue
        if j not in cache:
            im = cv2.imread(frames[j]["path"])
            cache[j] = im
            if len(cache) > 40:
                cache.pop(next(iter(cache)))
        img = cache.get(j)
        if img is None:
            continue
        h, w = img.shape[:2]
        pts = voxel(s["pts"], 0.05)
        u, v, keep = project(pts, R_lc, t_lc, w, h)
        if not keep.any():
            continue
        bgr = img[v[keep].astype(int), u[keep].astype(int)]
        allpts.append(pts[keep])
        allcol.append(bgr[:, ::-1] / 255.0)          # BGR -> RGB
        used += 1
        if (i + 1) % 25 == 0:
            print(f"\r  {i+1}/{len(sweeps)}  {used} sweeps coloured  ",
                  end="", flush=True)
    print()
    if not allpts:
        sys.exit("no sweep had a camera frame close enough in time")

    P = np.vstack(allpts).astype(np.float32)
    C = np.vstack(allcol).astype(np.float32)
    out = os.path.join(root, "colored_cloud.npz")
    np.savez_compressed(out, points=P, colors=C)
    print(f"\n  {len(P):,} coloured points from {used} sweeps")
    print(f"  (these are in the LiDAR frame of each sweep - run survey_map.py")
    print(f"   first if you want them placed in the building)")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
