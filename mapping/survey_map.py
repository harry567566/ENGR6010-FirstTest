"""Turn a recorded survey into a map. Runs at home, no robot needed.

    python3 mapping/survey_map.py lab/data/survey_walk_20261022_143012
    python3 mapping/survey_map.py <dir> --no-icp        # trust odometry only
    python3 mapping/survey_map.py <dir> --max 400       # first 400 sweeps

lab/real_survey.py saves each LiDAR sweep as raw bytes exactly as it arrived,
plus the field layout, and never decodes it on the day. This is where it gets
decoded and assembled.

Two ways to find where each sweep was taken from:

    odometry   if the robot published a pose, use it. Cheap and drift-free
               relative to whatever the robot's own estimator does.
    ICP        otherwise, line each sweep up against the map built so far.
               Point-to-point ICP with a kd-tree, initialised from the last
               motion, which is enough indoors where the geometry is strong.

Both run here. If odometry exists it seeds ICP instead of replacing it, so a
bad pose does not quietly become a bad map.

Output, next to the survey:
    map.npz          the assembled cloud, voxel-downsampled
    occupancy.npy    a 2D grid for navigation: 0 unknown, 1 free, 2 occupied
    map_preview.png  a top-down picture to look at
    map_report.json  how far it travelled, how well each scan matched

Needs numpy, scipy, matplotlib. No open3d, no ROS.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

# Apple's Accelerate BLAS raises spurious divide-by-zero / overflow warnings
# from matmul on macOS with numpy 2.x, even for a clean finite array times
# the identity - verified directly. The guards around ICP below are what
# actually catch a diverged solve; these warnings carry no information and
# only hide the real ones.
np.seterr(divide="ignore", over="ignore", invalid="ignore")

# PointField datatype codes, from the ROS PointCloud2 definition.
_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
       5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


# ======================================================================
#  1  DECODE
# ======================================================================

def decode(raw, layout, width, height, point_step):
    """Raw PointCloud2 bytes -> (N,3) float32, using the recorded layout.

    The offsets come from the file, not from an assumption. A Go2 sweep is
    usually x,y,z,intensity as float32 at a 16-byte stride, but firmware
    differs and a wrong guess turns a corridor into noise."""
    fields = {f["name"]: f for f in layout.get("fields", [])}
    if not all(k in fields for k in ("x", "y", "z")):
        return None
    step = int(point_step or layout.get("point_step", 0))
    if step <= 0:
        return None
    n = len(raw) // step
    if n == 0:
        return None
    a = raw[:n * step].reshape(n, step)
    out = np.empty((n, 3), np.float32)
    for i, k in enumerate(("x", "y", "z")):
        f = fields[k]
        dt = _NP.get(int(f["datatype"]), np.float32)
        w = np.dtype(dt).itemsize
        off = int(f["offset"])
        out[:, i] = a[:, off:off + w].copy().view(dt).ravel()[:n]
    return out[np.isfinite(out).all(axis=1)]


# Preference order when a survey caught more than one cloud topic. They are
# in different coordinate frames and must never be merged: cloud_deskewed is
# motion-compensated and already in odom, cloud is in the sensor frame which
# sits about 165 degrees from the body on a Go2.
TOPIC_RANK = ["rt/uslam/cloud_map", "rt/utlidar/cloud_deskewed",
              "rt/utlidar/cloud_base", "rt/utlidar/cloud"]


def load_survey(root, max_sweeps=None, topic=None):
    meta = json.load(open(os.path.join(root, "meta.json")))
    # new recordings store a layout per topic; old ones store one layout
    by_topic = meta.get("cloud_layout_by_topic") or {}
    legacy = meta.get("cloud_layout") or {}
    cdir = os.path.join(root, "clouds")
    files = sorted(f for f in os.listdir(cdir)) if os.path.isdir(cdir) else []

    # group by the topic recorded with each sweep
    groups = {}
    for f in files:
        z = np.load(os.path.join(cdir, f))
        tp = str(z["topic"]) if "topic" in z.files else "(legacy)"
        groups.setdefault(tp, []).append((f, z))
    if not groups:
        return meta, [], []

    if topic is None:
        ranked = sorted(groups, key=lambda t: (
            TOPIC_RANK.index(t) if t in TOPIC_RANK else 99, -len(groups[t])))
        topic = ranked[0]
    if len(groups) > 1:
        print(f"  this survey has {len(groups)} cloud topics; using {topic}")
        for t in groups:
            mark = "  <-- using" if t == topic else ""
            print(f"      {t:<34}{len(groups[t]):>6} sweeps{mark}")
        print("      (they are in different frames and are never merged)")

    layout = by_topic.get(topic) or legacy
    chosen = groups[topic]
    if max_sweeps:
        chosen = chosen[:max_sweeps]
    sweeps = []
    for f, z in chosen:
        pts = decode(z["data"], layout, int(z["width"]), int(z["height"]),
                     int(z["point_step"]))
        if pts is not None and len(pts) > 50:
            sweeps.append({"t": float(z["t"]), "pts": pts, "file": f})
    odom = []
    p = os.path.join(root, "odom.jsonl")
    if os.path.exists(p):
        for ln in open(p):
            try:
                odom.append(json.loads(ln))
            except Exception:
                pass
    return meta, sweeps, odom


# ======================================================================
#  2  GEOMETRY
# ======================================================================

def voxel(pts, size):
    """One point per occupied cube. Keeps ICP fast and stops dense near
    returns from outvoting the far geometry that actually constrains it."""
    if len(pts) == 0:
        return pts
    key = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[np.sort(idx)]


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]], float)


def kabsch(P, Q):
    """The rotation and translation that best maps P onto Q."""
    ok = np.isfinite(P).all(1) & np.isfinite(Q).all(1)
    if ok.sum() < 3:
        return np.eye(3), np.zeros(3)
    P, Q = P[ok], Q[ok]
    cp, cq = P.mean(0), Q.mean(0)
    H = (P - cp).T @ (Q - cq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cq - R @ cp


def icp(src, dst_tree, dst_pts, R0, t0, iters=20, reject=1.0, tol=1e-4):
    """Point-to-point ICP. Returns R, t, mean residual, inlier fraction.

    Correspondences further than `reject` metres are dropped each round,
    which is what keeps a person walking past from dragging the whole scan.
    """
    R, t = R0.copy(), t0.copy()
    if not (np.isfinite(R).all() and np.isfinite(t).all()):
        return np.eye(3), np.zeros(3), float("nan"), 0.0
    err = float("inf")
    frac = 0.0
    for _ in range(iters):
        moved = src @ R.T + t
        d, j = dst_tree.query(moved, k=1, workers=-1)
        keep = d < reject
        frac = float(keep.mean())
        if keep.sum() < 30:
            break
        Rn, tn = kabsch(src[keep], dst_pts[j[keep]])
        new_err = float(d[keep].mean())
        R, t = Rn, tn
        if abs(err - new_err) < tol:
            err = new_err
            break
        err = new_err
    return R, t, err, frac


# ======================================================================
#  3  BUILD
# ======================================================================

def build(root, use_icp=True, max_sweeps=None, vox=0.08, map_vox=0.10,
          keyframe_m=0.25, occ_res=0.05, topic=None):
    from scipy.spatial import cKDTree

    meta, sweeps, odom = load_survey(root, max_sweeps, topic)
    if not sweeps:
        print("No decodable point clouds in this survey.")
        print("  If clouds/ is empty the robot never published one - check")
        print("  the probe output. If it is full, the layout in meta.json may")
        print("  be missing; print meta['cloud_layout'].")
        return None

    print(f"  {len(sweeps)} sweeps, "
          f"{np.mean([len(s['pts']) for s in sweeps]):.0f} points each")
    print(f"  odometry: {len(odom)} poses"
          + ("" if odom else "  (none - ICP alone)"))

    ot = np.array([o["t"] for o in odom]) if odom else None

    def odom_at(t):
        if ot is None or not len(ot):
            return None
        i = int(np.argmin(np.abs(ot - t)))
        if abs(ot[i] - t) > 0.5:
            return None
        o = odom[i]
        return quat_to_R(o["q"]), np.array(o["p"], float)

    world = []                      # accumulated map, in map frame
    occ = Occupancy(res=occ_res)
    poses = []
    rows = []
    R, t = np.eye(3), np.zeros(3)
    dR, dt = np.eye(3), np.zeros(3)     # last motion, used to predict
    tree = None
    acc = None
    last_kf = None

    t0 = time.time()
    for i, s in enumerate(sweeps):
        pts = voxel(s["pts"], vox)
        od = odom_at(s["t"])

        if i == 0:
            R, t = (od if od else (np.eye(3), np.zeros(3)))
            src = "odom" if od else "origin"
            err, frac = 0.0, 1.0
        else:
            if od:
                Rg, tg = od                       # odometry as the guess
                src = "odom"
            else:
                Rg, tg = dR @ R, dR @ t + dt      # constant velocity
                src = "predict"
            if use_icp and tree is not None:
                Rn, tn, err, frac = icp(pts, tree, acc, Rg, tg)
                if frac > 0.35:
                    Rg, tg = Rn, tn
                    src += "+icp"
                else:
                    err, frac = float("nan"), frac
                    src += "+icp failed"
            else:
                err, frac = float("nan"), float("nan")
            if not (np.isfinite(Rg).all() and np.isfinite(tg).all()):
                # a diverged ICP must not poison every sweep after it
                Rg, tg = R, t
                src += " (diverged, held)"
            dR = Rg @ R.T
            dt = tg - dR @ t
            if not (np.isfinite(dR).all() and np.isfinite(dt).all()):
                dR, dt = np.eye(3), np.zeros(3)
            R, t = Rg, tg

        w = pts @ R.T + t
        occ.add(w, t[:2])
        poses.append({"t": s["t"], "p": t.tolist(), "src": src,
                      "err": None if np.isnan(err) else round(err, 4),
                      "inliers": None if np.isnan(frac) else round(frac, 3)})

        # only grow the map when the dog has actually moved, otherwise a
        # pause fills the map with thousands of copies of one view
        if last_kf is None or np.linalg.norm(t - last_kf) > keyframe_m:
            world.append(w)
            last_kf = t.copy()
            acc = voxel(np.vstack(world), map_vox)
            if len(acc) > 400_000:
                acc = voxel(acc, map_vox * 1.5)
            tree = cKDTree(acc)
        elif tree is None:
            acc = voxel(w, map_vox)
            tree = cKDTree(acc)

        if (i + 1) % 20 == 0 or i == len(sweeps) - 1:
            print(f"\r  {i+1}/{len(sweeps)}  map {len(acc):>7} pts  "
                  f"pos ({t[0]:6.2f},{t[1]:6.2f})  ", end="", flush=True)
    print(f"\n  built in {time.time()-t0:.1f}s")

    cloud = voxel(np.vstack(world), map_vox) if world else np.zeros((0, 3))
    P = np.array([p["p"] for p in poses])
    dist = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()) \
        if len(P) > 1 else 0.0

    g, origin = occ.grid()
    return {"meta": meta, "cloud": cloud, "poses": poses, "path": P,
            "distance_m": dist, "sweeps": len(sweeps),
            "had_odom": bool(odom), "grid": g, "origin": origin}


# ======================================================================
#  4  OCCUPANCY  (what a navigator actually needs)
# ======================================================================

class Occupancy:
    """A 2D grid built sweep by sweep: 0 unknown, 1 free, 2 occupied.

    Free space has to be carved from each sweep separately, at the pose that
    sweep was taken from. Carving it afterwards from the merged map draws
    rays to points the sensor could not see from there, and they go straight
    through walls - which is exactly what the first version of this did.

    Only the band between z_lo and z_hi counts as an obstacle. Below is
    floor and above is ceiling; neither stops a 40 cm robot, and leaving
    them in fills every corridor solid.
    """

    def __init__(self, res=0.05, z_lo=0.15, z_hi=1.4, reach=8.0):
        self.res, self.z_lo, self.z_hi, self.reach = res, z_lo, z_hi, reach
        self.cells = {}                      # (ix, iy) -> 1 free, 2 occupied

    def _ix(self, xy):
        return np.floor(xy / self.res).astype(np.int32)

    def add(self, world_pts, pose_xy):
        band = world_pts[(world_pts[:, 2] > self.z_lo) &
                         (world_pts[:, 2] < self.z_hi)]
        if not len(band):
            return
        d = band[:, :2] - pose_xy
        n = np.linalg.norm(d, axis=1)
        keep = (n > 0.25) & (n < self.reach)
        if not keep.any():
            return
        d, n, hit = d[keep], n[keep], band[keep][:, :2]

        # free space: along each ray, stopping short of the hit itself
        steps = np.linspace(0.2, 0.9, 10)[:, None, None]
        ray = pose_xy[None, None, :] + steps * d[None, :, :]
        for ix, iy in self._ix(ray.reshape(-1, 2)):
            self.cells.setdefault((ix, iy), 1)

        for ix, iy in self._ix(hit):         # occupied always wins
            self.cells[(ix, iy)] = 2

    def grid(self):
        if not self.cells:
            return None, None
        k = np.array(list(self.cells.keys()))
        v = np.array(list(self.cells.values()), np.uint8)
        lo = k.min(0) - 2
        hi = k.max(0) + 2
        W, H = int(hi[0] - lo[0] + 1), int(hi[1] - lo[1] + 1)
        if W * H > 40_000_000:
            return None, None
        g = np.zeros((H, W), np.uint8)
        g[k[:, 1] - lo[1], k[:, 0] - lo[0]] = v
        return g, (lo.astype(float) * self.res, self.res)


# ======================================================================

def preview(out, res_m, origin, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    grid, path, cloud = out["grid"], out["path"], out["cloud"]
    fig, ax = plt.subplots(1, 2, figsize=(13, 6), dpi=140)

    if grid is not None:
        lo, res = origin
        ax[0].imshow(grid, origin="lower", cmap="Greys",
                     extent=[lo[0], lo[0] + grid.shape[1] * res,
                             lo[1], lo[1] + grid.shape[0] * res],
                     vmin=0, vmax=2, interpolation="nearest")
    if len(path):
        ax[0].plot(path[:, 0], path[:, 1], "-", lw=1.6, color="#d9534f")
        ax[0].plot(path[0, 0], path[0, 1], "o", ms=7, color="#2a9d4a")
        ax[0].plot(path[-1, 0], path[-1, 1], "s", ms=7, color="#d9534f")
    ax[0].set_title(f"occupancy  ·  path {out['distance_m']:.1f} m  "
                    f"(green = start)", fontsize=10)
    ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
    ax[0].set_aspect("equal")

    if len(cloud):
        k = min(len(cloud), 120_000)
        c = cloud[np.random.default_rng(0).choice(len(cloud), k, replace=False)]
        sc = ax[1].scatter(c[:, 0], c[:, 1], s=0.4, c=c[:, 2],
                           cmap="viridis", linewidths=0)
        plt.colorbar(sc, ax=ax[1], label="height z (m)", shrink=0.8)
    ax[1].set_title(f"point map  ·  {len(cloud):,} points", fontsize=10)
    ax[1].set_xlabel("x (m)"); ax[1].set_aspect("equal")
    fig.tight_layout()
    fig.savefig(png, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("survey")
    ap.add_argument("--no-icp", action="store_true")
    ap.add_argument("--max", type=int, default=None)
    ap.add_argument("--topic", default=None,
                    help="which cloud topic to map, if several were caught")
    ap.add_argument("--voxel", type=float, default=0.10)
    ap.add_argument("--res", type=float, default=0.05)
    a = ap.parse_args()

    root = a.survey.rstrip("/")
    if not os.path.isdir(root):
        sys.exit(f"not a directory: {root}")
    print(f"building a map from {root}\n")

    out = build(root, use_icp=not a.no_icp, max_sweeps=a.max,
                map_vox=a.voxel, occ_res=a.res, topic=a.topic)
    if out is None:
        return
    grid, origin = out["grid"], out["origin"]

    np.savez_compressed(os.path.join(root, "map.npz"),
                        cloud=out["cloud"], path=out["path"])
    if grid is not None:
        np.save(os.path.join(root, "occupancy.npy"), grid)
    preview(out, a.res, origin or (np.zeros(2), a.res),
            os.path.join(root, "map_preview.png"))

    errs = [p["err"] for p in out["poses"] if p["err"] is not None]
    rep = {"sweeps": out["sweeps"], "had_odometry": out["had_odom"],
           "distance_m": round(out["distance_m"], 2),
           "map_points": int(len(out["cloud"])),
           "scan_match_residual_m": {
               "median": round(float(np.median(errs)), 4) if errs else None,
               "p90": round(float(np.percentile(errs, 90)), 4) if errs else None},
           "occupancy": None if grid is None else {
               "shape": list(grid.shape), "res_m": a.res,
               "free_cells": int((grid == 1).sum()),
               "occupied_cells": int((grid == 2).sum())},
           "poses": out["poses"]}
    with open(os.path.join(root, "map_report.json"), "w") as f:
        json.dump(rep, f, indent=1)

    print(f"\n  travelled      {out['distance_m']:.1f} m")
    print(f"  map            {len(out['cloud']):,} points")
    if errs:
        print(f"  scan residual  median {np.median(errs)*100:.1f} cm, "
              f"p90 {np.percentile(errs,90)*100:.1f} cm")
    if grid is not None:
        print(f"  occupancy      {grid.shape[1]}x{grid.shape[0]} cells at "
              f"{a.res} m, {(grid==2).sum():,} occupied")
    print(f"\nwrote map.npz, occupancy.npy, map_preview.png, "
          f"map_report.json in\n  {root}")


if __name__ == "__main__":
    main()
