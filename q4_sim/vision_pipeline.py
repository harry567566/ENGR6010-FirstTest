"""Run the five methods from Question 1 on the synthetic Go2 camera data.

Every part is graded against ground truth that the simulator gives for free:
the segmentation render says which object is under each pixel, and the depth
plus the camera poses give the true optical flow (flow_gt.py).

    python vision_pipeline.py            all five parts
    python vision_pipeline.py --only 3   one part

Writes results/q4_*.png and results/q4_results.json.
"""
import argparse
import json
import os
import time

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import channels  # noqa: E402
from flow_gt import epe, flow_to_color, gt_flow, load_depth  # noqa: E402
from scene import LABELS  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data", "sim_run")
OUT = os.path.join(HERE, "results")
SOIL, WEED, ROBOT, SKY = LABELS["soil"], LABELS["weed"], LABELS["robot"], LABELS["background"]
OBJECTS = ["weed", "red_box", "gray_box", "soil_box", "blue_cyl", "yellow_ball"]


class Run:
    def __init__(self, path=DATA):
        self.path = path
        m = np.load(os.path.join(path, "meta.npz"))
        self.K = m["K"]
        self.t, self.label = m["t"], m["label"]
        self.R, self.C = m["cam_R"], m["cam_pos"]
        self.n = len(self.t)

    def rgb(self, i):
        return cv2.imread(os.path.join(self.path, f"rgb_{i:04d}.png"))

    def gray(self, i):
        return cv2.cvtColor(self.rgb(i), cv2.COLOR_BGR2GRAY)

    def seg(self, i):
        return cv2.imread(os.path.join(self.path, f"seg_{i:04d}.png"), cv2.IMREAD_GRAYSCALE)

    def depth(self, i):
        return load_depth(os.path.join(self.path, f"depth_{i:04d}.png"))

    def flow(self, i):
        """True flow from frame i to i+1, and where it is valid (no sky, robot, occlusion)."""
        s = self.seg(i)
        f, v = gt_flow(self.depth(i), self.R[i], self.C[i], self.R[i + 1], self.C[i + 1], self.K,
                       depth1=self.depth(i + 1), robot_mask=(s == ROBOT))
        return f, v & (s != SKY)


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  figure -> {os.path.relpath(p, HERE)}")


# ---------------------------------------------------------------- 1 color space
def part1_color(run):
    print("\n[1] Color space: which channel separates weed from soil, and where hue breaks")
    from sklearn.metrics import roc_auc_score
    frames = [i for i in range(run.n) if (run.seg(i) == WEED).sum() > 400][:40]
    rng = np.random.default_rng(0)
    rows = []
    for i in frames:
        bgr, s = run.rgb(i), run.seg(i)
        spaces = {"B": bgr[..., 0], "G": bgr[..., 1], "R": bgr[..., 2]}
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
        ycc = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
        spaces.update({"H": hsv[..., 0], "S": hsv[..., 1], "V": hsv[..., 2],
                       "L": lab[..., 0], "a": lab[..., 1], "b": lab[..., 2],
                       "Y": ycc[..., 0], "Cr": ycc[..., 1], "Cb": ycc[..., 2]})
        w = np.flatnonzero((s == WEED).ravel())
        g = np.flatnonzero((s == SOIL).ravel())
        w = rng.choice(w, min(300, len(w)), replace=False)
        g = rng.choice(g, min(300, len(g)), replace=False)
        rows.append((np.concatenate([w, g]), np.r_[np.ones(len(w)), np.zeros(len(g))], spaces))
    names = list(rows[0][2].keys())
    auc = {}
    for nm in names:
        x = np.concatenate([r[2][nm].ravel()[r[0]].astype(np.float32) for r in rows])
        y = np.concatenate([r[1] for r in rows])
        a = roc_auc_score(y, x)
        auc[nm] = float(max(a, 1 - a))

    # hue stability: nudge every pixel by (+1, -1, +1) in RGB, as in Reading 2
    def hue_shift(bgr, mask):
        h0 = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[..., 0].astype(np.int16)
        n = np.clip(bgr.astype(np.int16) + np.array([1, -1, 1]), 0, 255).astype(np.uint8)
        h1 = cv2.cvtColor(n, cv2.COLOR_BGR2HSV)[..., 0].astype(np.int16)
        d = np.abs(h1 - h0)
        d = np.minimum(d, 180 - d) * 2.0          # OpenCV hue is 0..179 -> degrees
        return float(d[mask].mean())
    hue = {}
    for obj in ["red_box", "gray_box"]:
        best = max(range(run.n), key=lambda i: (run.seg(i) == LABELS[obj]).sum())
        hue[obj] = hue_shift(run.rgb(best), run.seg(best) == LABELS[obj])
    sat = {}
    for obj in ["red_box", "gray_box"]:
        best = max(range(run.n), key=lambda i: (run.seg(i) == LABELS[obj]).sum())
        sat[obj] = float(cv2.cvtColor(run.rgb(best), cv2.COLOR_BGR2HSV)[..., 1][run.seg(best) == LABELS[obj]].mean())

    order = sorted(auc, key=auc.get, reverse=True)
    print("  AUC for weed vs soil, one channel at a time (1.0 = perfect, 0.5 = useless):")
    print("   " + "  ".join(f"{k}={auc[k]:.3f}" for k in order))
    print(f"  hue shift for a 1-unit RGB nudge: red box {hue['red_box']:.2f} deg "
          f"(S={sat['red_box']:.0f}), gray box {hue['gray_box']:.2f} deg (S={sat['gray_box']:.0f})")

    # figure
    i = frames[len(frames) // 2]
    bgr = run.rgb(i)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
    fig, ax = plt.subplots(1, 5, figsize=(17, 3.2), gridspec_kw={"width_ratios": [1, 1, 1, 1, 1.25]})
    for a_, img, tt in [(ax[0], cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), "RGB frame"),
                        (ax[1], bgr[..., 2], "R channel"), (ax[2], hsv[..., 0], "H (hue)"),
                        (ax[3], lab[..., 1], "Lab a (green to red)")]:
        a_.imshow(img, cmap=None if img.ndim == 3 else "gray")
        a_.set_title(tt)
        a_.axis("off")
    ax[4].barh(order[::-1], [auc[k] for k in order[::-1]],
               color=["#3b9668" if auc[k] > 0.95 else "#999999" for k in order[::-1]])
    ax[4].set_xlim(0.5, 1.0)
    ax[4].set_title("weed vs soil, AUC per channel")
    ax[4].axvline(0.95, ls="--", color="k", lw=0.8)
    save(fig, "q4_color.png")
    return {"auc_weed_vs_soil": auc, "hue_shift_deg": hue, "mean_saturation": sat,
            "frames_used": len(frames)}


# ---------------------------------------------------------------- 2 pyramid
def part2_pyramid(run):
    print("\n[2] Gaussian pyramid: tracking with and without the pyramid, against true flow")
    bins = [0, 2, 5, 10, 20, 40, 90]
    levels = [0, 1, 2, 3]
    err = {L: [[] for _ in bins[:-1]] for L in levels}
    n_pts = 0
    for i in range(run.n - 1):
        g0, g1, s = run.gray(i), run.gray(i + 1), run.seg(i)
        mask = ((s != SKY) & (s != ROBOT)).astype(np.uint8) * 255
        pts = cv2.goodFeaturesToTrack(g0, 300, 0.01, 7, mask=mask)
        if pts is None:
            continue
        fl, val = run.flow(i)
        p = pts.reshape(-1, 2)
        ui = np.clip(np.round(p[:, 0]).astype(int), 0, 639)
        vi = np.clip(np.round(p[:, 1]).astype(int), 0, 479)
        ok = val[vi, ui]
        p, true = p[ok], fl[vi[ok], ui[ok]]
        if len(p) == 0:
            continue
        n_pts += len(p)
        mag = np.linalg.norm(true, axis=1)
        b = np.digitize(mag, bins) - 1
        for L in levels:
            q, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p.reshape(-1, 1, 2).astype(np.float32), None,
                                                winSize=(21, 21), maxLevel=L,
                                                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
            e = np.linalg.norm(q.reshape(-1, 2) - p - true, axis=1)
            e[st.ravel() == 0] = np.inf                     # lost point = failure
            for k in range(len(bins) - 1):
                err[L][k].extend(e[b == k].tolist())
    labels = [f"{bins[k]}-{bins[k + 1]}" for k in range(len(bins) - 1)]
    success = {L: [float(np.mean(np.array(err[L][k]) < 1.0)) if err[L][k] else np.nan
                   for k in range(len(bins) - 1)] for L in levels}
    counts = [len(err[0][k]) for k in range(len(bins) - 1)]
    print(f"  {n_pts} tracked corners over {run.n - 1} frame pairs")
    print("  share of corners tracked to within 1 px of the true position:")
    print("   motion (px)  " + "".join(f"{x:>9}" for x in labels))
    print("   corners      " + "".join(f"{c:>9}" for c in counts))
    for L in levels:
        print(f"   levels={L}     " + "".join(f"{100 * v:>8.0f}%" for v in success[L]))

    fig, ax = plt.subplots(1, 2, figsize=(14, 3.6), gridspec_kw={"width_ratios": [1.4, 1]})
    img = run.rgb(run.n // 3)
    pyr, cur = [], img
    for _ in range(4):
        pyr.append(cur)
        cur = cv2.pyrDown(cur)
    canvas = np.full((480, 640 + 320 + 10, 3), 255, np.uint8)
    canvas[:, :640] = pyr[0]
    y = 0
    for lv in pyr[1:]:
        h, w = lv.shape[:2]
        canvas[y:y + h, 650:650 + w] = lv
        y += h + 10
    ax[0].imshow(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    ax[0].set_title("Gaussian pyramid: 640x480, 320x240, 160x120, 80x60")
    ax[0].axis("off")
    for L, c in zip(levels, ["#b03030", "#d08030", "#3070b0", "#3b9668"]):
        ax[1].plot(range(len(labels)), [100 * v for v in success[L]], "o-", color=c,
                   label=f"{L} extra levels" if L else "no pyramid")
    ax[1].set_xticks(range(len(labels)))
    ax[1].set_xticklabels(labels)
    ax[1].set_xlabel("true motion between frames (pixels)")
    ax[1].set_ylabel("% tracked within 1 px")
    ax[1].set_title("Lucas-Kanade, 21x21 window")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    save(fig, "q4_pyramid.png")
    return {"bins_px": labels, "corners_per_bin": counts,
            "success_within_1px": {str(L): success[L] for L in levels}, "corners": n_pts}


# ---------------------------------------------------------------- 3 optical flow
def part3_flow(run):
    print("\n[3] Optical flow: dense methods against the true flow")
    dis = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
    clahe = cv2.createCLAHE(4.0, (8, 8))
    methods = {
        "Farneback": lambda a, b: cv2.calcOpticalFlowFarneback(a, b, None, 0.5, 4, 21, 5, 7, 1.5, 0),
        "Farneback+CLAHE": lambda a, b: cv2.calcOpticalFlowFarneback(clahe.apply(a), clahe.apply(b),
                                                                     None, 0.5, 4, 21, 5, 7, 1.5, 0),
        "DIS": lambda a, b: dis.calc(a, b, None),
    }
    groups = {"forward": [], "turn": [], "sidestep": []}
    res = {m: {g: [] for g in groups} for m in methods}
    true_mag = {g: [] for g in groups}
    t_used = {m: 0.0 for m in methods}
    for i in range(run.n - 1):
        lab = str(run.label[i])
        g = "turn" if lab.startswith("turn") else lab if lab in groups else None
        if g is None or str(run.label[i + 1]) != lab:
            continue
        a, b = run.gray(i), run.gray(i + 1)
        fl, val = run.flow(i)
        true_mag[g].append(float(np.median(np.linalg.norm(fl[val], axis=1))))
        for m, fn in methods.items():
            t0 = time.perf_counter()
            est = fn(a, b)
            t_used[m] += time.perf_counter() - t0
            res[m][g].append(epe(est, fl, val))
    table = {m: {g: float(np.mean(v)) for g, v in res[m].items() if v} for m in methods}
    med = {g: float(np.median(v)) for g, v in true_mag.items() if v}
    print("  mean end-point error in pixels (lower is better)")
    print("   motion       true |flow|  " + "".join(f"{m:>17}" for m in methods))
    for g in groups:
        if g in med:
            print(f"   {g:<12}{med[g]:>9.1f}px  " + "".join(f"{table[m][g]:>15.2f}px" for m in methods))
    n_pairs = sum(len(v) for v in true_mag.values())
    print("   time per pair  " + "".join(f"{1000 * t_used[m] / n_pairs:>15.1f}ms" for m in methods))

    # figure: one forward pair and one turning pair
    picks = []
    for want in ["forward", "turn_left"]:
        picks.append(next(i for i in range(10, run.n - 1)
                          if str(run.label[i]) == want and str(run.label[i + 1]) == want))
    fig, ax = plt.subplots(2, 4, figsize=(16, 6.2))
    for r, i in enumerate(picks):
        fl, val = run.flow(i)
        a, b = run.gray(i), run.gray(i + 1)
        mx = np.percentile(np.linalg.norm(fl[val], axis=1), 99)
        panels = [(cv2.cvtColor(run.rgb(i), cv2.COLOR_BGR2RGB), f"frame {i} ({run.label[i]})"),
                  (np.where(val[..., None], cv2.cvtColor(flow_to_color(fl, mx), cv2.COLOR_BGR2RGB), 255),
                   "true flow (from depth + pose)"),
                  (cv2.cvtColor(flow_to_color(methods["Farneback"](a, b), mx), cv2.COLOR_BGR2RGB),
                   f"Farneback  EPE {epe(methods['Farneback'](a, b), fl, val):.1f}px"),
                  (cv2.cvtColor(flow_to_color(methods["DIS"](a, b), mx), cv2.COLOR_BGR2RGB),
                   f"DIS  EPE {epe(methods['DIS'](a, b), fl, val):.1f}px")]
        for c, (img, tt) in enumerate(panels):
            ax[r, c].imshow(img)
            ax[r, c].set_title(tt, fontsize=10)
            ax[r, c].axis("off")
    save(fig, "q4_flow.png")
    return {"mean_epe_px": table, "median_true_flow_px": med,
            "ms_per_pair": {m: 1000 * t_used[m] / n_pairs for m in methods}, "pairs": n_pairs}


# ---------------------------------------------------------------- 4 grabcut
def part4_grabcut(run):
    print("\n[4] GrabCut: a rectangle around each object, scored against the true mask")
    out, crops = {}, []
    for obj in OBJECTS:
        lid = LABELS[obj]
        best, best_area, best_cc = None, 0, None
        for i in range(run.n):
            m = (run.seg(i) == lid).astype(np.uint8)
            n, cc, stats, _ = cv2.connectedComponentsWithStats(m, 8)
            for k in range(1, n):
                x, y, w, h, area = stats[k]
                if x > 5 and y > 5 and x + w < 635 and y + h < 475 and area > best_area:
                    best, best_area, best_cc = i, area, (cc == k)
        if best is None:
            continue
        img = run.rgb(best)
        ys, xs = np.nonzero(best_cc)
        pad = max(8, int(0.15 * max(np.ptp(xs), np.ptp(ys))))
        x0, y0 = max(0, xs.min() - pad), max(0, ys.min() - pad)
        x1, y1 = min(639, xs.max() + pad), min(479, ys.max() + pad)
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.setRNGSeed(0)
        cv2.grabCut(img, mask, (int(x0), int(y0), int(x1 - x0), int(y1 - y0)),
                    np.zeros((1, 65), np.float64), np.zeros((1, 65), np.float64), 5,
                    cv2.GC_INIT_WITH_RECT)
        pred = (mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)
        iou = float((pred & best_cc).sum() / max((pred | best_cc).sum(), 1))
        box = np.zeros_like(best_cc)
        box[y0:y1 + 1, x0:x1 + 1] = True
        ring = box & ~best_cc
        f = img.reshape(-1, 3).astype(np.float32)
        mo, mr = f[best_cc.ravel()].mean(0), f[ring.ravel()].mean(0)
        sr = f[ring.ravel()].std(0).mean() + 1e-6
        cnr = float(np.linalg.norm(mo - mr) / sr)
        out[obj] = {"frame": int(best), "area_px": int(best_area), "iou": iou,
                    "contrast_to_noise": cnr}
        print(f"  {obj:<12} frame {best:>3}  area {best_area:>6}px  "
              f"contrast/noise {cnr:>5.2f}   IoU {iou:.3f}")
        crop = img[y0:y1 + 1, x0:x1 + 1].copy()
        over = crop.copy()
        over[~pred[y0:y1 + 1, x0:x1 + 1]] = (over[~pred[y0:y1 + 1, x0:x1 + 1]] * 0.25).astype(np.uint8)
        cnt, _ = cv2.findContours(best_cc[y0:y1 + 1, x0:x1 + 1].astype(np.uint8), cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(over, cnt, -1, (0, 255, 255), 1)
        crops.append((obj, crop, over, iou, cnr))

    fig, ax = plt.subplots(2, len(crops), figsize=(2.6 * len(crops), 5.4))
    for c, (obj, crop, over, iou, cnr) in enumerate(crops):
        ax[0, c].imshow(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        ax[0, c].set_title(f"{obj}\ncontrast/noise {cnr:.1f}", fontsize=9)
        ax[1, c].imshow(cv2.cvtColor(over, cv2.COLOR_BGR2RGB))
        ax[1, c].set_title(f"GrabCut IoU {iou:.2f}", fontsize=9)
        ax[0, c].axis("off")
        ax[1, c].axis("off")
    fig.suptitle("top: the rectangle given to GrabCut.  bottom: its cut (dark = background), "
                 "yellow = true outline", fontsize=10)
    save(fig, "q4_grabcut.png")
    return out


# ---------------------------------------------------------------- 5 supervised
def part5_learning(run):
    print("\n[5] Supervised learning: weed pixels, exact labels, six classifiers")
    from sklearn.linear_model import LogisticRegression
    from sklearn.metrics import f1_score
    from sklearn.naive_bayes import GaussianNB
    from sklearn.neighbors import KNeighborsClassifier
    from sklearn.neural_network import MLPClassifier
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler
    from sklearn.svm import LinearSVC
    from sklearn.tree import DecisionTreeClassifier

    has_weed = [i for i in range(run.n) if (run.seg(i) == WEED).sum() > 150]
    cut = int(0.6 * len(has_weed))
    train_f, test_f = has_weed[:cut], has_weed[cut:]
    rng = np.random.default_rng(0)

    def sample(frames, per=300):
        X, y = [], []
        for i in frames:
            F = channels.feature_stack(run.rgb(i)).reshape(-1, len(channels.NAMES))
            s = run.seg(i).ravel()
            pos = np.flatnonzero(s == WEED)
            neg = np.flatnonzero((s != WEED) & (s != SKY) & (s != ROBOT))
            pos = rng.choice(pos, min(per, len(pos)), replace=False)
            neg = rng.choice(neg, min(per, len(neg)), replace=False)
            X.append(F[np.r_[pos, neg]])
            y.append(np.r_[np.ones(len(pos)), np.zeros(len(neg))])
        return np.concatenate(X), np.concatenate(y)
    Xtr, ytr = sample(train_f)
    Xte, yte = sample(test_f)
    # VEG divides by r and b, so near-black pixels give huge values; float64 avoids
    # overflow inside the matrix products
    Xtr, Xte = Xtr.astype(np.float64), Xte.astype(np.float64)
    print(f"  train: {len(train_f)} frames, {len(ytr)} pixels   test: {len(test_f)} later frames, "
          f"{len(yte)} pixels")

    models = {
        "Lasso logistic (L1)": make_pipeline(StandardScaler(), LogisticRegression(
            C=0.01, penalty="l1", solver="liblinear", max_iter=5000, random_state=0)),
        "Decision tree": DecisionTreeClassifier(max_depth=8, random_state=0),
        "Naive Bayes": make_pipeline(StandardScaler(), GaussianNB()),
        "k-nearest neighbors": make_pipeline(StandardScaler(), KNeighborsClassifier(5)),
        "Linear SVM": make_pipeline(StandardScaler(), LinearSVC(C=1.0, random_state=0)),
        "Neural net (MLP)": make_pipeline(StandardScaler(), MLPClassifier((32, 16), max_iter=500,
                                                                          random_state=0)),
    }
    out = {}
    for nm, mdl in models.items():
        t0 = time.perf_counter()
        mdl.fit(Xtr, ytr)
        t_fit = time.perf_counter() - t0
        t0 = time.perf_counter()
        p = mdl.predict(Xte)
        t_pred = (time.perf_counter() - t0) / len(yte) * 640 * 480 * 1000
        out[nm] = {"f1": float(f1_score(yte, p)), "fit_s": t_fit, "ms_per_image": t_pred}
        print(f"  {nm:<22} F1 {out[nm]['f1']:.3f}   fit {t_fit:6.2f}s   ~{t_pred:7.1f} ms per image")
    lasso = models["Lasso logistic (L1)"][-1]
    kept = [channels.NAMES[j] for j in np.flatnonzero(lasso.coef_.ravel())]
    out["Lasso logistic (L1)"]["kept"] = kept
    print(f"  at C=0.01 the lasso kept {len(kept)} of 25 channels")

    # the lasso path: make the penalty stronger until only a few channels survive
    # saga stops at max_iter on this almost separable data before the weights reach
    # exactly zero, so the path uses liblinear's coordinate descent, which does
    path = []
    for C in [1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 1e-1]:
        mdl = make_pipeline(StandardScaler(), LogisticRegression(
            C=C, penalty="l1", solver="liblinear", max_iter=5000, random_state=0)).fit(Xtr, ytr)
        k = [channels.NAMES[j] for j in np.flatnonzero(mdl[-1].coef_.ravel())]
        f1 = float(f1_score(yte, mdl.predict(Xte))) if k else 0.0
        path.append({"C": C, "kept": k, "f1": f1})
        print(f"   C={C:<7g} kept {len(k):>2}: {', '.join(k[:6])}{' ...' if len(k) > 6 else ''}"
              f"   test F1 {f1:.3f}")
    out["lasso_path"] = path

    # full-image predictions on one test frame
    i = test_f[len(test_f) // 2]
    F = channels.feature_stack(run.rgb(i)).reshape(-1, 25)
    truth = run.seg(i) == WEED
    fig, ax = plt.subplots(1, 4, figsize=(17, 3.3))
    ax[0].imshow(cv2.cvtColor(run.rgb(i), cv2.COLOR_BGR2RGB))
    ax[0].set_title(f"test frame {i}")
    ax[1].imshow(truth, cmap="Greens")
    ax[1].set_title("true weed pixels (from the simulator)")
    for a_, nm in [(ax[2], "Lasso logistic (L1)"), (ax[3], "Decision tree")]:
        pm = models[nm].predict(F).reshape(480, 640).astype(bool)
        iou = (pm & truth).sum() / max((pm | truth).sum(), 1)
        a_.imshow(pm, cmap="Greens")
        a_.set_title(f"{nm}: IoU {iou:.2f}")
        out[nm]["full_frame_iou"] = float(iou)
    for a_ in ax:
        a_.axis("off")
    save(fig, "q4_learning.png")
    return {"classifiers": out, "train_frames": len(train_f), "test_frames": len(test_f),
            "train_pixels": int(len(ytr)), "test_pixels": int(len(yte))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, choices=range(1, 6))
    a = ap.parse_args()
    run = Run()
    print(f"{run.n} frames from {os.path.relpath(DATA, HERE)}")
    parts = [part1_color, part2_pyramid, part3_flow, part4_grabcut, part5_learning]
    path = os.path.join(OUT, "q4_results.json")
    results = json.load(open(path)) if os.path.exists(path) else {}
    for k, fn in enumerate(parts, 1):
        if a.only and a.only != k:
            continue
        results[fn.__name__] = fn(run)
    os.makedirs(OUT, exist_ok=True)
    with open(path, "w") as f:
        json.dump(results, f, indent=1, default=float)
    print(f"\nnumbers -> {os.path.relpath(path, HERE)}")


if __name__ == "__main__":
    main()
