"""True optical flow between two simulator frames.

The scene is static and only the camera moves, so every pixel's motion is fixed
by geometry. For pixel (u, v) with depth d in frame t:

    1. back-project it to a 3D point in the camera frame,
    2. move that point into world coordinates with the camera pose at t,
    3. project it into the camera at t+1.

The difference between where it lands and where it started is the true flow.

MuJoCo camera frame: x to the right, y up, and the camera looks along -z.
cam_xmat maps camera-frame vectors to world vectors.

Pixels are marked invalid when the point leaves the image, lands behind the
camera, is hidden by something closer in frame t+1, or belongs to the robot
itself (the robot moves with the camera, so it breaks the static-scene rule).
"""
import numpy as np


def load_depth(path):
    import cv2
    return cv2.imread(path, cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0


def gt_flow(depth0, R0, C0, R1, C1, K, depth1=None, robot_mask=None, occl_tol=0.05):
    H, W = depth0.shape
    f, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    d = depth0.astype(np.float64)

    pc = np.stack([(u - cx) * d / f, -(v - cy) * d / f, -d], axis=-1)   # camera t
    Xw = pc @ R0.T + C0                                                  # world
    pc1 = (Xw - C1) @ R1                                                 # camera t+1
    z1 = -pc1[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u1 = cx + f * pc1[..., 0] / z1
        v1 = cy - f * pc1[..., 1] / z1
    flow = np.stack([u1 - u, v1 - v], axis=-1).astype(np.float32)

    valid = (z1 > 0.05) & (u1 >= 0) & (u1 <= W - 1) & (v1 >= 0) & (v1 <= H - 1)
    if depth1 is not None:
        ui = np.clip(np.round(u1), 0, W - 1).astype(int)
        vi = np.clip(np.round(v1), 0, H - 1).astype(int)
        seen = depth1[vi, ui]
        valid &= np.abs(seen - z1) <= occl_tol * np.maximum(z1, 1e-6)
    if robot_mask is not None:
        valid &= ~robot_mask
    return flow, valid


def epe(flow_est, flow_true, valid):
    """Mean end-point error in pixels over the valid pixels."""
    e = np.linalg.norm(flow_est - flow_true, axis=-1)
    return float(e[valid].mean()) if valid.any() else float("nan")


def flow_to_color(flow, max_mag=None):
    """Hue = direction, brightness = speed (the HW1 / Middlebury convention)."""
    import cv2
    mag, ang = cv2.cartToPolar(flow[..., 0].astype(np.float32), flow[..., 1].astype(np.float32))
    if max_mag is None:
        max_mag = np.percentile(mag, 99) + 1e-6
    hsv = np.zeros(flow.shape[:2] + (3,), np.uint8)
    hsv[..., 0] = (ang * 180 / np.pi / 2).astype(np.uint8)
    hsv[..., 1] = 255
    hsv[..., 2] = np.clip(mag / max_mag * 255, 0, 255).astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
