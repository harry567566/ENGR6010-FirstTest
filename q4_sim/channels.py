"""The 25 pixel features from my Reading 3 lasso experiment (lasso_weed.py).

Ten agricultural vegetation indices, twelve color-space channels and three
texture measures, all computed from one RGB image. Copied unchanged so the
supervised-learning part of Question 4 uses exactly the features the Reading 3
lasso chose from.
"""
import cv2
import numpy as np

EPS = 1e-6

NAMES = ["ExG", "ExR", "ExB", "ExGR", "CIVE", "NDI", "VEG", "GLI", "NGRDI",
         "MExG", "R", "G", "B", "H", "S", "V", "L", "a", "bb", "Y", "Cr",
         "Cb", "sd5", "sobel", "lap"]

PRETTY = {"bb": "Lab-b", "a": "Lab-a", "L": "Lab-L", "sd5": "local sd 5x5",
          "sobel": "Sobel edges", "lap": "Laplacian"}


def _chroma(bgr):
    """Normalised chromatic coordinates r, g, b that all the indices use."""
    f = bgr.astype(np.float32)
    B, G, R = f[..., 0], f[..., 1], f[..., 2]
    tot = R + G + B + EPS
    return R / tot, G / tot, B / tot, R, G, B


def channel_bank(bgr):
    """Returns (dict of name -> float32 image, list of names in a fixed order)."""
    r, g, b, R, G, B = _chroma(bgr)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    grayf = gray.astype(np.float32)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab).astype(np.float32)
    ycc = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb).astype(np.float32)

    # local texture: standard deviation in a 5x5 window, via E[x^2] - E[x]^2
    k = (5, 5)
    m = cv2.blur(grayf, k)
    m2 = cv2.blur(grayf * grayf, k)
    local_sd = np.sqrt(np.maximum(m2 - m * m, 0))

    out = {
        # --- vegetation indices (the agricultural literature's standards) ---
        "ExG":   2 * g - r - b,
        "ExR":   1.4 * r - g,
        "ExB":   1.4 * b - g,
        "ExGR":  (2 * g - r - b) - (1.4 * r - g),
        "CIVE":  0.441 * r - 0.811 * g + 0.385 * b + 18.78745,
        "NDI":   (g - r) / (g + r + EPS),
        "VEG":   g / (np.power(r + EPS, 0.667) * np.power(b + EPS, 0.333) + EPS),
        "GLI":   (2 * G - R - B) / (2 * G + R + B + EPS),
        "NGRDI": (G - R) / (G + R + EPS),
        "MExG":  1.262 * G - 0.884 * R - 0.311 * B,
        # --- raw and colour-space channels ---
        "R": R, "G": G, "B": B,
        "H": hsv[..., 0], "S": hsv[..., 1], "V": hsv[..., 2],
        "L": lab[..., 0], "a": lab[..., 1], "bb": lab[..., 2],
        "Y": ycc[..., 0], "Cr": ycc[..., 1], "Cb": ycc[..., 2],
        # --- texture ---
        "sd5":  local_sd,
        "sobel": cv2.magnitude(cv2.Sobel(grayf, cv2.CV_32F, 1, 0, ksize=3),
                               cv2.Sobel(grayf, cv2.CV_32F, 0, 1, ksize=3)),
        "lap":  np.abs(cv2.Laplacian(grayf, cv2.CV_32F, ksize=3)),
    }
    return out, list(out.keys())


def feature_stack(bgr):
    """The design matrix for one photograph, as height x width x 25."""
    bank, _ = channel_bank(bgr)
    return np.stack([bank[n] for n in NAMES], axis=-1).astype(np.float32)
