"""Figure of the synthetic data: camera image, true labels and depth for a few frames."""
import os
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from vision_pipeline import Run, OUT
from scene import LABELS

PAL = {"background": (0.16, 0.16, 0.16), "soil": (0.55, 0.38, 0.24), "weed": (0.2, 0.8, 0.2),
       "red_box": (0.85, 0.1, 0.1), "gray_box": (0.65, 0.65, 0.65), "soil_box": (0.62, 0.45, 0.28),
       "blue_cyl": (0.15, 0.35, 0.9), "yellow_ball": (0.95, 0.8, 0.1), "robot": (1, 1, 1)}
run = Run()
picks = [20, 60, 110, 150]
lut = np.zeros((256, 3))
for k, v in LABELS.items():
    lut[v] = PAL[k]
fig, ax = plt.subplots(3, len(picks), figsize=(4 * len(picks), 8.4))
for c, i in enumerate(picks):
    ax[0, c].imshow(cv2.cvtColor(run.rgb(i), cv2.COLOR_BGR2RGB))
    ax[0, c].set_title(f"t = {run.t[i]:.1f} s  ({run.label[i]})")
    ax[1, c].imshow(lut[run.seg(i)])
    im = ax[2, c].imshow(np.clip(run.depth(i), 0, 8), cmap="turbo")
for r, name in enumerate(["camera image", "true label of every pixel", "depth (m, sky clipped at 8)"]):
    ax[r, 0].set_ylabel(name, fontsize=11)
for a in ax.ravel():
    a.set_xticks([]); a.set_yticks([])
fig.colorbar(im, ax=ax[2, :].tolist(), fraction=0.02, pad=0.01)
out = os.path.join(OUT, "q4_data.png")
fig.savefig(out, dpi=110, bbox_inches="tight"); print("figure ->", out)
