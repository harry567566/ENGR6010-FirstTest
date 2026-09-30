"""Draw the control panel's app icon: the real Go2 model, rendered with MuJoCo, on a blue rounded square.

    ~/miniconda3/envs/go2/bin/python tools/make_panel_icon.py      ->  tools/panel_icon.png (1024 x 1024)
"""
import os

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "..", "third_party", "unitree_mujoco", "unitree_robots", "go2", "go2.xml")
OUT = os.path.join(HERE, "panel_icon.png")
S = 1024          # icon size
R = 2048          # render size (drawn twice as big, then shrunk, for smooth edges)


def render_dog():
    spec = mujoco.MjSpec.from_file(XML)
    spec.visual.global_.offwidth = spec.visual.global_.offheight = R
    spec.worldbody.add_light(pos=[1.5, -1.0, 2.5], dir=[-0.5, 0.35, -1], diffuse=[0.85, 0.85, 0.85],
                             specular=[0.3, 0.3, 0.3])
    spec.worldbody.add_light(pos=[-1.5, 1.5, 1.5], dir=[0.5, -0.5, -0.6], diffuse=[0.35, 0.35, 0.4],
                             specular=[0, 0, 0])
    m = spec.compile()
    d = mujoco.MjData(m)
    d.qpos[:3] = [0, 0, 0.3]
    d.qpos[3:7] = [1, 0, 0, 0]
    for leg in ("FL", "FR", "RL", "RR"):  # standing pose
        for joint, angle in (("hip", 0.0), ("thigh", 0.8), ("calf", -1.5)):
            d.qpos[m.jnt_qposadr[m.joint(f"{leg}_{joint}_joint").id]] = angle
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.02, 0, 0.2]
    cam.distance, cam.azimuth, cam.elevation = 1.15, 138, -16
    r = mujoco.Renderer(m, R, R)
    r.update_scene(d, cam)
    rgb = r.render()
    r.enable_segmentation_rendering()
    r.update_scene(d, cam)
    mask = r.render()[..., 0] >= 0  # background is -1
    rgba = np.dstack([rgb, (mask * 255).astype(np.uint8)])
    img = Image.fromarray(rgba, "RGBA")
    return img.crop(img.getbbox())


def rounded_background():
    # blue vertical gradient in the macOS icon shape: an 824 px rounded square with a 100 px margin
    top, bottom = np.array([72, 132, 250]), np.array([22, 58, 150])
    t = np.linspace(0, 1, S)[:, None, None]
    grad = (top + (bottom - top) * t).astype(np.uint8).repeat(S, axis=1)
    bg = Image.fromarray(np.dstack([grad, np.full((S, S), 255, np.uint8)]), "RGBA")
    shape = Image.new("L", (S, S), 0)
    ImageDraw.Draw(shape).rounded_rectangle([100, 100, S - 100, S - 100], radius=185, fill=255)
    out = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    out.paste(bg, (0, 0), shape)
    return out, shape


def main():
    icon, shape = rounded_background()
    dog = render_dog()
    w = 700
    dog = dog.resize((w, round(dog.height * w / dog.width)), Image.LANCZOS)
    x, y = (S - dog.width) // 2, (S - dog.height) // 2 + 30

    shadow = Image.new("RGBA", (S, S), (0, 0, 0, 0))  # soft shadow under the feet
    ImageDraw.Draw(shadow).ellipse([x + 60, y + dog.height - 40, x + dog.width - 60, y + dog.height + 30],
                                   fill=(5, 20, 60, 150))
    shadow = shadow.filter(ImageFilter.GaussianBlur(22))
    icon.alpha_composite(Image.composite(shadow, Image.new("RGBA", (S, S)), shape))
    icon.alpha_composite(dog, (x, y))
    icon.save(OUT)
    print("saved", OUT)


if __name__ == "__main__":
    main()
