"""Build a small outdoor test scene around the Go2 model from Go2Py.

The original Go2Py scene is an empty dark floor, which is useless as test data.
This adds a soil-colored floor with texture, sunlight, and objects chosen to
test each method from Question 1:

    weeds        green plants on brown soil     -> color space, GrabCut, supervised learning
    red box      strongly saturated color       -> color space (hue works well)
    gray box     almost no saturation           -> color space (hue is unstable near gray)
    soil box     same color as the ground       -> GrabCut camouflage failure case
    blue / yellow objects                       -> more colors for the flow and segmentation

All added objects are visual only (no collision), so they never disturb the
walking controller. The simulator's segmentation render gives the exact object
under every pixel, which is the ground truth the Reading 2 and Reading 3 work
never had.

The scene is built with MuJoCo's MjSpec API on top of Go2Py's go2.xml, so no
third-party file is edited. build_scene() writes a stand-alone XML that
Go2Sim(xml_path=...) can load.
"""
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GO2_XML = os.path.abspath(os.path.join(HERE, "..", "third_party", "Go2Py", "Go2Py",
                                       "assets", "mujoco", "go2.xml"))
OUT_XML = os.path.join(HERE, "go2_field_scene.xml")

# label ids used in every mask this project writes
LABELS = {"background": 0, "soil": 1, "weed": 2, "red_box": 3, "gray_box": 4,
          "soil_box": 5, "blue_cyl": 6, "yellow_ball": 7, "robot": 8}

WEEDS = [(1.6, 0.35), (2.2, -0.45), (2.9, 0.15), (3.4, -0.7), (4.1, 0.55),
         (4.6, -0.2), (5.3, 0.4), (5.9, -0.5), (6.6, 0.1), (7.2, -0.35)]
SOIL_RGB = (0.45, 0.33, 0.22)


def _visual(g, rgba):
    g.rgba = rgba
    g.contype = 0
    g.conaffinity = 0
    g.group = 1


def build_scene(seed=0, out=OUT_XML):
    rng = np.random.default_rng(seed)
    spec = mujoco.MjSpec.from_file(GO2_XML)
    # absolute mesh paths so the XML can live in this folder
    spec.meshdir = os.path.abspath(os.path.join(os.path.dirname(GO2_XML), ".."))

    # --- soil floor: brown with random darker specks (texture helps optical flow)
    tex = spec.add_texture(name="soil", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT,
                           mark=mujoco.mjtMark.mjMARK_RANDOM, random=0.35,
                           rgb1=SOIL_RGB, rgb2=SOIL_RGB,
                           markrgb=(0.30, 0.22, 0.14), width=512, height=512)
    mat = spec.add_material(name="soil")
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "soil"
    mat.texrepeat = (8, 8)
    mat.texuniform = True
    mat.reflectance = 0.0
    for g in spec.geoms:
        if g.name == "floor":
            g.material = "soil"

    # --- Go2Py's "front_camera" uses euler="1.5708 0 0", which points it along
    # the robot's +y axis, so it actually looks out of the dog's LEFT side.
    # Point it forward (+x) with the image "up" along +z: camera x = robot -y,
    # camera y = robot +z, camera z = robot -x (a MuJoCo camera looks along -z).
    for cam in spec.cameras:
        if cam.name == "front_camera":
            cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
            cam.alt.xyaxes = [0, -1, 0, 0, 0, 1]

    # --- sunlight and a softer fill light
    wb = spec.worldbody
    sun = wb.add_light(name="sun", pos=(2, -1, 6), dir=(-0.2, 0.15, -1))
    sun.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    sun.diffuse = (0.8, 0.78, 0.72)
    sun.ambient = (0.35, 0.35, 0.38)
    sun.castshadow = True
    spec.visual.headlight.ambient = (0.25, 0.25, 0.25)
    spec.visual.headlight.diffuse = (0.3, 0.3, 0.3)
    # room for the 1280x720 third-person video next to the 640x480 dog camera
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720

    # --- weeds: a cluster of flat green leaves around each center
    for i, (x, y) in enumerate(WEEDS):
        body = wb.add_body(name=f"weed_{i}", pos=(x, y, 0.0))
        n_leaves = int(rng.integers(5, 9))
        base = np.array([0.18, 0.52, 0.14]) + rng.normal(0, 0.04, 3)
        for k in range(n_leaves):
            ang = 2 * np.pi * k / n_leaves + rng.normal(0, 0.2)
            r = rng.uniform(0.03, 0.06)
            g = body.add_geom(name=f"weed_{i}_leaf_{k}", type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                              size=(rng.uniform(0.035, 0.055), rng.uniform(0.018, 0.028), 0.008),
                              pos=(r * np.cos(ang), r * np.sin(ang), 0.012 + 0.004 * k),
                              euler=(0, rng.uniform(-0.3, 0.3), ang))
            _visual(g, (*np.clip(base + rng.normal(0, 0.03, 3), 0, 1), 1))
        g = body.add_geom(name=f"weed_{i}_stem", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                          size=(0.015, 0, 0), pos=(0, 0, 0.02))
        _visual(g, (0.30, 0.45, 0.15, 1))

    def obj(name, gtype, size, pos, rgba):
        b = wb.add_body(name=name, pos=pos)
        g = b.add_geom(name=name, type=gtype, size=size)
        _visual(g, rgba)

    B, C, S = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE
    obj("red_box", B, (0.12, 0.12, 0.12), (3.0, 0.75, 0.12), (0.85, 0.12, 0.10, 1))
    obj("gray_box", B, (0.12, 0.12, 0.12), (3.8, -0.65, 0.12), (0.52, 0.52, 0.53, 1))
    obj("soil_box", B, (0.13, 0.13, 0.10), (5.0, 0.72, 0.10), (*[c * 1.03 for c in SOIL_RGB], 1))
    obj("blue_cyl", C, (0.09, 0.16, 0), (4.4, 0.85, 0.16), (0.12, 0.30, 0.85, 1))
    obj("yellow_ball", S, (0.11, 0, 0), (6.2, -0.75, 0.11), (0.95, 0.80, 0.10, 1))

    spec.compile()
    xml = spec.to_xml()
    with open(out, "w") as f:
        f.write(xml)
    return out


def label_lookup(model):
    """geom id -> label id, for turning a segmentation render into a mask."""
    lut = np.zeros(model.ngeom, np.uint8)
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gid]) or ""
        if name == "floor":
            lut[gid] = LABELS["soil"]
        elif name.startswith("weed_"):
            lut[gid] = LABELS["weed"]
        elif name in LABELS:
            lut[gid] = LABELS[name]
        elif name == "heightfield":
            lut[gid] = LABELS["soil"]
        elif body and body != "world":
            lut[gid] = LABELS["robot"]
    return lut


if __name__ == "__main__":
    path = build_scene()
    m = mujoco.MjModel.from_xml_path(path)
    print(f"wrote {path}: {m.ngeom} geoms, {m.nbody} bodies, {m.nlight} lights")
