"""Decoder for the dog's own SLAM cloud, rt/uslam/frontend/cloud_world_ds.

This is NOT the decoder the Wi-Fi library ships. There are two:

    libvoxel.wasm   rt/utlidar/voxel_map_compressed   a mesh, 11 arguments,
                    needs `origin` and `resolution`, ships with the library
    libslam.wasm    rt/uslam/frontend/cloud_world_ds  points, 15 arguments,
                    needs an axis-aligned box, does NOT ship with it

They are different binaries with incompatible signatures, so the sweep
decoder cannot read the SLAM map and vice versa. libslam.wasm is vendored
in lab/vendor/ from legion1581/unitree_ui, which recovered it from the
Unitree app.

The input is six bytes a point - three uint16, quantised inside the box the
message carries. The wasm expands them and, more usefully, deduplicates
against a 64 MB dictionary it keeps between calls, so what comes back is
the map so far rather than this frame alone. Call reset() to start a new
map; otherwise frames accumulate exactly as the phone app's map does.
"""
import ctypes
import os

import numpy as np
from wasmtime import Config, Engine, Store, Module, Instance, Func, FuncType
from wasmtime import ValType

MAX_POINTS = 1_000_000
DICT_BYTES = 67_108_864


class SlamDecoder:
    def __init__(self, wasm_path=None):
        if wasm_path is None:
            wasm_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "vendor", "libslam.wasm")
        cfg = Config()
        cfg.wasm_multi_value = True
        self.store = Store(Engine(cfg))
        module = Module.from_file(self.store.engine, wasm_path)

        # one import: emscripten_resize_heap. Returning 0 means "cannot
        # grow", which is correct here - every buffer is allocated up front
        # and the heap never needs to move.
        grow = Func(self.store, FuncType([ValType.i32()], [ValType.i32()]),
                    lambda _n: 0)
        self.inst = Instance(self.store, module, [grow])
        ex = self.inst.exports(self.store)

        self.mem = ex["b"]
        ctors, self._reset_fn = ex["c"], ex["d"]
        self.generate, self.malloc = ex["e"], ex["f"]
        ctors(self.store)

        base = int.from_bytes(self.mem.data_ptr(self.store), "little")
        size = self.mem.data_len(self.store)
        self.HEAPU8 = (ctypes.c_uint8 * size).from_address(base)
        self._base = base

        self.p_in = self.malloc(self.store, 200_000)
        self.p_direct = self.malloc(self.store, 300_000)
        self.p_outcount = self.malloc(self.store, 4)
        self.p_added = self.malloc(self.store, 4)
        self.p_dict = self.malloc(self.store, DICT_BYTES)
        self.p_idx = self.malloc(self.store, MAX_POINTS * 4)
        self.p_out = self.malloc(self.store, MAX_POINTS * 12)

    # ------------------------------------------------------------------
    def reset(self):
        """Forget the accumulated map and start again."""
        ctypes.memset(self._base + self.p_dict, 0, DICT_BYTES)
        ctypes.memset(self._base + self.p_idx, 0, MAX_POINTS * 4)
        self._reset_fn(self.store)

    def decode(self, data, meta):
        """One message -> (whole_map, this_frame), both (N, 3) float32.

        `meta` is the message's own `data` dict, which carries the box the
        points were quantised into. Missing bounds are not guessable, so a
        message without them is dropped rather than decoded into nonsense."""
        try:
            xmin, xmax = float(meta["xmin"]), float(meta["xmax"])
            ymin, ymax = float(meta["ymin"]), float(meta["ymax"])
            zmin, zmax = float(meta["zmin"]), float(meta["zmax"])
        except (KeyError, TypeError, ValueError):
            return None, None

        raw = bytes(data)
        n = len(raw) // 6
        if n <= 0:
            return None, None
        ctypes.memmove(self._base + self.p_in, raw, len(raw))

        self.generate(self.store, xmin, xmax, ymin, ymax, zmin, zmax, n,
                      self.p_in, MAX_POINTS, self.p_dict, self.p_idx,
                      self.p_direct, self.p_out, self.p_added, self.p_outcount)

        total = ctypes.c_int32.from_address(
            self._base + self.p_outcount).value
        total = max(0, min(total, MAX_POINTS))
        whole = np.ctypeslib.as_array(
            (ctypes.c_float * (total * 3)).from_address(
                self._base + self.p_out)).reshape(-1, 3).copy()
        frame = np.ctypeslib.as_array(
            (ctypes.c_float * (n * 3)).from_address(
                self._base + self.p_direct)).reshape(-1, 3).copy()
        return whole, frame


if __name__ == "__main__":                 # a smoke test without a robot
    d = SlamDecoder()
    meta = {"xmin": -5.0, "xmax": 5.0, "ymin": -5.0, "ymax": 5.0,
            "zmin": -1.0, "zmax": 3.0}
    rng = np.random.default_rng(0)
    q = rng.integers(0, 65535, (2000, 3), dtype=np.uint16)
    whole, frame = d.decode(q.tobytes(), meta)
    print("whole", None if whole is None else whole.shape,
          "frame", None if frame is None else frame.shape)
    if frame is not None and len(frame):
        print("frame range x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f"
              % (frame[:, 0].min(), frame[:, 0].max(),
                 frame[:, 1].min(), frame[:, 1].max(),
                 frame[:, 2].min(), frame[:, 2].max()))
