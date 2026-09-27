"""Make Go2Py run on a Mac without an NVIDIA GPU.

Go2Py's walk-these-ways checkpoint stores its config in parameters.pkl, and that
pickle contains torch tensors that were saved on 'cuda:0'. Unpickling them on a
machine without CUDA fails. Instead of editing Go2Py's code, this module
redirects those tensors to the CPU before Go2Py is imported.

Import this module first:
    import go2py_compat          # noqa: F401
    from Go2Py.sim.mujoco import Go2Sim
"""
import io
import os
import sys

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
GO2PY = os.path.join(HERE, "..", "third_party", "Go2Py")
if GO2PY not in sys.path:
    sys.path.insert(0, os.path.abspath(GO2PY))

_orig = torch.storage._load_from_bytes


def _load_on_cpu(b):
    return torch.load(io.BytesIO(b), map_location="cpu", weights_only=False)


torch.storage._load_from_bytes = _load_on_cpu
