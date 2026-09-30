#!/usr/bin/env bash
# Make the Python environment that lets this Mac talk to the real Go2 directly (no VM).
#   bash lab/setup_mac_sdk.sh
# cyclonedds 0.10.2 has a ready-made macOS wheel; the SDK itself is plain Python.
# Needs third_party/unitree_sdk2_python (tools/get_third_party.sh) and the go2 conda env (Python 3.10).
set -e
cd "$(dirname "$0")/.."
~/miniconda3/envs/go2/bin/python -m venv .venv_robot
.venv_robot/bin/pip install -q cyclonedds==0.10.2 numpy
.venv_robot/bin/pip install -q --no-deps ./third_party/unitree_sdk2_python
.venv_robot/bin/python -c "from unitree_sdk2py.go2.sport.sport_client import SportClient; print('Mac SDK ready')"
