#!/usr/bin/env bash
# Make the Python environment that talks to the Go2 over Wi-Fi (WebRTC, like the phone app).
#   bash lab/setup_wifi_sdk.sh
# Library: legion1581/unitree_webrtc_connect 2.2.0 (works on Go2 AIR / PRO / EDU).
# It is installed without its pyaudio dependency, which needs a C build and is only used by an audio example.
# Pillow replaces opencv-python for saving photos (opencv and av ship two copies of the same video library).
set -e
cd "$(dirname "$0")/.."
~/miniconda3/envs/go2/bin/python -m venv .venv_wifi
.venv_wifi/bin/pip install -q "aiortc>=1.9.0" pycryptodome requests curl_cffi wasmtime lz4 numpy packaging \
    sounddevice pydub pillow
.venv_wifi/bin/pip install -q --no-deps unitree_webrtc_connect==2.2.0
.venv_wifi/bin/python -c "from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection; print('Wi-Fi SDK ready')"
