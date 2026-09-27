#!/usr/bin/env bash
# Set up the Unitree Go2 development environment on Ubuntu 22.04 (ARM64, UTM VM).
# Run inside the VM:   bash setup_vm.sh 2>&1 | tee setup_vm.log
#
# 1. build CycloneDDS 0.10.x from source (PyPI has no Linux ARM64 wheel for 0.10.2)
# 2. install unitree_sdk2_python against it
# 3. install unitree_mujoco (the official simulator that speaks the same DDS messages)
# 4. install a desktop so the simulator window can be seen in UTM
set -euo pipefail
cd "$HOME"
step() { echo; echo "=================== $* ==================="; date; }

step "0. system packages"
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    git cmake build-essential python3-pip python3-venv python3-dev \
    libglfw3 libgl1-mesa-dri mesa-utils xvfb x11-apps imagemagick >/dev/null
python3 --version

step "1. CycloneDDS 0.10.x (C library)"
if [ ! -d cyclonedds ]; then
  git clone -q -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds
fi
mkdir -p cyclonedds/build cyclonedds/install
cd cyclonedds/build
cmake .. -DCMAKE_INSTALL_PREFIX="$HOME/cyclonedds/install" -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF >/dev/null
cmake --build . --target install -j"$(nproc)" >/dev/null
cd "$HOME"
ls cyclonedds/install/lib | grep -E "libddsc" | head -3
grep -q CYCLONEDDS_HOME ~/.bashrc || echo 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install' >> ~/.bashrc
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"

step "2. unitree_sdk2_python"
if [ ! -d unitree_sdk2_python ]; then
  git clone -q https://github.com/unitreerobotics/unitree_sdk2_python.git
fi
cd unitree_sdk2_python
# Ubuntu 22.04 ships setuptools 59. With it, "pip3 install -e ." as a normal user
# falls back to "setup.py develop" and tries to write into /usr/local, which fails
# with "can't create or remove files in install directory". A newer pip and
# setuptools do a proper per-user editable install instead.
python3 -m pip install -q --user --upgrade pip setuptools wheel
python3 -m pip install -q --user -e .
cd "$HOME"
python3 -c "import importlib.metadata as md, unitree_sdk2py; print('cyclonedds', md.version('cyclonedds'), '| unitree_sdk2py', md.version('unitree_sdk2py'), 'OK')"

step "3. unitree_mujoco (python simulator)"
pip3 install -q mujoco pygame 2>&1 | tail -1
if [ ! -d unitree_mujoco ]; then
  git clone -q https://github.com/unitreerobotics/unitree_mujoco.git
fi
python3 -c "import mujoco; print('mujoco', mujoco.__version__)"

step "4. desktop (for the simulator window in UTM)"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ubuntu-desktop-minimal >/dev/null
# X11 instead of Wayland: MuJoCo's GLFW viewer and screenshots are simpler on X11
sudo sed -i 's/^#\?WaylandEnable=.*/WaylandEnable=false/' /etc/gdm3/custom.conf
sudo systemctl set-default graphical.target

step "done"
df -h / | tail -1
