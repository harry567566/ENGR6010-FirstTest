#!/usr/bin/env bash
# Download the four projects this work builds on, at the exact commits I used.
# They are not copied into this repository; run this once from the repo root:
#   bash tools/get_third_party.sh
set -e
mkdir -p third_party && cd third_party
get() {  # name url commit
  [ -d "$1" ] && { echo "$1 already there"; return; }
  git init -q "$1" && git -C "$1" remote add origin "$2"
  git -C "$1" fetch -q --depth 1 origin "$3" && git -C "$1" checkout -q FETCH_HEAD
  echo "$1 @ ${3:0:7}"
}
get Go2Py               https://github.com/machines-in-motion/Go2Py.git            eff240b82b0ff6dae6ba5ee934a381f38fe64c0d
get unitree_mujoco      https://github.com/unitreerobotics/unitree_mujoco.git      1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
get unitree_sdk2_python https://github.com/unitreerobotics/unitree_sdk2_python.git 814556d15970dd2ecf1c9984e845ca02ab07e206
get unitree_rl_gym_go2  https://github.com/UESTC-REVERIE/unitree_rl_gym_go2.git    0f5344f69780970f7c284670b7f59cb2c3d325f5
