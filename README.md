# ENGR 6010 First Test — Harry He

ENGR 6010 AI in Robotics, Dr. Hongbo Zhang.

**The report is [`ENGR6010_FirstTest_HarryHe.pdf`](ENGR6010_FirstTest_HarryHe.pdf)** (Word version next to it).

- Pages 1–3: **Question 1**, a review paper on color spaces, Gaussian pyramids, optical flow, GrabCut and supervised learning.
- Pages 4–10: **Questions 2–4**, a technical note on the Unitree Go2 environment, the demo apps, and data and simulation.

## What is in this repository

| Folder | Question | What it holds |
|---|---|---|
| `q2_sdk/` | 2 | VM setup script, the three SDK examples changed for the simulator, logs, screenshots |
| `q3_go2py/` | 3 | Go2Py walking demo and its results |
| `q4_sim/` | 4 | synthetic data recorder, true optical flow, the five-method pipeline, the unitree_rl_gym Go2 run |
| `lab/` | 2, 4 | what to run on the real robot: checklist, read-only data collector, network script |
| `tools/` | – | fetches the third-party projects; builds the report |

## Setup (Mac, Apple Silicon)

```bash
conda create -n go2 python=3.10 && conda activate go2
pip install -r requirements.txt
bash tools/get_third_party.sh        # Go2Py, unitree_mujoco, unitree_sdk2_python, unitree_rl_gym_go2
```

The Ubuntu 22.04 VM is set up by `q2_sdk/setup_vm.sh` (CycloneDDS built from source, unitree_sdk2py, MuJoCo, unitree_mujoco, desktop).

## How to reproduce

```bash
# Question 2 (inside the Ubuntu VM)
bash q2_sdk/setup_vm.sh
bash q2_sdk/run_examples_vm.sh          # helloworld, stand example, wireless controller
python q2_sdk/timestep_check.py         # (Mac) why the dog slid at a 5 ms time step

# Question 3
python q3_go2py/demo_walk.py            # video + tracking plot
mjpython q3_go2py/demo_walk.py --view   # live window

# Question 4
python q4_sim/record_sim.py             # 170 frames: image, true labels, depth, camera pose
python q4_sim/vision_pipeline.py        # the five methods, graded against the simulator's ground truth
python q4_sim/rl_gym_go2_sim.py         # unitree_rl_gym Go2 policy in MuJoCo, world vs body frame
```

`q4_sim/data/sim_run/` holds the exact frames used in the report, so `vision_pipeline.py` gives the same numbers.
Recording again gives slightly different frames, because Go2Py's controller adds random noise.

## Main results

| | Result |
|---|---|
| Q2 | Ubuntu 22.04 ARM64 VM in UTM; CycloneDDS 0.10.5 + unitree_sdk2py 1.0.1; three examples run on the simulated Go2 (it stands up from 0.117 m to 0.384 m) |
| Q3 | Go2Py's walk-these-ways controller follows forward, turn, sideways and backward commands within 0.01 m/s |
| Q4 color | Lab a separates weed from soil perfectly (AUC 1.000); the G channel only 0.741; hue moves 27.2° on gray vs 1.1° on red |
| Q4 pyramid | fast turn (40–90 px/frame): 0% of corners tracked within 1 px without a pyramid, 30% with 3 extra levels |
| Q4 flow | DIS 2.3 / 12.9 / 2.0 px error (forward / turn / sidestep); Farneback fails on the smooth simulated soil |
| Q4 GrabCut | IoU 0.88–0.98 on boxes and ball, 0.67 on the weed; every error is a shadow |
| Q4 learning | six classifiers F1 0.989–0.999; the lasso keeps 3 of 25 channels (ExG, R, Lab a) at F1 0.986 |
| Q4 rl_gym | with world-frame velocity (as in deploy_go2.py) the dog falls when it turns; with body-frame velocity it does not |

## Problems found and fixed

1. **Go2Py's `front_camera` looks sideways.** Its `euler="1.5708 0 0"` points it along the robot's +y axis. `q4_sim/scene.py` points it forward.
2. **unitree_mujoco's default 5 ms step makes the standing dog slide** about 1 m/s. 2 ms (the real robot's control period) keeps it still.
3. **`deploy_go2.py` gives the policy world-frame velocity** while it was trained on body-frame velocity, so the dog falls when turning.
4. **OpenCV Farneback returns about zero motion on weak texture**, because each update divides by `det + 1e-3` (optflowgf.cpp).

Also on the way: UTM did not insert the cloud-init CD, and `hdiutil -hfs` made a seed image Linux could not read; Ubuntu 22.04's setuptools 59 breaks `pip install -e .` for a normal user; Go2Py's checkpoint holds CUDA tensors (`q3_go2py/go2py_compat.py` maps them to the CPU).

## Still to do with the real robot

See [`lab/LAB_CHECKLIST.md`](lab/LAB_CHECKLIST.md): bridge the VM to the robot, run the read-only camera and state collector (`lab/collect_real.py`), then the sport-mode example with the instructor.

## Credits

Built on [Go2Py](https://github.com/machines-in-motion/Go2Py), [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco),
[unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python), [Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds)
and [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) pull request #62 ([UESTC-REVERIE/unitree_rl_gym_go2](https://github.com/UESTC-REVERIE/unitree_rl_gym_go2)).
`q4_sim/channels.py` is copied from my Reading 3 assignment.
