# Lab checklist: connecting the real Unitree Go2

Everything below was prepared and tested against the simulator. Only the steps
marked **real robot** still need the dog in the lab.

## Before the lab
- The Ubuntu 22.04 VM (UTM, "Go2-Ubuntu22") has CycloneDDS 0.10.5, unitree_sdk2py 1.0.1,
  MuJoCo and unitree_mujoco installed. User `harry` (the password is in my local cloud-init file, which is not in this repository).
- The scripts are already in the VM under `~/q2/lab/`.
- Bring a USB-C to Ethernet adapter and an Ethernet cable.

## 1. Network (about 5 minutes)
1. Plug the adapter into the Mac and the cable into the Go2's Ethernet port.
2. In UTM: stop the VM, open **Edit > Devices > New... > Network**,
   set **Network Mode = Bridged (Advanced)** and **Bridged Interface = the USB LAN adapter**. Save and start the VM.
   (Keep the first network card as it is, so the VM still has internet and SSH.)
3. In the VM terminal: `ip -br link` shows the new card, usually `enp0s2`.
4. `bash ~/q2/lab/robot_net_up.sh enp0s2` sets 192.168.123.222/24 and pings the robot (192.168.123.161).

## 2. Read-only tests (safe, the robot does not move)
Replace `enp0s2` with the interface from step 1.
```
cd ~/unitree_sdk2_python/example
python3 go2/front_camera/capture_image.py enp0s2        # saves img.jpg from the front camera
python3 ~/q2/lab/collect_real.py enp0s2 --secs 30 --fps 5   # camera frames + state, for Question 4
```
For `collect_real.py`, someone can walk the dog with the remote while it records.
It saves `data/real_run_<time>/` with the frames, `frames.csv` and `state.csv`.

## 3. Examples that move the robot (**real robot**, only with the instructor)
Clear 2 m around the dog, keep the remote in hand, and know how to make it lie down.
```
python3 go2/high_level/go2_sport_client.py enp0s2
```
It asks for an option number. Safe ones: `2` stand_down, `1` stand_up, `0` damp,
`3` move forward (short), `6` stop_move. **Do not** try flips, hand stand or jumps.

Do not run `go2/low_level/go2_stand_example.py` on the real robot unless the
instructor says so: it switches off the robot's own controller and drives the
motors directly.

## 4. After the lab
Copy the data to the Mac and run the Question 4 pipeline on it:
```
scp -r harry@<vm-ip>:~/q2/lab/data/real_run_* ~/Desktop/2026fall/ENGR6010/firsttest/q4_sim/data/
```
