#!/usr/bin/env bash
# Run the three Unitree SDK examples inside the Ubuntu VM against the simulated Go2.
#   bash run_examples_vm.sh            (logs and screenshots go to ~/q2_logs)
#
# Example 1  helloworld publisher + subscriber   (DDS only, no robot)
# Example 2  go2 low-level stand example          (the dog stands up in unitree_mujoco)
# Example 3  wireless controller reader           (reads rt/lowstate from the dog)
#
# The simulator window is drawn on a virtual X display (Xvfb) so screenshots can be
# taken over SSH. On the VM desktop the same commands open a normal window.
set -u
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
LOG="$HOME/q2_logs"; mkdir -p "$LOG"
EXD="$HOME/q2/sim_examples"
shot() { import -display :99 -window root "$LOG/$1"; echo "  screenshot $1"; }

echo "== Example 1: helloworld =="
cd "$HOME/unitree_sdk2_python/example/helloworld"
(timeout 30 python3 -u subscriber.py > "$LOG/ex1_subscriber.log" 2>&1 &)
sleep 2
timeout 12 python3 -u publisher.py > "$LOG/ex1_publisher.log" 2>&1
sleep 1
echo "  published $(grep -c 'Publish success' "$LOG/ex1_publisher.log"), received $(grep -c 'Subscribe success' "$LOG/ex1_subscriber.log")"

echo "== starting the simulator (unitree_mujoco, Go2) =="
cd "$HOME/unitree_mujoco/simulate_python"
sed -i 's/^USE_JOYSTICK = 1/USE_JOYSTICK = 0/' config.py      # no gamepad in a VM
# The default 5 ms physics step makes the standing dog slide backwards about 1 m/s
# (the stiff PD gains of the stand example, kp=60 kd=5, are too much for a 5 ms
# contact step). 2 ms is the real robot's control period and keeps it still.
sed -i 's/^SIMULATE_DT = 0.005/SIMULATE_DT = 0.002/' config.py
Xvfb :99 -screen 0 1280x800x24 > /dev/null 2>&1 &
XVFB=$!
export DISPLAY=:99
sleep 2
python3 -u unitree_mujoco.py > "$LOG/simulator.log" 2>&1 &
SIM=$!
sleep 6
shot sim_0_lying.png

echo "== recording rt/lowstate during examples 2 and 3 =="
cd "$EXD"
timeout 30 python3 -u record_lowstate.py 20 "$LOG/lowstate.csv" > "$LOG/recorder.log" 2>&1 &
REC=$!
sleep 1

echo "== Example 3: wireless controller (reads the remote-control bytes in rt/lowstate) =="
timeout 4 python3 -u wireless_controller_go2_sim.py > "$LOG/ex3_wireless_controller.log" 2>&1
echo "  printed $(grep -c 'debug unitreeRemoteController' "$LOG/ex3_wireless_controller.log") controller states"

echo "== Example 2: stand up =="
timeout 20 python3 -u go2_stand_sim.py > "$LOG/ex2_stand.log" 2>&1 &
STAND=$!
sleep 2.0; shot sim_1_rising.png
sleep 1.3; shot sim_2_standing.png
sleep 2.0; shot sim_3_lowering.png
wait $STAND
sleep 0.5; shot sim_4_final.png
echo "  $(tail -1 "$LOG/ex2_stand.log")"
wait $REC
cat "$LOG/recorder.log"

kill $SIM $XVFB 2>/dev/null
echo "== done =="
ls -la "$LOG"
