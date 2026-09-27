"""Record the simulated Go2's state from DDS while another example runs.

Subscribes to the same two topics the real robot publishes:
    rt/lowstate          12 motor angles, foot forces, IMU
    rt/sportmodestate    body position (the simulator fills this from MuJoCo)

    python3 record_lowstate.py 20 lowstate.csv      record 20 s
"""
import csv
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
out = sys.argv[2] if len(sys.argv) > 2 else "lowstate.csv"
rows, t0, height = [], [None], [float("nan")] * 3


def on_high(msg: SportModeState_):
    height[:] = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]


def on_low(msg: LowState_):
    t = time.time()
    if t0[0] is None:
        t0[0] = t
    q = [msg.motor_state[i].q for i in range(12)]
    rows.append([round(t - t0[0], 4)] + q + list(msg.foot_force)[:4]
                + list(msg.imu_state.rpy) + list(height))


ChannelFactoryInitialize(1, "lo")
hi = ChannelSubscriber("rt/sportmodestate", SportModeState_)
hi.Init(on_high, 10)
lo = ChannelSubscriber("rt/lowstate", LowState_)
lo.Init(on_low, 10)
time.sleep(secs)

names = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
         "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t"] + names + ["foot_FR", "foot_FL", "foot_RR", "foot_RL",
                                "roll", "pitch", "yaw", "body_x", "body_y", "body_height"])
    w.writerows(rows)
dur = rows[-1][0] if rows else 0
print(f"recorded {len(rows)} lowstate messages in {dur:.1f} s "
      f"({len(rows) / max(dur, 1e-6):.0f} Hz) -> {out}")
