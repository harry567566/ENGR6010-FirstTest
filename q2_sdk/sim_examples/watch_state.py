"""Print the simulated Go2's state twice a second, in words, while another example runs.

    python3 watch_state.py 10        watch for 10 seconds
"""
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
state = {"h": None, "q": None, "n": 0}


def on_high(msg: SportModeState_):
    state["h"] = float(msg.position[2])


def on_low(msg: LowState_):
    state["q"] = [msg.motor_state[i].q for i in range(3)]   # front-right hip, thigh, calf
    state["n"] += 1


ChannelFactoryInitialize(1, "lo")
ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(on_high, 10)
ChannelSubscriber("rt/lowstate", LowState_).Init(on_low, 10)

t0 = time.time()
while time.time() - t0 < secs:
    time.sleep(0.5)
    if state["q"] is None:
        print("[dog] no message yet - is the simulator running?", flush=True)
        continue
    h = state["h"]
    hip, thigh, calf = state["q"]
    pose = "lying" if h is not None and h < 0.15 else ("standing" if h is not None and h > 0.33 else "moving")
    print(f"[dog] {time.time() - t0:4.1f}s  body height {h:.3f} m ({pose})   "
          f"front-right leg: thigh {thigh:+.2f}  calf {calf:+.2f} rad   {state['n']} messages", flush=True)
