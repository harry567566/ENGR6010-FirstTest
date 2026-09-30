"""Watch the state of a REAL Unitree Go2. READ-ONLY: it never sends a command.

Run with the robot connected by Ethernet, on the Mac or inside the Ubuntu VM:
    .venv_robot/bin/python lab/real_state.py en7     # Mac, 20 seconds
    python3 real_state.py enp0s2 60                  # VM, 60 seconds (Ctrl+C stops it early)

The first argument is the network card that is on 192.168.123.x
(see: ifconfig on the Mac, ip -br addr in the VM). Twice per second it prints:
    battery voltage / current / charge, IMU roll-pitch-yaw, body height,
    sport mode number, front-right leg joint angles, 4 foot forces,
    and how many messages per second arrive on each topic.

Data sources (unitree_sdk2py, unitree_go IDL):
    rt/lowstate        LowState_       power_v, power_a, bms_state.soc,
                                       imu_state.rpy, motor_state[i].q, foot_force
    rt/sportmodestate  SportModeState_ mode, body_height, position, velocity
(The robot also publishes lower-rate copies: rt/lf/lowstate, rt/lf/sportmodestate.)
"""
import math
import sys
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

# SportModeState_.mode numbers, from Unitree's unitree_ros2 README.
MODE_NAMES = {0: "idle/stand", 1: "balanceStand", 2: "pose", 3: "locomotion",
              5: "lieDown", 6: "jointLock", 7: "damping", 8: "recoveryStand",
              10: "sit", 11: "frontFlip", 12: "frontJump", 13: "frontPounce"}

DOMAIN_ID = 0  # the real robot always uses DDS domain 0

# Latest message and message counters, filled by the DDS callbacks.
lock = threading.Lock()
latest = {"low": None, "sport": None}
count = {"low": 0, "sport": 0}


def on_lowstate(msg: LowState_):
    # Called for every rt/lowstate message (a few hundred per second on the
    # real robot), so it only stores the message and counts it.
    with lock:
        latest["low"] = msg
        count["low"] += 1


def on_sportstate(msg: SportModeState_):
    with lock:
        latest["sport"] = msg
        count["sport"] += 1


def main():
    if len(sys.argv) < 2:
        print("usage: python3 real_state.py <network_interface> [seconds]")
        print("example: python3 real_state.py enp0s2 30")
        sys.exit(1)
    iface = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    try:
        ChannelFactoryInitialize(DOMAIN_ID, iface)
    except Exception:
        # Cyclone DDS prints "<iface>: does not match an available interface."
        print(f"cannot use network interface '{iface}'. List the cards with: ifconfig (Mac) or ip -br addr (VM)")
        sys.exit(2)

    low_sub = ChannelSubscriber("rt/lowstate", LowState_)
    low_sub.Init(on_lowstate, 10)
    sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sport_sub.Init(on_sportstate, 10)

    print(f"listening on {iface} (domain {DOMAIN_ID}) for {seconds:.0f} s. "
          "Read-only: the robot is not commanded.")
    t_start = time.time()
    t_prev = t_start
    prev = dict(count)
    try:
        while time.time() - t_start < seconds:
            time.sleep(0.5)
            now = time.time()
            with lock:
                low, sport = latest["low"], latest["sport"]
                n = dict(count)
            dt = now - t_prev
            rate_low = (n["low"] - prev["low"]) / dt
            rate_sport = (n["sport"] - prev["sport"]) / dt
            t_prev, prev = now, n

            if low is None:
                print(f"[{now - t_start:5.1f}s] no rt/lowstate yet. Check: cable, "
                      f"'ping 192.168.123.161', interface name '{iface}'.")
                continue

            roll, pitch, yaw = (math.degrees(a) for a in low.imu_state.rpy)
            # Joint order: FR hip, thigh, calf = motor_state[0], [1], [2]  (radians)
            fr = [low.motor_state[i].q for i in range(3)]
            feet = list(low.foot_force)  # order FR, FL, RR, RL (raw sensor units)

            line = (f"[{now - t_start:5.1f}s] "
                    f"batt {low.power_v:5.2f} V {low.power_a:5.2f} A {low.bms_state.soc:3d}% | "
                    f"rpy {roll:6.1f} {pitch:6.1f} {yaw:7.1f} deg | "
                    f"FR q {fr[0]:+.2f} {fr[1]:+.2f} {fr[2]:+.2f} rad | "
                    f"feet {feet[0]:4d} {feet[1]:4d} {feet[2]:4d} {feet[3]:4d}")
            if sport is not None:
                mode = MODE_NAMES.get(sport.mode, "?")
                line += (f" | height {sport.body_height:.3f} m"
                         f" | mode {sport.mode} ({mode})"
                         f" | vx {sport.velocity[0]:+.2f} m/s")
            line += f" | rate low {rate_low:4.0f}/s sport {rate_sport:4.0f}/s"
            print(line)
    except KeyboardInterrupt:
        print("stopped by Ctrl+C")

    low_sub.Close()
    sport_sub.Close()
    print(f"done. received {count['low']} lowstate and {count['sport']} sportmodestate messages.")


if __name__ == "__main__":
    main()
