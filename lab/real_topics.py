"""List every topic this Go2 is actually publishing. READ-ONLY.

    .venv_robot/bin/python lab/real_topics.py en7 20
    python3 real_topics.py enp0s2 30

real_probe.py checks nine topic names that I guessed from the SDK examples
and from what other people's Go2s publish. If this robot's firmware names
the point cloud something else, that probe reports "SILENT" and we would
wrongly conclude there is no LiDAR.

This does not guess. It asks DDS itself. Every DDS participant publishes a
built-in topic describing its own writers, so subscribing to
DCPSPublication gives the real list: topic name, type name, and which
participant is publishing it. Whatever the dog is sending, it is in here.

Run this once, early. Together with real_probe.py it settles what this
robot can give us, and everything written afterwards is written against
the answer rather than against a guess.

Writes lab/data/topics_<time>.json.
"""
import json
import os
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "data")
DOMAIN_ID = 0

# Names worth calling out when they appear, so the important ones do not
# get lost in a list of forty.
INTERESTING = [
    ("cloud", "POINT CLOUD - this is the LiDAR"),
    ("lidar", "LiDAR related"),
    ("odom", "odometry - a pose estimate"),
    ("mapping", "on-board mapping"),
    ("pose", "pose"),
    ("imu", "inertial measurement"),
    ("lowstate", "joints, IMU, battery"),
    ("sportmode", "high-level mode and velocity"),
    ("lowcmd", "LOW-LEVEL MOTOR COMMAND - writing here drives the motors"),
    ("video", "camera"),
    ("audio", "microphone or speaker"),
    ("wirelesscontroller", "the handheld remote"),
]


def why(topic):
    t = topic.lower()
    for key, note in INTERESTING:
        if key in t:
            return note
    return ""


def main():
    if len(sys.argv) < 2:
        print("usage: python3 real_topics.py <network_interface> [seconds]")
        sys.exit(1)
    iface = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from cyclonedds.builtin import (BuiltinDataReader,
                                    BuiltinTopicDcpsPublication)
    from cyclonedds.core import Qos, Policy
    from cyclonedds.util import duration

    print(f"listening for DDS announcements on {iface} for {seconds:.0f} s")
    print("READ-ONLY: this never writes to the robot\n")

    ChannelFactoryInitialize(DOMAIN_ID, iface)
    # The SDK keeps the participant it just made on the class itself
    # (ChannelFactory is a singleton). Reuse it, so this scanner is on the
    # same domain and the same interface as every other script here.
    from unitree_sdk2py.core.channel import ChannelFactory
    dp = ChannelFactory._ChannelFactory__participant
    if dp is None:
        sys.exit("DDS did not start: is the interface name right?")

    rd = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)

    found = {}
    t0 = time.time()
    while time.time() - t0 < seconds:
        for s in rd.take(N=200):
            try:
                name = s.topic_name
                typ = s.type_name
            except Exception:
                continue
            if name.startswith("DCPS"):
                continue                      # discovery's own chatter
            e = found.setdefault(name, {"type": typ, "writers": 0})
            e["writers"] += 1
        el = time.time() - t0
        print(f"\r  {el:4.0f}s   {len(found)} topics seen   ", end="", flush=True)
        time.sleep(0.3)
    print("\n")

    if not found:
        print("  NOTHING. Either the robot is not reachable on this interface,")
        print("  or nothing is publishing. Check 1-1 first.\n")
    else:
        print(f"{'topic':<46}{'writers':>8}   type / note")
        print("-" * 104)
        for name in sorted(found):
            e = found[name]
            note = why(name)
            short = e["type"].split("::")[-1]
            print(f"{name:<46}{e['writers']:>8}   {short}"
                  + (f"   <<< {note}" if note else ""))

        print("\n" + "=" * 104)
        clouds = [n for n in found if "cloud" in n.lower()]
        odoms = [n for n in found if "odom" in n.lower()
                 or "mapping" in n.lower() or "pose" in n.lower()]
        if clouds:
            print(f"  POINT CLOUD TOPICS: {', '.join(clouds)}")
            print("  -> put the right one at the top of CLOUD_TOPICS in "
                  "real_survey.py")
        else:
            print("  No topic with 'cloud' in the name.")
            print("  -> the LiDAR may be switched off. The SDK example")
            print("     go2_utlidar_switch.py publishes 'ON' to "
                  "rt/utlidar/switch.")
        if odoms:
            print(f"  POSE / ODOMETRY TOPICS: {', '.join(odoms)}")
            print("  -> if one of these carries a pose, we do not have to "
                  "write odometry ourselves.")
        else:
            print("  No pose or odometry topic: position has to come from "
                  "rt/sportmodestate, which drifts.")

    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, f"topics_{datetime.now():%Y%m%d_%H%M%S}.json")
    with open(p, "w") as f:
        json.dump({"when": datetime.now().isoformat(timespec="seconds"),
                   "interface": iface, "seconds": seconds,
                   "topics": found}, f, indent=1)
    print(f"\nwrote {p}")


if __name__ == "__main__":
    main()
