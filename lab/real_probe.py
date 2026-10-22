"""Find out what this Go2 actually is and what it will give us. READ-ONLY.

    .venv_robot/bin/python lab/real_probe.py en7          # Mac, 25 s
    python3 real_probe.py enp0s2 40                       # VM

This is the first thing to run in the lab, before anything else. It never
sends a command and never moves the robot. It answers, in one pass, the
questions the rest of the final project depends on:

    which model and firmware is this, and does the SDK reach it at all
    is there a LiDAR, is it spinning, and does it publish a point cloud
    how many points per sweep, at what rate, with what packet loss
    does the robot publish its own odometry or a map (some firmware runs
        LIO-SAM on board, which would mean we do not have to write one)
    what else is on the bus that we did not know about

Everything it learns is written to lab/data/probe_<timestamp>.json, so the
answers come home with us even if the session is short.

Why it matters: the robot stays in the lab, so every script after this one
has to be written at home against whatever this file reports. Guessing the
topic names and finding out on the day wastes the session.
"""
import json
import os
import sys
import threading
import time
from datetime import datetime

DOMAIN_ID = 0
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "data")

# Topics worth asking about. The first group the SDK documents; the second
# group is what Go2 firmware has been seen to publish for the L1 LiDAR and
# its on-board mapping, and is exactly what we are here to confirm or rule
# out. An entry that never arrives is a result too.
WANTED = [
    # (topic, IDL module, class name, why we care)
    ("rt/lowstate", "unitree_go", "LowState_",
     "joints, IMU, battery, foot forces"),
    ("rt/sportmodestate", "unitree_go", "SportModeState_",
     "mode, body height, position, velocity"),
    ("rt/utlidar/cloud", "sensor_msgs", "PointCloud2_",
     "THE POINT CLOUD - the thing we came for"),
    ("rt/utlidar/cloud_deskewed", "sensor_msgs", "PointCloud2_",
     "motion-compensated cloud, if the firmware makes one"),
    ("rt/utlidar/lidar_state", "unitree_go", "LidarState_",
     "rotation speed, packet loss, cloud size"),
    ("rt/utlidar/robot_pose", "geometry_msgs", "PoseStamped_",
     "THE DOG'S OWN LIDAR ODOMETRY - if this is live we need no ICP"),
    ("rt/utlidar/robot_odom", "nav_msgs", "Odometry_",
     "same pose as an Odometry message, with velocity"),
    ("rt/utlidar/height_map_array", "unitree_go", "HeightMap_",
     "the dog's own traversability height map"),
    ("rt/uslam/localization/odom", "nav_msgs", "Odometry_",
     "ON-BOARD SLAM pose - if this answers, the dog is already mapping"),
    ("rt/uslam/cloud_map", "sensor_msgs", "PointCloud2_",
     "ON-BOARD SLAM map - the whole project, already done by the dog"),
    ("rt/uslam/frontend/cloud_world_ds", "sensor_msgs", "PointCloud2_",
     "on-board SLAM downsampled world cloud"),
    ("rt/lio_sam_ros2/mapping/odometry", "nav_msgs", "Odometry_",
     "on-board LIO-SAM odometry, on some firmware"),
    ("rt/utlidar/cloud_base", "sensor_msgs", "PointCloud2_",
     "body-frame copy of the cloud, on some firmware"),
    ("rt/secondary_imu", "unitree_go", "IMUState_",
     "the second IMU, if fitted"),
]


def _load(mod, cls):
    """Import one IDL class, or report why it is not available."""
    paths = {
        "unitree_go": "unitree_sdk2py.idl.unitree_go.msg.dds_",
        "sensor_msgs": "unitree_sdk2py.idl.sensor_msgs.msg.dds_",
        "geometry_msgs": "unitree_sdk2py.idl.geometry_msgs.msg.dds_",
        "nav_msgs": "unitree_sdk2py.idl.nav_msgs.msg.dds_",
    }
    try:
        m = __import__(paths[mod], fromlist=[cls])
        return getattr(m, cls), None
    except Exception as e:
        return None, f"{type(e).__name__}: {e}"


class Watch:
    """Counts messages on one topic and keeps the first and last one."""

    def __init__(self, topic, why):
        self.topic, self.why = topic, why
        self.n = 0
        self.first = None
        self.last = None
        self.t0 = None
        self.t1 = None
        self.lock = threading.Lock()

    def on(self, msg):
        with self.lock:
            now = time.time()
            if self.first is None:
                self.first, self.t0 = msg, now
            self.last, self.t1 = msg, now
            self.n += 1

    def hz(self):
        if self.n < 2 or self.t1 is None or self.t1 <= self.t0:
            return 0.0
        return (self.n - 1) / (self.t1 - self.t0)


def describe_cloud(msg):
    """Pull the useful numbers out of a PointCloud2 without decoding it."""
    try:
        fields = [f.name for f in msg.fields]
    except Exception:
        fields = []
    n = 0
    try:
        n = int(msg.width) * int(msg.height)
    except Exception:
        pass
    return {
        "points_per_message": n,
        "width": int(getattr(msg, "width", 0)),
        "height": int(getattr(msg, "height", 0)),
        "point_step_bytes": int(getattr(msg, "point_step", 0)),
        "row_step_bytes": int(getattr(msg, "row_step", 0)),
        "fields": fields,
        "is_dense": bool(getattr(msg, "is_dense", False)),
        "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        "data_bytes": len(getattr(msg, "data", []) or []),
    }


def describe_lidar_state(msg):
    out = {}
    for k in ("firmware_version", "software_version", "sdk_version",
              "sys_rotation_speed", "com_rotation_speed", "error_state",
              "cloud_frequency", "cloud_packet_loss_rate", "cloud_size",
              "cloud_scan_num", "imu_frequency", "imu_packet_loss_rate"):
        v = getattr(msg, k, None)
        if v is not None:
            out[k] = v if isinstance(v, str) else float(v)
    return out


def describe_lowstate(msg):
    try:
        bms = msg.bms_state
        return {
            "battery_soc_percent": int(bms.soc),
            "power_v": round(float(msg.power_v), 2),
            "power_a": round(float(msg.power_a), 2),
            "imu_rpy_deg": [round(float(x) * 57.2958, 2)
                            for x in msg.imu_state.rpy],
            "n_motors_reporting": sum(
                1 for m in msg.motor_state if abs(float(m.q)) > 1e-9),
            "foot_force": [int(x) for x in msg.foot_force],
        }
    except Exception as e:
        return {"error": f"{type(e).__name__}: {e}"}


def describe_sport(msg):
    try:
        return {
            "mode": int(msg.mode),
            "body_height_m": round(float(msg.body_height), 3),
            "position_m": [round(float(x), 3) for x in msg.position],
            "velocity_mps": [round(float(x), 3) for x in msg.velocity],
            "foot_raise_height_m": round(float(msg.foot_raise_height), 3),
        }
    except Exception as e:
        return {"error": f"{type(e).__name__}: {e}"}


def main():
    if len(sys.argv) < 2:
        print("usage: python3 real_probe.py <network_interface> [seconds]")
        print("example (Mac):  .venv_robot/bin/python lab/real_probe.py en7")
        print("example (VM):   python3 real_probe.py enp0s2 40")
        sys.exit(1)
    iface = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0

    from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                             ChannelSubscriber)

    print(f"probing on {iface} for {seconds:.0f} s - READ ONLY, "
          f"the robot will not move")
    print("  STAND THE DOG UP FIRST. Lying down, the LiDAR rate drops from")
    print("  about 14 Hz to 7-10 Hz with blackouts of several seconds.\n")
    ChannelFactoryInitialize(DOMAIN_ID, iface)

    watches, skipped = {}, {}
    for topic, mod, cls, why in WANTED:
        klass, err = _load(mod, cls)
        if klass is None:
            skipped[topic] = f"no IDL for {mod}.{cls} in this SDK ({err})"
            continue
        w = Watch(topic, why)
        try:
            sub = ChannelSubscriber(topic, klass)
            sub.Init(w.on, 10)
            watches[topic] = w
        except Exception as e:
            skipped[topic] = f"subscribe failed: {type(e).__name__}: {e}"

    print(f"listening on {len(watches)} topics"
          + (f", {len(skipped)} unavailable" if skipped else "") + "\n")
    t0 = time.time()
    while time.time() - t0 < seconds:
        time.sleep(1.0)
        live = [t for t, w in watches.items() if w.n]
        print(f"\r  {time.time()-t0:4.0f}s   {len(live)}/{len(watches)} "
              f"topics alive   ", end="", flush=True)
    print("\n")

    # ---------------------------------------------------------------- report
    report = {"when": datetime.now().isoformat(timespec="seconds"),
              "interface": iface, "seconds": seconds,
              "topics": {}, "unavailable": skipped}

    print(f"{'topic':<38}{'msgs':>7}{'Hz':>8}   what it is")
    print("-" * 92)
    for topic, mod, cls, why in WANTED:
        w = watches.get(topic)
        if w is None:
            print(f"{topic:<38}{'-':>7}{'-':>8}   NOT AVAILABLE IN SDK")
            continue
        print(f"{topic:<38}{w.n:>7}{w.hz():>8.1f}   "
              f"{why if w.n else 'SILENT - nothing published'}")
        entry = {"messages": w.n, "hz": round(w.hz(), 2), "why": why}
        if w.n and w.last is not None:
            if cls == "PointCloud2_":
                entry["sample"] = describe_cloud(w.last)
            elif cls == "LidarState_":
                entry["sample"] = describe_lidar_state(w.last)
            elif cls == "LowState_":
                entry["sample"] = describe_lowstate(w.last)
            elif cls == "SportModeState_":
                entry["sample"] = describe_sport(w.last)
            else:
                entry["sample"] = {"note": "arrived, not decoded here"}
        report["topics"][topic] = entry

    # ------------------------------------------------------------- verdicts
    def alive(t):
        w = watches.get(t)
        return bool(w and w.n)

    cloud = next((t for t in ("rt/uslam/cloud_map",
                              "rt/utlidar/cloud_deskewed",
                              "rt/utlidar/cloud_base",
                              "rt/utlidar/cloud") if alive(t)), None)
    verdict = {
        "sdk_reaches_robot": alive("rt/lowstate"),
        "lidar_publishing": cloud is not None,
        "lidar_topic": cloud,
        "onboard_odometry": any(alive(t) for t in (
            "rt/utlidar/robot_pose", "rt/utlidar/robot_odom",
            "rt/uslam/localization/odom",
            "rt/lio_sam_ros2/mapping/odometry")),
        "onboard_slam": any(alive(t) for t in (
            "rt/uslam/cloud_map", "rt/uslam/localization/odom")),
        "height_map": alive("rt/utlidar/height_map_array"),
    }
    report["verdict"] = verdict

    print("\n" + "=" * 92)
    print("  WHAT THIS MEANS FOR THE PROJECT")
    print("=" * 92)
    if not verdict["sdk_reaches_robot"]:
        print("  The SDK is not reaching this robot at all. Either the cable")
        print("  route is not up, or this is not an EDU and we must use the")
        print("  Wi-Fi/WebRTC scripts instead. Nothing below is meaningful.")
    else:
        print("  The SDK reaches the robot: lowstate is arriving.")
        if cloud:
            s = report["topics"][cloud].get("sample", {})
            print(f"  LiDAR IS PUBLISHING on {cloud}")
            print(f"    {s.get('points_per_message', '?')} points per message, "
                  f"{report['topics'][cloud]['hz']:.1f} Hz, "
                  f"fields {s.get('fields')}")
            print("    -> mapping and navigation are on the table.")
            if cloud == "rt/utlidar/cloud":
                print("    NOTE: this is the SENSOR frame, rotated about 165")
                print("          degrees from the body on a Go2. Prefer")
                print("          cloud_deskewed if it appears.")
        else:
            print("  NO POINT CLOUD on any topic we tried.")
            print("    -> either the LiDAR is switched off (rt/utlidar/switch")
            print("       takes 'ON'), this model has none, or the topic has")
            print("       another name on this firmware. Check the topic list.")
        if verdict.get("onboard_slam"):
            print("  *** THE DOG IS RUNNING ITS OWN SLAM. rt/uslam/* is live.")
            print("      Record rt/uslam/cloud_map and the map is already")
            print("      built - the project becomes using it, not making it.")
        if verdict["onboard_odometry"]:
            print("  The robot publishes its own pose. We do not have to")
            print("  write odometry, only the map and the navigator.")
        if verdict.get("height_map"):
            print("  It also publishes a height map: a traversability grid")
            print("  for free, which is most of what a navigator needs.")
        else:
            print("  No on-board pose. Position must come from "
                  "rt/sportmodestate (drifts) or from our own scan matching.")

    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, f"probe_{datetime.now():%Y%m%d_%H%M%S}.json")
    with open(p, "w") as f:
        json.dump(report, f, indent=1, default=str)
    print(f"\nwrote {p}")
    print("Bring this file home - everything else is written against it.")


if __name__ == "__main__":
    main()
