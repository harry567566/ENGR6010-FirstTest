"""Everything this robot exposes, in one place.

    .venv_wifi/bin/python lab/go2_map.py              print the reference
    .venv_wifi/bin/python lab/go2_map.py --probe      ask the dog as well
    .venv_wifi/bin/python lab/go2_map.py --md > ...   as a markdown file

The reference is generated from the installed library's own tables rather
than typed out here, so it cannot drift away from the code that will
actually be called. What it adds on top is which route each thing is
reachable on, and what it is for - the library has the names and the
numbers but not the map.

Routes, because this is the thing that decides what is possible:

    cable   Ethernet, 192.168.123.161, DDS/CycloneDDS, EDU and X only.
            Full-rate state, rt/lowcmd (joint-level control), the raw
            LiDAR topics. This is the only route to low-level control.
    wifi    the dog's own hotspot 192.168.12.1, WebRTC, every model.
            Commands, camera, the compressed cloud, the on-board SLAM
            and navigation. No DDS, so no rt/lowcmd and no joint control.

A thing reachable on wifi is reachable in simulation only if the simulator
implements it; a thing on cable is DDS and MuJoCo speaks DDS, so the cable
surface is the one that transfers from sim to the real robot unchanged.
"""
import argparse
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from unitree_webrtc_connect import constants as C
except ImportError:
    C = None

# What each thing is for, and where it works. Only the prose is here; the
# names and numbers come from the library.
NOTES = {
    # ---- state the robot publishes
    "rt/lowstate": ("cable", "全速状态:12 个电机、IMU、电池、脚底受力。1000 Hz 级"),
    "rt/lf/lowstate": ("wifi", "同样的内容,降频版(约 2 Hz)。Wi-Fi 上只有这个"),
    "rt/sportmodestate": ("cable", "模式、身高、速度、位置,约 50 Hz"),
    "rt/lf/sportmodestate": ("wifi", "同上,约 20 Hz"),
    "rt/lowcmd": ("cable", "★ 关节级控制。强化学习部署唯一的入口,Wi-Fi 上没有"),
    "rt/utlidar/cloud": ("cable", "原始点云,传感器坐标系(相对机身转了约 165°)"),
    "rt/utlidar/cloud_deskewed": ("cable", "去运动畸变的点云,已在 odom 坐标系"),
    "rt/utlidar/voxel_map_compressed": ("wifi", "压缩体素点云,libvoxel.wasm 解码"),
    "rt/utlidar/robot_pose": ("both", "狗自己的雷达里程计"),
    "rt/utlidar/robot_odom": ("cable", "同样的位姿,带速度"),
    "rt/utlidar/lidar_state": ("both", "雷达转速、丢包率、固件版本"),
    "rt/utlidar/switch": ("both", "开关雷达,发字符串 \"ON\" / \"OFF\""),
    # ---- the robot's own SLAM
    "rt/uslam/client_command": ("wifi", "★ 建图/定位/导航/巡逻的命令入口。纯字符串"),
    "rt/uslam/server_log": ("wifi", "SLAM 服务的回复,成功失败都在这里"),
    "rt/uslam/frontend/cloud_world_ds": ("wifi", "★ 建图时的世界点云,libslam.wasm 解码"),
    "rt/uslam/frontend/odom": ("wifi", "建图时的位姿"),
    "rt/uslam/cloud_map": ("wifi", "整张地图。更可靠的拿法是文件传输 map.pcd"),
    "rt/uslam/localization/odom": ("wifi", "定位模式下的位姿"),
    "rt/uslam/localization/cloud_world": ("wifi", "定位用的世界点云,PointCloud2"),
    "rt/uslam/navigation/global_path": ("wifi", "★ 规划出来的全局路径,PointCloud2"),
    # ---- command topics
    "rt/api/sport/request": ("both", "运动命令总入口,用 api_id 区分"),
    "rt/api/obstacles_avoid/request": ("wifi", "★ 避障服务:开关 + 带避障的移动"),
    "rt/api/motion_switcher/request": ("both", "查/换当前控制器(mcf / sport_mode)"),
    "rt/api/vui/request": ("both", "灯和声音"),
    "rt/api/bashrunner/request": ("cable", "在狗上跑脚本,能查硬件和固件版本"),
}

USLAM_COMMANDS = {
    "mapping": ["start", "stop", "cancel", "get_status", "get_cloud_map",
                "set_map_pose", "run_mapping_process"],
    "localization": ["start", "stop", "get_status",
                     "set_initial_pose/{x}/{y}/{yaw}", "set_initial_pose_type"],
    "navigation": ["start", "stop", "get_status",
                   "set_goal_pose/{x}/{y}/{yaw}"],
    "patrol": ["start", "go", "pause", "stop", "get_status",
               "add_patrol_point/{x}/{y}/{yaw}", "clear_all_patrol_points",
               "get_patrol_points", "set_total_time_limit/{s}",
               "set_bms_soc_limit/{min}/{max}"],
    "autocharge": ["start", "stop", "get_status", "set_plate_distance/{m}",
                   "go_to_charge_and_stop_patrol"],
    "common": ["get_map_id", "set_map_id/{id}", "get_map_file",
               "enable_joystick_control", "disable_joystick_control"],
    "frontend": ["start", "stop", "restart", "get_status"],
    "control": ["start", "stop", "move_velocity", "stand_up", "stand_down",
                "recovery_stand", "stop_move", "get_status"],
}

USLAM_HOW = """\
rt/uslam/client_command 收的是【纯字符串】,不是 api_id 那种结构。
必须用 publish_without_callback —— publish() 会等一个从 payload 里挖
uuid 的 future,纯字符串没有 uuid,所以会永远卡住。

    ps.publish_without_callback("rt/uslam/client_command", "mapping/start")

建图的完整流程:
    1  mapping/start
    2  遛狗(cloud_world_ds 以约 5 Hz 流出来)
    3  mapping/stop   等 server_log 回 mapping/stop/success
    4  common/set_map_id/<自己起的id>   狗不会自己生成 id
    5  把 map.pcd / map.pgm / map.txt 用文件传输拖回来

导航:
    navigation/start                      必须先起来
    navigation/set_goal_pose/x/y/yaw      yaw 要在 [-pi, pi] 内
状态会以 navigation/state_transition/<STATE> 的形式回到 server_log:
    WAITING TRACKING REACHED NO_PATH TIMEOUT GOAL_OCCUPIED FAILURE

注意:狗只有【一个】地图槽。每次 mapping/start 都会覆盖上一张。
"""

ROUTE_NOTE = """\
两条路,决定了什么做得到:

  网线  192.168.123.161,DDS/CycloneDDS,只有 EDU 和 X 支持
        全速状态、rt/lowcmd(关节级控制)、原始点云。
        ★ 强化学习部署、仿真迁移,只有这条路。
        MuJoCo 仿真器发的也是 DDS,所以在仿真里写的东西
        换到真狗上不用改——这是 sim→real 的关键。

  Wi-Fi 狗自己的热点 192.168.12.1,WebRTC,所有型号都行
        命令、摄像头、压缩点云、板载 SLAM 和导航。
        没有 DDS,所以没有 rt/lowcmd,控不了单个关节。
        狗一次只接一个 Wi-Fi 客户端。

也就是说:现场遛狗、建图、导航走 Wi-Fi 最方便;
要把仿真里训出来的策略放到真狗上,必须插网线。
"""


def table(title, rows, widths=(34, 8, 54)):
    out = [f"\n{title}\n" + "─" * (sum(widths) + 4)]
    for a, b, c in rows:
        out.append(f"{str(a):<{widths[0]}}  {str(b):<{widths[1]}}  {c}")
    return "\n".join(out)


def build():
    out = [__doc__.split("\n\n")[0], "", ROUTE_NOTE]

    if C is None:
        out.append("!! unitree_webrtc_connect 没装:用 .venv_wifi 的 python 跑")
        return "\n".join(out)

    rows = []
    for name, topic in sorted(C.RTC_TOPIC.items(), key=lambda kv: kv[1]):
        route, why = NOTES.get(topic, ("wifi", ""))
        rows.append((topic, route, why))
    out.append(table("话题  (库里 RTC_TOPIC 的全部内容)", rows))

    cmds = getattr(C, "SPORT_CMD_MCF", None) or C.SPORT_CMD
    rows = [(k, v, "") for k, v in sorted(cmds.items(), key=lambda kv: kv[1])]
    out.append(table("运动命令  rt/api/sport/request  (mcf 固件)",
                     rows, (28, 8, 40)))

    if hasattr(C, "OBSTACLES_AVOID_API"):
        rows = [("SWITCH_SET", 1001, '{"enable": true}  开关避障'),
                ("SWITCH_GET", 1002, "{}  查当前状态"),
                ("MOVE", 1003, '{"x","y","yaw","mode":0}  ★ 带避障的移动'),
                ("USE_REMOTE_COMMAND_FROM_API", 1004,
                 '{"is_remote_commands_from_api": bool}')]
        out.append(table("避障服务  rt/api/obstacles_avoid/request",
                         rows, (30, 8, 46)))

    rows = []
    for ns, cs in USLAM_COMMANDS.items():
        for c in cs:
            rows.append((f"{ns}/{c}", "wifi", ""))
    out.append(table("板载 SLAM 命令  rt/uslam/client_command  (纯字符串)",
                     rows, (44, 6, 30)))
    out.append("\n" + USLAM_HOW)

    out.append("""
还没验证的 / 下一步
────────────────────────────────────────────────────────────
  · voxel_map_compressed 的坐标系和尺度 —— 1-1b 窗口底下那行会报
  · 相机-雷达外参 —— 1-1b 的「相机+雷达」视图看点落得准不准
  · 板载 SLAM 在这只狗上有没有真的跑起来 —— 按「开始建图」看 server_log
  · 导航能不能用 —— 要先 localization 成功
  · 相机标定是 640x480 的,而串流是 1280x720,纵向需要重标
""")
    return "\n".join(out)


async def probe(target):
    """Ask the dog itself which services it is running."""
    from wifi_common import connect, disconnect
    conn = await connect(target)
    print("\n=== 狗自己报的服务列表 ===")
    try:
        ans = await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(
                "rt/api/robot_state/request", {"api_id": 1003}), 6.0)
        data = ans.get("data", {}).get("data")
        if isinstance(data, str):
            data = json.loads(data)
        for sv in (data or []):
            print("  %-26s %s" % (sv.get("name"),
                                  "开" if sv.get("status") == 0 else sv))
    except Exception as e:
        print("  拿不到服务列表:", e)

    print("\n=== 哪些话题真的在发(听 8 秒) ===")
    seen = {}
    watch = ["rt/lf/lowstate", "rt/lf/sportmodestate",
             "rt/utlidar/voxel_map_compressed", "rt/utlidar/robot_pose",
             "rt/utlidar/lidar_state", "rt/uslam/server_log",
             "rt/uslam/frontend/cloud_world_ds", "rt/uslam/frontend/odom",
             "rt/uslam/localization/odom", "rt/uslam/navigation/global_path"]
    for t in watch:
        seen[t] = 0
        conn.datachannel.pub_sub.subscribe(
            t, (lambda tt: (lambda _m: seen.__setitem__(tt, seen[tt] + 1)))(t))
    await asyncio.sleep(8)
    for t in watch:
        n = seen[t]
        print("  %-42s %s" % (t, f"{n} 条" if n else "— 没有"))
    await disconnect(conn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true",
                    help="连上狗,问它自己有什么")
    ap.add_argument("--target", default=None)
    a = ap.parse_args()
    print(build())
    if a.probe:
        target = a.target
        if target is None:
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".go2_wifi_ip")
            target = (open(path).read().strip() if os.path.exists(path)
                      else "ap") or "ap"
        asyncio.run(probe(target))


if __name__ == "__main__":
    main()
