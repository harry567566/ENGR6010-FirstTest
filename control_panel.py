#!/usr/bin/env python3
"""ENGR 6010 - Unitree Go2 control panel.

One window with a button for every part of the work:
  * REAL ROBOT  - talks to the Go2 in the lab over Ethernet with the official Unitree SDK (DDS),
                  straight from this Mac (.venv_robot) or, as a backup, through the Ubuntu VM
  * Q2 simulation - the SDK examples against Unitree's simulator inside the VM
  * Q3 / Q4       - learned walking and vision, on this Mac
The right side explains the selected button and shows what the program prints.
The EN / 中文 button switches the language.

    ~/miniconda3/envs/go2/bin/python control_panel.py
"""
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext

HERE = os.path.dirname(os.path.abspath(__file__))
VM_DIR = os.path.join(HERE, "_vm")
VM_ID_FILE = os.path.join(VM_DIR, "vm_id.txt")
PY = sys.executable
MJPY = os.path.join(os.path.dirname(sys.executable), "mjpython")
FONT = "PingFang SC"
MONO = "Menlo"
HAND = "pointinghand" if sys.platform == "darwin" else "hand2"
PRACTICE_OUT = "/tmp/firsttest_practice"
ROBOT_IP = "192.168.123.161"
LAB = os.path.join(HERE, "lab")
LOCAL_DATA = os.path.join(LAB, "data")
ROBOT_PY = os.path.join(HERE, ".venv_robot", "bin", "python")  # made by lab/setup_mac_sdk.sh
WIFI_PY = os.path.join(HERE, ".venv_wifi", "bin", "python")    # made by lab/setup_wifi_sdk.sh
DOG_AP_IP = "192.168.12.1"  # the dog's address on its own Wi-Fi hotspot
WIFI_SCRIPT = {"real_state.py": "wifi_state.py", "real_photo.py": "wifi_photo.py",  # cable script -> Wi-Fi twin
               "collect_real.py": "wifi_record.py", "real_sport.py": "wifi_sport.py"}
ROUTES = [("cable", "Cable", "网线"), ("wifi", "Wi-Fi", "Wi-Fi"), ("vm", "VM (backup)", "虚拟机(备用)")]

# --- shell snippets used inside the VM --------------------------------------------------------
# The pattern "[u]nitree_mujoco.py" finds the simulator without matching the ssh command itself,
# and the start command spells the file name through a variable for the same reason.
VM_ENV = ("export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority "
          "CYCLONEDDS_HOME=$HOME/cyclonedds/install PYTHONIOENCODING=utf-8; ")
SIM_UP = 'pgrep -f "[u]nitree_mujoco.py" >/dev/null'
NEED_SIM = SIM_UP + ' || { echo "The simulator is not running: click 2-1 first. / 仿真狗还没打开:先点 2-1"; exit 1; }; '
QUIET = 'grep --line-buffered -v -E "multicast|WARNING: Please"'
# the robot's card in the VM: the one on 192.168.123.x, else the newest card that is not loopback or internet (enp0s1)
FIND_IF = ("IFACE=$(ip -br addr | awk '$3 ~ /^192[.]168[.]123[.]/ {print $1; exit}'); "
           "[ -n \"$IFACE\" ] || IFACE=$(ip -br link | awk '$1!=\"lo\" && $1!=\"enp0s1\" {i=$1} END {print i}'); ")
ROBOT_IF = (FIND_IF + "if [ -z \"$IFACE\" ]; then echo \"No robot network card in the VM yet: click '1-1 Connect to the dog' "
            "first. / 虚拟机里还没有连狗的网卡:先点 1-1\"; exit 2; fi; ")
NEED_ROBOT = (ROBOT_IF + f"ping -c 1 -W 1 {ROBOT_IP} >/dev/null || {{ echo \"Cannot reach the robot at {ROBOT_IP}: "
              "click '1-1 Connect to the dog', check the cable, and check the dog is on. "
              "/ 连不上狗:先点 1-1,检查网线和狗有没有开机\"; exit 3; }; ")


def utm(cmd):
    vid = open(VM_ID_FILE).read().strip()
    return ["osascript", "-e", f'tell application "UTM" to {cmd} virtual machine id "{vid}"']


def A(key, where, cmd, en, zh, say="", **kw):
    """One button. en / zh = (label, what it does, what you will see, why)."""
    d = dict(key=key, where=where, cmd=cmd, say=say, **kw)
    for lang, (label, what, see, why) in (("en", en), ("zh", zh)):
        d[lang] = dict(label=label, what=what, see=see, why=why)
    return d


def robot(script):
    return NEED_ROBOT + "cd ~/q2/lab; " + script


def R(key, run, en, zh, limit=20, **kw):
    """A real-robot button. run = (script, arguments after the network card). limit = seconds before it is ended."""
    return A(key, "robot", None, en, zh, run=run, limit=limit, **kw)


def wifi_target():
    """'ap' (the Mac on the dog's own hotspot), or the dog's IP on a shared router, written in lab/.go2_wifi_ip."""
    path = os.path.join(LAB, ".go2_wifi_ip")
    ip = open(path).read().strip() if os.path.exists(path) else ""
    return ip or "ap"


def wifi_dog_ip(target):
    return DOG_AP_IP if target == "ap" else target


def mac_wifi_ip():
    """The Mac's own address on Wi-Fi ('' if none). On the dog's hotspot it is 192.168.12.x."""
    try:
        ports = subprocess.run(["networksetup", "-listallhardwareports"], capture_output=True, text=True).stdout
        dev = re.search(r"Hardware Port: Wi-Fi\nDevice: (\S+)", ports).group(1)
        return subprocess.run(["ipconfig", "getifaddr", dev], capture_output=True, text=True).stdout.strip()
    except (OSError, AttributeError):
        return ""


def dog_wifi_port_open(ip, timeout=1.5):
    """Does the dog's Wi-Fi control (WebRTC signaling) port answer? New firmware uses 9991, older 8081."""
    for port in (9991, 8081):
        try:
            with socket.create_connection((ip, port), timeout=timeout):
                return True
        except OSError:
            pass
    return False


def mac_robot_iface():
    """The Mac network card that has a 192.168.123.x address (set by 1-1), or None."""
    try:
        out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    except OSError:
        return None
    card = None
    for line in out.splitlines():
        if line and not line[0].isspace():
            card = line.split(":")[0]
        elif line.strip().startswith("inet 192.168.123."):
            return card
    return None


ACTIONS = [
    (("1  Real robot", "1  真狗", "in the lab, over the Ethernet cable", "在实验室,用网线连"), [
        A("r_net", "robot", None,
          ("1-1  Connect to the dog", "Finds the Mac's USB-Ethernet port with the cable, gives it an address on the "
           f"Go2's network (192.168.123.222) and pings the dog ({ROBOT_IP}). macOS asks for your Mac password once.",
           "'ROBOT REACHABLE' and three ping replies. The top right then says 'Dog connected' (green dot).",
           "The dog's computer only answers on its own wired 192.168.123.x network, so the Mac joins it first. "
           "(Wi-Fi: checks that the Mac is on the dog's Wi-Fi and that the dog answers. VM backup: restarts the VM "
           "once to give it a card on the cable, then does the same inside Ubuntu with 192.168.123.223.)"),
          ("1-1  连上狗", "找到插着网线的 USB 转网口,给它一个狗那个网络里的地址 192.168.123.222,然后 ping 狗"
           f"({ROBOT_IP})。macOS 会弹窗要一次 Mac 开机密码。",
           "看到 'ROBOT REACHABLE' 和 3 次 ping 回复,右上角变成 '狗 已连上'(绿点)。",
           "狗的电脑只在它自己的有线网络 192.168.123.x 上回话,所以 Mac 先加入这个网络。"
           "(Wi-Fi:检查 Mac 是否连着狗的 Wi-Fi、狗有没有回应。虚拟机备用路线:重启一次虚拟机,给它加一张"
           "接网线的网卡,再在 Ubuntu 里做同样的事,地址用 192.168.123.223。)"),
          say="The Go2 is at 192.168.123.161 on its own wired network, so my computer joins that network first."),
        A("w_live", "mac_window", [WIFI_PY, os.path.join(LAB, "wifi_live.py")],
          ("1-1b  Live view over Wi-Fi: camera, LiDAR, WASD",
           "Opens one window that holds a SINGLE Wi-Fi connection and shows the front camera, the LiDAR "
           "point cloud from above, and the dog's state - all at once. Every other Wi-Fi button opens its own "
           "connection and hangs up, which costs seconds and makes the dog answer '429 Too Many Requests' when "
           "two land together. Opening the window commands nothing; driving is behind a tick box inside it.",
           "A window with the camera on the left and the LiDAR on the right. Tick 'allow motion' and W A S D "
           "drive, Q E turn, space stops.",
           "The point cloud does come over Wi-Fi - not as DDS, which really is cable-only, but on the robot's "
           "own WebRTC channel, the one the phone app draws its map from."),
          ("1-1b  Wi-Fi 实时画面:摄像头 + 雷达 + WASD",
           "打开一个窗口,用【一条】常驻 Wi-Fi 连接同时显示前置摄像头、俯视的激光雷达点云和狗的状态。"
           "其他每个 Wi-Fi 按钮都是自己连一次再断开,既慢,两个挨着按还会让狗回 '429 Too Many Requests'。"
           "打开窗口本身不给狗发任何命令;开狗要在窗口里单独勾选。",
           "一个窗口:左边摄像头,右边雷达俯视图。勾上'允许运动'后 W A S D 走、Q E 转、空格停。",
           "点云确实能走 Wi-Fi 拿到 —— 不是 DDS(DDS 确实只能走网线),而是狗自己的 WebRTC 通道,"
           "手机 App 画地图用的就是它。"),
          say="One Wi-Fi connection, held open, carrying the camera, the LiDAR and the keyboard at the same time."),
        R("r_state", ("real_state.py", "10"),
          ("1-2  Read robot state (safe)", "Listens to the robot's messages for 10 s. Sends nothing, so the robot "
           "does not move.", "Battery voltage, body tilt, body height, leg angles, foot forces, message rate.",
           "The same rt/lowstate messages as in the simulator. Always check this before any motion command."),
          ("1-2  读取狗的状态(安全)", "听狗发来的消息 10 秒。什么都不发,狗不会动。",
           "电池电压、身体倾斜、身体高度、腿的角度、脚底受力、每秒多少条消息。",
           "和仿真器里一样的 rt/lowstate 消息。每次发运动命令前,都先用这个确认连接正常。"),
          say="This only listens. It reads the same rt/lowstate messages I used in the simulator."),
        R("r_probe", ("real_probe.py", "25"),
          ("1-2b  What IS this dog? (safe)",
           "Listens on nine topics for 25 s and reports what this particular Go2 "
           "publishes: model, firmware, whether the LiDAR is spinning and sending "
           "a point cloud, how many points per sweep, and whether the robot "
           "publishes its own odometry. Sends nothing.",
           "A table of every topic with its message rate, a verdict in plain "
           "words, and lab/data/probe_<time>.json to bring home.",
           "Run this FIRST, before anything else. Every script written after "
           "today is written against that JSON file, so guessing topic names "
           "is what wastes a lab session."),
          ("1-2b  这只狗到底有什么?(安全)",
           "监听九个话题 25 秒,报告这只 Go2 到底发什么:型号、固件、"
           "激光雷达转不转、发不发点云、每帧多少点、狗自己发不发里程计。"
           "什么都不发送,狗不会动。",
           "一张所有话题和频率的表、一段大白话结论,以及 "
           "lab/data/probe_<时间>.json 带回家。",
           "第一个跑这个,先于一切。今天之后写的所有代码都照着那个 JSON 写。"
           "靠猜话题名字是最浪费实验室时间的事。"),
          say="This only listens. It tells me what this robot can give me "
              "before I write anything that depends on it.",
          limit=40),
        R("r_topics", ("real_topics.py", "20"),
          ("1-2c  List every topic the dog publishes (safe)",
           "Asks DDS itself what this robot is broadcasting, instead of "
           "checking names we guessed. 1-2b tests nine likely topics; if this "
           "firmware calls the point cloud something else, only this finds it.",
           "Every topic with its type and a note on the important ones, plus "
           "lab/data/topics_<time>.json.",
           "Run this right after 1-2b. Between them they settle what the "
           "robot can give us, with no guessing left."),
          ("1-2c  列出狗发的每一个话题(安全)",
           "直接问 DDS 这只狗在广播什么,而不是去试我们猜的名字。"
           "1-2b 只测九个可能的话题;如果这个固件给点云起了别的名字,"
           "只有这个找得到。",
           "所有话题及其类型,重要的会单独标出,存 lab/data/topics_<时间>.json。",
           "紧跟在 1-2b 后面跑。两个加起来就把'这只狗能给什么'问清楚了,不剩猜测。"),
          say="This asks DDS what the robot actually publishes, so I am not "
              "guessing topic names.",
          limit=35),
        R("r_photo", ("real_photo.py", "{photo}"),
          ("1-3  Take a photo (safe)", "Asks the robot's front camera for one image (VideoClient).",
           "The photo opens on the Mac and is saved in lab/data/photos/.", "Read-only; the robot does not move."),
          ("1-3  拍一张照片(安全)", "让狗的前置摄像头拍一张照片(VideoClient)。",
           "照片会在 Mac 上打开,并存到 lab/data/photos/。", "只读,狗不会动。")),
        R("r_record", ("collect_real.py", "--secs", "20", "--fps", "5", "--out", "{run}"),
          ("1-4  Record data 20 s (safe)", "Records camera frames and robot state for 20 s, for Question 4. "
           "Someone can walk the dog with the remote while it records.",
           "The recording is saved in lab/data/ on the Mac and the folder opens.", "Read-only."),
          ("1-4  录 20 秒数据(安全)", "录 20 秒摄像头画面和狗的状态,给 Q4 用。录的时候可以有人用遥控器遛狗。",
           "录好的数据存在 Mac 的 lab/data/,并打开文件夹。", "只读。"), limit=45),
        R("r_survey", ("real_survey.py", "--secs", "180", "--fps", "4",
                       "--name", "walk"),
          ("1-4b  Survey the building, 3 min (safe)",
           "Records everything at once for three minutes: state at 50 Hz, the "
           "LiDAR point cloud if this robot has one, on-board odometry if it "
           "publishes any, and camera frames at 4 fps - all on one clock. "
           "Sends nothing; someone drives with the remote.",
           "lab/data/survey_walk_<time>/ with meta.json, state.jsonl, "
           "odom.jsonl, clouds/ and frames/.",
           "Point clouds are saved as raw bytes plus their field layout, "
           "not decoded here. The robot stays in the lab, so anything this "
           "misses cannot be collected again later."),
          ("1-4b  扫一遍楼,3 分钟(安全)",
           "三分钟内把能录的全录下来:50 Hz 的状态、有雷达就录点云、"
           "狗发里程计就录里程计、4 fps 的相机画面 —— 全部挂在同一个时钟上。"
           "什么都不发送;让人用遥控器遛狗。",
           "存到 lab/data/survey_walk_<时间>/,含 meta.json、state.jsonl、"
           "odom.jsonl、clouds/、frames/。",
           "点云按原始字节保存,外加字段布局,不在现场解码 —— "
           "狗留在实验室,今天漏掉的东西没法回头再采。"),
          say="This only listens. It records the point cloud, the state and "
              "the camera on one clock so they can be lined up afterwards.",
          limit=240),
        R("m_standup", ("real_sport.py", "standup", "--yes"),
          ("1-5  Stand up", "Sends StandUp to the robot's own controller (SportClient).",
           "The dog stands up and holds still.",
           "My program only says WHAT to do. The dog's built-in controller decides how to move the 12 motors."),
          ("1-5  站起来", "给狗自带的控制器发 StandUp(SportClient)。", "狗站起来,保持不动。",
           "我的程序只说'做什么',怎么动 12 个电机由狗自带的控制器决定。"),
          motion=True, say="These are high-level commands. The robot's own controller moves the legs."),
        R("m_balance", ("real_sport.py", "balance", "--yes"),
          ("1-6  Balance stand", "BalanceStand: stand and keep balance, ready to walk.", "The dog stands, slightly active.", ""),
          ("1-6  平衡站立", "BalanceStand:站着保持平衡,准备走路。", "狗站着,会轻微调整。", ""), motion=True),
        R("m_walk", ("real_sport.py", "move", "0.3", "0", "0", "3", "--yes"),
          ("1-7  Walk forward 0.3 m/s, 3 s", "Move(0.3, 0, 0), re-sent 20 times a second for 3 seconds, then "
           "StopMove. If the dog is standing but not in balance stand (for example right after Stand up), it sends "
           "BalanceStand first; if the dog is lying down, it refuses.",
           "The dog walks forward and stops. The output then says how far the dog measured it moved (up to about "
           "0.9 m; less, because the dog speeds up gradually).", "Speed and time are limited in real_sport.py: at "
           "most 0.3 m/s and 3 s."),
          ("1-7  往前走 3 秒(0.3 米/秒)", "Move(0.3, 0, 0),每秒重发 20 次,持续 3 秒,然后 StopMove。狗站着但不在"
           "平衡站立(比如刚站起来)时,会先自动发 BalanceStand;狗趴着时不执行。",
           "狗往前走然后停。输出里会写狗自己测量走了多远(最多约 0.9 米;狗是慢慢加速的,所以会少一些)。",
           "real_sport.py 里限制了速度和时间:最多 0.3 米/秒、3 秒。"), motion=True, limit=30),
        R("m_turn", ("real_sport.py", "move", "0", "0", "0.5", "3", "--yes"),
          ("1-8  Turn left 0.5 rad/s, 3 s", "Move(0, 0, 0.5) for 3 seconds, then StopMove (BalanceStand first if "
           "needed, like 1-7).", "The dog turns left on the spot. The output says how many degrees it measured "
           "(up to about 85).", "Speed and time are limited in real_sport.py: at most 0.5 rad/s and 3 s."),
          ("1-8  原地左转 3 秒(0.5 弧度/秒)", "Move(0, 0, 0.5) 持续 3 秒,然后 StopMove(需要时先发 BalanceStand,"
           "和 1-7 一样)。", "狗原地左转。输出里会写狗测量转了多少度(最多约 85 度)。",
           "real_sport.py 里限制了速度和时间:最多 0.5 弧度/秒、3 秒。"), motion=True, limit=30),
        R("m_standdown", ("real_sport.py", "standdown", "--yes"),
          ("1-9  Lie down", "StandDown: the dog lowers its body to the ground.", "The dog lies down.",
           "The safe way to finish. Do this before Damp."),
          ("1-9  趴下", "StandDown:狗把身体放低到地上。", "狗趴下。", "结束时的安全做法,要在 Damp 之前做。"), motion=True),
        A("m_stop", "robot", None,
          ("■  STOP moving", "Ends any motion command still running, then sends StopMove: stop walking at once. "
           "Always allowed, no questions asked.",
           "The dog stops walking and stands.", "Your first button if anything looks wrong. The remote (L2 + B) also works."),
          ("■  停止运动", "先掐掉还在运行的运动命令,再发 StopMove:立刻停止走路。随时可以按,不会弹窗。", "狗停下,站着。",
           "看到任何不对劲,第一个按这个。遥控器(L2 + B)也可以停。")),
        R("m_damp", ("real_sport.py", "damp", "--yes"),
          ("Damp (motors go soft)", "Damp: all motors go limp. If the dog is standing it will DROP to the ground.",
           "The dog collapses and its legs go loose.", "Only after 'Lie down', or if the instructor says so."),
          ("Damp(电机放松)", "Damp:所有电机变软。如果狗是站着的,会直接摔到地上。", "狗瘫下去,腿变软。",
           "只在'趴下'之后用,或者老师让你用的时候。"),
          motion=True, confirm=("DAMP makes all motors go limp. A standing dog will DROP.\n\nIs the dog already lying down?",
                                "DAMP 会让所有电机变软,站着的狗会直接摔下来。\n\n狗已经趴下了吗?")),
    ]),
    (("Ubuntu VM", "Ubuntu 虚拟机", "needed for Q2 and for the backup route", "Q2 和备用路线要用"), [
        A("vm_start", "host", ["bash", "tools/vm_start.sh"],
          ("Start the VM", "Opens UTM, starts the Ubuntu virtual machine and waits until Ubuntu answers.",
           "About 30 s later: 'VM is up.' here, and 'VM on' (green dot) at the top right.",
           "The Q2 simulator (and the backup route to the real dog) run inside Ubuntu."),
          ("启动虚拟机", "打开 UTM,启动 Ubuntu 虚拟机,等到 Ubuntu 能连上为止。",
           "大约 30 秒后这里显示 'VM is up.',右上角变成 '虚拟机 已连接'(绿点)。",
           "Q2 的仿真器(还有连真狗的备用路线)在 Ubuntu 里跑。")),
        A("vm_stop", "host", ["bash", "tools/vm_stop.sh"],
          ("Shut down the VM", "Shuts Ubuntu down cleanly and gives the 8 GB of memory back to the Mac. "
           "If the VM is stuck, it forces it off, and closes UTM if UTM itself hangs.",
           "'VM off.' here, and 'VM off' at the top right.", "Also the way out when the VM is stuck."),
          ("关闭虚拟机", "让 Ubuntu 正常关机,把 8 GB 内存还给 Mac。虚拟机卡住时会强制关掉,UTM 本身卡死就把 UTM 也关掉。",
           "这里显示 'VM off.',右上角显示 '虚拟机 关'。", "虚拟机卡住时也用这个。")),
    ]),
    (("2  Q2 simulation", "2  Q2 仿真", "Unitree's simulator in the VM; the dog appears in the UTM window",
      "虚拟机里的官方仿真器;狗出现在 UTM 窗口里"), [
        A("sim_on", "vm", "cd ~/unitree_mujoco/simulate_python; S=unitree_mujoco; "
                          f"if {SIM_UP}; then echo 'The simulator is already running.'; else "
                          "(nohup python3 ${S}.py > /tmp/sim.log 2>&1 &); sleep 5; "
                          f"if {SIM_UP}; then echo 'Simulator started: look at the Ubuntu window in UTM.'; "
                          "else echo 'Did not start. Last lines of the log:'; tail -5 /tmp/sim.log; fi; fi",
          ("2-1  Start the simulated dog", "Starts unitree_mujoco, Unitree's own simulator. It sends and receives "
           "the same DDS messages as the real Go2.", "The dog is NOT in this panel: the UTM window (the Ubuntu desktop) "
           "comes to the front and shows 'MuJoCo : go2 scene' with a dog lying down. Click this panel to come back.",
           "A stand-in for the real dog. 2-3 and 2-4 talk to it."),
          ("2-1  打开仿真狗", "打开 Unitree 官方仿真器 unitree_mujoco,它收发的消息和真狗一模一样。",
           "狗不在这个面板里:UTM 窗口(Ubuntu 桌面)会跳到最前面,里面有 'MuJoCo : go2 scene',一只狗趴着。点一下面板就能回来。",
           "真狗的替身,2-3、2-4 都在跟它通信。"),
          show_vm=True, say="unitree_mujoco sends and receives the same DDS messages as the real Go2."),
        A("hello", "vm", "cd ~/unitree_sdk2_python/example/helloworld; "
                         "(timeout 10 python3 -u subscriber.py 2>&1 | sed -u 's/^/[received] /' &); sleep 1.5; "
                         "timeout 6 python3 -u publisher.py 2>&1 | sed -u 's/^/[sent] /'; sleep 2.5",
          ("2-2  Example 1: send / receive", "The SDK's helloworld: one program publishes on a topic, another receives.",
           "Pairs of [sent] and [received] with the same time stamp. No dog needed.",
           "DDS is like a radio channel: whoever listens on a topic gets the message."),
          ("2-2  例子 1:发消息/收消息", "SDK 的 helloworld:一个程序往频道里发,另一个收。",
           "每秒一对 [sent] 和 [received],时间戳一样。不需要狗。",
           "DDS 像广播频道:谁订阅这个频道谁就能收到。"),
          say="helloworld shows DDS works: one program publishes, the other receives the same message."),
        A("stand", "vm", NEED_SIM + "cd ~/q2/sim_examples; "
                         f"(timeout 11 python3 -u watch_state.py 10 2>&1 | {QUIET} &); sleep 1; "
                         f"timeout 20 python3 -u go2_stand_sim.py 2>&1 | {QUIET}; sleep 4",
          ("2-3  Example 2: stand up (low-level)", "Sends target angles to all 12 motors every 2 ms (rt/lowcmd) "
           "and reads the state back (rt/lowstate).",
           "In the UTM window (it comes to the front) the dog stands up and lowers again; here the body height "
           "goes 0.117 m -> 0.384 m -> 0.117 m.",
           "Nothing is learned here: a fixed list of joint angles. This low-level mode is NOT used on the real "
           "dog today, it bypasses the dog's own controller."),
          ("2-3  例子 2:站起来(底层控制)", "每 2 毫秒给 12 个电机发目标角度(rt/lowcmd),再读回状态(rt/lowstate)。",
           "UTM 窗口会跳到前面,里面的狗站起来再放低;这里显示身体高度 0.117 → 0.384 → 0.117 米。",
           "这里没有学习,是写死的关节角度。真狗今天不用这种底层模式,它会绕过狗自带的控制器。"),
          show_vm=True, say="The low-level example sets all 12 motor angles every 2 ms. It is a fixed sequence, not learned."),
        A("remote", "vm", NEED_SIM + "cd ~/q2/sim_examples; "
                          f"timeout 3 python3 -u wireless_controller_go2_sim.py 2>&1 | {QUIET} "
                          "| awk 'NR%8000<=21 {print; fflush()}'; "
                          "echo '(All zeros: there is no remote control in the VM.)'",
          ("2-4  Example 3: read the remote", "Decodes the remote-control bytes inside rt/lowstate.",
           "Lx, Rx, A, B ... all zero.", "On the real dog these change when you press the remote."),
          ("2-4  例子 3:读遥控器", "把 rt/lowstate 里的遥控器数据翻译成摇杆和按键。", "Lx、Rx、A、B……全是 0。",
           "真狗上按遥控器,这些数就会变。")),
        A("sim_off", "vm", 'pkill -f "[u]nitree_mujoco.py" && echo "Simulator stopped." || echo "It was not running."',
          ("2-5  Stop the simulated dog", "Closes the simulator.", "The MuJoCo window disappears.", ""),
          ("2-5  关掉仿真狗", "关掉仿真器。", "MuJoCo 窗口消失。", "")),
    ]),
    (("3  Q3 learned walking", "3  Q3 学会走路", "opens a window on this Mac", "在 Mac 上打开新窗口"), [
        A("go2py", "mac_window", [MJPY, "demo_walk.py", "--view"],
          ("3-1  Go2Py walking policy", "Go2Py's walk-these-ways network, trained with reinforcement learning.",
           "A new window: stand, walk, turn, sidestep, walk back, stop, repeating. Close it to end.",
           "Every 0.02 s the network reads the joints and the speed command and outputs 12 joint targets."),
          ("3-1  Go2Py 走路网络", "Go2Py 用强化学习训练好的走路神经网络。",
           "新窗口:站 → 走 → 转弯 → 横走 → 后退 → 停,循环。关窗口结束。",
           "每 0.02 秒,网络读一次关节状态和速度命令,输出 12 个关节的目标角度。"),
          cwd="q3_go2py", say="This policy was trained with reinforcement learning. It turns a speed command into 12 joint targets 50 times a second."),
    ]),
    (("4  Q4 simulation + vision", "4  Q4 仿真 + 视觉", "on this Mac", "在 Mac 上"), [
        A("rl_world", "mac_window", [MJPY, "rl_gym_go2_sim.py", "--view", "--frame", "world"],
          ("4-1  rl_gym: original (falls)", "unitree_rl_gym's Go2 policy, given the velocity in WORLD coordinates "
           "as deploy_go2.py does.", "Walks fine, then falls soon after the turn starts (about 7 s).",
           "After a turn the world 'forward' is not the dog's 'forward', so the policy over-corrects."),
          ("4-1  rl_gym 原版(会摔)", "unitree_rl_gym 的 Go2 网络,按原脚本把'世界坐标系'的速度给它。",
           "直走没事,大约第 7 秒开始转弯后很快摔倒。", "转弯后地图上的'前'不再是狗的'前',网络拼命纠正,结果摔倒。"),
          cwd="q4_sim", say="The deploy script used world-frame velocity; the policy was trained on body-frame velocity."),
        A("rl_body", "mac_window", [MJPY, "rl_gym_go2_sim.py", "--view"],
          ("4-2  rl_gym: fixed", "The same policy with the velocity turned into the dog's own body frame.",
           "It turns smoothly and never falls.", "One coordinate rotation fixes it."),
          ("4-2  rl_gym 修正版", "同一个网络,速度换成狗自己的坐标系。", "转弯很稳,一直不倒。", "一个坐标变换就修好了。"),
          cwd="q4_sim"),
        A("eye", "mac", ["open", "q4_sim/data/sim_run/front_camera.mp4", "q4_sim/data/sim_run/third_person.mp4"],
          ("4-3  The dog's eye (videos)", "Opens the dog's front-camera video and a follow camera.",
           "Soil, weeds, boxes and a ball; the dog walks, turns fast and sidesteps.",
           "Every frame also has the true label and depth of each pixel, which a real camera cannot give."),
          ("4-3  狗的眼睛(视频)", "打开狗的前置摄像头视频和跟拍视频。", "土、杂草、盒子、球;狗走、快速转弯、横走。",
           "每一帧还存了每个像素的真实标签和深度,真实摄像头给不了。"),
          cwd="."),
        A("vision", "mac", [PY, "-W", "ignore", "-u", "-c",
                            f"import vision_pipeline as v; v.OUT='{PRACTICE_OUT}'; v.main()"],
          ("4-4  Run the five vision methods (30 s)", "Runs color space, pyramid, optical flow, GrabCut and "
           "supervised learning on the 170 synthetic frames, graded against the simulator's ground truth.",
           "[1] Lab-a separates weed from soil perfectly (1.000), G only 0.741. [2] Fast turn: 0% without a "
           "pyramid, 30% with. [3] DIS about 2 px error, Farneback fails on weak texture. [4] GrabCut 0.88-0.98, "
           "weed 0.67 (the errors are shadows). [5] Six classifiers all near F1 0.99.",
           f"Figures go to {PRACTICE_OUT}; the report's files are not changed."),
          ("4-4  跑五个视觉方法(30 秒)", "在 170 帧合成数据上跑颜色空间、金字塔、光流、GrabCut、监督学习,用标准答案打分。",
           "[1] Lab-a 分杂草和土满分 1.000,G 只有 0.741。[2] 快速转弯:不用金字塔 0%,用了 30%。"
           "[3] DIS 误差约 2 像素,Farneback 在弱纹理上失效。[4] GrabCut 0.88-0.98,杂草 0.67(错的都是影子)。"
           "[5] 6 个分类器 F1 都约 0.99。", f"结果存到 {PRACTICE_OUT},不会改动作业文件。"),
          cwd="q4_sim", after=["open", PRACTICE_OUT],
          say="In simulation I know the exact depth and camera pose, so I can grade optical flow against the truth."),
        A("report", "mac", ["open", "ENGR6010_FirstTest_HarryHe.pdf"],
          ("Open the report (PDF)", "Opens the First Test report.", "Pages 1-3 Q1 review, pages 4-10 technical note.", ""),
          ("打开报告 PDF", "打开 First Test 报告。", "第 1-3 页 Q1 综述,第 4-10 页技术报告。", ""), cwd="."),
    ]),
]

NOISE = ("multicast", "p_gains", "Warning", "warn(", "warnings.warn", "torch.jit.load", "ret = a @ b")
UI = {  # small interface texts
    "title": ("Go2 Control Panel", "机器狗控制面板"),
    "tagline": ("Robot = nerves (Q2: computer <-> dog messages) + cerebellum (Q3/Q4: learned walking) + eyes (Q4: vision)",
                "机器狗 = 神经(Q2:电脑和狗通信)+ 小脑(Q3/Q4:学会走路)+ 眼睛(Q4:视觉)"),
    "allow": ("Allow motion: the instructor is here and 2 m around the dog is clear",
              "允许运动:老师在场,狗周围 2 米没人没东西"),
    "stopall": ("Close programs opened by this panel", "关掉面板打开的程序"),
    "output": ("Program output", "程序输出"),
    "clear": ("Clear", "清空"),
    "where_robot": ("Runs on: this Mac -> real dog, by the connection chosen at the top of card 1",
                    "在哪跑:这台 Mac → 真狗,走卡片 1 顶上选的连接方式"),
    "where_vm": ("Runs in: the Ubuntu VM", "在哪跑:Ubuntu 虚拟机"),
    "where_host": ("Runs on: this Mac (controls the VM)", "在哪跑:这台 Mac(控制虚拟机)"),
    "where_mac": ("Runs on: this Mac", "在哪跑:这台 Mac"),
    "moves": ("This moves the real robot.", "这会让真狗动起来。"),
    "route": ("Connection:", "连接方式:"),
    "route_h": ("Connection to the real dog", "连真狗的方式"),
    "route_txt": ("Cable (default): Python on this Mac talks to the dog over the USB-Ethernet cable with the official "
                  "Unitree SDK. Needs a Go2 EDU. The Mac's Wi-Fi stays free for the internet.\n\n"
                  "Wi-Fi: the Mac joins the dog's own Wi-Fi and talks to it the way the phone app does (WebRTC). "
                  "Works on Go2 AIR, PRO and EDU. Close the phone app first: the dog takes one Wi-Fi client at a time, "
                  "so click one button at a time and wait for 'done'. Firmware 1.1.15 or newer needs the dog's AES key "
                  "in lab/.go2_aes_key. On the dog's Wi-Fi the Mac has no internet, unless the iPhone is connected by "
                  "USB with Personal Hotspot on.\n\n"
                  "VM (backup): the cable scripts run inside Ubuntu. Only if the cable route does not work; 1-1 then "
                  "restarts the VM once to give it a network card on the cable.",
                  "网线(默认):Mac 上的 Python 用 Unitree 官方 SDK,通过 USB 转网口的网线和狗通信。需要 Go2 EDU。"
                  "Mac 的 Wi-Fi 空着,可以继续上网。\n\n"
                  "Wi-Fi:Mac 连上狗自己的 Wi-Fi,用手机 App 同样的方式(WebRTC)和狗通信。AIR、PRO、EDU 都行。"
                  "先关掉手机 App:狗一次只接受一个 Wi-Fi 连接,所以一次只点一个按钮,等显示 'done' 再点下一个。"
                  "固件 1.1.15 或更新需要狗的 AES 密钥(放在 lab/.go2_aes_key)。连着狗的 Wi-Fi 时 Mac 没有外网,"
                  "除非用数据线连 iPhone 并打开个人热点。\n\n"
                  "虚拟机(备用):网线那套脚本在 Ubuntu 里跑。只有网线直连不行时才用;这时 1-1 会重启一次虚拟机,"
                  "给它加一张接网线的网卡。"),
    "what": ("What it does:  ", "在做什么:"), "see": ("You will see:  ", "你会看到:"), "why": ("Why:  ", "原理:"),
    "say": ("If asked, you can say:", "老师问起来可以说:"),
    "need_allow": ("Tick 'Allow motion' first.", "请先勾选 '允许运动'。"),
    "confirm_motion": ("The dog will MOVE: {label}.\n\nIs the instructor here and is the area 2 m around the dog clear?",
                       "狗要动了:{label}。\n\n老师在场吗?狗周围 2 米内没有人和东西吗?"),
}


# --- look ---------------------------------------------------------------------------------------
# Every color is set here by hand. Left to the system, macOS dark mode turns the default text white,
# which is unreadable on the light cards.
C = dict(
    bg="#EDF0F4", card="#FFFFFF", line="#D5DAE1", text="#1B1F24", muted="#5F6873",
    btn="#F3F5F8", btn_hover="#E3E8EF", btn_down="#D3DAE4",
    accent="#1F5FBF", accent_soft="#E8F0FC",
    move="#FFF1DC", move_hover="#FFE2B8", move_text="#7A4100",
    off="#F1F2F4", off_text="#6B7480",
    danger="#D92D20", danger_hover="#B42318", danger_soft="#FDECEA", danger_soft_hover="#F9D6D1",
    ok="#12A150", warn="#B86E00", err="#D92D20", idle="#8A94A0",
    say="#1A7F37", console="#111827", console_text="#E5E7EB",
)
# style -> (background, background under the mouse, text color, number color)
BTN = {
    "normal": (C["btn"], C["btn_hover"], C["text"], C["accent"]),
    "move": (C["move"], C["move_hover"], C["move_text"], C["move_text"]),
    "danger": (C["danger"], C["danger_hover"], "#FFFFFF", "#FFFFFF"),
    "danger_soft": (C["danger_soft"], C["danger_soft_hover"], C["danger_hover"], C["danger_hover"]),
}


def inside(widget, other):
    """Is `other` the widget itself or one of its children?"""
    if other is None:
        return False
    w, o = str(widget), str(other)
    return o == w or o.startswith(w + ".")


class FlatButton(tk.Frame):
    """A button drawn with labels, so it looks the same in light and dark mode (native Mac buttons ignore colors).
    A label like "1-5  Stand up" is shown as a small step number plus the text."""

    def __init__(self, parent, command, style="normal", size=13, bold=False, pady=7):
        super().__init__(parent, bg=C["line"], padx=1, pady=1, cursor=HAND)
        self.command, self.style, self.enabled, self.hover = command, style, True, False
        self.body = tk.Frame(self, cursor=HAND)
        self.body.pack(fill="both", expand=True)
        self.num = tk.Label(self.body, font=(FONT, size - 1, "bold"), anchor="w", cursor=HAND)
        self.text = tk.Label(self.body, font=(FONT, size, "bold" if bold else "normal"), anchor="w",
                             justify="left", wraplength=310, cursor=HAND)
        self.text.pack(side="left", fill="x", expand=True, padx=(12, 10), pady=pady)
        for w in (self, self.body, self.num, self.text):
            w.bind("<Enter>", self._enter)
            w.bind("<Leave>", self._leave)
            w.bind("<ButtonPress-1>", self._press)
            w.bind("<ButtonRelease-1>", self._release)
        self._paint()

    def set_label(self, label):
        num, _, rest = label.partition("  ")
        if rest and (num[0].isdigit() or num == "■"):
            self.num.config(text=num, width=4 if num[0].isdigit() else 2)
            self.num.pack(side="left", padx=(12, 0), before=self.text)
            self.text.config(text=rest)
            self.text.pack_configure(padx=(4, 10))
        else:
            self.num.pack_forget()
            self.text.config(text=label)
            self.text.pack_configure(padx=(12, 10))

    def set_enabled(self, on):
        self.enabled = on
        for w in (self, self.body, self.num, self.text):
            w.config(cursor=HAND if on else "")
        self._paint()

    def _paint(self, down=False):
        bg, hover, fg, num = BTN[self.style]
        if not self.enabled:
            bg, fg, num = C["off"], C["off_text"], C["off_text"]
        elif down:
            bg = C["btn_down"] if self.style == "normal" else hover
        elif self.hover:
            bg = hover
        for w in (self.body, self.num, self.text):
            w.config(bg=bg)
        self.text.config(fg=fg)
        self.num.config(fg=num)

    def _enter(self, _):
        self.hover = True
        self._paint()

    def _leave(self, _):
        x, y = self.winfo_pointerxy()
        self.hover = inside(self, self.winfo_containing(x, y))
        self._paint()

    def _press(self, _):
        if self.enabled:
            self._paint(down=True)

    def _release(self, _):
        x, y = self.winfo_pointerxy()
        self._paint()
        if inside(self, self.winfo_containing(x, y)):  # run() explains why a greyed-out button does nothing
            self.command()


class Toggle(tk.Frame):
    """A check box drawn with labels (same reason as FlatButton). get() / set() like a BooleanVar."""

    def __init__(self, parent, command, on_bg, on_fg, size=12, bg=C["card"]):
        super().__init__(parent, cursor=HAND)
        self.command, self.value, self.bg = command, False, bg
        self.on_bg, self.on_fg = on_bg, on_fg
        self.box = tk.Canvas(self, width=18, height=18, highlightthickness=0, bd=0, cursor=HAND)
        self.box.pack(side="left", padx=(8, 6), pady=4)
        self.text = tk.Label(self, font=(FONT, size), anchor="w", justify="left", wraplength=320, cursor=HAND)
        self.text.pack(side="left", fill="x", expand=True, padx=(0, 8), pady=4)
        for w in (self, self.box, self.text):
            w.bind("<ButtonRelease-1>", self._click)
        self._paint()

    def get(self):
        return self.value

    def set(self, value):
        self.value = bool(value)
        self._paint()

    def set_label(self, text):
        self.text.config(text=text)

    def _click(self, _):
        x, y = self.winfo_pointerxy()
        if inside(self, self.winfo_containing(x, y)):
            self.set(not self.value)
            self.command()

    def _paint(self):
        bg, fg = (self.on_bg, self.on_fg) if self.value else (self.bg, C["muted"])
        for w in (self, self.box, self.text):
            w.config(bg=bg)
        self.box.delete("all")
        if self.value:  # filled box with a white tick
            self.box.create_rectangle(1, 1, 17, 17, fill=self.on_fg, outline=self.on_fg)
            self.box.create_line(4, 9, 8, 13, 14, 5, fill="#FFFFFF", width=2)
        else:
            self.box.create_rectangle(1, 1, 17, 17, fill="#FFFFFF", outline=C["muted"], width=1.5)
        self.text.config(fg=fg)


class Segmented(tk.Frame):
    """A row of options where exactly one is chosen, like the EN / 中文 switch. options = [(code, English, 中文)]."""

    def __init__(self, parent, options, command):
        super().__init__(parent, bg=C["line"], padx=1, pady=1)
        self.command, self.value, self.labels = command, options[0][0], {}
        for code, en, zh in options:
            b = tk.Label(self, font=(FONT, 12, "bold"), padx=10, pady=3, cursor=HAND)
            b.pack(side="left", padx=(0, 1))
            b.bind("<ButtonRelease-1>", lambda e, c=code, b=b: self._release(c, b))
            self.labels[code] = (b, en, zh)
        self._paint()

    def get(self):
        return self.value

    def set_lang(self, lang):
        for b, en, zh in self.labels.values():
            b.config(text=en if lang == "en" else zh)

    def _release(self, code, b):
        x, y = b.winfo_pointerxy()
        if inside(b, b.winfo_containing(x, y)) and code != self.value:
            self.value = code
            self._paint()
            self.command()

    def _paint(self):
        for code, (b, _, _) in self.labels.items():
            on = code == self.value
            b.config(bg=C["accent"] if on else C["card"], fg="#FFFFFF" if on else C["text"])


class Chip(tk.Frame):
    """A status light: a colored dot and a short text."""

    def __init__(self, parent):
        super().__init__(parent, bg=C["line"], padx=1, pady=1)
        inner = tk.Frame(self, bg=C["card"])
        inner.pack()
        self.dot = tk.Label(inner, text="●", font=(FONT, 12), bg=C["card"], fg=C["idle"])
        self.dot.pack(side="left", padx=(10, 3), pady=3)
        self.text = tk.Label(inner, font=(FONT, 12), bg=C["card"], fg=C["text"])
        self.text.pack(side="left", padx=(0, 10), pady=3)

    def set(self, color, text):
        self.dot.config(fg=color)
        self.text.config(text=text)


def card(parent, **pack):
    """A white box with a thin border. Returns the inside frame."""
    outer = tk.Frame(parent, bg=C["line"], padx=1, pady=1)
    outer.pack(**pack)
    inner = tk.Frame(outer, bg=C["card"], padx=14, pady=12)
    inner.pack(fill="both", expand=True)
    return inner


INTRO = [  # (text style, English, Chinese) for the start page
    ("h", "How to use\n", "怎么用\n"),
    (None, "A click on the left runs that button right away. This card then explains it, and the dark box below "
           "shows what the program prints.\n\n",
     "点左边的按钮会马上运行;这里解释它在做什么,下面的黑框是程序的输出。\n\n"),
    ("k", "In the lab (real dog)\n", "在实验室(真狗)\n"),
    ("step", "1   Choose Cable or Wi-Fi at the top of card 1. Cable: plug it into the Mac and the dog. Wi-Fi: join "
             "the dog's Wi-Fi and close the phone app. Turn the dog on.\n"
             "2   1-1 Connect to the dog, then 1-2 Read robot state.\n"
             "3   With the instructor: tick 'Allow motion', then 1-5 -> 1-6 -> 1-7 -> 1-9.\n\n",
     "1   在卡片 1 顶上选网线或 Wi-Fi。网线:插上 Mac 和狗。Wi-Fi:连上狗的 Wi-Fi,关掉手机 App。狗开机。\n"
     "2   1-1 连上狗,再 1-2 读状态。\n"
     "3   老师在场时:勾选 '允许运动',再按 1-5 → 1-6 → 1-7 → 1-9。\n\n"),
    ("k", "At home\n", "在家\n"),
    ("step", "First 'Start the VM' and wait for 'VM on' at the top. Then 2-1 and 2-3 show the simulated dog in the "
             "UTM window. 3-1 to 4-4 do not need the VM and open their own window on this Mac.\n\n",
     "先点「启动虚拟机」,等顶上显示 '虚拟机 已连接',再点 2-1、2-3,仿真狗在 UTM 窗口里。"
     "3-1 到 4-4 不用虚拟机,会在 Mac 上开自己的窗口。\n\n"),
    ("k", "Closing\n", "关闭\n"),
    ("step", "Closing this window closes everything it opened, the VM and UTM included. Nothing starts by itself "
             "when you open it again.\n\n",
     "关掉这个窗口,它打开过的东西(包括虚拟机和 UTM)都会一起关掉。再打开时什么都不会自己启动。\n\n"),
    ("warn", "The red STOP (top right) is always there. The remote (L2 + B) also stops the dog.",
     "右上角红色的「停止」随时能按。遥控器(L2 + B)也能停狗。"),
]


class Panel:
    def __init__(self, root):
        self.root = root
        self.lang = "en"
        self.q = queue.Queue()
        self.procs = []
        self.buttons = {}  # key -> list of (FlatButton, action); STOP has two buttons
        self.current = None
        self.status = {"vm": ("?", None), "sim": None, "robot": ("?", "")}
        self.logfile = os.path.join(LOCAL_DATA, "logs", time.strftime("panel_%Y%m%d.log"))  # a copy of the dark box
        try:  # always draw this window in light mode, whatever the system setting
            root.tk.call("::tk::unsupported::MacWindowStyle", "appearance", root, "aqua")
        except tk.TclError:
            pass
        w, h = min(1400, root.winfo_screenwidth() - 60), min(920, root.winfo_screenheight() - 90)
        root.geometry(f"{w}x{h}")
        root.minsize(1240, 680)
        root.configure(bg=C["bg"])
        icon = os.path.join(HERE, "tools", "panel_icon.png")  # the dog icon, also in the Dock
        if os.path.exists(icon):
            self.icon = tk.PhotoImage(file=icon)
            root.iconphoto(True, self.icon)
        actions = {a["key"]: a for _, items in ACTIONS for a in items}

        # ---- top bar: title, language, status lights, STOP --------------------------------------
        top = tk.Frame(root, bg=C["card"])
        top.pack(fill="x")
        tk.Frame(root, bg=C["line"], height=1).pack(fill="x")
        row = tk.Frame(top, bg=C["card"])  # row 1: title, language, status lights, STOP; row 2: tagline
        row.pack(fill="x", padx=20, pady=(12, 0))
        self.title_lbl = tk.Label(row, font=(FONT, 21, "bold"), bg=C["card"], fg=C["text"])
        self.title_lbl.pack(side="left")
        tk.Label(row, text="ENGR 6010 · First Test", font=(FONT, 12), bg=C["card"], fg=C["muted"]).pack(
            side="left", padx=(12, 16), pady=(6, 0))
        seg = tk.Frame(row, bg=C["line"], padx=1, pady=1)
        seg.pack(side="left", pady=(4, 0))
        self.lang_btns = {}
        for code, name in (("en", "EN"), ("zh", "中文")):
            b = tk.Label(seg, text=name, font=(FONT, 12, "bold"), padx=12, pady=2, cursor=HAND)
            b.pack(side="left")
            b.bind("<ButtonRelease-1>", lambda e, c=code: self.set_lang(c))
            self.lang_btns[code] = b
        stop = actions["m_stop"]
        self.top_stop = FlatButton(row, lambda: self.run(stop), style="danger", size=16, bold=True, pady=8)
        self.top_stop.pack(side="right", padx=(10, 0))
        self.buttons["m_stop"] = [(self.top_stop, stop)]
        chips = tk.Frame(row, bg=C["card"])
        chips.pack(side="right")
        self.chip = {}
        for k in ("vm", "sim", "robot"):
            self.chip[k] = Chip(chips)
            self.chip[k].pack(side="left", padx=4)
        self.tag_lbl = tk.Label(top, font=(FONT, 12), bg=C["card"], fg=C["muted"], anchor="w")
        self.tag_lbl.pack(fill="x", padx=20, pady=(2, 12))

        # ---- body: buttons on the left, explanation and output on the right ----------------------
        body = tk.Frame(root, bg=C["bg"])
        body.pack(fill="both", expand=True, padx=16, pady=14)
        left_outer = tk.Frame(body, bg=C["bg"])
        left_outer.pack(side="left", fill="y")
        self.canvas = tk.Canvas(left_outer, width=430, bg=C["bg"], highlightthickness=0, bd=0, yscrollincrement=15)
        sb = tk.Scrollbar(left_outer, orient="vertical", command=self.canvas.yview)
        left = tk.Frame(self.canvas, bg=C["bg"])
        win = self.canvas.create_window((0, 0), window=left, anchor="nw")
        left.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(win, width=e.width))
        self.canvas.configure(yscrollcommand=sb.set)
        self.canvas.pack(side="left", fill="y")
        sb.pack(side="left", fill="y", padx=(4, 0))
        self.left_outer = left_outer
        for ev in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            root.bind_all(ev, self.wheel)

        self.section_lbls = []
        for (t_en, t_zh, s_en, s_zh), items in ACTIONS:
            box = card(left, fill="x", pady=(0, 12))
            title = tk.Label(box, font=(FONT, 15, "bold"), bg=C["card"], fg=C["text"], anchor="w")
            title.pack(fill="x")
            sub = tk.Label(box, font=(FONT, 11), bg=C["card"], fg=C["muted"], anchor="w", justify="left",
                           wraplength=390)
            sub.pack(fill="x", pady=(0, 8))
            self.section_lbls.append((title, sub, t_en, t_zh, s_en, s_zh))
            if items[0]["key"] == "r_net":
                row = tk.Frame(box, bg=C["card"])
                row.pack(fill="x", pady=(0, 8))
                self.route_lbl = tk.Label(row, font=(FONT, 11), bg=C["card"], fg=C["muted"])
                self.route_lbl.pack(side="left", padx=(0, 8))
                self.route_seg = Segmented(row, ROUTES, self.route_changed)
                self.route_seg.pack(side="left")
            for a in items:
                if a["key"] == "m_standup":
                    tk.Frame(box, bg=C["line"], height=1).pack(fill="x", pady=(10, 8))
                    self.allow = Toggle(box, self.refresh_motion, on_bg=C["move"], on_fg=C["move_text"], size=12)
                    self.allow.pack(fill="x", pady=(0, 6))
                style = ("danger" if a["key"] == "m_stop" else "danger_soft" if a["key"] == "m_damp"
                         else "move" if a.get("motion") else "normal")
                b = FlatButton(box, lambda a=a: self.run(a), style=style, bold=a["key"] == "m_stop")
                b.pack(fill="x", pady=2)
                self.buttons.setdefault(a["key"], []).append((b, a))
        self.stop_btn = FlatButton(left, self.stop_all, size=12)
        self.stop_btn.pack(fill="x", pady=(0, 12))
        self.route = "cable"  # plain copy of the connection switch for the status thread

        right = tk.Frame(body, bg=C["bg"])
        right.pack(side="left", fill="both", expand=True, padx=(16, 0))
        ebox = card(right, fill="x")
        self.explain = tk.Text(ebox, height=13, wrap="word", font=(FONT, 14), relief="flat", bd=0,
                               bg=C["card"], fg=C["text"], highlightthickness=0, padx=4, pady=2,
                               spacing1=2, spacing3=4, cursor="arrow")
        self.explain.pack(fill="x")
        self.explain.tag_configure("h", font=(FONT, 19, "bold"), foreground=C["text"], spacing3=6)
        self.explain.tag_configure("where", font=(FONT, 12), foreground=C["muted"], spacing3=10)
        self.explain.tag_configure("k", font=(FONT, 14, "bold"), foreground=C["accent"])
        self.explain.tag_configure("en", font=(FONT, 14, "italic"), foreground=C["say"], lmargin1=14, lmargin2=14)
        self.explain.tag_configure("warn", font=(FONT, 14, "bold"), foreground=C["danger"])
        self.explain.tag_configure("step", lmargin1=14, lmargin2=34)

        obox = card(right, fill="both", expand=True, pady=(12, 0))
        orow = tk.Frame(obox, bg=C["card"])
        orow.pack(fill="x", pady=(0, 8))
        self.out_lbl = tk.Label(orow, font=(FONT, 14, "bold"), bg=C["card"], fg=C["text"])
        self.out_lbl.pack(side="left")
        self.clear_btn = FlatButton(orow, lambda: self.log.delete("1.0", "end"), size=11, pady=2)
        self.clear_btn.pack(side="right")
        self.log = scrolledtext.ScrolledText(obox, wrap="word", font=(MONO, 12), bg=C["console"],
                                             fg=C["console_text"], insertbackground=C["console_text"],
                                             relief="flat", bd=0, highlightthickness=0, padx=12, pady=10)
        self.log.configure(selectbackground="#3A4A66", inactiveselectbackground="#3A4A66", selectforeground="#FFFFFF")
        self.log.pack(fill="both", expand=True)
        for tag, col in (("info", "#7DD3FC"), ("send", "#FBBF24"), ("recv", "#67E8F9"), ("state", "#86EFAC"),
                         ("bad", "#FCA5A5")):
            self.log.tag_configure(tag, foreground=col)
        self.apply_lang()
        self.root.after(60, self.pump)
        threading.Thread(target=self.watch, daemon=True).start()
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.createcommand("::tk::mac::Quit", self.quit)  # Cmd+Q and Dock > Quit

    def wheel(self, e):
        """Mouse wheel / trackpad: scroll the button list, but only while the pointer is over it."""
        x, y = self.root.winfo_pointerxy()
        if not inside(self.left_outer, self.root.winfo_containing(x, y)):
            return
        if e.num in (4, 5):  # Linux
            step = -3 if e.num == 4 else 3
        elif sys.platform == "darwin":
            if e.state & 0x1:  # Shift is set for a sideways trackpad swipe
                return
            step = -e.delta
        else:
            step = -(e.delta // 120) * 3
        self.canvas.yview_scroll(step, "units")

    # ------------------------------------------------------------------ language
    def t(self, key):
        en, zh = UI[key]
        return en if self.lang == "en" else zh

    def set_lang(self, lang):
        self.lang = lang
        self.apply_lang()

    def apply_lang(self):
        self.root.title(self.t("title") + " - ENGR 6010")
        self.title_lbl.config(text=self.t("title"))
        self.tag_lbl.config(text=self.t("tagline"))
        for code, b in self.lang_btns.items():
            on = code == self.lang
            b.config(bg=C["accent"] if on else C["card"], fg="#FFFFFF" if on else C["text"])
        for title, sub, t_en, t_zh, s_en, s_zh in self.section_lbls:
            title.config(text=t_en if self.lang == "en" else t_zh)
            sub.config(text=s_en if self.lang == "en" else s_zh)
        for pairs in self.buttons.values():
            for b, a in pairs:
                b.set_label(a[self.lang]["label"])
        self.top_stop.set_label("■  " + ("STOP" if self.lang == "en" else "停止"))
        self.allow.set_label(self.t("allow"))
        self.route_lbl.config(text=self.t("route"))
        self.route_seg.set_lang(self.lang)
        self.stop_btn.set_label(self.t("stopall"))
        self.clear_btn.set_label(self.t("clear"))
        self.out_lbl.config(text=self.t("output"))
        self.refresh_motion()
        self.show_status()
        if self.current:
            self.show(self.current)
        else:
            self.show_intro()

    def refresh_motion(self):
        for pairs in self.buttons.values():
            for b, a in pairs:
                if a.get("motion"):
                    b.set_enabled(self.allow.get())

    def route_changed(self):
        self.route = self.route_seg.get()
        self.current = None
        self.fill([("h", self.t("route_h") + "\n"), (None, self.t("route_txt"))])

    def fill(self, parts):
        e = self.explain
        e.config(state="normal", wrap="char" if self.lang == "zh" else "word")
        e.delete("1.0", "end")
        for tag, text in parts:
            e.insert("end", text, tag)
        e.config(state="disabled")

    def show_intro(self):
        self.fill([(tag, text) for tag, en, zh in INTRO for text in [en if self.lang == "en" else zh]])

    def show(self, a):
        self.current = a
        d = a[self.lang]
        where = ("robot" if a["where"] == "robot" else "vm" if a["where"] == "vm" else
                 "host" if a["where"] == "host" else "mac")
        parts = [("h", d["label"].strip() + "\n"), ("where", self.t("where_" + where) + "\n")]
        for k in ("what", "see", "why"):
            if d[k]:
                parts += [("k", self.t(k)), (None, d[k] + "\n")]
        if a.get("motion"):
            parts.append(("warn", "\n" + self.t("moves") + "\n"))
        if a.get("say"):
            parts += [("k", "\n" + self.t("say") + "\n"), ("en", a["say"] + "\n")]
        self.fill(parts)

    def write(self, line, tag=None):
        if tag is None:
            tag = ("send" if line.startswith("[sent]") else "recv" if line.startswith("[received]")
                   else "state" if line.startswith(("[dog]", "[狗的状态]", "[robot]")) else None)
        self.log.insert("end", line + "\n", tag)
        self.log.see("end")
        try:
            os.makedirs(os.path.dirname(self.logfile), exist_ok=True)
            with open(self.logfile, "a", encoding="utf-8") as f:
                f.write(time.strftime("%H:%M:%S ") + line + "\n")
        except OSError:
            pass

    # ------------------------------------------------------------------ running
    def ssh_base(self):
        ip = open(os.path.join(VM_DIR, "vm_ip.txt")).read().strip()
        return ip, ["-i", os.path.join(VM_DIR, "vm_key"), "-o", "StrictHostKeyChecking=no",
                    "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=4",
                    "-o", "ServerAliveInterval=15"]

    def vm_cmd(self, remote):
        ip, opts = self.ssh_base()
        return ["ssh", *opts, f"harry@{ip}", VM_ENV + remote]

    def sync_to_vm(self):
        """Copy the Mac's copy of the robot scripts into the VM; the Mac is the only place to edit them."""
        ip, opts = self.ssh_base()
        n = 0
        for src, dst in ((os.path.join(HERE, "q2_sdk", "sim_examples"), "q2/sim_examples"),
                         (os.path.join(HERE, "lab"), "q2/lab")):
            files = [os.path.join(src, f) for f in sorted(os.listdir(src)) if f.endswith((".py", ".sh"))]
            subprocess.run(["ssh", *opts, f"harry@{ip}", f"mkdir -p ~/{dst}"], capture_output=True, timeout=10)
            r = subprocess.run(["scp", "-q", *opts, *files, f"harry@{ip}:~/{dst}/"], capture_output=True, timeout=20)
            n += len(files) if r.returncode == 0 else 0
        return n

    def run(self, a):
        self.show(a)
        label = a[self.lang]["label"].strip()
        if a.get("motion") and not self.allow.get():
            self.write(self.t("need_allow"), "bad")
            return
        if a.get("motion") or a.get("confirm"):
            text = a["confirm"][0 if self.lang == "en" else 1] if a.get("confirm") else \
                self.t("confirm_motion").format(label=label)
            if not messagebox.askyesno(label, text, icon="warning", default="no", parent=self.root):
                return
        self.write(f"\n▶ {label}", "info")
        if a["where"] == "robot":
            threading.Thread(target=self.run_robot, args=(a, label, self.route), daemon=True).start()
            return
        if a["where"] == "mac_window":
            for p in self.procs:
                if p.kind == "mac_window" and p.poll() is None:
                    p.terminate()
        if a["where"] == "vm":
            if a.get("show_vm"):  # the simulator draws on the Ubuntu desktop, which is the UTM window
                subprocess.Popen(["osascript", "-e", 'tell application "UTM" to activate'])
                self.write("  -> the dog is in the UTM window (Ubuntu desktop), now in front" if self.lang == "en"
                           else "  -> 狗在 UTM 窗口(Ubuntu 桌面)里,已经放到最前面", "info")
            # copying the scripts can take seconds when the VM is slow or off: do it off the window's thread
            threading.Thread(target=self.run_vm, args=(a, label), daemon=True).start()
            return
        self.start(a["cmd"], os.path.join(HERE, a.get("cwd", ".")), a, label, after=a.get("after"))

    def run_vm(self, a, label):
        if a["key"] != "vm_stop":
            try:
                n = self.sync_to_vm()
            except (OSError, subprocess.SubprocessError):
                n = 0
            self.q.put(("line", f"  synced {n} scripts from the Mac into the VM" if n else
                        "  could not reach the VM - is it running? / 连不上虚拟机:开了吗?", "info" if n else "bad"))
        self.start(self.vm_cmd(a["cmd"]), HERE, a, label, after=a.get("after"))

    def start(self, cmd, cwd, a, label, after=None, fetch=None, limit=None, wifi=False):
        """Start a program and stream what it prints into the black box. Safe to call from any thread."""
        try:
            p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        except OSError as err:
            self.q.put(("line", f"✗ could not start: {err}", "bad"))
            return
        p.kind, p.key, p.wifi = a["where"], a["key"], wifi
        self.procs.append(p)
        if limit:  # end it after `limit` seconds (SIGTERM; a walk command still sends StopMove)
            timer = threading.Timer(limit, lambda: p.poll() is None and p.terminate())
            timer.daemon = True  # does not keep the program alive after the window is closed
            timer.start()
        threading.Thread(target=self.reader, args=(p, label, after, fetch), daemon=True).start()

    def run_robot(self, a, label, route):
        """Real-robot buttons over the cable (default), over Wi-Fi, or through the Ubuntu VM (backup)."""
        key = a["key"]
        bad = lambda text: self.q.put(("line", text, "bad"))
        if key == "r_net":
            target = wifi_target()
            cmd = {"cable": ["bash", "lab/mac_net_up.sh"], "vm": ["bash", "lab/add_robot_nic.sh"],
                   "wifi": ["bash", "lab/wifi_check.sh"] + ([] if target == "ap" else [target])}[route]
            self.start(cmd, HERE, a, label)
            return
        if route == "wifi":
            self.run_wifi(a, label)
            return
        if route == "vm":
            if key != "m_stop":
                try:
                    n = self.sync_to_vm()
                except (OSError, subprocess.SubprocessError):
                    n = 0
                self.q.put(("line", f"  synced {n} scripts from the Mac into the VM" if n else
                            "  could not reach the VM - is it running? / 连不上虚拟机:开了吗?", "info" if n else "bad"))
            if key == "m_stop":
                remote = (ROBOT_IF + "cd ~/q2/lab; S=real_sport; "
                          # first end any motion command still running (the pattern does not match this line)
                          "pkill -f '[r]eal_sport.py .* (standup|balance|move)' && echo 'ended the running motion command'; "
                          "timeout 15 python3 -u ${S}.py $IFACE stop --yes")
                fetch = None
            else:
                script, *args = a["run"]
                args = " ".join(args).replace("{photo}", "/tmp/go2_photo.jpg").replace(
                    "{run}", "~/q2/lab/data/real_run_$(date +%H%M%S)")
                remote = robot(f"rm -f /tmp/go2_photo.jpg; timeout {a['limit']} python3 -u {script} $IFACE {args}")
                fetch = (("/tmp/go2_photo.jpg", "photos", True) if "{photo}" in a["run"] else
                         ("~/q2/lab/data/", "", False) if "{run}" in a["run"] else None)
            self.start(self.vm_cmd(remote), HERE, a, label, fetch=fetch)
            return
        # straight from the Mac
        if not os.path.exists(ROBOT_PY):
            bad("The Mac robot environment is missing: run  bash lab/setup_mac_sdk.sh  (or tick the VM route).")
            return
        iface = mac_robot_iface()
        if key == "m_stop":
            # end motion commands still running; a walk sends StopMove on the way out, and a command that has
            # not gone out yet is not sent at all (real_sport.py checks for STOP before sending)
            for p in self.procs:
                if getattr(p, "key", "") in ("m_standup", "m_balance", "m_walk", "m_turn") and p.poll() is None:
                    p.terminate()
                    self.q.put(("line", "  ended the running motion command", "info"))
            subprocess.run(["pkill", "-f", "[r]eal_sport.py .* (standup|balance|move)"])
            if not iface:
                bad("The Mac is not connected to the dog (no 192.168.123.x card). Use the remote: L2 + B. "
                    "/ Mac 没连上狗,用遥控器 L2 + B 停狗。")
                return
            self.start([ROBOT_PY, "-u", os.path.join(LAB, "real_sport.py"), iface, "stop", "--yes"], HERE, a, label,
                       limit=15)
            return
        if not iface:
            bad("The Mac is not connected to the dog yet: click 1-1 Connect to the dog first. "
                "/ Mac 还没连上狗:先点 1-1。")
            return
        if subprocess.run(["ping", "-c", "1", "-t", "1", ROBOT_IP], capture_output=True).returncode != 0:
            bad(f"Cannot reach the robot at {ROBOT_IP} on {iface}: check the cable and that the dog is on. "
                "/ 连不上狗:检查网线和狗有没有开机。")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        photo = os.path.join(LOCAL_DATA, "photos", f"photo_{stamp}.jpg")
        run_dir = os.path.join(LOCAL_DATA, f"real_run_{stamp}")
        os.makedirs(os.path.dirname(photo), exist_ok=True)
        script, *args = a["run"]
        args = [x.replace("{photo}", photo).replace("{run}", run_dir) for x in args]
        after = ["open", photo] if "{photo}" in a["run"] else ["open", run_dir] if "{run}" in a["run"] else None
        self.start([ROBOT_PY, "-u", os.path.join(LAB, script), iface, *args], HERE, a, label, after=after,
                   limit=a["limit"])

    def run_wifi(self, a, label):
        """Real-robot buttons over Wi-Fi (WebRTC, like the phone app). The dog takes one Wi-Fi client at a time."""
        key = a["key"]
        bad = lambda text: self.q.put(("line", text, "bad"))
        if not os.path.exists(WIFI_PY):
            bad("The Wi-Fi environment is missing: run  bash lab/setup_wifi_sdk.sh")
            return
        target = wifi_target()
        busy = [p for p in self.procs if getattr(p, "wifi", False) and p.poll() is None]
        if key == "m_stop":
            # end motion commands still running: a walk sends StopMove over its own connection and hangs up,
            # and a command that has not gone out yet is not sent at all
            for p in busy:
                if p.key in ("m_standup", "m_balance", "m_walk", "m_turn"):
                    p.terminate()
                    self.q.put(("line", "  ended the running motion command", "info"))
            subprocess.run(["pkill", "-f", "[w]ifi_sport.py .* (standup|balance|move)"])
            t0 = time.time()  # give them a moment to hang up, since the dog takes one Wi-Fi client
            while any(p.poll() is None for p in busy) and time.time() - t0 < 6:
                time.sleep(0.1)
            self.start([WIFI_PY, "-u", os.path.join(LAB, "wifi_sport.py"), target, "stop", "--yes"], HERE, a, label,
                       limit=30, wifi=True)
            return
        if busy:
            bad("Wait for the running Wi-Fi command to finish: the dog accepts one Wi-Fi client at a time. "
                "/ 等上一个 Wi-Fi 命令结束:狗一次只接受一个 Wi-Fi 连接。")
            return
        if target == "ap" and not mac_wifi_ip().startswith("192.168.12."):
            bad("The Mac is not on the dog's Wi-Fi: join it in the Wi-Fi menu (top right of the screen), then 1-1. "
                "/ Mac 没连上狗的 Wi-Fi:在屏幕右上角的 Wi-Fi 菜单里连上狗的网络,再点 1-1。")
            return
        if not dog_wifi_port_open(wifi_dog_ip(target)):
            bad(f"The dog does not answer over Wi-Fi at {wifi_dog_ip(target)}: is it on? Click 1-1 to check. "
                "/ 狗没有回应:开机了吗?点 1-1 检查。")
            return
        script = a["run"][0]
        if script not in WIFI_SCRIPT:
            # These read DDS topics straight off the wired bus, and the dog's
            # hotspot does not carry DDS - it carries WebRTC. Without this the
            # KeyError below killed the worker thread and the button printed
            # nothing at all, which is the worst way to say "not supported".
            bad(f"{label}: this one needs the Ethernet cable. It reads DDS topics directly, and the dog's "
                f"Wi-Fi carries WebRTC instead. For the camera and the point cloud over Wi-Fi, use 1-1b. "
                f"/ 这个按钮要插网线:它直接读 DDS 话题,而狗的 Wi-Fi 走的是 WebRTC。"
                f"想在 Wi-Fi 上看摄像头和点云,用 1-1b。")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        photo = os.path.join(LOCAL_DATA, "photos", f"photo_{stamp}.jpg")
        run_dir = os.path.join(LOCAL_DATA, f"wifi_run_{stamp}")
        os.makedirs(os.path.dirname(photo), exist_ok=True)
        args = [x.replace("{photo}", photo).replace("{run}", run_dir) for x in a["run"][1:]]
        after = ["open", photo] if "{photo}" in a["run"] else ["open", run_dir] if "{run}" in a["run"] else None
        # a Wi-Fi command first spends a few seconds connecting, so it gets more time than over the cable
        self.start([WIFI_PY, "-u", os.path.join(LAB, WIFI_SCRIPT[script]), target, *args], HERE, a, label,
                   after=after, limit=a["limit"] + 15, wifi=True)

    def fetch(self, spec):
        """Copy a result from the VM to the Mac. spec = (remote path, local subfolder, is_file)."""
        remote, sub, is_file = spec
        ip, opts = self.ssh_base()
        dest = os.path.join(LOCAL_DATA, sub)
        os.makedirs(dest, exist_ok=True)
        if is_file:
            local = os.path.join(dest, time.strftime("photo_%Y%m%d_%H%M%S.jpg"))
            r = subprocess.run(["scp", "-q", *opts, f"harry@{ip}:{remote}", local], capture_output=True, timeout=30)
            if r.returncode == 0:
                subprocess.Popen(["open", local])
                return f"  saved {os.path.relpath(local, HERE)}"
        else:
            r = subprocess.run(["scp", "-q", "-r", *opts, f"harry@{ip}:{remote}.", dest], capture_output=True,
                               timeout=120)
            if r.returncode == 0:
                subprocess.Popen(["open", dest])
                return f"  copied to {os.path.relpath(dest, HERE)}/"
        return "  could not copy the result from the VM"

    def reader(self, p, label, after, fetch):
        for line in p.stdout:
            line = line.rstrip()
            if line and not any(n in line for n in NOISE):
                self.q.put(line)
        rc = p.wait()
        if rc == 0 and after:
            subprocess.Popen(after)
        if rc == 0 and fetch:
            self.q.put(self.fetch(fetch))
        self.q.put(("done", label, rc))

    def pump(self):
        try:
            while True:
                item = self.q.get_nowait()
                if isinstance(item, tuple) and item[0] == "line":
                    self.write(item[1], item[2])
                elif isinstance(item, tuple):
                    _, label, rc = item
                    self.procs = [p for p in self.procs if p.poll() is None]
                    ok = rc in (0, -15, None)
                    self.write(f"■ {label}: done" + ("" if ok else f" (exit {rc})"), "info" if ok else "bad")
                else:
                    self.write(item)
        except queue.Empty:
            pass
        self.root.after(60, self.pump)

    def stop_all(self):
        n = 0
        for p in self.procs:
            if p.poll() is None:
                p.terminate()
                n += 1
        self.write(f"■ stopped {n} program(s) started here. The simulator has its own stop button; "
                   "the real dog has the STOP button.", "info")

    def watch(self):
        """Every 4 s: is the VM up, is the simulator running, can we reach the dog? (in a background thread)"""
        probe = (f"{SIM_UP} && echo SIM || echo NOSIM; "
                 + FIND_IF +
                 f"if [ -z \"$IFACE\" ]; then echo NOCARD; elif ping -c 1 -W 1 {ROBOT_IP} >/dev/null 2>&1; "
                 "then echo ROBOT; else echo NOROBOT; fi")
        while True:
            vm, sim, vm_robot = "unknown", None, "vmoff"
            try:
                r = subprocess.run(self.vm_cmd(probe), capture_output=True, text=True, timeout=10)
                out = r.stdout.split()
                if r.returncode == 0 and len(out) == 2:
                    vm, sim = "on", out[0] == "SIM"
                    vm_robot = {"ROBOT": "ok", "NOROBOT": "noreply", "NOCARD": "nocard"}[out[1]]
                else:
                    raise RuntimeError
            except Exception:
                if subprocess.run(["pgrep", "-x", "UTM"], capture_output=True).returncode != 0:
                    vm = "off"  # UTM is closed (asking it for the status would open it)
                else:
                    try:
                        st = subprocess.run(utm("get status of"), capture_output=True, text=True,
                                            timeout=8).stdout.strip()
                        vm = {"stopped": "off", "starting": "starting", "started": "starting",
                              "stopping": "stopping"}.get(st, vm)
                    except Exception:
                        pass
            if self.route == "vm":
                robot = (vm_robot, "VM")
            elif self.route == "wifi":
                target = wifi_target()
                ip = wifi_dog_ip(target)
                if target == "ap" and not mac_wifi_ip().startswith("192.168.12."):
                    robot = ("nowifi", "Wi-Fi")
                elif (subprocess.run(["ping", "-c", "1", "-t", "1", ip], capture_output=True).returncode == 0
                      or dog_wifi_port_open(ip)):
                    robot = ("ok", "Wi-Fi")
                else:
                    robot = ("noreply", "Wi-Fi")
            else:
                iface = mac_robot_iface()
                if not iface:
                    robot = ("nocard", "")
                elif subprocess.run(["ping", "-c", "1", "-t", "1", ROBOT_IP], capture_output=True).returncode == 0:
                    robot = ("ok", iface)
                else:
                    robot = ("noreply", iface)
            self.status = {"vm": (vm, None), "sim": sim, "robot": robot}
            self.root.after(0, self.show_status)
            threading.Event().wait(4)

    def show_status(self):
        en = self.lang == "en"
        vm = self.status["vm"][0]
        self.chip["vm"].set(*{
            "on": (C["ok"], "VM on" if en else "虚拟机 已连接"),
            "off": (C["idle"], "VM off" if en else "虚拟机 关"),
            "starting": (C["warn"], "VM starting..." if en else "虚拟机 启动中"),
            "stopping": (C["warn"], "VM shutting down..." if en else "虚拟机 关机中"),
            "unknown": (C["err"], "VM not reachable" if en else "虚拟机 连不上"),
            "?": (C["idle"], "VM: checking..." if en else "虚拟机 检查中..."),
        }[vm])
        sim = self.status["sim"]
        self.chip["sim"].set(*((C["ok"], "Simulator on" if en else "仿真狗 开") if sim else
                               (C["idle"], "Simulator off" if en else "仿真狗 关")))
        state, where = self.status["robot"]
        self.chip["robot"].set(*{
            "ok": (C["ok"], (f"Dog connected ({where})" if en else f"狗 已连上({where})")),
            "noreply": (C["warn"], (f"Dog: no reply on {where}" if en else f"狗 没回应({where})")),
            "nocard": (C["idle"], ("Dog: not connected" if en else "狗 未连接") +
                       (" (VM has no dog card)" if en and where == "VM" else "(虚拟机没有连狗的网卡)" if where == "VM" else "")),
            "vmoff": (C["idle"], "Dog: VM is off" if en else "狗:虚拟机没开"),
            "nowifi": (C["idle"], "Dog: join its Wi-Fi first" if en else "狗:先连上狗的 Wi-Fi"),
            "?": (C["idle"], "Dog: checking..." if en else "狗 检查中..."),
        }[state])

    def quit(self):
        """Closing the panel closes everything it started: its programs on this Mac (a walk still sends StopMove
        on the way out), then the Ubuntu VM with its simulator, then UTM. Opened photos, videos and the PDF stay."""
        if getattr(self, "closing", False):
            return
        self.closing = True
        for p in self.procs:
            if p.poll() is None:
                p.terminate()
        if subprocess.run(["pgrep", "-x", "UTM"], capture_output=True).returncode == 0:
            # runs on after the window is gone: clean shutdown (forced if the VM is stuck), then close UTM
            subprocess.Popen(["bash", os.path.join(HERE, "tools", "vm_stop.sh"), "--quit-utm"], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Panel(root)
    root.mainloop()
