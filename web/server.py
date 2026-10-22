"""Web front end for the Go2 control panel.

    .venv_robot/bin/python web/server.py          then open http://127.0.0.1:8742

The tkinter panel (control_panel.py) still works and is still the fallback.
This serves the same buttons to a browser instead, which the tkinter one
cannot do: a browser can draw the LiDAR point cloud in 3D, show the path on
a canvas, and be opened from a phone on the same Wi-Fi while the dog walks.

Nothing about the robot logic is reimplemented here. This imports
control_panel and uses its ACTIONS table and its helpers unchanged:

    ACTIONS            the same buttons, the same scripts, the same limits
    mac_robot_iface()  the same way of finding the 192.168.123.x card
    wifi_target()      the same Wi-Fi target file
    ROBOT_PY, LAB      the same paths

Only the presentation layer is new. If a button behaves differently here
than in the tkinter panel, that is a bug in this file, not a design choice.

No new Python packages: this is http.server from the standard library, and
log lines reach the browser over Server-Sent Events, which is plain HTTP.
The React build is vendored in web/static/vendor/, so the lab does not need
internet or npm.
"""
import json
import os
import queue
import socket
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STATIC = os.path.join(HERE, "static")
CLIENTLOG = os.path.join(os.path.dirname(STATIC), "clientlog.json")
sys.path.insert(0, ROOT)

import control_panel as CP          # the existing logic, unchanged
from web.live import LIVE

PORT = int(os.environ.get("GO2_PANEL_PORT", "8742"))

# --------------------------------------------------------------- session
# Joining the dog's Wi-Fi costs the Mac its internet, so a lab session is
# worked through offline and only looked at afterwards. Everything the
# panel does is therefore written to disk as it happens: the browser's view
# of it is a convenience, the file is the record.
SESSION_DIR = os.path.join(CP.LOCAL_DATA, "sessions")
SESSION = os.path.join(SESSION_DIR,
                       time.strftime("session_%Y%m%d_%H%M%S.jsonl"))
session_lock = threading.Lock()


def record(kind, **fields):
    """Append one event to this session's file. Never raises."""
    try:
        os.makedirs(SESSION_DIR, exist_ok=True)
        rec = dict(t=time.strftime("%Y-%m-%d %H:%M:%S"), kind=kind, **fields)
        with session_lock, open(SESSION, "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    except Exception:
        pass                      # a lab session must not die over a log


# ------------------------------------------------------------- lifetime
# Close the last window and the program stops, like any other app. The
# count of open SSE streams is the only honest measure of "is anyone
# looking at this": a reload drops it to zero for a moment, which is what
# the grace period is for. GO2_PANEL_STAY=1 keeps it running regardless,
# for a lab session driven from more than one window.
IDLE_QUIT = 0 if os.environ.get("GO2_PANEL_STAY") else 12.0

# Open pages, by the id each one sends with its status poll. Counting SSE
# sockets instead looked obvious and was wrong: Chrome keeps closed pages'
# sockets in a pool, so a shut window still looked like a viewer. A page
# that is really gone simply stops checking in.
viewers = {}                       # client id -> last seen
viewers_lock = threading.Lock()
_empty_since = {"t": time.time()}


def seen(cid):
    if not cid:
        return
    with viewers_lock:
        viewers[cid] = time.time()


def gone(cid):
    with viewers_lock:
        viewers.pop(cid, None)


def viewer_count(max_age=8.0):
    now = time.time()
    with viewers_lock:
        for k in [k for k, t in viewers.items() if now - t > max_age]:
            del viewers[k]
        return len(viewers)


def watch_for_empty(srv):
    """Stop the program once the last window is gone, like any other app."""
    def loop():
        while True:
            time.sleep(2.0)
            with procs_lock:
                busy_now = any(p.poll() is None for p in procs.values())
            # never pull the floor out from under a command the robot is
            # still carrying out, even if every window was closed
            if viewer_count() or busy_now:
                _empty_since["t"] = time.time()
                continue
            if time.time() - _empty_since["t"] > IDLE_QUIT:
                print("no window left - stopping", flush=True)
                record("quit", why="no window open")
                srv.shutdown()
                return
    threading.Thread(target=loop, daemon=True).start()


# ----------------------------------------------------------------- state
subs = []                   # open SSE listeners
subs_lock = threading.Lock()
procs = {}                  # key -> Popen, so a running action can be stopped
procs_lock = threading.Lock()


def emit(kind, **fields):
    """Push one event to every open browser."""
    msg = json.dumps(dict(kind=kind, t=time.time(), **fields))
    with subs_lock:
        dead = []
        for q in subs:
            try:
                q.put_nowait(msg)
            except Exception:
                dead.append(q)
        for q in dead:
            subs.remove(q)


def line(text, tone="plain"):
    emit("line", text=text, tone=tone)
    record("line", text=text, tone=tone)


# ------------------------------------------------------------- the actions

def action_list():
    """The cards and buttons, straight out of control_panel.ACTIONS.

    ACTIONS is a list of (header, [action, ...]), where header is
    (title_en, title_zh, subtitle_en, subtitle_zh). The browser gets the
    same grouping the tkinter panel uses, rather than a grouping invented
    here, so the two panels cannot drift apart."""
    cards = []
    for head, items in CP.ACTIONS:
        t_en, t_zh, s_en, s_zh = (list(head) + ["", "", "", ""])[:4]
        acts = []
        for a in items:
            if not isinstance(a, dict):
                continue
            acts.append({
                "key": a.get("key"),
                "where": a.get("where"),
                "motion": bool(a.get("motion")),
                "limit": a.get("limit", 20),
                "has_run": bool(a.get("run")),
                "en": a.get("en", {}),
                "zh": a.get("zh", {}),
            })
        cards.append({"en": {"title": t_en, "sub": s_en},
                      "zh": {"title": t_zh, "sub": s_zh},
                      "actions": acts})
    return cards


def all_actions():
    for _head, items in CP.ACTIONS:
        for a in items:
            if isinstance(a, dict):
                yield a


def find(key):
    for a in all_actions():
        if a.get("key") == key:
            return a
    return None



def _spawn(key, label, cmd, limit, after=None, tag=None):
    """Run one command, stream its output to the browser, remember the pid."""
    # a snapshot of what the Mac could see at the moment the button was
    # pressed, so a failure can be read back without guessing
    record("action", key=key, label=label, route=tag or "cable",
           cmd=cmd, limit=limit, status=status())
    line(f"▶ {label}", "head")
    line("  " + " ".join(os.path.basename(c) for c in cmd[:3]) +
         (" " + " ".join(cmd[3:]) if len(cmd) > 3 else ""), "dim")

    def worker():
        try:
            p = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
        except Exception as e:
            line(f"✗ could not start: {e}", "bad")
            emit("done", key=key, ok=False)
            return
        p.tag = tag
        with procs_lock:
            procs[key] = p
        t0 = time.time()
        try:
            for raw in p.stdout:
                line(raw.rstrip("\n"))
                if time.time() - t0 > limit + 5:
                    p.terminate()
                    line(f"  stopped after {limit}s", "info")
                    break
        except Exception as e:
            line(f"✗ {e}", "bad")
        p.wait()
        with procs_lock:
            procs.pop(key, None)
        ok = p.returncode == 0
        line(f"■ {label}: {'done' if ok else f'exit {p.returncode}'}",
             "good" if ok else "bad")
        if ok and after:
            subprocess.Popen(after, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        emit("done", key=key, ok=ok)

    threading.Thread(target=worker, daemon=True).start()


def run_wifi(key, a, label):
    """The Wi-Fi route: WebRTC, the same way the phone app talks to the dog.

    Mirrors control_panel.run_wifi, including the rule that matters most in
    the lab - the dog accepts one Wi-Fi client at a time, so a second
    command sent while one is running simply fails."""
    if not os.path.exists(CP.WIFI_PY):
        line("The Wi-Fi environment is missing: run  "
             "bash lab/setup_wifi_sdk.sh  /  Wi-Fi 环境没装,先跑这个脚本。", "bad")
        emit("done", key=key, ok=False)
        return

    target = CP.wifi_target()

    if key == "r_net":                       # 1-1 over Wi-Fi is a check script
        cmd = ["bash", os.path.join(CP.LAB, "wifi_check.sh")]
        if target != "ap":
            cmd.append(target)
        _spawn(key, label, cmd, 40, tag="wifi")
        return

    script = a["run"][0]
    if script not in CP.WIFI_SCRIPT:
        # No WebRTC twin, so try DDS over the Wi-Fi card instead of
        # refusing. If the dog publishes its topics there, this works and
        # the assumption that it is cable-only was wrong; if it does not,
        # the script says "0 messages" and we know, from the robot rather
        # than from a guess.
        wif = wifi_iface()
        if not wif:
            line(f"{label}: no Wi-Fi interface found on this Mac.", "bad")
            emit("done", key=key, ok=False)
            return
        line(f"  no WebRTC version of this one, so trying DDS over the "
             f"Wi-Fi card ({wif}). If the dog does not publish its topics "
             f"there, it will report nothing - that is the answer, not a "
             f"crash.  /  这个没有 WebRTC 版本,改成用 Wi-Fi 网卡({wif})"
             f"直接试 DDS。狗要是不在 Wi-Fi 上发话题,它会报 0 条消息 —— "
             f"那就是答案,不是崩溃。", "info")
        _, args = a["run"][0], [x for x in a["run"][1:]]
        _spawn(key, label,
               [CP.ROBOT_PY, "-u", os.path.join(CP.LAB, script), wif, *args],
               a.get("limit", 20), tag="wifi-dds")
        return

    with procs_lock:
        busy = [(k, p) for k, p in procs.items()
                if getattr(p, "tag", None) == "wifi" and p.poll() is None]

    if key == "m_stop":
        for k, p in busy:
            if k in ("m_standup", "m_balance", "m_walk", "m_turn"):
                p.terminate()
                line("  ended the running motion command", "info")
        subprocess.run(["pkill", "-f",
                        "[w]ifi_sport.py .* (standup|balance|move)"])
        # give them a moment to hang up: the dog takes one Wi-Fi client
        t0 = time.time()
        while any(p.poll() is None for _k, p in busy) and time.time() - t0 < 6:
            time.sleep(0.1)
        _spawn(key, label,
               [CP.WIFI_PY, "-u", os.path.join(CP.LAB, "wifi_sport.py"),
                target, "stop", "--yes"], 30, tag="wifi")
        return

    if busy:
        line("Wait for the running Wi-Fi command to finish: the dog accepts "
             "one Wi-Fi client at a time.  /  等上一个 Wi-Fi 命令结束:"
             "狗一次只接受一个 Wi-Fi 连接。", "bad")
        emit("done", key=key, ok=False)
        return

    if target == "ap" and not CP.mac_wifi_ip().startswith("192.168.12."):
        line("The Mac is not on the dog's Wi-Fi: join it in the Wi-Fi menu, "
             "then 1-1.  /  Mac 没连上狗的 Wi-Fi:先在右上角 Wi-Fi 菜单里连上"
             "狗的网络,再点 1-1。", "bad")
        emit("done", key=key, ok=False)
        return

    if not CP.dog_wifi_port_open(CP.wifi_dog_ip(target)):
        line(f"The dog does not answer over Wi-Fi at "
             f"{CP.wifi_dog_ip(target)}: is it on, and is the phone app "
             f"closed?  /  狗在 Wi-Fi 上没回应:开机了吗?手机 App 关了吗?",
             "bad")
        emit("done", key=key, ok=False)
        return

    stamp = time.strftime("%Y%m%d_%H%M%S")
    photo = os.path.join(CP.LOCAL_DATA, "photos", f"photo_{stamp}.jpg")
    run_dir = os.path.join(CP.LOCAL_DATA, f"wifi_run_{stamp}")
    os.makedirs(os.path.dirname(photo), exist_ok=True)
    raw = a["run"]
    args = [x.replace("{photo}", photo).replace("{run}", run_dir)
            for x in raw[1:]]
    after = (["open", photo] if "{photo}" in " ".join(raw)
             else ["open", run_dir] if "{run}" in " ".join(raw) else None)
    # a Wi-Fi command spends a few seconds connecting before it does
    # anything, so it gets more time than the same command over the cable
    _spawn(key, label,
           [CP.WIFI_PY, "-u", os.path.join(CP.LAB, CP.WIFI_SCRIPT[script]),
            target, *args],
           a.get("limit", 20) + 15, after=after, tag="wifi")


def run_action(key, route):
    """Start one action, over the cable or over Wi-Fi.

    Mirrors control_panel.run_robot and control_panel.run_wifi. The logic
    is not reinvented here; the checks, the script names and the time
    limits are the ones the tkinter panel uses."""
    a = find(key)
    if a is None:
        line(f"unknown action {key}", "bad")
        emit("done", key=key, ok=False)
        return
    label = a["zh"]["label"]

    # r_net and m_stop carry no run script of their own - the panel runs
    # mac_net_up.sh / wifi_check.sh / a stop command for them - so they are
    # routed before the "is this a robot script" guard, which used to
    # reject 1-1 outright and, worse, return without a done event, leaving
    # the button stuck on "running" and every other button disabled.
    if a.get("where") != "robot" or not (a.get("run") or key in ("r_net",
                                                                 "m_stop")):
        line(f"{label}: this button is not a robot script; "
             f"use the tkinter panel for it", "bad")
        emit("done", key=key, ok=False)
        return

    if route == "wifi":
        run_wifi(key, a, label)
        return

    if key == "m_stop":
        n = 0
        with procs_lock:
            for k, p in list(procs.items()):
                if k in ("m_standup", "m_balance", "m_walk", "m_turn") \
                        and p.poll() is None:
                    p.terminate()
                    n += 1
        subprocess.run(["pkill", "-f",
                        "[r]eal_sport.py .* (standup|balance|move)"])
        line(f"  ended {n} running motion command(s)" if n
             else "  nothing was running", "info")
        if not a.get("run"):            # nothing further to spawn
            emit("done", key=key, ok=True)
            return

    if not os.path.exists(CP.ROBOT_PY):
        line("The Mac robot environment is missing: run  "
             "bash lab/setup_mac_sdk.sh", "bad")
        emit("done", key=key, ok=False)
        return
    iface = CP.mac_robot_iface()
    if not iface and key != "r_net":
        line("The Mac is not connected to the dog yet: click 1-1 first. / "
             "Mac 还没连上狗,先点 1-1。", "bad")
        emit("done", key=key, ok=False)
        return

    if key == "r_net":
        _spawn(key, label, ["bash", os.path.join(CP.LAB, "mac_net_up.sh")],
               a.get("limit", 20), tag="cable")
        return

    stamp = time.strftime("%Y%m%d_%H%M%S")
    photo = os.path.join(CP.LOCAL_DATA, "photos", f"photo_{stamp}.jpg")
    run_dir = os.path.join(CP.LOCAL_DATA, f"real_run_{stamp}")
    os.makedirs(os.path.dirname(photo), exist_ok=True)
    raw = a["run"]
    script, args = raw[0], [x.replace("{photo}", photo).replace("{run}", run_dir)
                            for x in raw[1:]]
    # open what the script produced, the way the tkinter panel does
    after = (["open", photo] if "{photo}" in " ".join(raw)
             else ["open", run_dir] if "{run}" in " ".join(raw) else None)
    _spawn(key, label,
           [CP.ROBOT_PY, "-u", os.path.join(CP.LAB, script), iface, *args],
           a.get("limit", 20), after=after, tag="cable")



# ------------------------------------------------------- recorded surveys

def _data_dir():
    return os.path.join(CP.LAB, "data")


def list_surveys():
    """Every recorded survey, and whether a map has been built from it."""
    d = _data_dir()
    if not os.path.isdir(d):
        return []
    out = []
    for name in sorted(os.listdir(d), reverse=True):
        p = os.path.join(d, name)
        if not (os.path.isdir(p) and name.startswith("survey_")):
            continue
        meta = {}
        try:
            meta = json.load(open(os.path.join(p, "meta.json")))
        except Exception:
            pass
        rep = None
        try:
            rep = json.load(open(os.path.join(p, "map_report.json")))
        except Exception:
            pass
        out.append({
            "name": name,
            "seconds": meta.get("seconds"),
            "counts": meta.get("counts", {}),
            "has_map": os.path.exists(os.path.join(p, "map.npz")),
            "distance_m": (rep or {}).get("distance_m"),
            "map_points": (rep or {}).get("map_points"),
            "had_odometry": (rep or {}).get("had_odometry"),
        })
    return out


def _safe(name):
    if "/" in name or ".." in name or not name.startswith("survey_"):
        return None
    p = os.path.join(_data_dir(), name)
    return p if os.path.isdir(p) else None


def map_cloud(name, max_points=300000):
    """A built map as raw float32 xyz, for the same GPU path the live
    cloud uses."""
    import numpy as np
    p = _safe(name)
    if not p:
        return None
    f = os.path.join(p, "map.npz")
    if not os.path.exists(f):
        return None
    try:
        c = np.load(f)["cloud"]
        if len(c) > max_points:
            idx = np.random.default_rng(0).choice(len(c), max_points,
                                                  replace=False)
            c = c[np.sort(idx)]
        return np.ascontiguousarray(c, dtype="<f4").tobytes()
    except Exception:
        return None


def map_path(name):
    import numpy as np
    p = _safe(name)
    if not p:
        return {"path": []}
    f = os.path.join(p, "map.npz")
    if not os.path.exists(f):
        return {"path": []}
    try:
        z = np.load(f)
        return {"path": np.asarray(z["path"], float).tolist()}
    except Exception:
        return {"path": []}


def cable_only_keys():
    """Buttons with no WebRTC twin: they speak DDS directly.

    That does NOT mean they cannot work over Wi-Fi. Whether the dog
    publishes its DDS topics on the Wi-Fi interface as well as the wired
    one is a property of the robot nobody here has measured, so these are
    run over the Wi-Fi card and allowed to report what they find. Blocking
    them on a guess is how a lab session gets wasted."""
    out = []
    for a in all_actions():
        run = a.get("run")
        if (a.get("where") == "robot" and run
                and a["key"] != "r_net"
                and run[0] not in CP.WIFI_SCRIPT):
            out.append(a["key"])
    return sorted(out)


_status_cache = {"t": 0.0, "v": None}


def wifi_iface():
    """The Mac's Wi-Fi interface name (en0), not its address.

    DDS needs the interface, not the IP, and on the dog's own hotspot the
    Wi-Fi card is the only card facing the robot."""
    try:
        import re
        out = subprocess.run(["networksetup", "-listallhardwareports"],
                             capture_output=True, text=True).stdout
        m = re.search(r"Hardware Port: Wi-Fi\nDevice: (\S+)", out)
        return m.group(1) if m else None
    except OSError:
        return None


def _dog_wifi_up(ip):
    """Is the dog's Wi-Fi control port answering? Fast, or not at all.

    control_panel's version waits 1.5 s on each of two ports, so it costs
    3 s when nothing answers - and the panel polls status every 2.5 s, so
    that probe was never not running. On a LAN a port either answers in a
    few milliseconds or is not there."""
    for port in (9991, 8081):
        try:
            with socket.create_connection((ip, port), timeout=0.4):
                return True
        except OSError:
            pass
    return False


def status(max_age=2.0):
    """The panel's view of what is connected.

    Cached: it shells out to ifconfig, networksetup and ipconfig and opens
    sockets, which is far too much to redo for every poll of every open
    tab."""
    now = time.time()
    if _status_cache["v"] is not None and now - _status_cache["t"] < max_age:
        return _status_cache["v"]
    v = _status_uncached()
    _status_cache.update(t=now, v=v)
    return v


def _status_uncached():
    iface = CP.mac_robot_iface()
    reachable = False
    if iface:
        reachable = subprocess.run(
            ["ping", "-c", "1", "-t", "1", CP.ROBOT_IP],
            capture_output=True).returncode == 0
    with procs_lock:
        busy = [k for k, p in procs.items() if p.poll() is None]

    target = CP.wifi_target()
    mac_wifi = CP.mac_wifi_ip()
    on_dog_wifi = (mac_wifi.startswith("192.168.12.") if target == "ap"
                   else bool(mac_wifi))
    wifi_ok = on_dog_wifi and _dog_wifi_up(CP.wifi_dog_ip(target))
    return {"iface": iface or "", "robot_ip": CP.ROBOT_IP,
            "reachable": reachable, "busy": busy,
            "robot_py": os.path.exists(CP.ROBOT_PY),
            "wifi_target": target,
            "wifi_py": os.path.exists(CP.WIFI_PY),
            "mac_wifi_ip": mac_wifi,
            "dog_wifi_ip": CP.wifi_dog_ip(target),
            "wifi_reachable": wifi_ok,
            "viewers": viewer_count(),
            "wifi_iface": wifi_iface() or "",
            "dds_over_wifi": cable_only_keys()}


# ----------------------------------------------------------- the handover
# The lab has no internet while the Mac is on the dog's Wi-Fi, so a session
# is worked through blind and only read afterwards. These two turn the
# session file into something that can simply be handed over.

def session_summary():
    try:
        recs = [json.loads(x) for x in open(SESSION) if x.strip()]
    except Exception:
        recs = []
    acts = [r for r in recs if r["kind"] == "action"]
    bad = [r for r in recs if r["kind"] == "line" and r.get("tone") == "bad"]
    return {"file": SESSION, "events": len(recs),
            "actions": len(acts), "errors": len(bad),
            "started": recs[0]["t"] if recs else None,
            "last": recs[-1]["t"] if recs else None,
            "keys": [a["key"] for a in acts]}


def export_session():
    """One readable file holding everything this session saw.

    Written as Markdown rather than a zip: the point is to be pasted into a
    chat or committed to the repo, not archived."""
    try:
        recs = [json.loads(x) for x in open(SESSION) if x.strip()]
    except Exception as e:
        return {"ok": False, "msg": f"no session file yet ({e})"}

    out = [f"# Go2 panel session  {time.strftime('%Y-%m-%d %H:%M')}", ""]
    try:
        import platform
        out += [f"- macOS {platform.mac_ver()[0]}, python "
                f"{sys.version.split()[0]}",
                f"- cable python: {CP.ROBOT_PY} "
                f"({'present' if os.path.exists(CP.ROBOT_PY) else 'MISSING'})",
                f"- wifi python:  {CP.WIFI_PY} "
                f"({'present' if os.path.exists(CP.WIFI_PY) else 'MISSING'})",
                ""]
    except Exception:
        pass

    st = status()
    out += ["## Connection at export time", "```",
            json.dumps(st, ensure_ascii=False, indent=1), "```", ""]

    try:
        out += ["## What the browser reported", "```"]
        for r in json.load(open(CLIENTLOG))[-6:]:
            out.append(json.dumps(r, ensure_ascii=False))
        out += ["```", ""]
    except Exception:
        pass

    out.append("## What happened")
    for r in recs:
        if r["kind"] == "action":
            s_ = r.get("status", {})
            out += ["", f"### {r['t']}  {r.get('label', r['key'])} "
                        f"[{r['key']}, route={r.get('route')}]",
                    "```", "cmd: " + " ".join(map(str, r.get("cmd", []))),
                    f"iface={s_.get('iface')!r} reachable={s_.get('reachable')} "
                    f"wifi_reachable={s_.get('wifi_reachable')} "
                    f"mac_wifi_ip={s_.get('mac_wifi_ip')!r}", "```", ""]
        elif r["kind"] == "line":
            mark = {"bad": "!! ", "head": "", "info": " . "}.get(r.get("tone"), "   ")
            out.append("    " + mark + r.get("text", ""))

    path = os.path.join(SESSION_DIR,
                        time.strftime("report_%Y%m%d_%H%M%S.md"))
    try:
        os.makedirs(SESSION_DIR, exist_ok=True)
        with open(path, "w") as f:
            f.write("\n".join(out) + "\n")
        subprocess.Popen(["open", "-R", path], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    except Exception as e:
        return {"ok": False, "msg": str(e)}
    return {"ok": True, "path": path, "events": len(recs)}


# ------------------------------------------------------------------ HTTP

class Handler(SimpleHTTPRequestHandler):
    # HTTP/1.1, so the socket is reused instead of torn down after every
    # reply. The panel polls four endpoints and holds one SSE stream open;
    # at HTTP/1.0 that is a new TCP connection roughly ten times a second,
    # and Chrome allows six per host. The SSE takes one and the rest fight
    # over five, which shows up as a stream of net::ERR_ABORTED and a page
    # that quietly stops updating. Keep-alive needs an accurate
    # Content-Length on every response, which _json, _bin, index() and the
    # static handler all set.
    protocol_version = "HTTP/1.1"

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=STATIC, **kw)

    def log_message(self, fmt, *a):
        """Quiet by default; GO2_ACCESS_LOG=<path> turns it on.

        The console belongs to the robot scripts' output, so request lines
        are off unless something needs tracing."""
        path = os.environ.get("GO2_ACCESS_LOG")
        if not path:
            return
        try:
            with open(path, "a") as f:
                f.write("%s %s %s\n" % (time.strftime("%H:%M:%S"),
                                        self.client_address[0], fmt % a))
        except Exception:
            pass

    def end_headers(self):
        """Never let the browser cache the panel.

        SimpleHTTPRequestHandler sends Last-Modified and honours
        If-Modified-Since, so Chrome happily keeps serving an old app.js
        after the panel is updated. When a file is renamed or removed, that
        stale copy references something that no longer exists and the page
        renders black with an error nobody sees. The panel is served from
        localhost and is a few hundred kB, so there is nothing to gain by
        caching it."""
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def send_header(self, key, value):
        # drop the validators that make a 304 possible in the first place
        if key.lower() in ("last-modified", "etag"):
            return
        super().send_header(key, value)

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _bin(self, blob, ctype):
        if blob is None:
            # 204 still needs a length under keep-alive, or the browser waits
            # for a body that never comes and eventually aborts
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(blob)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(blob)
        except Exception:
            pass

    def do_GET(self):
        # SimpleHTTPRequestHandler answers 304 from send_head(), before any
        # of our headers are written, so dropping Last-Modified on the way
        # out is not enough - the conditional request has to go too.
        for h in ("If-Modified-Since", "If-None-Match"):
            if h in self.headers:
                del self.headers[h]
        path, _, query = self.path.partition("?")
        if path == "/api/actions":
            return self._json(action_list())
        if path == "/api/status":
            # the page's id rides along on a poll it already makes, so
            # knowing whether a window is still open costs no extra request
            from urllib.parse import parse_qs
            seen(parse_qs(query).get("c", [""])[0])
            return self._json(status())
        if self.path == "/api/events":
            return self.sse()
        if self.path == "/api/live":
            return self._json(LIVE.state())
        if self.path == "/api/cloud":
            # raw float32 xyz, straight into a GPU buffer in the browser
            return self._bin(LIVE.cloud_bytes(), "application/octet-stream")
        if self.path.startswith("/api/frame"):
            return self._bin(LIVE.frame(), "image/jpeg")
        if self.path == "/api/clientlog":          # what the browser saw
            try:
                return self._json(json.load(open(CLIENTLOG)))
            except Exception:
                return self._json([])
        if self.path == "/api/session":
            return self._json(session_summary())
        if self.path == "/api/session/export":
            return self._json(export_session())
        if self.path == "/api/version":
            return self._json({"build": self.build_id()})
        if self.path == "/api/surveys":
            return self._json(list_surveys())
        if self.path.startswith("/api/map/"):
            return self._bin(map_cloud(self.path[len("/api/map/"):]),
                             "application/octet-stream")
        if self.path.startswith("/api/path/"):
            return self._json(map_path(self.path[len("/api/path/"):]))
        if self.path in ("/", "/index.html"):
            return self.index()
        return super().do_GET()

    def build_id(self):
        """One hash over every asset, so a page can tell it is out of date."""
        import hashlib
        h = hashlib.md5()
        for name in sorted(os.listdir(STATIC)):
            if name.endswith((".js", ".css", ".html")):
                try:
                    with open(os.path.join(STATIC, name), "rb") as f:
                        h.update(name.encode()); h.update(f.read())
                except OSError:
                    pass
        return h.hexdigest()[:12]

    def index(self):
        """Serve index.html with a fresh content hash on every asset.

        The hashes are computed here rather than written into the file, so
        editing app.js is enough - there is no second place to remember to
        update, and a browser can never pair a new index.html with an old
        script."""
        import hashlib
        import re
        try:
            html = open(os.path.join(STATIC, "index.html")).read()
        except OSError:
            self.send_error(404)
            return

        def ver(name):
            try:
                with open(os.path.join(STATIC, name), "rb") as f:
                    return hashlib.md5(f.read()).hexdigest()[:8]
            except OSError:
                return "0"

        def sub(m):
            attr, name = m.group(1), m.group(2)
            if name.startswith("vendor/"):        # pinned, never changes
                return f'{attr}="{name}"'
            return f'{attr}="{name}?v={ver(name)}"'

        html = re.sub(r'(src|href)="([^"?]+)(?:\?v=[^"]*)?"', sub, html)
        # the page learns which build it is, so it can notice a newer one
        html = html.replace("</head>",
                            f'<script>window.BUILD="{self.build_id()}";'
                            f'</script>\n</head>')
        body = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def sse(self):
        # the one response whose length is unknown: tell the client the
        # socket closes when the stream does, instead of promising keep-alive
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        q = queue.Queue(maxsize=2000)
        with subs_lock:
            subs.append(q)
        try:
            self.wfile.write(b": connected\n\n")
            self.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=5)
                    self.wfile.write(f"data: {msg}\n\n".encode())
                except queue.Empty:
                    # also how a closed tab is noticed: this write fails
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except Exception:
            pass
        finally:
            with subs_lock:
                if q in subs:
                    subs.remove(q)

    def client_log(self, rec):
        """Append one browser self-report.

        The panel runs in a browser I cannot attach a debugger to, and a
        page that fails to draw cannot be asked what happened. So it says
        so itself, once, on load."""
        try:
            try:
                recs = json.load(open(CLIENTLOG))
            except Exception:
                recs = []
            rec["at"] = time.strftime("%H:%M:%S")
            recs = (recs + [rec])[-40:]
            with open(CLIENTLOG, "w") as f:
                json.dump(recs, f, ensure_ascii=False, indent=1)
        except Exception:
            pass

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            body = {}
        if self.path == "/api/run":
            key = body.get("key", "")
            run_action(key, body.get("route", "cable"))
            return self._json({"started": key})
        if self.path == "/api/clientlog":
            # body is already decoded above; this is a dict, not a string
            self.client_log(dict(body) if isinstance(body, dict) else {})
            return self._json({"ok": True})
        if self.path == "/api/bye":
            # the window is closing; drop it now instead of waiting it out
            gone((body or {}).get("c"))
            return self._json({"ok": True})
        if self.path == "/api/live/start":
            ok, msg = LIVE.start(CP.mac_robot_iface())
            line(("  " if ok else "✗ ") + msg, "info" if ok else "bad")
            return self._json({"ok": ok, "msg": msg})
        return self._json({"error": "unknown"}, 404)


SHOW_TAB = """
on run argv
  set target to item 1 of argv
  try
    tell application "Google Chrome"
      repeat with w in windows
        repeat with i from 1 to (count of tabs of w)
          if (URL of tab i of w) contains target then
            set active tab index of w to i
            set index of w to 1
            activate
            return "focused"
          end if
        end repeat
      end repeat
      open location ("http://" & target)
      activate
      return "opened"
    end tell
  on error
    return "no-chrome"
  end try
end run
"""


def show(url):
    """Bring the panel up, in the tab it is already in if there is one.

    Plain `open <url>` adds another tab every time the icon is clicked,
    which leaves a row of identical panels all polling the same server."""
    target = url.replace("http://", "")
    try:
        r = subprocess.run(["osascript", "-", target], input=SHOW_TAB,
                           capture_output=True, text=True, timeout=10)
        if "no-chrome" not in r.stdout:
            return
    except Exception:
        pass
    try:                                      # not Chrome, or AppleScript off
        subprocess.Popen(["open", url], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    except Exception:
        pass


def main():
    if not os.path.isdir(STATIC):
        sys.exit(f"missing {STATIC}")
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    if IDLE_QUIT:
        watch_for_empty(srv)
    url = f"http://127.0.0.1:{PORT}"
    print(f"Go2 panel (web)  ->  {url}")
    n = sum(len(c["actions"]) for c in action_list())
    print(f"  {n} buttons in {len(action_list())} cards, taken from control_panel.ACTIONS")
    print(f"  the tkinter panel still works and is the fallback")
    print(f"  closing the last window stops this; GO2_PANEL_STAY=1 to keep it")
    print(f"  Ctrl+C to stop\n")
    if not os.environ.get("GO2_PANEL_NO_OPEN"):
        show(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
