"""Send ONE high-level sport command to a REAL Go2 over Wi-Fi (WebRTC). The Wi-Fi twin of real_sport.py.

THIS MAKES THE ROBOT MOVE. Clear 2 m around it and keep the remote in hand
(remote emergency stop: hold L2 then press B = damping).

    .venv_wifi/bin/python lab/wifi_sport.py ap standup          # Mac on the dog's own Wi-Fi
    .venv_wifi/bin/python lab/wifi_sport.py ap balance
    .venv_wifi/bin/python lab/wifi_sport.py ap move 0.2 0 0 2   # walk: vx vy vyaw seconds
    .venv_wifi/bin/python lab/wifi_sport.py ap stop
    .venv_wifi/bin/python lab/wifi_sport.py ap standdown
    .venv_wifi/bin/python lab/wifi_sport.py ap damp             # motors soft: lie the dog down first
    .venv_wifi/bin/python lab/wifi_sport.py 192.168.1.23 stop   # dog and Mac on the same router
Add --dry-run to only print what would be sent. Add --yes to skip the 'Press Enter' question.

The command ids are the same as the cable SDK (and the same in the newer MCF firmware mode):
Damp 1001, BalanceStand 1002, StopMove 1003, StandUp 1004, StandDown 1005, RecoveryStand 1006, Move 1008.
move limits (clamped here): |vx| <= 0.3 m/s, |vy| <= 0.2 m/s, |vyaw| <= 0.5 rad/s, seconds <= 3.
"""
import asyncio
import json
import math
import random
import sys
import time

from wifi_common import STOP, connect, disconnect, watch_for_stop

SPORT_TOPIC = "rt/api/sport/request"
SWITCHER_TOPIC = "rt/api/motion_switcher/request"
LIMITS = {"vx": 0.3, "vy": 0.2, "vyaw": 0.5, "seconds": 3.0}
MOVE_PERIOD = 0.05  # re-send the Move command every 0.05 s (20 times per second)

# the dog's sport mode (rt/lf/sportmodestate "mode") decides whether Move can make it walk
MODE_NAMES = {0: "idle stand", 1: "balance stand", 2: "pose", 3: "walking", 5: "lying down", 6: "joints locked",
              7: "damping", 8: "recovery stand", 10: "sitting"}
STANDING_LOCKED = (0, 2, 6, 8)  # standing, but Move does little there until BalanceStand
DOWN = (5, 7, 10)               # lying, soft or sitting: stand it up first

# action -> (name, api id, what it does, asks for Enter first?)
ACTIONS = {
    "standup":   ("StandUp",       1004, "stand up and lock the joints", True),
    "balance":   ("BalanceStand",  1002, "switch to balance stand (needed before walking)", True),
    "recovery":  ("RecoveryStand", 1006, "recover to standing (e.g. after a fall)", True),
    "stop":      ("StopMove",      1003, "stop moving, keep the current stance", False),
    "standdown": ("StandDown",     1005, "lie down on the floor", False),
    "damp":      ("Damp",          1001, "damping: all motors go soft (robot sinks if standing)", False),
    "move":      ("Move",          1008, "walk with a velocity, then StopMove", True),
}
SAFE_TO_REPEAT = ("stop", "damp", "standdown")


def usage():
    print(__doc__)
    sys.exit(1)


def clamp(name, value):
    lim = LIMITS[name]
    clamped = max(-lim, min(lim, value))
    if clamped != value:
        print(f"  note: {name} = {value} clamped to {clamped}")
    return clamped


def parse_args(argv):
    dry_run, yes = "--dry-run" in argv, "--yes" in argv
    args = [a for a in argv if a not in ("--dry-run", "--yes")]
    if len(args) < 2 or args[1] not in ACTIONS:
        usage()
    target, action, move = args[0], args[1], None
    if action == "move":
        if len(args) != 6:
            print("move needs 4 numbers: vx vy vyaw seconds   e.g.  move 0.2 0 0 2")
            sys.exit(1)
        vx, vy, vyaw, secs = (float(x) for x in args[2:6])
        vx, vy, vyaw = clamp("vx", vx), clamp("vy", vy), clamp("vyaw", vyaw)
        if not 0.0 <= secs <= LIMITS["seconds"]:
            print(f"  note: seconds = {secs} clamped to 0..{LIMITS['seconds']}")
            secs = min(max(secs, 0.0), LIMITS["seconds"])
        move = (vx, vy, vyaw, secs)
    elif len(args) != 2:
        print(f"'{action}' takes no extra numbers")
        sys.exit(1)
    return target, action, move, dry_run, yes


async def request(conn, topic, api_id, parameter=None, timeout=5.0):
    """A command that waits for the dog's answer. Returns its status code (0 = OK, -1 = no answer)."""
    payload = {"api_id": api_id}
    if parameter is not None:
        payload["parameter"] = parameter
    try:
        answer = await asyncio.wait_for(conn.datachannel.pub_sub.publish_request_new(topic, payload), timeout)
    except asyncio.TimeoutError:
        return -1
    return answer.get("data", {}).get("header", {}).get("status", {}).get("code", -1)


def send_move(conn, vx, vy, vyaw):
    """Move gets no answer from the dog (same as the cable SDK's Move): it is sent and not waited for."""
    conn.datachannel.pub_sub.publish_without_callback(SPORT_TOPIC, {
        "header": {"identity": {"id": int(time.time() * 1000) % 2147483648 + random.randint(0, 1000),
                                "api_id": 1008},
                   "policy": {"priority": 0, "noreply": True}},
        "parameter": json.dumps({"x": vx, "y": vy, "z": vyaw}),
        "binary": [],
    })


async def check_mode(conn):
    """Read-only: which controller is active? An empty name means sport mode was released (low-level control),
    and then sport commands do nothing. No answer at all is reported and ignored (newer firmware may not reply)."""
    try:
        answer = await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(SWITCHER_TOPIC, {"api_id": 1001}), 3.0)
        name = json.loads(answer["data"]["data"]).get("name", "")
    except Exception:
        print("  motion switcher: no answer (continuing)")
        return True
    print(f"  motion switcher mode: {name!r}")
    if not name:
        print("  sport mode is RELEASED (low-level mode), so sport commands will not work.")
        print("  Ask the instructor, or reboot the robot to get sport mode back.")
        return False
    return True


def pose(state):
    """(x, y, heading) from the latest rt/lf/sportmodestate message, or None."""
    msg = state.get("msg") or {}
    pos, rpy = msg.get("position"), (msg.get("imu_state") or {}).get("rpy")
    return (float(pos[0]), float(pos[1]), float(rpy[2])) if pos and rpy else None


def moved(start, end):
    """How far and how much it turned between two poses, as the dog measured it."""
    dist = math.hypot(end[0] - start[0], end[1] - start[1])
    turn = math.degrees(math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2])))
    return f"measured by the dog: moved {dist:.2f} m, turned {turn:+.0f} degrees"


async def main():
    target, action, move, dry_run, yes = parse_args(sys.argv[1:])
    method, api_id, what, confirm = ACTIONS[action]
    print(f"Wi-Fi target {target}")
    print(f"about to send: {method} (API {api_id}) = {what}")
    if move:
        vx, vy, vyaw, secs = move
        print(f"  Move(vx={vx}, vy={vy}, vyaw={vyaw}) every {MOVE_PERIOD} s for {secs} s, then StopMove")
    if dry_run:
        print("dry run: nothing sent")
        return
    stop_event = asyncio.Event()
    watch_for_stop(stop_event)
    conn = await connect(target)
    try:
        if action not in ("stop", "damp") and not await check_mode(conn):
            sys.exit(3)
        if confirm and not yes:
            await asyncio.to_thread(input, "Press Enter to send (Ctrl+C to cancel) ... ")
        if STOP["requested"] and action not in SAFE_TO_REPEAT:
            print(f"STOP was pressed before {method} went out: nothing sent")
            if action != "move":
                return
        if action != "move":
            code = await request(conn, SPORT_TOPIC, api_id)
            if code != 0 and action in SAFE_TO_REPEAT:
                code = await request(conn, SPORT_TOPIC, api_id)  # safe to repeat: try once more
            print(f"{method} returned {'0 (OK)' if code == 0 else '-1 (no answer)' if code == -1 else code}")
            return
        # make sure the dog can walk, keep sending the velocity, always stop, then say how far it went
        state = {}
        conn.datachannel.pub_sub.subscribe("rt/lf/sportmodestate", lambda m: state.update(msg=m.get("data", {})))
        t_wait = time.time() + 1.5
        while "msg" not in state and time.time() < t_wait:
            await asyncio.sleep(0.05)
        mode = state.get("msg", {}).get("mode")
        name = MODE_NAMES.get(mode, "unknown")
        if mode in DOWN:
            print(f"the dog is {name} (mode {mode}): click Stand up, then Balance stand, then walk. Nothing sent.")
            sys.exit(4)
        if mode in STANDING_LOCKED and not STOP["requested"]:
            print(f"the dog is in '{name}' (mode {mode}), where Move does little: sending BalanceStand first")
            code = await request(conn, SPORT_TOPIC, 1002)
            print(f"BalanceStand returned {'0 (OK)' if code == 0 else '-1 (no answer)' if code == -1 else code}")
            await asyncio.sleep(1.5)
        elif mode is None:
            print("no sport state from the dog yet (mode unknown): walking anyway")
        start = pose(state)
        n_sent, t_end = 0, None
        try:
            while not STOP["requested"] and (t_end is None or time.time() < t_end):
                send_move(conn, vx, vy, vyaw)
                if t_end is None:
                    t_end = time.time() + secs  # the clock starts when the first Move has gone out
                n_sent += 1
                try:  # wait one period, but wake at once on STOP
                    await asyncio.wait_for(stop_event.wait(), MOVE_PERIOD)
                except asyncio.TimeoutError:
                    pass
            if STOP["requested"]:
                print("STOP: stopping")
        finally:
            code = await request(conn, SPORT_TOPIC, 1003)
            if code != 0:
                code = await request(conn, SPORT_TOPIC, 1003)
            print(f"Move sent {n_sent} times")
            print(f"StopMove returned {'0 (OK)' if code == 0 else '-1 (no answer)' if code == -1 else code}")
        await asyncio.sleep(1.0)  # let the dog settle, then read where it thinks it is
        end = pose(state)
        if start and end:
            print(moved(start, end))
    finally:
        await disconnect(conn)


if __name__ == "__main__":
    asyncio.run(main())
