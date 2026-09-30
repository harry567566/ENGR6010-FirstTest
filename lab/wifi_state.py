"""Watch the state of a REAL Go2 over Wi-Fi. READ-ONLY: it never sends a command. The Wi-Fi twin of real_state.py.

    .venv_wifi/bin/python lab/wifi_state.py ap          # Mac on the dog's own Wi-Fi, 20 seconds
    .venv_wifi/bin/python lab/wifi_state.py ap 60       # 60 seconds

Twice per second it prints battery, body tilt, body height, sport mode, front-right leg angles,
foot forces and how many messages per second arrive. Over Wi-Fi the dog sends the lower-rate copies
of the state topics (rt/lf/lowstate, rt/lf/sportmodestate), so expect fewer messages than on the cable.
"""
import asyncio
import math
import sys
import time

from wifi_common import STOP, connect, disconnect, watch_for_stop

latest = {"low": None, "sport": None}
count = {"low": 0, "sport": 0}


def on_low(message):
    latest["low"] = message.get("data", {})
    count["low"] += 1


def on_sport(message):
    latest["sport"] = message.get("data", {})
    count["sport"] += 1


def line(t, rates):
    low, sport = latest["low"] or {}, latest["sport"] or {}
    rpy = [math.degrees(a) for a in (low.get("imu_state", {}).get("rpy") or [0, 0, 0])]
    q = [m.get("q", 0.0) for m in (low.get("motor_state") or [])[:3]] or [0.0, 0.0, 0.0]
    feet = low.get("foot_force") or [0, 0, 0, 0]
    soc = low.get("bms_state", {}).get("soc", 0)
    return (f"[{t:5.1f}s] batt {low.get('power_v', 0):5.2f} V {soc:3}% | "
            f"rpy {rpy[0]:6.1f} {rpy[1]:6.1f} {rpy[2]:7.1f} deg | FR q {q[0]:+.2f} {q[1]:+.2f} {q[2]:+.2f} rad | "
            f"feet {' '.join(f'{f:4}' for f in feet[:4])} | height {sport.get('body_height', 0):.3f} m | "
            f"mode {sport.get('mode', '?')} | rate low {rates[0]:4.0f}/s sport {rates[1]:4.0f}/s")


async def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    target = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    stop_event = asyncio.Event()
    watch_for_stop(stop_event)
    conn = await connect(target)
    try:
        conn.datachannel.pub_sub.subscribe("rt/lf/lowstate", on_low)
        conn.datachannel.pub_sub.subscribe("rt/lf/sportmodestate", on_sport)
        print(f"listening for {seconds:.0f} s. Read-only: the robot is not commanded.", flush=True)
        t0 = t_prev = time.time()
        prev = dict(count)
        while time.time() - t0 < seconds and not STOP["requested"]:
            try:
                await asyncio.wait_for(stop_event.wait(), 0.5)
            except asyncio.TimeoutError:
                pass
            now = time.time()
            rates = [(count[k] - prev[k]) / max(now - t_prev, 1e-6) for k in ("low", "sport")]
            prev, t_prev = dict(count), now
            if latest["low"] is None:
                print(f"[{now - t0:5.1f}s] no state message yet", flush=True)
            else:
                print(line(now - t0, rates), flush=True)
        print(f"done. received {count['low']} lowstate and {count['sport']} sportmodestate messages.")
    finally:
        await disconnect(conn)


if __name__ == "__main__":
    asyncio.run(main())
