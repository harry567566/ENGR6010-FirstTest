"""Shared parts of the Wi-Fi scripts (wifi_state.py, wifi_photo.py, wifi_record.py, wifi_sport.py).

Over Wi-Fi the Go2 is reached the way the phone app reaches it: WebRTC, through the library
unitree_webrtc_connect (legion1581). It works on Go2 AIR, PRO and EDU. Environment: .venv_wifi
(made by lab/setup_wifi_sdk.sh).

The first argument of every Wi-Fi script says where the dog is:
    ap              the Mac is on the dog's own Wi-Fi hotspot (the dog is always 192.168.12.1 there)
    192.168.x.y     the dog and the Mac are on the same router; the dog's address on that router

Firmware 1.1.15 and newer also needs the dog's own AES key (32 hex characters). Put it in
lab/.go2_aes_key or in the environment variable GO2_AES_KEY. Fetch it once, with internet, using the
Unitree account the dog is bound to:  .venv_wifi/bin/unitree-fetch-aes-key --email ... --device-type Go2

Only ONE Wi-Fi client can be connected at a time: close the Unitree phone app first, and run one script
at a time.
"""
import asyncio
import os
import signal
import sys

DOG_AP_IP = "192.168.12.1"  # the dog's address on its own hotspot
KEY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".go2_aes_key")

STOP = {"requested": False}  # set by SIGTERM (panel STOP / time limit) or Ctrl+C


def aes_key():
    key = os.environ.get("GO2_AES_KEY", "").strip()
    if not key and os.path.exists(KEY_FILE):
        key = open(KEY_FILE).read().strip()
    return key or None


def watch_for_stop(event):
    """SIGTERM or Ctrl+C: remember it and wake anything waiting on `event` (inside the running loop)."""
    loop = asyncio.get_running_loop()

    def stop():
        STOP["requested"] = True
        event.set()

    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, stop)


async def connect(target, timeout=25.0):
    """Open the WebRTC connection to the dog. Exits with a clear message if that fails."""
    from unitree_webrtc_connect import unitree_auth as auth
    from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

    key = aes_key()
    if target == "ap":
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, aes_128_key=key)
        where = f"the dog's own Wi-Fi ({DOG_AP_IP})"
    else:
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=target, aes_128_key=key)
        where = f"{target} on the shared router"
    print(f"connecting over Wi-Fi to {where}" + (" with the AES key" if key else "") + " ...", flush=True)
    try:
        await asyncio.wait_for(conn.connect(), timeout)
    except auth.RobotBusyError:
        fail(4, "The dog already has a Wi-Fi client: close the Unitree phone app (or any other program) and try again.",
             "狗已经被别的程序连着:先关掉手机上的 Unitree App,再试。")
    except auth.AesKeyRequiredError:
        # Nothing here can be fixed without internet and the Unitree account
        # the robot is bound to, which for a lab robot is the instructor's.
        # So say that, rather than pointing at a document.
        fail(5, "This dog's firmware (Go2 1.1.15 or newer) needs its own "
                "AES key, and there is no way around it over Wi-Fi. The key "
                "comes from the Unitree account the DOG is registered to - "
                "the instructor's, not yours - and fetching it needs "
                "internet:  .venv_wifi/bin/unitree-fetch-aes-key --email "
                "<that account> --device-type Go2 --sn <the dog's serial>. "
                "Save it in lab/.go2_aes_key. Until then use the Ethernet "
                "cable, which needs no key.",
             "这只狗的固件(Go2 1.1.15 以上)必须有它自己的 AES 密钥,走 Wi-Fi "
             "绕不过去。密钥要用【这只狗绑定的那个 Unitree 账号】去云端取 —— "
             "那是老师的账号,不是你的 —— 而且取密钥需要联网:\n"
             "  .venv_wifi/bin/unitree-fetch-aes-key --email <老师的账号> "
             "--device-type Go2 --sn <狗的序列号>\n"
             "取到后存进 lab/.go2_aes_key。在那之前,用网线 —— 网线不需要密钥。")
    except auth.AesKeyRejectedError:
        fail(5, "The dog rejected the AES key in lab/.go2_aes_key: it belongs to another dog, or it is mistyped.",
             "狗不认这个 AES 密钥:可能是别的狗的,或者抄错了。")
    except (auth.LocalSignalingPortError, OSError, ValueError):
        fail(2, f"Cannot reach the dog at {where}. Is the Mac's Wi-Fi on the dog's network, and is the dog on?",
             "连不上狗:Mac 的 Wi-Fi 连的是狗的网络吗?狗开机了吗?")
    except (asyncio.TimeoutError, auth.NoSdpAnswerError, auth.DataChannelTimeoutError):
        fail(2, f"The dog did not finish the Wi-Fi handshake within {timeout:.0f} s. Move closer, check that the phone "
                "app is closed, then try again.", "Wi-Fi 握手没完成:靠近一点,确认手机 App 已关,再试一次。")
    except Exception as err:  # anything else the library raises
        fail(2, f"Wi-Fi connection failed: {err}", "Wi-Fi 连接失败,见上面的英文原因。")
    print("connected", flush=True)
    return conn


async def disconnect(conn):
    """Always hang up: the dog accepts only one Wi-Fi client, so a dangling connection blocks the next one."""
    try:
        await asyncio.wait_for(conn.disconnect(), 5)
    except Exception:
        pass


def fail(code, en, zh):
    print(f"{en} / {zh}", flush=True)
    sys.exit(code)
