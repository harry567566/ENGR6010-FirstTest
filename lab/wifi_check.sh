#!/usr/bin/env bash
# Wi-Fi route, step 1-1: is the Mac on the dog's network, and does the dog answer?
#   bash lab/wifi_check.sh                 the dog's own hotspot (the dog is 192.168.12.1)
#   bash lab/wifi_check.sh 192.168.1.23    the dog and the Mac on the same router
# Nothing is changed on the Mac: joining the dog's Wi-Fi is done in the Wi-Fi menu (top right of the screen).
set -u
cd "$(dirname "$0")/.."
TARGET="${1:-ap}"
WIFI=$(networksetup -listallhardwareports | awk '/Hardware Port: Wi-Fi/{getline; print $2}')
IP=$(ipconfig getifaddr "$WIFI" 2>/dev/null)
echo "Mac Wi-Fi ($WIFI) address: ${IP:-none}"

if [ "$TARGET" = ap ]; then
  DOG=192.168.12.1
  case "$IP" in
    192.168.12.*) echo "The Mac is on the dog's own Wi-Fi." ;;
    *) echo "The Mac is NOT on the dog's Wi-Fi yet. Click the Wi-Fi icon at the top right of the screen and join"
       echo "the dog's network (its name usually starts with Go2; ask the instructor for the password), then click 1-1 again."
       echo "/ Mac 还没连上狗的 Wi-Fi:点屏幕右上角的 Wi-Fi 图标,选狗的网络(名字一般以 Go2 开头,密码问老师),再点一次 1-1。"
       exit 1 ;;
  esac
else
  DOG="$TARGET"
fi

echo "checking the dog at $DOG ..."
ping -c 2 -W 1000 "$DOG" >/dev/null 2>&1 && echo "  ping: answers" || echo "  ping: no answer (some dogs do not answer ping)"
if nc -z -G 2 "$DOG" 9991 2>/dev/null || nc -z -G 2 "$DOG" 8081 2>/dev/null; then
  echo "DOG REACHABLE over Wi-Fi at $DOG"
else
  echo "The dog's Wi-Fi control port does not answer at $DOG: is the dog on (wait 1 minute after power on)?"
  echo "/ 狗的 Wi-Fi 控制端口没有回应:狗开机了吗?开机后要等 1 分钟。"
  exit 3
fi
echo "Close the Unitree phone app: the dog accepts only one Wi-Fi client at a time. / 先关掉手机上的 Unitree App。"

if [ -s lab/.go2_aes_key ] || [ -n "${GO2_AES_KEY:-}" ]; then
  echo "AES key: found (needed only on firmware 1.1.15 or newer)"
else
  echo "AES key: none (fine on firmware 1.1.14 or older; newer firmware needs it, see LAB_CHECKLIST.md)"
fi

if curl -s -m 3 -o /dev/null https://www.apple.com; then
  echo "Internet: yes, through $(route -n get default 2>/dev/null | awk '/interface:/{print $2}')"
else
  echo "Internet: no. The dog does not need it. To keep internet as well, connect the iPhone with a USB cable"
  echo "and turn on Personal Hotspot. / 没有外网:控制狗不需要网;想同时上网,就用数据线连 iPhone 并打开个人热点。"
fi
