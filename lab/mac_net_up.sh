#!/usr/bin/env bash
# Connect this Mac directly to the Go2 over the USB-Ethernet adapter (no VM), then ping the dog.
#   bash lab/mac_net_up.sh
# The Go2's onboard computer is 192.168.123.161; the development computer uses 192.168.123.222/24.
# Setting a network address needs an administrator, so macOS asks for the Mac password once.
# Only the wired adapter is changed; Wi-Fi (internet) stays as it is.
set -u
ROBOT=192.168.123.161
ME=192.168.123.222

# ---- which wired port has the cable? --------------------------------------------------------
# network services with a device; skip Wi-Fi, Bluetooth, Thunderbolt, iPhone and Jetson (Tegra) links
SVC=""; DEV=""
while IFS='|' read -r name dev; do
  [ "$(ifconfig "$dev" 2>/dev/null | awk '/status:/{print $2}')" = "active" ] && { SVC="$name"; DEV="$dev"; break; }
done < <(networksetup -listnetworkserviceorder | awk '
  /^\([0-9]+\) /{sub(/^\([0-9]+\) /,""); name=$0}
  /Device: en/{d=$0; sub(/.*Device: /,"",d); sub(/\).*/,"",d)
               if (name !~ /Wi-Fi|Bluetooth|Thunderbolt|iPhone|Tegra/) print name"|"d}')
if [ -z "$DEV" ]; then
  echo "No wired port with a cable found on the Mac."
  echo "Plug the USB-C to Ethernet adapter into the Mac and the cable into the dog, turn the dog on, then try again."
  echo "/ Mac 上没找到插着网线的有线网口:把 USB-C 转网口插到 Mac,网线接狗,狗开机后再试。"
  exit 1
fi
echo "wired port with a cable: $DEV ($SVC)"

# ---- give it 192.168.123.222 (once) ---------------------------------------------------------
if ifconfig "$DEV" | grep -q "inet $ME "; then
  echo "$DEV already has $ME"
else
  echo "setting $DEV to $ME (macOS asks for your Mac password)..."
  osascript - "$SVC" "$ME" <<'OSA' || { echo "Not changed (password dialog cancelled?)."; exit 1; }
on run argv
	do shell script "/usr/sbin/networksetup -setmanual " & quoted form of (item 1 of argv) & " " & (item 2 of argv) & " 255.255.255.0" with administrator privileges
end run
OSA
  for i in $(seq 1 10); do ifconfig "$DEV" | grep -q "inet $ME " && break; sleep 1; done
fi
ifconfig "$DEV" | awk '/inet /{print "  " $1, $2}'

# ---- ping the dog ---------------------------------------------------------------------------
echo "pinging the robot..."
if ping -c 3 -W 1000 "$ROBOT"; then
  echo "ROBOT REACHABLE on $DEV"
else
  echo "no reply: check the cable, that the dog is on (wait 1 minute after power on), and turn off any VPN."
  echo "/ ping 不通:检查网线、狗是否开机(开机后等 1 分钟)、关掉 VPN(比如快帆)。"
  exit 3
fi
