#!/usr/bin/env bash
# Backup route (through the VM): give the UTM Ubuntu VM a second network card, bridged to the Mac's
# wired (USB-Ethernet) port that is plugged into the Go2, then set it up and ping the dog.
# Run on the Mac:   bash lab/add_robot_nic.sh [enX]
#
# 1. find the Mac's active wired interface (or use the one given)
# 2. if the VM does not have a card on it yet: shut the VM down, add a Bridged network device on that
#    interface through UTM's AppleScript
# 3. make sure the VM is running, then run robot_net_up.sh on the new card inside the VM
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"
VMD="$HERE/_vm"
VID="$(cat "$VMD/vm_id.txt")"
CFG="$HOME/Library/Containers/com.utmapp.UTM/Data/Documents/Go2-Ubuntu22.utm/config.plist"
status() { osascript -e "tell application \"UTM\" to get status of virtual machine id \"$VID\"" 2>/dev/null; }

# ---- 1. which Mac port is the cable in? -------------------------------------------------
IF="${1:-}"
if [ -z "$IF" ]; then
  while read -r port dev; do
    st=$(ifconfig "$dev" 2>/dev/null | awk '/status:/{print $2}')
    [ "$st" = "active" ] && { IF="$dev"; echo "wired port with a cable: $dev ($port)"; break; }
  done < <(networksetup -listallhardwareports | awk '/Hardware Port:/{sub(/Hardware Port: /,""); p=$0}
            /Device:/{ if (p !~ /Wi-Fi|Bluetooth|Thunderbolt|iPhone|Tegra/) print p"|"$2 }' | tr ' ' '_' | tr '|' ' ')
fi
if [ -z "$IF" ]; then
  echo "No wired port with a cable found on the Mac."
  echo "Plug the USB-C to Ethernet adapter into the Mac and the cable into the dog, turn the dog on, then try again."
  echo "/ Mac 上没找到插着网线的有线网口:把 USB-C 转网口插到 Mac,网线接狗,狗开机后再试。"
  exit 1
fi

# ---- 2. add the card, unless the VM already has one on this port -----------------------------
if plutil -convert json -o - "$CFG" 2>/dev/null | python3 -c "
import json,sys; c=json.load(sys.stdin)
sys.exit(0 if any(n.get('Mode')=='Bridged' and n.get('BridgeInterface')=='$IF' for n in c.get('Network',[])) else 1)"; then
  echo "The VM already has a card bridged to $IF."
else
  echo "shutting the VM down..."
  "$VMD/vmssh" -o ConnectTimeout=4 '(sleep 1; sudo systemctl poweroff) >/dev/null 2>&1 &' >/dev/null 2>&1
  for i in $(seq 1 45); do [ "$(status)" = "stopped" ] && break; sleep 2; done
  if [ "$(status)" != "stopped" ]; then
    osascript -e "tell application \"UTM\" to stop virtual machine id \"$VID\" by request" >/dev/null 2>&1
    for i in $(seq 1 20); do [ "$(status)" = "stopped" ] && break; sleep 2; done
  fi
  [ "$(status)" = "stopped" ] || { echo "The VM did not stop; stop it in UTM and run this again."; exit 1; }

  osascript - "$VID" "$IF" <<'OSA' || { echo "UTM refused the change."; exit 1; }
on run argv
	tell application "UTM"
		set vm to virtual machine id (item 1 of argv)
		set cfg to configuration of vm
		set nis to network interfaces of cfg
		set end of nis to {mode:bridged, host interface:(item 2 of argv)}
		set network interfaces of cfg to nis
		update configuration of vm with cfg
	end tell
end run
OSA
  echo "Added bridged card on $IF."
fi

# ---- 3. VM running? then set up the card inside it --------------------------------------------
if ! "$VMD/vmssh" -o ConnectTimeout=3 true >/dev/null 2>&1; then
  [ "$(status)" = "started" ] || osascript -e "tell application \"UTM\" to start virtual machine id \"$VID\"" >/dev/null
  echo "starting the VM..."
  for i in $(seq 1 40); do
    "$VMD/vmssh" -o ConnectTimeout=3 true >/dev/null 2>&1 && break
    sleep 3
  done
fi
"$VMD/vmssh" 'mkdir -p ~/q2/lab' && "$VMD/vmssh" --scp "$HERE/lab/robot_net_up.sh" "harry@$(cat "$VMD/vm_ip.txt"):q2/lab/" \
  || { echo "Cannot reach the VM yet; wait a few seconds and click this button again."; exit 1; }
# the VM card that belongs to this port has the MAC address UTM stored for it
MAC=$(plutil -convert json -o - "$CFG" | python3 -c "
import json,sys; c=json.load(sys.stdin)
print(next(n['MacAddress'] for n in c['Network'] if n.get('Mode')=='Bridged' and n.get('BridgeInterface')=='$IF'))")
"$VMD/vmssh" "MAC=$MAC bash -s" <<'EOS'
echo "VM network cards:"; ip -br link | sed "s/^/  /"
IFACE=$(ip -br link | awk -v m="$MAC" 'tolower($3)==tolower(m) {print $1}')
[ -n "$IFACE" ] || { echo "The VM does not show the new card ($MAC)."; exit 2; }
echo "robot card in the VM: $IFACE"
bash ~/q2/lab/robot_net_up.sh "$IFACE"
EOS
