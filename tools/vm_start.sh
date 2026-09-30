#!/usr/bin/env bash
# Start the Ubuntu VM and wait until it answers.   bash tools/vm_start.sh
# UTM is opened first and asked for the VM's status until it answers: a start command sent while UTM is
# still opening can leave the VM stuck at "starting" with nothing running.
set -u
cd "$(dirname "$0")/.."
VID="$(cat _vm/vm_id.txt)"
status() { perl -e 'alarm 8; exec @ARGV' osascript -e "tell application \"UTM\" to get status of virtual machine id \"$VID\"" 2>/dev/null; }

open -ga UTM
for i in $(seq 1 20); do [ -n "$(status)" ] && break; sleep 1; done
st="$(status)"
case "$st" in
  started) echo "The VM is already running." ;;
  stopped) osascript -e "tell application \"UTM\" to start virtual machine id \"$VID\"" >/dev/null
           echo "starting Ubuntu (about 30 s)..." ;;
  "")      echo "UTM does not answer. Click 'Shut down the VM' (it also resets UTM), then try again."; exit 1 ;;
  *)       echo "The VM is '$st'. If it stays like this, click 'Shut down the VM', then start again."; exit 1 ;;
esac
for i in $(seq 1 30); do
  _vm/vmssh -o ConnectTimeout=3 true 2>/dev/null && { echo "VM is up."; exit 0; }
  sleep 3
done
echo "Ubuntu did not answer within 90 s: click 'Shut down the VM', then 'Start the VM' again."
exit 1
