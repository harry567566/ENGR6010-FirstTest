#!/usr/bin/env bash
# Shut the Ubuntu VM down, even when it is stuck.   bash tools/vm_stop.sh [--quit-utm]
# 1. a clean shutdown through SSH   2. UTM's force stop   3. if UTM itself does not answer, close UTM.
# --quit-utm also closes UTM at the end (the control panel does this when it is closed).
set -u
cd "$(dirname "$0")/.."
VID="$(cat _vm/vm_id.txt)"
utm() { perl -e 'alarm 10; exec @ARGV' osascript -e "tell application \"UTM\" to $1 virtual machine id \"$VID\"$2" 2>/dev/null; }
close_utm() {
  perl -e 'alarm 10; exec @ARGV' osascript -e 'tell application "UTM" to quit' >/dev/null 2>&1
  sleep 2
  pgrep -x UTM >/dev/null && { kill -TERM $(pgrep -x UTM); sleep 3; }
  pgrep -x UTM >/dev/null && kill -KILL $(pgrep -x UTM)
}

pgrep -x UTM >/dev/null || { echo "UTM is not running, so the VM is already off."; exit 0; }
if _vm/vmssh -o ConnectTimeout=3 '(sleep 1; sudo systemctl poweroff) >/dev/null 2>&1 &' 2>/dev/null; then
  echo "shutting Ubuntu down..."
  for i in $(seq 1 30); do [ "$(utm "get status of" "")" = stopped ] && break; sleep 2; done
fi
if [ "$(utm "get status of" "")" != stopped ]; then
  echo "no clean shutdown: forcing the VM off"
  utm stop " by force" >/dev/null
  sleep 3
  if [ "$(utm "get status of" "")" != stopped ]; then
    echo "UTM does not answer: closing UTM"
    close_utm
  fi
fi
[ "${1:-}" = "--quit-utm" ] && pgrep -x UTM >/dev/null && close_utm
echo "VM off."
