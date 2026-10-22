#!/usr/bin/env bash
# Make the double-clickable Mac apps for the Go2 control panel.
#   bash tools/make_panel_app.sh              -> two apps on the Desktop
#   bash tools/make_panel_app.sh /Applications
#
# It builds TWO apps from one icon:
#
#   机器狗控制面板.app        the web panel (React in the browser) - the main one
#   机器狗控制面板-经典.app    the original tkinter window - the fallback
#
# Both run the same buttons: web/server.py imports control_panel and uses its
# ACTIONS table, so neither app can have a feature the other lacks. The web
# one adds what a tkinter window cannot do - the live camera, the LiDAR point
# cloud in 3D, and the recorded maps.
#
# What the panels print goes to ~/Library/Logs/Go2Panel.log.
set -e
FT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${1:-$HOME/Desktop}"
PY="$HOME/miniconda3/envs/go2/bin/python"
RPY="$FT/.venv_robot/bin/python"
[ -x "$PY" ] || { echo "missing $PY (the go2 conda env)"; exit 1; }

make_icns() {   # $1 = app path
  local SET
  SET="$(mktemp -d)/AppIcon.iconset"
  mkdir -p "$SET"
  for s in 16 32 128 256 512; do
    sips -z $s $s "$FT/tools/panel_icon.png" --out "$SET/icon_${s}x${s}.png" >/dev/null
    sips -z $((s * 2)) $((s * 2)) "$FT/tools/panel_icon.png" --out "$SET/icon_${s}x${s}@2x.png" >/dev/null
  done
  iconutil -c icns "$SET" -o "$1/Contents/Resources/AppIcon.icns"
}

plist() {       # $1 = app path, $2 = bundle name, $3 = identifier
  cat > "$1/Contents/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>$2</string>
  <key>CFBundleDisplayName</key><string>$2</string>
  <key>CFBundleIdentifier</key><string>$3</string>
  <key>CFBundleExecutable</key><string>launcher</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>2.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>NSLocalNetworkUsageDescription</key>
  <string>Talks to the Unitree Go2 over the Ethernet cable and to the Ubuntu VM.</string>
  <key>NSAppleEventsUsageDescription</key>
  <string>Starts and stops the Ubuntu VM in UTM.</string>
</dict>
</plist>
EOF
}

finish() {      # $1 = app path
  chmod +x "$1/Contents/MacOS/launcher"
  codesign --force --deep -s - "$1" 2>/dev/null || true
  touch "$1"
  echo "made $1"
}

# ------------------------------------------------------------- the web app
APP="$DEST/机器狗控制面板.app"
rm -rf "$APP"; mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
make_icns "$APP"; plist "$APP" "机器狗控制面板" "edu.engr6010.go2panel.web"
cat > "$APP/Contents/MacOS/launcher" <<EOF
#!/bin/bash
# Opens the Go2 web panel. Made by tools/make_panel_app.sh.
#
# Three things this has to get right, because they are what makes it feel
# like an app rather than a script:
#   - double-click starts it
#   - closing the window stops it (the server quits when no page checks in)
#   - clicking the icon again focuses the page that is already open,
#     instead of leaving a row of identical tabs behind
export PATH="\$HOME/miniconda3/envs/go2/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
export LANG=en_US.UTF-8 PYTHONIOENCODING=utf-8
LOG="\$HOME/Library/Logs/Go2Panel.log"
DIR="$FT"
URL="http://127.0.0.1:8742"
cd "\$DIR" || exit 1
{ echo ""; echo "===== $(date '+%F %T')  web panel starting ====="; } >> "\$LOG"

show() {
  /usr/bin/osascript - "127.0.0.1:8742" <<'AS' >/dev/null 2>&1 && return 0
on run argv
  set target to item 1 of argv
  tell application "Google Chrome"
    repeat with w in windows
      repeat with i from 1 to (count of tabs of w)
        if (URL of tab i of w) contains target then
          set active tab index of w to i
          set index of w to 1
          activate
          return
        end if
      end repeat
    end repeat
    open location ("http://" & target)
    activate
  end tell
end run
AS
  open "\$URL"
}

# The robot venv has the Unitree SDK, so live telemetry works.
PYBIN="\$DIR/.venv_robot/bin/python"
[ -x "\$PYBIN" ] || PYBIN="\$HOME/miniconda3/envs/go2/bin/python"

# Start it only if nothing is serving yet. The server is launched in the
# background and this script exits, rather than exec-ing into it: if the
# launcher became the server, macOS would consider the app "running" and
# a second double-click would do nothing at all - not even bring the page
# to the front.
if ! curl -s -o /dev/null --max-time 2 "\$URL/api/status"; then
  GO2_PANEL_NO_OPEN=1 nohup "\$PYBIN" web/server.py >> "\$LOG" 2>&1 &
  for i in \$(seq 1 40); do
    curl -s -o /dev/null --max-time 1 "\$URL/api/status" && break
    sleep 0.25
  done
else
  echo "already running - focusing the existing tab" >> "\$LOG"
fi
show
EOF
finish "$APP"

# -------------------------------------------------------- the classic app
APP2="$DEST/机器狗控制面板-经典.app"
rm -rf "$APP2"; mkdir -p "$APP2/Contents/MacOS" "$APP2/Contents/Resources"
make_icns "$APP2"; plist "$APP2" "机器狗控制面板-经典" "edu.engr6010.go2panel.tk"
cat > "$APP2/Contents/MacOS/launcher" <<EOF
#!/bin/bash
# Opens the original tkinter panel - the fallback if the browser one fails.
export PATH="\$HOME/miniconda3/envs/go2/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
export LANG=en_US.UTF-8 PYTHONIOENCODING=utf-8
cd "$FT" || exit 1
{ echo ""; echo "===== \$(date '+%F %T')  tkinter panel starting ====="; } >> "\$HOME/Library/Logs/Go2Panel.log"
exec "$PY" control_panel.py >> "\$HOME/Library/Logs/Go2Panel.log" 2>&1
EOF
finish "$APP2"

echo ""
echo "Double-click 机器狗控制面板 for the browser panel."
echo "机器狗控制面板-经典 is the old tkinter window, kept as the fallback."
