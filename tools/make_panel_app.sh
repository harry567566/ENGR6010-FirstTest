#!/usr/bin/env bash
# Make a double-clickable Mac app with an icon that opens the control panel (no Terminal window).
#   bash tools/make_panel_app.sh              -> ~/Desktop/机器狗控制面板.app
#   bash tools/make_panel_app.sh /Applications
# The icon is tools/panel_icon.png (redraw it with tools/make_panel_icon.py).
# What the panel prints goes to ~/Library/Logs/Go2Panel.log.
set -e
FT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${1:-$HOME/Desktop}"
APP="$DEST/机器狗控制面板.app"
PY="$HOME/miniconda3/envs/go2/bin/python"
[ -x "$PY" ] || { echo "missing $PY (the go2 conda env)"; exit 1; }

rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"

# icon: all the sizes macOS wants, packed into one .icns
SET="$(mktemp -d)/AppIcon.iconset"
mkdir -p "$SET"
for s in 16 32 128 256 512; do
  sips -z $s $s "$FT/tools/panel_icon.png" --out "$SET/icon_${s}x${s}.png" >/dev/null
  sips -z $((s * 2)) $((s * 2)) "$FT/tools/panel_icon.png" --out "$SET/icon_${s}x${s}@2x.png" >/dev/null
done
iconutil -c icns "$SET" -o "$APP/Contents/Resources/AppIcon.icns"

cat > "$APP/Contents/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>机器狗控制面板</string>
  <key>CFBundleDisplayName</key><string>机器狗控制面板</string>
  <key>CFBundleIdentifier</key><string>edu.engr6010.go2panel</string>
  <key>CFBundleExecutable</key><string>launcher</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>NSLocalNetworkUsageDescription</key>
  <string>Talks to the Unitree Go2 over the Ethernet cable and to the Ubuntu VM.</string>
  <key>NSAppleEventsUsageDescription</key>
  <string>Starts and stops the Ubuntu VM in UTM.</string>
</dict>
</plist>
EOF

cat > "$APP/Contents/MacOS/launcher" <<EOF
#!/bin/bash
# Opens the ENGR 6010 Go2 control panel (made by tools/make_panel_app.sh).
export PATH="\$HOME/miniconda3/envs/go2/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
export LANG=en_US.UTF-8 PYTHONIOENCODING=utf-8
cd "$FT" || exit 1
exec "$PY" control_panel.py >> "\$HOME/Library/Logs/Go2Panel.log" 2>&1
EOF
chmod +x "$APP/Contents/MacOS/launcher"

codesign --force --deep -s - "$APP" 2>/dev/null || true   # local signature, so macOS remembers its permissions
touch "$APP"
echo "made $APP"
