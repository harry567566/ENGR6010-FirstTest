#!/bin/bash
# Double-click this to open the Go2 panel in a browser.
# The tkinter panel (机器狗控制面板) still works and is the fallback.
cd "$(dirname "$0")/.." || exit 1
exec .venv_robot/bin/python web/server.py
