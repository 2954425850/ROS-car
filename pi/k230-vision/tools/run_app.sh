#!/bin/sh
# 在 K230 上跑 /sdcard/k230vision/app.py，用 timeout 包住防止挂死终端。
# 用法: run_app.sh <seconds> [logfile]
DUR="${1:-1800}"
LOG="${2:-/tmp/k230_app.log}"
exec >"$LOG" 2>&1
echo "RUN_APP start $(date +%H:%M:%S) dur=${DUR}s"
exec timeout "$DUR" /home/cy/.local/bin/mpremote connect /dev/ttyACM0 exec "exec(open('/sdcard/k230vision/app.py').read())"
