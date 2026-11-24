#!/usr/bin/env bash
# 安装开机自启（systemd 系统单元）。卸载：bash uninstall.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
UNIT="$HERE/../../systemd/system/upper.service"

sudo cp "$UNIT" /etc/systemd/system/upper.service
sudo systemctl daemon-reload
sudo systemctl enable --now upper
sleep 2
systemctl --no-pager --lines=0 status upper || true
echo
echo "打开  http://$(hostname -I | awk '{print $1}'):8850"
