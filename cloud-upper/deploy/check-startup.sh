#!/usr/bin/env bash
# 查看后端启动全过程的原始日志（排错用）
echo "=== systemctl status ==="
systemctl status robot-upper --no-pager | head -14
echo
echo "=== 本次启动的完整日志（最后 60 行）==="
journalctl -u robot-upper --no-pager -n 60 --since "5 min ago"
echo
echo "=== 端口 ==="
ss -tlnp | grep -E ':(8080|8555|8556)' || echo "NO LISTENER"
echo
echo "=== 关键字检索 ==="
journalctl -u robot-upper --no-pager --since "10 min ago" | grep -E 'Flyway|Migrating|Successfully applied|Schema-validation|Tomcat started|Started UpperComputer|CommandPump|视频收流|结果收流|ERROR|Caused by' | tail -30
