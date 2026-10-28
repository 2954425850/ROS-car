#!/usr/bin/env bash
# 启动后端（dev profile：DEBUG 日志、放行开发种子口令、注册 Mock 机器人网关）
# server.address 钉死回环：不靠 UFW 兜底，进程本身就不监听对外网卡
set -euo pipefail
set -a; . /opt/robot-upper/env; set +a
exec java -Xmx512m -jar /opt/robot-upper/robot-upper.jar \
  --spring.profiles.active=dev \
  --server.address=127.0.0.1
