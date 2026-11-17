#!/usr/bin/env bash
# carhealth 一期故障注入。默认只读地跑 doctor；需要停服务的 case 会提示 sudo。
set -u

CTL="${CARHEALTHCTL:-carhealthctl}"
case "${1:-}" in
  bridge)
    sudo systemctl stop k230-bridge
    echo "已停 k230-bridge，5s 后看状态（期望 k230_link=ABSENT，且无人重启它）"
    sleep 5
    "$CTL" status
    ;;
  bridge-restore)
    sudo systemctl start k230-bridge
    echo "已恢复 k230-bridge"
    ;;
  power-now)
    echo "伪造当前欠压 0x1（期望 power FAIL；健康的底盘/臂仍 PASS，不被 BLOCKED 掩盖）"
    "$CTL" doctor --no-daemon --fake-throttled 0x1
    ;;
  power-history)
    echo "伪造历史欠压 0x50000（期望 power 只 WARN，不是 FAIL）"
    "$CTL" doctor --no-daemon --fake-throttled 0x50000
    ;;
  board)
    echo "把 K230 指向不可达地址（期望 board FAIL，k230_link/k230_video BLOCKED）"
    "$CTL" doctor --no-daemon --board-host 203.0.113.1
    ;;
  build-stale)
    touch "$(git -C "${CARHEALTH_REPO:-$HOME/ROS-car}" rev-parse --show-toplevel)/pi/raspot_ws/src/arm_grasp/setup.py"
    echo "制造 build 过期（期望出现 build.stale 规则结论）"
    "$CTL" doctor --no-daemon
    ;;
  voice)
    XDG_RUNTIME_DIR="/run/user/$(id -u)" systemctl --user stop voice-chatbot
    echo "已停 voice-chatbot，6s 后看状态（期望 voice=STALE）"
    sleep 6
    "$CTL" status
    ;;
  voice-restore)
    XDG_RUNTIME_DIR="/run/user/$(id -u)" systemctl --user start voice-chatbot
    echo "已恢复 voice-chatbot"
    ;;
  history)
    "$CTL" history --since 30m
    ;;
  *)
    cat <<'EOF'
用法: inject_fault.sh <case>

  bridge          停 k230-bridge，验证 k230_link=ABSENT
  bridge-restore  恢复 k230-bridge
  power-now       伪造当前欠压，验证 BLOCKED 不掩盖健康组件
  power-history   伪造历史欠压，验证 doctor 不会永久 FAIL
  board           伪造板子不可达，验证 board 与 k230_* BLOCKED
  build-stale     制造 build 过期，验证配置漂移规则
  voice           停 voice-chatbot，验证 voice=STALE
  voice-restore   恢复 voice-chatbot
  history         看最近 30 分钟时间线
EOF
    exit 2
    ;;
esac
