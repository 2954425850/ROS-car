#!/usr/bin/env bash
# 语音助手启动包装。
#
# 为什么要包装：systemd 用户单元不会 source ROS 的 setup.bash，
# 而 CarController 需要 rclpy。不写这一层，症状是
# 「ModuleNotFoundError: No module named 'rclpy'」——
# 而且因为 MCP 那边有先例（PATH 不含 ~/.local/bin 导致静默降级），
# 这种失败很容易被当成「小车功能没写对」而查错方向。
set -e

# 脚本自身所在目录 = 语音助手根目录。
# 用推导而不是写死 /home/cy/voice-chatbot，这样代码搬到
# ~/ROS-car/pi/voice-chatbot 之后不用改这里。
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# **ROS 缺失不致命**：小车功能降级（CarController 会在日志里大声报错），
# 但天气、音乐、闲聊都还得能用。所以这里只警告，不退出。
#
# 不能直接 `source` 了事：`set -e` 下 source 一个不存在的文件会立刻退出，
# 而 systemd 单元是 Restart=on-failure —— 那就变成每 10 秒重启一次的
# 崩溃循环，且症状看起来像「加了小车之后助手起不来了」。
for setup in /opt/ros/jazzy/setup.bash "$HERE/../l150pro_ws/install/setup.bash"; do
    if [ -f "$setup" ]; then
        # shellcheck disable=SC1090
        source "$setup"
    else
        echo "警告：找不到 $setup —— 小车功能将不可用，其余功能照常" >&2
    fi
done

cd "$HERE"
exec /usr/bin/python3 main.py "$@"
