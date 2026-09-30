#!/usr/bin/env bash
# L150Pro 底盘驱动启动器
# 桌面图标 / 应用菜单 / 终端 都指向这个文件。想改参数改这里，不用重新生成图标。

cd "$HOME" || exit 1

source /opt/ros/jazzy/setup.bash
source "$HOME/l150pro_ws/install/setup.bash"

echo "=============================================="
echo "  L150Pro 底盘驱动"
echo "=============================================="
echo

# ---- 1) 串口在不在 ----
if [ ! -e /dev/l150pro ]; then
    echo "❌ 找不到 /dev/l150pro"
    echo "   板子没插好 / 没上电 / 这根 USB 线不是数据线？"
    echo
    echo "   当前串口设备："
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "   （一个都没有）"
    echo
    echo "   正常应该看到 /dev/l150pro -> ttyACM0"
    echo
    read -rp "按回车关闭…"
    exit 1
fi
echo "✅ 串口  /dev/l150pro -> $(readlink -f /dev/l150pro)"

# ---- 2) 串口有没有被别的进程占着 ----
# 用 fuser 而不是 pgrep：pgrep -f 会匹配到自己的命令行（自匹配误报），fuser 不会。
holder="$(fuser /dev/l150pro 2>/dev/null)"
if [ -n "$holder" ]; then
    echo
    echo "⚠️  串口 /dev/l150pro 已经被这些进程占着："
    for pid in $holder; do
        ps -o pid=,cmd= -p "$pid" 2>/dev/null | sed 's/^/     /'
    done
    echo "   多半是已经有一个驱动在跑了。两个一起跑会互抢串口，都活不成。"
    echo "   建议先关掉那个窗口（或在那边按 Ctrl-C）。"
    read -rp "   仍要强行继续吗？[y/N] " a
    case "$a" in [yY]*) ;; *) echo "已取消。"; exit 0;; esac
fi

# ---- 3) 起驱动 ----
echo
echo "启动中……（这个窗口要一直开着。要停就按 Ctrl-C）"
echo "----------------------------------------------------------------"
echo

ros2 launch l150pro_driver l150pro.launch.py \
    port:=/dev/l150pro use_ekf:=false

echo
echo "----------------------------------------------------------------"
echo "驱动已退出。"
echo
echo "提示：如果上面报「帧全被 CRC 拒绝」，检查是不是接的【串口1】（不是串口3）。"
read -rp "按回车关闭窗口。"
