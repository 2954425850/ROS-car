#!/usr/bin/env bash
# L150Pro 键盘遥控测试 —— 一键开
# 桌面图标 / 应用菜单都指向这个文件。
#
# 干两件事：
#   1) 驱动没在跑就把它拉起来（另开一个窗口，沿用 l150pro_start_driver.sh 的检查）
#   2) 在本窗口跑键盘遥控 teleop_l150pro.py（底盘 + 云台）
#
# 用法：
#   双击图标                                = 驱动 + 遥控
#   ./l150pro_start_test.sh --no-driver     = 只跑遥控（自己另外管驱动）
#   ./l150pro_start_test.sh --dry-run       = 演练：不接车，只验证按键映射
#   其余参数原样透传给 teleop（--help 看全部）

set -u

TELEOP="$HOME/teleop_l150pro.py"
DRIVER_SH="$HOME/l150pro_start_driver.sh"
PORT=/dev/l150pro
WAIT_SECS="${L150PRO_WAIT_SECS:-25}"   # 可用环境变量覆盖，便于测试

start_driver=1
teleop_args=()
for a in "$@"; do
    case "$a" in
        --no-driver) start_driver=0 ;;
        --dry-run)   start_driver=0; teleop_args+=("$a") ;;   # 不接车的演练，驱动没必要起
        *)           teleop_args+=("$a") ;;
    esac
done

# 用 fuser 而不是 pgrep：pgrep -f 会匹配到自己的命令行（自匹配误报），fuser 不会。
port_held() { [ -n "$(fuser "$PORT" 2>/dev/null)" ]; }

echo "=============================================="
echo "  L150Pro 键盘遥控测试"
echo "=============================================="
echo

if [ ! -f "$TELEOP" ]; then
    echo "❌ 找不到遥控程序：$TELEOP"
    read -rp "按回车关闭…"
    exit 1
fi

if [ "$start_driver" = 1 ]; then
    # ---- 1) 串口在不在 ----
    if [ ! -e "$PORT" ]; then
        echo "❌ 找不到 $PORT"
        echo "   板子没插好 / 没上电 / 这根 USB 线不是数据线？"
        echo
        echo "   当前串口设备："
        ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "   （一个都没有）"
        echo
        echo "   正常应该看到 $PORT -> ttyACM0"
        echo
        read -rp "按回车关闭…"
        exit 1
    fi
    echo "✅ 串口  $PORT -> $(readlink -f "$PORT")"

    # ---- 2) 驱动在不在跑 ----
    if port_held; then
        echo "✅ 驱动已经在跑，不重复启动"
        echo "   （两个实例会互抢串口、都活不成，所以这里只认串口占用）"
    else
        echo
        spawned=0
        echo "→ 驱动没在跑，另开一个窗口把它拉起来……"
        if command -v gnome-terminal >/dev/null 2>&1 && \
           [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
            gnome-terminal --title="L150Pro 驱动" -- \
                bash -c "$DRIVER_SH; exec bash" &
            spawned=1
        else
            echo "⚠️  没有可用的图形终端（没 DISPLAY 或没 gnome-terminal）。"
            echo "   请自己另开一个终端执行：$DRIVER_SH"
        fi

        printf "   等待驱动就绪"
        i=0
        while [ "$i" -lt $((WAIT_SECS * 2)) ]; do
            port_held && break
            printf "."
            sleep 0.5
            i=$((i + 1))
        done
        echo

        if port_held; then
            echo "✅ 驱动已就绪"
            sleep 1          # 再给节点一点时间把话题挂上
        else
            echo "⚠️  等了 ${WAIT_SECS} 秒，串口仍没被占住 —— 驱动多半没起来。"
            if [ "$spawned" = 1 ]; then
                echo "   去看那个新开的「L150Pro 驱动」窗口里报了什么。"
            else
                echo "   注意：这次并没有把驱动启动起来（没有图形终端）。"
                echo "   请先手动跑：$DRIVER_SH"
            fi
            echo
            read -rp "   仍要继续跑遥控吗（遥控会提示 cmd_vel 没人订阅）？[y/N] " a
            case "$a" in [yY]*) ;; *) echo "已取消。"; exit 0;; esac
        fi
    fi
    echo
fi

echo "----------------------------------------------------------------"
echo "进入键盘遥控：h 看帮助，x 或 Ctrl-C 退出"
echo "（驱动跑在另一个窗口，退出遥控不会关掉它）"
echo "----------------------------------------------------------------"
echo
sleep 1

# teleop 自己会 source ROS 并 exec，所以这里直接把它交出去、不再套一层
exec python3 "$TELEOP" ${teleop_args[@]+"${teleop_args[@]}"}
