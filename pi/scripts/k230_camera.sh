#!/usr/bin/env bash
# K230 摄像头实时预览 —— 一键开
# 桌面图标 / 应用菜单都指向这个文件。
#
# 干一件事：把 K230 的 RTSP 流拉过来、软解、开一个窗口显示实时画面。
#
# 为什么整条管道套在一个 while 里反复重启 —— 这不是偷懒，是这条流的要求：
#   K230 的 RTSP 会话会被板子**频繁掐断**（30 分钟长稳实测 ~57 次真实断连，
#   平均 33 秒一次，板子自己每次 ≤1.6 秒恢复）。断的那一刻 rtspsrc 会直接报错
#   退出 —— 但**一次断开不等于「流结束了」**，所以必须循环重连。
#   别改成去调 rtspsrc 的 timeout/retry 指望它自愈：这条流断的频度下，
#   那种自愈根本追不上，实测过，是死路。
#
# 断流期间画面会黑多久：实测 **3~4.5 秒**。拆开就是「板子自愈 ~1.6s +
#   重启管道握手 + 等下一个 IDR」。这段躲不掉 —— K230 不能主动请求 IDR，
#   GOP 是 25 帧，所以重连后最多要等一整个 GOP 才有干净画面。
#
# 用法：
#   ./k230_camera.sh                        开窗口看实时画面（双击桌面图标走这条）
#   ./k230_camera.sh --test                 不开窗口，只验证流拉不拉得动（SSH 里用）
#   K230_HOST=192.168.1.113 ./k230_camera.sh    板子换了 IP
#   K230_RETRY_DELAY=2 ./k230_camera.sh         断流后多等一会儿再连
#
# 退出：关掉视频窗口 / 按 Ctrl-C / 关掉本终端窗口 —— 三条路都会把 gst 收干净，
#       不会留下野 gst-launch 进程。

set -u

K230_HOST="${K230_HOST:-192.168.1.112}"
RTSP_PORT="${K230_RTSP_PORT:-8554}"
RTSP_URL="${K230_RTSP_URL:-rtsp://$K230_HOST:$RTSP_PORT/k230}"

RETRY_DELAY="${K230_RETRY_DELAY:-1}"    # 断流后隔多久再连（文档给的建议值就是 1 秒）
QUICK_FAILS="${K230_QUICK_FAILS:-5}"    # 连续几次「秒断」就提醒板子可能没了
GST_LOG="${TMPDIR:-/tmp}/k230_camera_gst.log"

# 解码链的三个要点（要改先看 k230-vision/docs/，那里有实测记录）：
#   * h264parse 必需 —— rtph264depay 默认吐 AVCC（长度前缀），不是 Annex-B；
#   * Pi 上没有能用的硬解（/dev/video19 那个 rpivid 没有 GStreamer 元素能驱动），
#     一律软解 avdec_h264，720p 大约吃 17~20% 单核，够用；
#   * latency/drop-on-latency 跟 k230-vision 的 k230ctl 保持同一组值，是那边实测过的。
PIPE=(
    rtspsrc "location=$RTSP_URL" protocols=tcp latency=200 drop-on-latency=true
    ! rtph264depay
    ! h264parse
    ! "video/x-h264,stream-format=byte-stream,alignment=au"
    ! avdec_h264
    ! videoconvert
)

case "${1:-}" in
    --test)  MODE=test;   PIPE+=(! fakesink sync=false) ;;
    --help|-h)
        sed -n '2,25p' "$0"
        exit 0 ;;
    "")      MODE=window; PIPE+=(! autovideosink) ;;
    *)       echo "不认识的参数：$1（只有 --test / --help）"; exit 2 ;;
esac

# 收尾统一走这里：先 SIGINT 给 gst-launch（配合 -e 让它走 EOS 正常收尾），
# 给它 2 秒；还赖着不走就 -9，绝不留野 gst。
gst_pid=""
cleanup() {
    trap - INT TERM HUP                  # 先摘 trap，免得自己再被信号打断、重入
    if [ -n "$gst_pid" ] && kill -0 "$gst_pid" 2>/dev/null; then
        kill -INT "$gst_pid" 2>/dev/null
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$gst_pid" 2>/dev/null || break
            sleep 0.2
        done
        kill -9 "$gst_pid" 2>/dev/null
    fi
    rm -f "$GST_LOG"
    echo
    echo "已退出。"
    exit 0
}
trap cleanup INT TERM HUP

echo "=============================================="
echo "  K230 摄像头实时预览"
echo "=============================================="
echo

# ---- 1) 板子在不在 ----
if ! ping -c1 -W1 "$K230_HOST" >/dev/null 2>&1; then
    echo "❌ ping 不通 $K230_HOST"
    echo "   → 板子可能没上电、没连上这个 WiFi，或者 IP 变了。"
    echo "     板上电后大概要等 20 秒才起得来。"
    echo "     要是 IP 真变了：K230_HOST=新IP $0"
    echo
    read -rp "按回车关闭…"
    exit 1
fi
echo "✅ 板子在线        $K230_HOST"

# ---- 2) RTSP 端口在不在 ----
# 用 bash 内建的 /dev/tcp，不靠 nc 装没装；外面套 timeout 兜住「连上了但不说话」。
if ! timeout 3 bash -c "exec 3<>/dev/tcp/$K230_HOST/$RTSP_PORT" 2>/dev/null; then
    echo "❌ $K230_HOST:$RTSP_PORT 连不上（板子活着，但 RTSP 没在推流）"
    echo "   → 板子上的推流程序没跑起来，这个状态下拉不到任何画面。"
    echo "     去板子那边把 RTSP 推流起起来，再双击本图标。"
    echo
    read -rp "按回车关闭…"
    exit 1
fi
echo "✅ RTSP 端口开着   $K230_HOST:$RTSP_PORT"

# ---- 3) 解码链要用的元素 ----
missing=""
for e in rtspsrc rtph264depay h264parse avdec_h264; do
    gst-inspect-1.0 "$e" >/dev/null 2>&1 || missing="$missing $e"
done
if [ -n "$missing" ]; then
    echo "❌ GStreamer 缺元素：$missing"
    echo "   → 装一下：sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav"
    echo
    read -rp "按回车关闭…"
    exit 1
fi
echo "✅ 解码链就绪      avdec_h264 软解（Pi 上没有能用的硬解）"

# ---- 4) 开窗口得有显示 ----
if [ "$MODE" = window ] && [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    echo "❌ 没有图形显示（DISPLAY 和 WAYLAND_DISPLAY 都是空的）"
    echo "   → 本脚本要开窗口，得在桌面会话里跑 —— 双击桌面图标就行。"
    echo "     只想确认流还在不在，用：$0 --test"
    echo
    read -rp "按回车关闭…"
    exit 1
fi

echo
if [ "$MODE" = test ]; then
    echo "（--test：不开窗口，只解码。看得到帧率输出就说明流是活的）"
else
    echo "画面窗口马上出来。断了会自己重连，不用管。"
fi
echo "退出：关掉视频窗口 / Ctrl-C / 关掉本终端"
echo "----------------------------------------------------------------"
echo

attempt=0
quick=0
while :; do
    attempt=$((attempt + 1))
    if [ "$attempt" -gt 1 ]; then
        echo "→ 第 $attempt 次连接（上一次断开是正常的：这条流平均 33 秒断一次）"
    fi

    started=$(date +%s)
    gst-launch-1.0 -e -q "${PIPE[@]}" 2>"$GST_LOG" &
    gst_pid=$!
    wait "$gst_pid"
    gst_pid=""
    alive=$(( $(date +%s) - started ))

    # 用户自己把视频窗口点掉 —— 那是"我看完了"，不是断流，别再弹一个新窗口出来。
    if grep -qi "window was closed" "$GST_LOG" 2>/dev/null; then
        echo "视频窗口被关掉了，退出。"
        rm -f "$GST_LOG"
        exit 0
    fi
    # 把 gst 的报错压成一行 —— 整页 debug 对用户没用
    if [ -s "$GST_LOG" ]; then
        reason="$(grep -m1 -E '^(ERROR|WARNING|错误|警告)' "$GST_LOG" 2>/dev/null | cut -c1-110)"
        [ -n "$reason" ] && echo "   gst：$reason"
    fi
    echo "   这一轮活了 ${alive} 秒"

    # 「秒断」说明多半不是正常的周期性断流，而是板子那边出事了
    if [ "$alive" -lt 3 ]; then
        quick=$((quick + 1))
    else
        quick=0
    fi

    if [ "$quick" -ge "$QUICK_FAILS" ]; then
        echo "⚠️  连续 $quick 次都在 3 秒内就断了 —— 板子多半已经不在推流了。"
        echo "   去看看 K230 是不是掉电了 / 推流程序挂了。"
        echo "   （这边继续重连，板子一回来画面就自己恢复）"
        quick=0
        sleep 5
    else
        sleep "$RETRY_DELAY"
    fi
done
