#!/bin/bash
# ============================================
# JARVIS Voice Chatbot - 树莓派5 安装脚本
# 使用方法: bash install.sh
# ============================================
set -e

echo "=== 安装系统依赖 ==="
sudo apt-get update
sudo apt-get install -y python3-pip python3-pyaudio portaudio19-dev

echo "=== 安装 Python 包 ==="
pip3 install --break-system-packages -r requirements.txt

echo "=== 校验 MCP 依赖 ==="
# mcp 被钉在 <2（见 requirements.txt 的说明）。这里显式验一次 ——
# 版本漂到 2.x 时的报错信息极难指向根因（会报 FastMCP 不存在），早失败早省事。
python3 - <<'PYCHECK'
import importlib.metadata as md
from mcp.server.fastmcp import FastMCP          # noqa: F401
from mcp.client.stdio import stdio_client       # noqa: F401
import qq_music_api                             # noqa: F401
print(f"  mcp {md.version('mcp')} / qq-music-mcp {md.version('qq-music-mcp')} OK")
PYCHECK

echo "=== 配置唤醒模块串口权限 ==="
# 硬件唤醒模块走 USB 转串口，普通用户默认没有读写权限
sudo usermod -aG dialout "$USER"
echo "  已把 $USER 加入 dialout 组（需重新登录/重启才生效）"
echo "  检查设备: ls -l /dev/ttyUSB*"
echo "  建议用 udev 规则固定成 /dev/myspeech（与 config.yaml 默认值一致），例如:"
echo "    sudo tee /etc/udev/rules.d/99-myspeech.rules >/dev/null <<'EOF'"
echo "    SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7523\", SYMLINK+=\"myspeech\""
echo "    EOF"
echo "    sudo udevadm control --reload && sudo udevadm trigger"

echo "=== 配置 USB 声卡 ==="
# USB 声卡在 card 1, plughw:1,0 支持 16kHz 重采样
# 验证录音:
echo "测试录音 3 秒..."
arecord -D plughw:1,0 -d 3 -f S16_LE -r 16000 -c 1 /tmp/test_mic.wav 2>/dev/null && echo "  麦克风 OK ($(du -h /tmp/test_mic.wav | cut -f1))"

# 验证播放:
echo "测试播放..."
aplay -D plughw:1,0 /tmp/test_mic.wav 2>/dev/null && echo "  扬声器 OK"

echo ""
echo "=== 配置 API Key ==="
# 请在 ~/.bashrc 或当前 shell 中设置:
echo "请设置以下环境变量:"
echo "  export DASHSCOPE_API_KEY=sk-xxxxxxxx"
echo "  export DEEPSEEK_API_KEY=sk-xxxxxxxx"
echo ""
echo "=== 运行 ==="
echo "  cd ~/voice-chatbot"
echo "  python3 main.py"
echo ""
echo "安装完成!"
