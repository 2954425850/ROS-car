# /sdcard/main.py  —— CanMV 上电自启钩子（2026-09-19 恢复）
#
# 恢复前补了一件事：**app.py 主循环里加了 os.exitpoint()**。
# 原注释说"Ctrl-C 本来任何时候都能打断"，但实测不是 —— 自启后 raw REPL
# 进不去、只能拔卡（NOTES.md 记过）。原因是 MicroPython 只在 exitpoint 处
# 处理挂起的键盘中断。补上之后 Ctrl-C / CanMV IDE 的停止按钮就能打断它。
#
# 救援路径（不依赖板子活着）：
#   1. mpremote 打断 -> 改 /sdcard/k230vision/ 下的文件
#   2. 读卡器改卡（/sdcard 是 FAT，Windows 直接改）—— 最可靠
#   3. K230 的 USB 从 Pi 上拔下来插 PC，PC 上 mpremote 直连 —— 绕过 Pi
import sys
import time

sys.path.insert(0, "/sdcard/k230vision")

try:
    import app
    app.main()
except KeyboardInterrupt:
    print("APP INTERRUPTED (Ctrl-C)")
except Exception as e:
    print("APP CRASH", repr(e))
    time.sleep(5)
