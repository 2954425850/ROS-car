# /sdcard/main.py
"""CanMV 上电自启钩子。**只在长稳通过后才安装**（2026-09-17 Task 7 已通过）。

按用户 2026-09-17 的决定，这里就是计划原版：`import app; app.main()` 外套 try/except，
**不加哨兵文件、不加开机延时窗口**。

理由（用户已拍板）：整体一起断电时 Pi 的启动时间远长于任何窗口，窗口没用；
而 Ctrl-C 本来任何时候都能打断（app.py 主循环到处都有 sleep），窗口也不多给什么。

恢复路径（已记录在 NOTES.md）：
1. Pi 能 SSH 上、板子能被打断 -> 用 mpremote 改 `main.py` / `config.py`
2. 读卡器改卡（/sdcard 是 FAT，Windows 能直接改）—— 最可靠，不依赖 Pi 也不依赖板子活着
3. 把 K230 的 USB 从 Pi 上拔下来插到 PC，PC 上装 mpremote 直接连它 —— 绕过 Pi
"""
import sys
import time

sys.path.insert(0, "/sdcard/k230vision")

try:
    import app
    app.main()
except Exception as e:
    print("APP CRASH", repr(e))
    time.sleep(5)
