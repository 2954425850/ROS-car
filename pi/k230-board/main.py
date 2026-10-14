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
import machine

sys.path.insert(0, "/sdcard/k230vision")

# 2026-10-14 加崩溃自愈：app.main() 现在返回 True/False（正常/崩溃）。
# 崩溃后同一 MicroPython 会话无法重建 MPP 管线（MediaManager link failed(9)），
# 只有 machine.reset() 一条路，所以崩溃退出就复位重来；但 60 秒内连续崩
# CRASH_LIMIT 次就停下来停在 REPL 等救援，别无限复位转圈。
CRASH_MARK = "/sdcard/k230vision/_crash_ts"
CRASH_WINDOW_S = 60        # 距上次崩溃超过这个秒数，连续计数清零
CRASH_LIMIT = 3            # 窗口内连续崩这么多次就不再复位，停在 REPL


def _read_crash():
    try:
        with open(CRASH_MARK) as f:
            parts = f.read().strip().split(":")
            n = int(parts[0])
            ts = int(parts[1]) if len(parts) > 1 else 0
            return n, ts
    except Exception:
        return 0, 0


def _write_crash(n, ts):
    try:
        with open(CRASH_MARK, "w") as f:
            f.write("%d:%d" % (n, ts))
    except Exception:
        pass


try:
    import app
    ok = app.main()
    if ok:
        _write_crash(0, 0)      # 正常退出（Ctrl-C 或清理完）清零崩溃计数
    else:
        # app.main() 在异常里退出（已打印 traceback 并清理完管线）。
        n, ts0 = _read_crash()
        now = int(time.time())
        if now - ts0 > CRASH_WINDOW_S:
            n = 0
        n += 1
        _write_crash(n, now)
        if n >= CRASH_LIMIT:
            print("APP CRASH LOOP (x%d/%ds) —— 停在 REPL 等待救援" % (n, CRASH_WINDOW_S))
        else:
            print("APP CRASH (x%d) —— 3s 后 machine.reset()" % n)
            time.sleep(3)
            machine.reset()
except KeyboardInterrupt:
    # app.main() 内部已单独捕获 Ctrl-C 并正常返回 True，通常到不了这里；
    # 留着兜底（比如 import app 阶段被打断）。
    print("APP INTERRUPTED (Ctrl-C)")
except Exception as e:
    print("APP CRASH (import/boot)", repr(e))
    time.sleep(5)
