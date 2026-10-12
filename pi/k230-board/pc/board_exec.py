"""把本地脚本送到 K230 上跑（PC 侧）。

为什么不用 mpremote：`/sdcard/main.py` 上电自启会挡死 raw REPL，而 **mpremote 打开串口会
翻转 DTR 复位板子** ⇒ 它一开就撞上正在启动的 app.py，永远进不去（2026-10-10 实测）。
这里在一个连接里做完全部动作：Ctrl-C 猛打 → Ctrl-A → 送码 → Ctrl-D → 收输出 →
收尾 Ctrl-B + Ctrl-D 软复位把 app 拉回来。

用法：
    python pc/board_exec.py t_tof.py                        # 只送一个脚本
    python pc/board_exec.py t_tof_locate.py --module tof_locate.py
                                                            # 先垫上本地模块再送脚本

`--module` 是给**还没部署到板子上的新模块**用的：板子 `/sdcard/k230vision/` 里可能没有它
（部署是另一回事），而本工具一次只送一个文件。做法是把模块源码拼在脚本前面，再加一个
`sys.modules` 垫片 —— **模块与测试两个文件都逐字不改**，测试里照常 `import x as y` 就能用。

⚠️ 被 `--module` 垫的模块，其**顶层代码会在板子上的 `__main__` 里执行**。别垫带
`if __name__ == "__main__":` 副作用的模块。
"""
import argparse, os, re, sys, time
import serial


def _shim(module_paths):
    """把本地模块注册进板子的 `sys.modules`，让测试脚本按原样 import 得到。

    ⚠️ MicroPython 的 raw REPL 里**没有** `sys.modules['__main__']`（2026-10-12 实测：
    直接引用它会 `KeyError: '__main__'`）。所以垫片是"造个空对象、把当前 globals 复制进去"。
    """
    out = []
    for p in module_paths or []:
        name = os.path.splitext(os.path.basename(p))[0]
        with open(p, encoding="utf-8") as f:
            out.append(f.read())
        out.append("\nimport sys as _sys\n"
                   "class _Mod(object):\n    pass\n"
                   "_m = _Mod()\n"
                   "for _k, _v in globals().items():\n    setattr(_m, _k, _v)\n"
                   "_sys.modules['%s'] = _m\n\n" % name)
    return "".join(out).encode("utf-8")


def run(port, code_path, modules=None, boot_wait_s=40.0, exec_timeout_s=150.0,
        baud=115200):
    code = _shim(modules) + open(code_path, "rb").read()
    s = serial.Serial(port, baud, timeout=0.2)
    s.reset_input_buffer()

    buf, end = b"", time.time() + boot_wait_s
    while time.time() < end:
        s.write(b"\x03\x03")
        time.sleep(0.25)
        d = s.read(8192)
        if d:
            buf += d
            if b">>>" in buf[-800:] or b"raw REPL" in buf[-800:]:
                break
    else:
        s.close()
        print("连板子失败：Ctrl-C 打不进 raw REPL", file=sys.stderr)
        return 1

    s.write(b"\x01")          # 确保在 raw REPL
    time.sleep(0.4)
    s.read(4096)
    # 分块慢送：USB-CDC **不看波特率**（`s.write(code)` 一坨丢下去是 USB 全速），
    # 板子 REPL 的接收环形缓冲吃不下 5KB 会**丢掉最前面那段**（实测：整文件一坨送
    # ⇒ 只有文件尾部执行，报 `line 3 ... NameError`）。分块 + 小间隔就全到了。
    for i in range(0, len(code), 128):
        s.write(code[i:i + 128])
        time.sleep(0.02)
    time.sleep(0.3)
    s.write(b"\x04")

    out, end = b"", time.time() + exec_timeout_s
    while time.time() < end:
        d = s.read(4096)
        if d:
            out += d
            if b"<<EOT>>" not in out and out.count(b"\x04") >= 2:
                break
        else:
            time.sleep(0.02)

    try:                      # 收尾：把 app.py 拉回来
        s.write(b"\x02"); time.sleep(0.3); s.write(b"\x04")
    except Exception:
        pass
    time.sleep(0.5)
    s.close()

    text = out.decode("utf-8", "replace").replace("\x04", "\n")
    print(text)
    bad = bool(re.search(r"\bFAIL\b|Traceback|EXC", text))
    return 1 if bad else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("script")
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--module", action="append", default=None,
                    help="先垫进板子 sys.modules 的本地模块（可重复）；"
                         "给还没部署到板子上的新模块用")
    a = ap.parse_args()
    sys.exit(run(a.port, a.script, modules=a.module))
