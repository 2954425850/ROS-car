"""把本地脚本送到 K230 上跑，或把本地文件写到板子上（PC 侧）。

为什么不用 mpremote：`/sdcard/main.py` 上电自启会挡死 raw REPL，而 **mpremote 打开串口会
翻转 DTR 复位板子** ⇒ 它一开就撞上正在启动的 app.py，永远进不去（2026-10-10 实测）。
这里在一个连接里做完全部动作：Ctrl-C 猛打 → Ctrl-A → 送码 → Ctrl-D → 收输出 →
收尾 Ctrl-B + Ctrl-D 软复位把 app 拉回来。

用法：
    python pc/board_exec.py t_tof.py                        # 只送一个脚本
    python pc/board_exec.py t_tof_locate.py --module tof_locate.py
                                                            # 先垫上本地模块再送脚本
    python pc/board_exec.py --put tof_locate.py             # 只把文件写到板子上
    python pc/board_exec.py --put app.py --put tof_locate.py t_app.py
                                                            # 先落盘两个文件，再跑脚本

`--module` 是给**还没部署到板子上的新模块**用的：板子 `/sdcard/k230vision/` 里可能没有它
（部署是另一回事），而本工具一次只送一个文件。做法是把模块源码拼在脚本前面，再加一个
`sys.modules` 垫片 —— **模块与测试两个文件都逐字不改**，测试里照常 `import x as y` 就能用。

⚠️ 被 `--module` 垫的模块，其**顶层代码会在板子上的 `__main__` 里执行**。别垫带
`if __name__ == "__main__":` 副作用的模块。

`--put <本地文件>`（可重复）把本地文件真正写到板子上（`<--dir，默认 /sdcard/k230vision>/
<basename>`）：这是 mpremote 进不去之后，唯一能把文件落盘的机制。每个文件板上按
**写 `<target>.new` → 回读核对大小 → 备份旧文件(`.bak-<ts>`) → `os.rename` 就位**
的顺序做，任一步失败都不碰旧文件、并且立刻中止（返回码 1）。二进制内容用 base64 传，
板上按 ≤256 字节分块 `f.write`，**收尾一定 `f.close()`**（板上不 close 会静默 0 字节）。
"""
import argparse, os, posixpath, re, sys, time
import serial

DEFAULT_DIR = "/sdcard/k230vision"


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


def _put_code(remote, data, ts):
    """生成在板上把 `data` 写到 `remote` 的 raw-REPL 脚本。

    板上流程（**顺序很关键**，任一步出错就只打印标记、由 PC 侧判返回码）：
      1. 写临时名 `<remote>.new`：open("wb") → 分块写（每块 ≤256 字节）→ **close()**；
      2. 回读 `<remote>.new` 长度核对；**不符只报 `PUTSIZERR`、绝不去动旧文件**；
      3. 旧文件存在则 `os.rename(<remote>, <remote>.bak-<ts>)`；
      4. `os.rename(<remote>.new, <remote>)` 就位。
    （板上 `os.rename` 覆盖已存在的路径会失败 ⇒ 第 3 步"先挪开"必不可少。）
    """
    import base64
    b64 = base64.b64encode(data).decode("ascii")
    lit = "\n".join('    "%s"' % b64[i:i + 76] for i in range(0, len(b64), 76))
    head = ("import ubinascii as _ub, os as _os\n"
            "_p = %r\n"
            "_d = _ub.a2b_base64(\n%s\n)\n" % (remote, lit))
    body = (
        "try:\n"
        "    _t = _p + \".new\"\n"
        "    _f = open(_t, \"wb\")\n"
        "    for _i in range(0, len(_d), 256):\n"
        "        _f.write(_d[_i:_i + 256])\n"
        "    _f.close()\n"
        "    _g = open(_t, \"rb\")\n"
        "    _n = len(_g.read())\n"
        "    _g.close()\n"
        "    if _n != %d:\n"
        "        print(\"PUTSIZERR %%s %%d %%d\" %% (_p, _n, %d))\n"
        "    else:\n"
        "        _bk = 0\n"
        "        try:\n"
        "            _os.stat(_p)\n"
        "            _os.rename(_p, _p + \".bak-\" + %r)\n"
        "            _bk = 1\n"
        "        except OSError:\n"
        "            pass\n"
        "        _os.rename(_t, _p)\n"
        "        print(\"PUTOK %%s %%d %%d\" %% (_p, _n, _bk))\n"
        "except Exception as _e:\n"
        "    print(\"PUTERR %%s %%r\" %% (_p, _e))\n"
    ) % (len(data), len(data), ts)
    return (head + body).encode("utf-8")


def _exec_raw(s, code, timeout_s):
    """在一个已经在 raw REPL 的串口上送一段代码去执行，返回解码后的输出文本。"""
    # 分块慢送：USB-CDC **不看波特率**（`s.write(code)` 一坨丢下去是 USB 全速），
    # 板子 REPL 的接收环形缓冲吃不下 5KB 会**丢掉最前面那段**（实测：整文件一坨送
    # ⇒ 只有文件尾部执行，报 `line 3 ... NameError`）。分块 + 小间隔就全到了。
    for i in range(0, len(code), 128):
        s.write(code[i:i + 128])
        time.sleep(0.02)
    time.sleep(0.3)
    s.write(b"\x04")

    out, end = b"", time.time() + timeout_s
    while time.time() < end:
        d = s.read(4096)
        if d:
            out += d
            if b"<<EOT>>" not in out and out.count(b"\x04") >= 2:
                break
        else:
            time.sleep(0.02)
    return out.decode("utf-8", "replace").replace("\x04", "\n")


def put_file(s, local, remote_dir, ts, timeout_s):
    """把本地 `local` 写到板子 `<remote_dir>/<basename>`；返回 True/False 并打一行结果。"""
    try:
        with open(local, "rb") as f:
            data = f.read()
    except OSError as e:
        print("PUT  读本地失败 %s: %s" % (local, e), file=sys.stderr)
        return False

    remote = posixpath.join(remote_dir, os.path.basename(local))
    text = _exec_raw(s, _put_code(remote, data, ts), timeout_s)
    n = len(data)
    m = re.search(r"PUTOK (\S+) (\d+) (\d+)", text)
    if not m:
        print("PUT  %s -> %s 失败（本地 %d 字节）" % (local, remote, n))
        for ln in text.strip().splitlines()[-6:]:
            print("      | " + ln)
        return False
    board_n, bak = int(m.group(2)), int(m.group(3))
    print("PUT  %s  local=%d  board=%d  backup=%s  -> %s"
          % (local, n, board_n, "yes" if bak else "no", remote))
    if board_n != n:
        print("PUT  板上字节数与本地不一致，中止", file=sys.stderr)
        return False
    return True


K230_USB_ID = (0x1209, 0xABD1)     # 庐山派 CanMV


def _check_is_k230(port):
    """开口前核对 USB VID:PID 是 K230。不是就拒绝 —— 别把 Ctrl-C/Ctrl-A 灌进别的板子。

    ★ 2026-10-21：ttyACM 编号随枚举顺序变，那次开机 K230 和底盘板对调了，
      对着 /dev/ttyACM1（那天是底盘）跑了两次本工具。查不到 USB 信息（Windows COM
      口、非 USB 设备）时只警告不拦，免得 PC 侧没法用。
    """
    try:
        from serial.tools import list_ports
        real = os.path.realpath(port)
        for p in list_ports.comports():
            if os.path.realpath(p.device) == real or p.device == port:
                if p.vid is None:
                    break
                if (p.vid, p.pid) != K230_USB_ID:
                    raise SystemExit("拒绝：%s 是 %04x:%04x（%s），不是 K230 %04x:%04x。"
                                     "Pi 上请用 /dev/k230（systemd/99-k230.rules）"
                                     % (port, p.vid, p.pid, p.description or "?",
                                        K230_USB_ID[0], K230_USB_ID[1]))
                return
    except SystemExit:
        raise
    except Exception as e:
        print("WARN  查不到 %s 的 USB 信息（%s），未核对是不是 K230" % (port, e), file=sys.stderr)
        return
    print("WARN  %s 不在串口列表里或无 USB 信息，未核对是不是 K230" % port, file=sys.stderr)


def run(port, code_path=None, modules=None, boot_wait_s=40.0, exec_timeout_s=150.0,
        baud=115200, puts=None, remote_dir=DEFAULT_DIR):
    puts = list(puts or [])
    _check_is_k230(port)
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

    s.write(b"\x01")          # 确保在 raw REPL（之后各段直接接着送即可）
    time.sleep(0.4)
    s.read(4096)

    rc = 0
    ts = time.strftime("%Y%m%d-%H%M%S")   # 时间戳用 PC 本地时间，作为参数传给板子
    for local in puts:
        if not put_file(s, local, remote_dir, ts, exec_timeout_s):
            rc = 1
            break             # 报错退出，不要往下走

    if rc == 0 and code_path:
        text = _exec_raw(s, _shim(modules) + open(code_path, "rb").read(),
                         exec_timeout_s)
        print(text)
        if re.search(r"\bFAIL\b|Traceback|EXC", text):
            rc = 1

    try:                      # 收尾：把 app.py 拉回来
        s.write(b"\x02"); time.sleep(0.3); s.write(b"\x04")
    except Exception:
        pass
    time.sleep(0.5)
    s.close()
    return rc


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("script", nargs="?", default=None,
                    help="要送到板子上执行的脚本（可选）")
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--module", action="append", default=None,
                    help="先垫进板子 sys.modules 的本地模块（可重复）；"
                         "给还没部署到板子上的新模块用")
    ap.add_argument("--put", action="append", default=None, metavar="FILE",
                    help="把本地文件写到板子上（可重复）；默认落到 --dir 下、同名")
    ap.add_argument("--dir", default=DEFAULT_DIR,
                    help="--put 的板端目标目录（默认 %s）" % DEFAULT_DIR)
    a = ap.parse_args()
    if not a.script and not a.put:
        ap.error("至少要给一个 script 位置参数，或一个 --put 文件")
    sys.exit(run(a.port, a.script, modules=a.module, puts=a.put, remote_dir=a.dir))
