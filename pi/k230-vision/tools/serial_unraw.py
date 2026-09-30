#!/usr/bin/env python3
"""假设板子卡在 MicroPython 的 raw-REPL 模式（mpremote 被 timeout 杀掉留下的），
raw 模式里 Ctrl-C 是无效的，必须用 Ctrl-B 退回 friendly REPL。
本脚本只发控制字符并读回显，不写任何板载文件。
"""
import time

import serial

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.3)


def show(label, dur=1.5):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < dur:
        d = s.read(4096)
        if d:
            buf += d
    print("%-34s %d bytes: %r" % (label, len(buf), buf[:500]))
    return buf


show("baseline", 1.0)

s.write(b"\x02")            # Ctrl-B: 从 raw REPL 退回 friendly REPL
s.flush()
show("after Ctrl-B")

s.write(b"\x03\r\n")
s.flush()
show("after Ctrl-C + CRLF")

s.write(b"\x01\x04")        # Ctrl-A (raw) + Ctrl-D (execute empty)
s.flush()
show("after Ctrl-A + Ctrl-D")

s.write(b"\x02")
s.flush()
show("after Ctrl-B again")

s.write(b"\x01")
s.flush()
time.sleep(0.3)
s.write(b"\x04")
s.flush()
show("after Ctrl-A + Ctrl-D (2)")

s.close()
print("done")
