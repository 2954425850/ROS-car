#!/usr/bin/env python3
"""尝试从 USB 层把 K230 叫醒：Ctrl-C 连发 + DTR/RTS 抖动。只读不写板载文件。"""
import time

import serial

PORT = "/dev/ttyACM0"


def drain(s, dur, label):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < dur:
        d = s.read(4096)
        if d:
            buf += d
    print("%-28s got %d bytes: %r" % (label, len(buf), buf[:400]))
    return buf


s = serial.Serial(PORT, 115200, timeout=0.2)
print("opened", PORT)
s.dtr = False
s.rts = False
time.sleep(0.3)
drain(s, 2.0, "after open, dtr/rts low")

for i in range(5):
    s.write(b"\x03" * 8)
    s.flush()
    time.sleep(0.4)
    drain(s, 0.6, "ctrl-c burst %d" % i)

s.write(b"\x04")          # EOT = mpremote 的 raw-repl 结束符
s.flush()
drain(s, 1.0, "after EOT")

print("--- DTR/RTS toggle (有些板子会硬复位) ---")
for v in (True, False, True, False):
    s.dtr = v
    s.rts = v
    time.sleep(0.4)
drain(s, 6.0, "after dtr/rts toggle")

print("--- 再来一轮 ctrl-c ---")
for i in range(3):
    s.write(b"\r\x03\x03\x03")
    s.flush()
    time.sleep(0.5)
    drain(s, 1.0, "post-toggle ctrl-c %d" % i)

s.close()
print("done")
