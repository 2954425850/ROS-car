#!/usr/bin/env python3
"""raw 读 /dev/ttyACM0，看板子到底在吐什么。不做任何 reset/写文件。"""
import sys
import time

import serial

PORT = "/dev/ttyACM0"
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0

s = serial.Serial(PORT, 115200, timeout=0.3)
s.reset_input_buffer()
# 先尝试用 Ctrl-C 打断（连发三次，MicroPython 需要）
s.write(b"\r\x03\x03\x03")
s.flush()

t0 = time.time()
buf = b""
while time.time() - t0 < DUR:
    d = s.read(4096)
    if d:
        buf += d

print("RAW_BYTES", len(buf))
print("RAW_REPR", repr(buf[:4000]))
s.close()
