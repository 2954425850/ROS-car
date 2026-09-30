#!/usr/bin/env python3
"""尝试对 root hub 的端口 1（K230 所在）做真正的 VBUS 断电-上电。

用 USBDEVFS_CONTROL 在 root hub 设备上发 SetPortFeature/ClearPortFeature(PORT_POWER)。
只有 hub 支持 PPPS 才会真的切电；不支持就返回 EPIPE/EPROTO，无副作用。
注意：只动 bus2 port1（K230 独占），port2 上的无线键鼠 hub 不受影响。
"""
import fcntl
import os
import struct
import time

import serial

USBDEVFS_CONTROL = 0xC0185500
PORT_POWER = 8
SET_FEATURE = 0x03
CLEAR_FEATURE = 0x01

bus, dev = 2, 1  # root hub of bus 2
path = "/dev/bus/usb/%03d/%03d" % (bus, dev)
PORT = 1


def ctrl(fd, req, value, index, length=0):
    buf = bytearray(length)
    data = struct.pack("<BBHHH I xxxxxxxx P", 0x23, req, value, index, length, 1000, buf)
    try:
        fcntl.ioctl(fd, USBDEVFS_CONTROL, data)
        return True, None
    except OSError as e:
        return False, e


fd = os.open(path, os.O_RDWR)
print("opened root hub", path)

ok, err = ctrl(fd, CLEAR_FEATURE, PORT_POWER, PORT)
print("ClearPortFeature(PORT_POWER) port%d ->" % PORT, ok, err)
time.sleep(4)

ok, err = ctrl(fd, SET_FEATURE, PORT_POWER, PORT)
print("SetPortFeature(PORT_POWER) port%d ->" % PORT, ok, err)
os.close(fd)

time.sleep(10)
print("ttyACM exists:", os.path.exists("/dev/ttyACM0"))

for attempt in range(4):
    try:
        s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.3)
    except Exception as e:
        print("open attempt %d failed: %r" % (attempt, e))
        time.sleep(5)
        continue
    s.write(b"\r\x03\x03\x03")
    s.flush()
    t0 = time.time()
    buf = b""
    while time.time() - t0 < 6.0:
        d = s.read(4096)
        if d:
            buf += d
    print("read attempt %d: %d bytes %r" % (attempt, len(buf), buf[:400]))
    s.close()
    if buf:
        break
    time.sleep(4)

print("done")
