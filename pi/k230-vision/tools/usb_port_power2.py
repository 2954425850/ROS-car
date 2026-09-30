#!/usr/bin/env python3
"""v2：用 ctypes 定义 usbdevfs_ctrltransfer（v1 的 struct 格式串非法）。
对 root hub 端口 1（K230 独占）做 VBUS 断电-上电。不支持 PPPS 时返回 EPIPE/EPROTO，无副作用。
"""
import ctypes
import fcntl
import os
import time

import serial

USBDEVFS_CONTROL = 0xC0185500
PORT_POWER = 8
SET_FEATURE = 0x03
CLEAR_FEATURE = 0x01
PORT = 1


class CtrlTransfer(ctypes.Structure):
    _fields_ = [
        ("bRequestType", ctypes.c_ubyte),
        ("bRequest", ctypes.c_ubyte),
        ("wValue", ctypes.c_ushort),
        ("wIndex", ctypes.c_ushort),
        ("wLength", ctypes.c_ushort),
        ("timeout", ctypes.c_uint),
        ("data", ctypes.c_void_p),
    ]


def ctrl(fd, req, value, index, length=0):
    buf = ctypes.create_string_buffer(max(length, 1))
    t = CtrlTransfer(0x23, req, value, index, length, 1000,
                     ctypes.cast(buf, ctypes.c_void_p))
    raw = ctypes.string_at(ctypes.byref(t), ctypes.sizeof(t))
    try:
        fcntl.ioctl(fd, USBDEVFS_CONTROL, raw)
        return True, None
    except OSError as e:
        return False, e


fd = os.open("/dev/bus/usb/002/001", os.O_RDWR)
print("opened root hub /dev/bus/usb/002/001, struct size", ctypes.sizeof(CtrlTransfer))

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
