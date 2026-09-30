#!/usr/bin/env python3
"""对 K230 的 USB 端口发一次真正的 bus reset（USBDEVFS_RESET ioctl），
比 unbind/bind 的"逻辑重枚举"更硬。需要 root。之后尝试读串口。
"""
import fcntl
import os
import time

import serial

USBDEVFS_RESET = 21780  # _IO('U', 20)


def read_sys(p):
    with open(p) as f:
        return f.read().strip()


bus = read_sys("/sys/bus/usb/devices/2-1/busnum")
dev = read_sys("/sys/bus/usb/devices/2-1/devnum")
path = "/dev/bus/usb/%s/%s" % (bus.zfill(3), dev.zfill(3))
print("target", path, "bus", bus, "dev", dev)

fd = os.open(path, os.O_RDWR)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    print("USBDEVFS_RESET ok")
except OSError as e:
    print("USBDEVFS_RESET failed:", e)
finally:
    os.close(fd)

time.sleep(8)

for attempt in range(3):
    try:
        s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.3)
    except Exception as e:
        print("open attempt %d failed: %r" % (attempt, e))
        time.sleep(5)
        continue
    s.reset_input_buffer()
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
    time.sleep(5)

print("done")
