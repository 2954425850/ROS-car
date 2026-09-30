# /sdcard/k230vision/t_sendfile.py
"""把板载文件经 WiFi 原样推给 Pi（TCP 裸字节流）。

为什么要它：`mpremote fs cp` 传 1.38 MB 走的是 REPL 通道，实测 90 s 都传不完；
而板子 WiFi 实测有 12 Mbps 余量。本脚本不碰媒体管线，随便跑。

用法（Pi 侧先监听）:
    nc -l 8557 > /home/cy/k230-vision/raw_chn0.bin
板子侧:
    timeout 90 mpremote connect /dev/ttyACM0 exec "exec(open('/sdcard/k230vision/t_sendfile.py').read())"
"""
import sys
import socket

sys.path.insert(0, "/sdcard/k230vision")

import config
import netup

SRC = sys.argv[1] if len(getattr(sys, "argv", [])) > 1 else "/sdcard/k230vision/raw_chn0.bin"
PORT = 8557

netup.init(config.WIFI_SSID, config.WIFI_PASS)
if not netup.connect(25):
    print("FAIL wifi")
    raise SystemExit
print("WIFI OK %s" % netup.ip())

s = socket.socket()
s.settimeout(5)
if not s.connect((config.RESULT_HOST, PORT)):
    pass
f = open(SRC, "rb")
n = 0
chunk = bytearray(4096)
try:
    while True:
        k = f.readinto(chunk)
        if not k:
            break
        s.send(chunk[:k])
        n += k
finally:
    f.close()
    s.close()
print("SENT %s bytes=%d" % (SRC, n))
