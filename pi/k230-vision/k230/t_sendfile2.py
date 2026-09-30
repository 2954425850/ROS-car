# /sdcard/k230vision/t_sendfile2.py
"""把板载文件经 WiFi 原样推给 Pi（TCP 裸字节流）。v2：修 v1 的 EAGAIN。

## v1（t_sendfile.py）的教训 —— 留着不删，因为它记了坑

`netup.connect()` 返回后**立刻**发包，只发出 **42340 B** 就
`OSError: [Errno 11] EAGAIN`（5 s send 超时内发不动）。
这正是 NOTES.md 里那条：「连上后十几秒内速率自适应还没爬升，
此时测吞吐会得到低 2~3 个数量级的假数字」——**必须预热**。

v2 的改动：`connect()` 之后**静置 WARMUP_S 秒**，send 超时放宽到 20 s，
并打印实际速率。

## 为什么需要它

`mpremote fs cp` 传 1.38 MB 走 REPL 通道，实测 90 s 都没传完；
WiFi 实测有 12 Mbps 余量。

## 用法

Pi 侧：`python3 tools/recv_file.py 8557 <out> &`
板子：`mpremote connect /dev/ttyACM0 exec "exec(open('/sdcard/k230vision/t_sendfile2.py').read())"`
"""
import sys
import time
import socket

sys.path.insert(0, "/sdcard/k230vision")

import config
import netup

SRC = "/sdcard/k230vision/raw_chn0.bin"
PORT = 8557
WARMUP_S = 18

netup.init(config.WIFI_SSID, config.WIFI_PASS)
if not netup.connect(25):
    print("FAIL wifi")
    raise SystemExit
print("WIFI OK %s" % netup.ip())
print("WARMUP %ds ..." % WARMUP_S)
time.sleep(WARMUP_S)

s = socket.socket()
s.settimeout(20)
s.connect((config.RESULT_HOST, PORT))
print("CONNECTED %s:%d" % (config.RESULT_HOST, PORT))

f = open(SRC, "rb")
n = 0
chunk = bytearray(4096)
t0 = time.time()
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

el = time.time() - t0
print("SENT %s bytes=%d in %.2fs = %.0f B/s" % (SRC, n, el, n / max(el, 1e-6)))
