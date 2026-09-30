#!/usr/bin/env python3
"""接收板子经 WiFi 推来的裸字节流并落盘。

为什么不用 `mpremote fs cp`：REPL 通道传 1.38 MB 实测 90 s 都传不完，
而板子 WiFi 有 12 Mbps 余量。配合板载 `t_sendfile.py` 使用。

用法: python3 recv_file.py <port> <outpath>
"""
import socket
import sys

port = int(sys.argv[1])
out = sys.argv[2]

s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", port))
s.listen(1)
print("listening %d -> %s" % (port, out), flush=True)
c, a = s.accept()
print("from %s" % (a,), flush=True)
n = 0
with open(out, "wb") as f:
    while True:
        d = c.recv(65536)
        if not d:
            break
        f.write(d)
        n += len(d)
c.close()
s.close()
print("RECV %d -> %s" % (n, out), flush=True)
