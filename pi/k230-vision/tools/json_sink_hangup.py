#!/usr/bin/env python3
"""探针（不是交付物）：收满 N 条消息后**直接挂断并退出**，
用来验证 reporter.py 的核心契约 —— 「发不出去就丢，绝不排队」。

用法: python3 json_sink_hangup.py <port> <hangup_after>
"""
import socket
import sys
import time

port = int(sys.argv[1])
hangup = int(sys.argv[2])

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", port))
srv.listen(1)
srv.settimeout(60)
print("listening %d, hangup after %d msgs" % (port, hangup), flush=True)
c, a = srv.accept()
print("connected from %s" % (a,), flush=True)
c.settimeout(10.0)

buf = b""
n = 0
t0 = time.time()
while True:
    try:
        d = c.recv(65536)
    except socket.timeout:
        print("RECV_TIMEOUT", flush=True)
        break
    if not d:
        print("EOF", flush=True)
        break
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        if not line.strip():
            continue
        n += 1
        if n >= hangup:
            print("HANGUP after %d msgs (t=%.1fs)" % (n, time.time() - t0), flush=True)
            c.close()
            srv.close()
            sys.exit(0)
print("exited with n=%d" % n, flush=True)
