#!/usr/bin/env python3
"""接收裸 H.264 字节流并落盘，供人工/工具验证。"""
import socket, sys, time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8555
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/k230.h264"
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", PORT))
srv.listen(1)
print(f"listening {PORT} -> {OUT}", flush=True)
c, a = srv.accept()
print("connected from", a, flush=True)
c.settimeout(5.0)
t0 = time.time(); n = 0; last = t0
with open(OUT, "wb") as f:
    while time.time() - t0 < DUR:
        try:
            d = c.recv(65536)
        except socket.timeout:
            if n: break
            continue
        if not d: break
        f.write(d); n += len(d); last = time.time()
dt = max(last - t0, 1e-6)
print(f"RESULT {n} bytes in {dt:.2f}s = {n*8/dt/1e6:.3f} Mbps", flush=True)
