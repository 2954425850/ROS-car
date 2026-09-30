#!/usr/bin/env python3
"""接收裸 H.264 字节流并落盘；**容忍发送端重连**（30 分钟长稳测试用）。

## 为什么需要这个变体

`tcp_h264_sink.py` 只 `accept()` **一次**：客户端一掉线，它就退出。
本项目的链路**真的会瞬时塌陷**（NOTES.md 实测到 0.161 Mbps，Task 3/4/7 各遇到一次），
而 `pusher.py` 的正确行为就是「单帧发送超时 -> 断线重连」。两者一撞：
sink 先退出 -> 板子的重连**永远连不上** -> 观察者误判成「板子挂了/代码有问题」。
（Task 7 实测：exp1 里 t≈32 s 一次 `PUSH LOST` 后，sink 立即 RESULT 退出，
板子随后 2 分钟里每一次 connect 都失败，看着像彻底坏了，其实只是没人 listen。）

本工具**一直 listen**，每次客户端进来就接着往同一个文件追加，并逐条打印
每次连接的结果，让「断了几次、断了多久、断了之后恢复没有」变成可验收的输出。

用法:
    python3 tcp_h264_sink_reconnect.py [port] [outfile] [duration_s]

用法与 `tcp_h264_sink.py` 兼容（同样的三个位置参数），可以互换。
"""
import socket
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8555
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/k230.h264"
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 1900.0

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", PORT))
srv.listen(1)
srv.settimeout(1.0)

t0 = time.time()
conns = 0
total = 0
idle_since = t0

print(f"LISTEN {PORT} -> {OUT} (max {DUR:.0f}s, reconnects allowed)", flush=True)

with open(OUT, "wb", buffering=0) as f:
    while time.time() - t0 < DUR:
        try:
            c, a = srv.accept()
        except socket.timeout:
            continue
        conns += 1
        gap = time.time() - idle_since
        c.settimeout(10.0)
        print(f"ACCEPT #{conns} from {a} at t={time.time()-t0:.1f}s "
              f"(idle {gap:.1f}s)", flush=True)
        n = 0
        tconn = time.time()
        while time.time() - t0 < DUR:
            try:
                d = c.recv(65536)
            except socket.timeout:
                break
            except OSError:
                break
            if not d:
                break
            f.write(d)
            n += len(d)
            total += len(d)
        dt = max(time.time() - tconn, 1e-6)
        print(f"CLOSED #{conns} {n} bytes in {dt:.2f}s = {n*8/dt/1e6:.3f} Mbps "
              f"(total {total} B)", flush=True)
        try:
            c.close()
        except OSError:
            pass
        idle_since = time.time()

dt = max(time.time() - t0, 1e-6)
print(f"TOTAL {total} bytes over {conns} conns in {dt:.2f}s = "
      f"{total*8/dt/1e6:.3f} Mbps avg", flush=True)
