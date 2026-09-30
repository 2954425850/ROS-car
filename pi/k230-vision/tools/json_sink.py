#!/usr/bin/env python3
"""接收 K230 的结构化结果（换行分隔的 JSON），打印带时间戳的摘要与速率。

用法:
    python3 json_sink.py [port] [duration_s]

输出关键行（验收用）:
    RESULT <n> messages in <dt>s = <rate> msg/s

另打印 FIRST_OBJ_MSG / LAST_OBJ_MSG —— **真实检出**的原始 JSON，
用来证明这条链路上跑的是 KPU 的输出而不是写死的假数据。
"""
import socket
import sys
import json
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8556
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", PORT))
srv.listen(1)
srv.settimeout(DUR + 15)
print(f"listening {PORT} (max {DUR}s)", flush=True)
try:
    c, a = srv.accept()
except socket.timeout:
    print("ACCEPT_TIMEOUT: nobody connected", flush=True)
    sys.exit(1)
print("connected from", a, flush=True)
c.settimeout(5.0)

buf = b""
t0 = None            # 第一条消息到达的时刻
t_last = None
n = 0
bad = 0
first_obj = None
last_obj = None
max_gap = 0
last_frame = None

while True:
    now = time.time()
    if t0 is not None and now - t0 > DUR:
        print("DURATION_REACHED", flush=True)
        break
    try:
        d = c.recv(65536)
    except socket.timeout:
        if n:
            print("RECV_TIMEOUT", flush=True)
            break
        continue
    if not d:
        print("EOF from peer", flush=True)
        break
    if t0 is None:
        t0 = time.time()
    t_last = time.time()
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        line = line.strip()
        if not line:
            continue
        try:
            o = json.loads(line)
        except Exception:
            bad += 1
            continue
        n += 1
        objs = o.get("objs", [])
        if n <= 5 or n % 25 == 0:
            print(f"[{n}] frame={o.get('frame')} objs={len(objs)} {objs[:2]}", flush=True)
        if objs:
            if first_obj is None:
                first_obj = (n, o)
            last_obj = (n, o)
        f = o.get("frame")
        if last_frame is not None and f is not None and (f - last_frame) > max_gap:
            max_gap = f - last_frame
        last_frame = f

span = (t_last - t0) if (t0 and t_last) else 1e-6
dt = max((t_last - t0) if (t0 and t_last) else (time.time() - (t0 or time.time())), 1e-6)
print(f"BAD_LINES {bad}", flush=True)
print(f"MAX_FRAME_GAP {max_gap}", flush=True)
if first_obj:
    print(f"FIRST_OBJ_MSG [{first_obj[0]}] {json.dumps(first_obj[1], ensure_ascii=False)}", flush=True)
if last_obj:
    print(f"LAST_OBJ_MSG [{last_obj[0]}] {json.dumps(last_obj[1], ensure_ascii=False)}", flush=True)
else:
    print("NO_OBJ_MSG: no message carried a real detection", flush=True)
print(f"RESULT {n} messages in {dt:.1f}s = {(n - 1) / dt:.1f} msg/s (span {span:.1f}s)", flush=True)
print(f"RESULT_RAW {n} messages", flush=True)
c.close()
srv.close()
