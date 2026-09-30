# /sdcard/k230vision/t_push_stress.py
"""Task 3 压力验证：接收端消失 -> 推流器发现 -> 接收端回来 -> 自动恢复。

协议（Pi 侧 orchestrator 负责杀/重启接收端）：
    接收端在 -> 跑 DUR 秒；中途 kill -9 掉 sink，8 s 后重启 sink。

期望：K230 侧出现 `LOST at ~10.x` 与 `RECONNECTED at ~18.x`，看门狗报
`STALLED`，且**板子不崩**（V4 风险点）。

与计划 Task 3 Step 1 那段样例代码的两处不同（都已在上报里说明）：
1. 断线时**仍然调用 p.pump()** —— pump 在未连接时会 GetStream→Release，
   把码流丢弃以保持 MPP 流动；否则 VENC 的 8 个输出缓冲被填满。
2. 重连走 `p.lost_events` / `p.connects` 计数判定，而不是靠 rc==-2，
   因为 rc==-2 在断线期间的每一轮都会返回。

**清理**：整个跑体包在 try/finally 里，finally 里 `p.close()` + `cam.stop()`。
（NOTES.md 的教训：探针漏 cleanup 会污染板子。）
"""
import sys
import time

sys.path.insert(0, "/sdcard/k230vision")
import config
import netup
from cam import Camera
from pusher import Pusher
from watchdog import Watchdog

DUR = 45          # 总时长（秒）

netup.init(config.WIFI_SSID, config.WIFI_PASS)
assert netup.connect(), "WiFi 连不上"
print("IP", netup.ip())

cam = Camera(config.WIDTH, config.HEIGHT)
cam.add_channel(config.PUSH_CHN, config.BITRATE_KBPS)
cam.start()
print("CAM STARTED")

p = Pusher(config.PUSH_HOST, config.PUSH_PORT,
           cam.encoder(config.PUSH_CHN), config.PUSH_CHN)
assert p.connect(), "TCP 连不上，检查 Pi 侧接收端是否已起"
print("PUSH CONNECTED")

wd = Watchdog(config.PUSH_STALL_WARN_MS)
t0 = time.time()
seen_lost = p.lost_events
seen_conn = p.connects
lost_at = None
reconnected_at = None
stalled_at = None
last_try = 0.0
last_report = t0
backoff = config.RECONNECT_BACKOFF_MS / 1000.0

try:
    while time.time() - t0 < DUR:
        p.pump()                       # 断线时也泵：取出即丢弃，保 MPP 流动
        wd.tick(p.frames_sent)

        if p.lost_events > seen_lost:  # 刚发现连接坏了
            seen_lost = p.lost_events
            if lost_at is None:
                lost_at = time.time()
            print("LOST at", round(time.time() - t0, 1))

        if p.connects > seen_conn:     # 重连成功
            seen_conn = p.connects
            if lost_at is not None and reconnected_at is None:
                reconnected_at = time.time()
                print("RECONNECTED at", round(time.time() - t0, 1))

        if wd.stalled and stalled_at is None:
            stalled_at = time.time()
            print("STALLED at", round(time.time() - t0, 1),
                  "idle_ms=%d" % wd.idle_ms)

        if not p.ok:
            now = time.time()
            if now - last_try >= backoff:
                last_try = now
                p.connect()

        now = time.time()
        if now - last_report >= 5:
            print("STATS t=%d sent=%d dropped=%d bytes=%d lost=%d conns=%d stalled=%s"
                  % (round(now - t0), p.frames_sent, p.frames_dropped, p.bytes_sent,
                     p.lost_events, p.connects, wd.stalled))
            last_report = now

        time.sleep(0.002)
finally:
    p.close()
    cam.stop()
    print("DONE sent=%d dropped=%d bytes=%d lost=%d conns=%d"
          % (p.frames_sent, p.frames_dropped, p.bytes_sent,
             p.lost_events, p.connects))
