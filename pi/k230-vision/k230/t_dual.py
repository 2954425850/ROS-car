# /sdcard/k230vision/t_dual.py
"""同时：chn0 推云端 + chn1 供局域网 RTSP。验证设计文档的 V5 风险点。

跑法（**每次之前先 machine.reset()**，见 NOTES.md）：
  ~/.local/bin/mpremote connect /dev/ttyACM0 exec "exec(open('/sdcard/k230vision/t_dual.py').read())"

Pi 侧配套：
  python3 /home/cy/k230-vision/tools/tcp_h264_sink.py 8555 /tmp/dual.h264 35
  timeout 15 gst-launch-1.0 -q rtspsrc location=rtsp://192.168.1.112:8554/k230 \\
      protocols=udp latency=200 ! rtph264depay ! filesink location=/tmp/dual_rtsp.h264

本脚本带完整 finally 清理（铁律：一个会话只建一条管线，且必须清干净）。
"""
import sys
import time

sys.path.insert(0, "/sdcard/k230vision")

import config
import netup
from cam import Camera
from pusher import Pusher
from rtsp_srv import RtspOut
from watchdog import Watchdog

RUN_S = 30.0

netup.init(config.WIFI_SSID, config.WIFI_PASS)
assert netup.connect(), "WiFi 连不上，先解决 Task 1"
print("WIFI", netup.ip() if hasattr(netup, "ip") else "")

cam = None
p = None
rt = None
try:
    cam = Camera(config.WIDTH, config.HEIGHT)
    cam.add_channel(config.PUSH_CHN, config.BITRATE_KBPS)
    cam.add_channel(config.RTSP_CHN, config.BITRATE_KBPS)
    cam.start()
    print("CAM STARTED (2 chans)")

    p = Pusher(config.PUSH_HOST, config.PUSH_PORT,
               cam.encoder(config.PUSH_CHN), config.PUSH_CHN)
    if not p.connect():
        print("PUSH CONNECT FAILED, 检查 Pi 侧 sink 是否已起")
    else:
        print("PUSH CONNECTED")

    rt = RtspOut(cam.encoder(config.RTSP_CHN), config.RTSP_CHN,
                 config.RTSP_PORT, config.RTSP_SESSION,
                 config.WIDTH, config.HEIGHT)
    rt.start()
    print("RTSP URL:", rt.get_url())

    wd = Watchdog(config.PUSH_STALL_WARN_MS)
    t0 = time.time()
    last_report = t0
    n_stalled = 0
    while time.time() - t0 < RUN_S:
        if p.pump() == -2:
            print("PUSH LOST at %.0f" % (time.time() - t0))
            p.connect()
        rt.pump()
        if wd.tick(p.frames_sent):
            n_stalled += 1
            print("STALLED at %.0f idle_ms=%d" % (time.time() - t0, wd.idle_ms))
        now = time.time()
        if now - last_report >= 5:
            print("STATS t=%.0f push sent=%d dropped=%d rtsp frames=%d bytes=%d"
                  % (now - t0, p.frames_sent, p.frames_dropped,
                     rt.frames, rt.bytes_sent))
            last_report = now
        time.sleep(0.003)
    print("STALL_COUNT %d" % n_stalled)

except BaseException as e:
    print("DUAL_EXC", repr(e))
    sys.print_exception(e)

finally:
    print("CLEANUP_START")
    if p:
        p.close()
    if rt:
        try:
            rt.stop()
        except BaseException as e:
            print("  rtsp stop EXC", repr(e))
    if cam:
        try:
            cam.stop()
        except BaseException as e:
            print("  cam stop EXC", repr(e))
    if p:
        print("DONE sent=%d dropped=%d bytes=%d lost=%d conns=%d"
              % (p.frames_sent, p.frames_dropped, p.bytes_sent,
                 p.lost_events, p.connects))
    if rt:
        print("RTSP frames=%d packs=%d bytes=%d empty=%d"
              % (rt.frames, rt.packs, rt.bytes_sent, rt.empty))
    print("CLEANUP_DONE")
