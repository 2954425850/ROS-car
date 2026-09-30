# /sdcard/k230vision/t_push.py
import sys, time
sys.path.insert(0, "/sdcard/k230vision")
import config
import netup
from cam import Camera
from pusher import Pusher

netup.init(config.WIFI_SSID, config.WIFI_PASS)
assert netup.connect(), "WiFi 连不上，先解决 Task 1"

cam = Camera(config.WIDTH, config.HEIGHT)
cam.add_channel(config.PUSH_CHN, config.BITRATE_KBPS)
cam.start()
print("CAM STARTED")

p = Pusher(config.PUSH_HOST, config.PUSH_PORT,
           cam.encoder(config.PUSH_CHN), config.PUSH_CHN)
assert p.connect(), "TCP 连不上，检查 Pi 侧接收端是否已起"
print("PUSH CONNECTED")

t0 = time.time()
last_report = t0
try:
    while time.time() - t0 < 15:
        rc = p.pump()
        if rc == -2:
            print("PUSH LOST, reconnecting")
            time.sleep(1)
            p.connect()
        now = time.time()
        if now - last_report >= 3:
            print("STATS sent=%d dropped=%d bytes=%d" % (p.frames_sent, p.frames_dropped, p.bytes_sent))
            last_report = now
finally:
    p.close()
    cam.stop()
    print("DONE sent=%d dropped=%d bytes=%d" % (p.frames_sent, p.frames_dropped, p.bytes_sent))
