# /sdcard/k230vision/t_isolate.py -- 隔离实验：WiFi 初始化后再起管线，只数帧不发网
import sys, time, gc
sys.path.insert(0, "/sdcard/k230vision")
import config, netup
print("MEM_BEFORE_WIFI", gc.mem_free())
netup.init(config.WIFI_SSID, config.WIFI_PASS)
print("WIFI", netup.connect(), netup.ip())
print("MEM_AFTER_WIFI", gc.mem_free())
from cam import Camera
from media.vencoder import StreamData
cam = Camera(config.PUSH_CHN, config.WIDTH, config.HEIGHT, config.BITRATE_KBPS)
cam.start()
print("CAM STARTED", "MEM_AFTER_CAM", gc.mem_free())
t0 = time.time(); n = 0; nb = 0
while time.time() - t0 < 5:
    sd = StreamData()
    if cam.encoder.GetStream(config.PUSH_CHN, sd, timeout=100) == 0:
        n += 1
        for i in range(sd.pack_cnt):
            nb += sd.data_size[i]
        cam.encoder.ReleaseStream(config.PUSH_CHN, sd)
el = time.time() - t0
print("ISOLATE frames=%d el=%.2f fps=%.1f kbps=%.0f" % (n, el, n / el, nb * 8 / el / 1000))
cam.stop()
print("STOPPED_OK")
