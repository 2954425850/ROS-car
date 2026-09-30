# /sdcard/k230vision/t_isolate2.py -- 原始可用配置(bare Sensor) + WiFi，只数帧
import sys, time, gc
sys.path.insert(0, "/sdcard/k230vision")
# no netup
pass
pass
from media.sensor import *
from media.media import *
from media.vencoder import *
sensor = None; enc = None; link = None
try:
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=1280, height=720, alignment=12)
    sensor.set_pixformat(Sensor.YUV420SP)
    enc = Encoder(); enc.SetOutBufs(0, 8, 1280, 720)
    attr = ChnAttrStr(enc.PAYLOAD_TYPE_H264, enc.H264_PROFILE_MAIN, 1280, 720, bit_rate=2048)
    enc.Create(0, attr)
    link = MediaManager.link(sensor.bind_info()["src"], (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, 0))
    enc.Start(0); sensor.run()
    print("STARTED MEM", gc.mem_free())
    t0 = time.time(); n = 0; nb = 0
    while time.time() - t0 < 5:
        sd = StreamData()
        if enc.GetStream(0, sd, timeout=100) == 0:
            n += 1
            for i in range(sd.pack_cnt):
                nb += sd.data_size[i]
            enc.ReleaseStream(0, sd)
    el = time.time() - t0
    print("BARE_WIFI frames=%d el=%.2f fps=%.1f kbps=%.0f" % (n, el, n / el, nb * 8 / el / 1000))
finally:
    try:
        if sensor: sensor.stop()
    except Exception: pass
    try:
        if link: del link
    except Exception: pass
    try:
        if enc: enc.Stop(0); enc.Destroy(0)
    except Exception: pass
    print("CLEANUP_DONE")
