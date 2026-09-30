# /sdcard/k230vision/t_fps_probe.py  -- 一次性探针：不联网，只数编码器输出帧
import sys, time
sys.path.insert(0, "/sdcard/k230vision")
from media.sensor import *
from media.media import *
from media.vencoder import *

CHN = 0


def measure(sw, sh, vw, vh, src_fps, dst_fps, gop, secs=5.0):
    sensor = None; encoder = None; link = None
    try:
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=sw, height=sh, alignment=12)
        try:
            sensor.set_framerate(src_fps)
            print("   set_framerate(%d) ok readback=%s" % (src_fps, sensor.get_framerate()))
        except Exception as e:
            print("   set_framerate(%d) FAILED: %s" % (src_fps, e))
        sensor.set_pixformat(Sensor.YUV420SP)
        encoder = Encoder()
        encoder.SetOutBufs(CHN, 8, vw, vh)
        attr = ChnAttrStr(encoder.PAYLOAD_TYPE_H264, encoder.H264_PROFILE_MAIN, vw, vh, bit_rate=2048)
        attr.src_frame_rate = src_fps
        attr.dst_frame_rate = dst_fps
        attr.gop_len = gop
        encoder.Create(CHN, attr)
        link = MediaManager.link(sensor.bind_info()["src"], (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, CHN))
        encoder.Start(CHN)
        sensor.run()
        t0 = time.time(); n = 0; nb = 0
        while time.time() - t0 < secs:
            sd = StreamData()
            if encoder.GetStream(CHN, sd, timeout=100) == 0:
                n += 1
                for i in range(sd.pack_cnt):
                    nb += sd.data_size[i]
                encoder.ReleaseStream(CHN, sd)
        el = time.time() - t0
        return n, el, nb
    finally:
        try:
            if sensor: sensor.stop()
        except Exception: pass
        try:
            if link: del link
        except Exception: pass
        try:
            if encoder: encoder.Stop(CHN); encoder.Destroy(CHN)
        except Exception: pass


cfgs = [
    ("A_720p_src25_dst25", 1280, 720, 1280, 720, 25, 25, 25),
    ("B_720p_src60_dst25", 1280, 720, 1280, 720, 60, 25, 25),
    ("C_1080p_src30_dst25", 1920, 1080, 1280, 720, 30, 25, 25),
    ("D_1080p_src30_dst30", 1920, 1080, 1280, 720, 30, 30, 25),
]
for name, sw, sh, vw, vh, s, d, g in cfgs:
    print("== %s ==" % name)
    try:
        n, el, nb = measure(sw, sh, vw, vh, s, d, g)
        print("RESULT %s frames=%d el=%.2f fps=%.1f kbps=%.0f" % (name, n, el, n / el, nb * 8 / el / 1000))
    except Exception as e:
        print("RESULT %s EXC %s" % (name, e))
    time.sleep(1)
print("PROBE DONE")
