# /sdcard/k230vision/fps_lib.py
# 探针库。铁律：
#   1) 一块板子同时只能有一条管线 —— 每次 run() 必须在 finally 里彻底拆干净；
#   2) 跑完立刻再跑一次仍能起来，才算 cleanup 正确（V4 检查）。
import time
from media.sensor import *
from media.media import *
from media.vencoder import *

CHN = 0


def run(name, sw, sh, vw, vh, sensor_fps, src_fps, dst_fps, gop, bitrate=2048, secs=5.0):
    print("== %s ==" % name)
    sensor = None; encoder = None; link = None
    try:
        if sensor_fps is None:
            sensor = Sensor()
        else:
            sensor = Sensor(fps=sensor_fps)
        print("   Sensor(fps=%s) constructed OK" % sensor_fps)
        sensor.reset()
        sensor.set_framesize(width=sw, height=sh, alignment=12)
        sensor.set_pixformat(Sensor.YUV420SP)
        encoder = Encoder()
        encoder.SetOutBufs(CHN, 8, vw, vh)
        attr = ChnAttrStr(encoder.PAYLOAD_TYPE_H264, encoder.H264_PROFILE_MAIN, vw, vh, bit_rate=bitrate)
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
        print("RESULT %s frames=%d el=%.2f fps=%.1f kbps=%.0f" % (name, n, el, n / el, nb * 8 / el / 1000))
        return n, el, nb
    except Exception as e:
        print("RESULT %s EXC %s" % (name, e))
        return None
    finally:
        try:
            if sensor: sensor.stop()
        except Exception as e:
            print("   cleanup sensor EXC:", e)
        try:
            if link: del link
        except Exception as e:
            print("   cleanup link EXC:", e)
        try:
            if encoder:
                encoder.Stop(CHN); encoder.Destroy(CHN)
        except Exception as e:
            print("   cleanup encoder EXC:", e)
        print("   CLEANUP_DONE")
