# /sdcard/k230vision/t_venc_probe.py
"""Task 4 前置调查：两个 VENC 通道能否共存，以及 Encoder() 到底该实例化几个。

MODE 由调用方在 exec 前注入：
  MODE="A" -> 每个通道各自 new 一个 Encoder()
  MODE="B" -> 只 new 一个 Encoder()，两个通道都挂在它上面

跑法（每次前必须 machine.reset()，见 NOTES.md）：
  ~/.local/bin/mpremote connect /dev/ttyACM0 exec "MODE='A'; exec(open('/sdcard/k230vision/t_venc_probe.py').read())"

本脚本带完整 finally 清理。
"""
import gc
import sys
import time

import config
from media.sensor import *
from media.media import *
from media.vencoder import *

MODE = globals().get("MODE", "A")
RUN_S = 4.0
NCH = 2
W = 1280
H = 720

print("PROBE MODE=%s NCH=%d" % (MODE, NCH))

sensor = None
encs = []
links = []
sd = [StreamData() for _ in range(NCH)]

try:
    sensor = Sensor(fps=config.SENSOR_FPS)
    sensor.reset()
    sensor.set_framesize(width=W, height=H, alignment=12)
    sensor.set_pixformat(Sensor.YUV420SP)

    def mkattr(e):
        a = ChnAttrStr(e.PAYLOAD_TYPE_H264, e.H264_PROFILE_MAIN, W, H,
                      bit_rate=config.BITRATE_KBPS)
        a.src_frame_rate = config.SENSOR_FPS
        a.dst_frame_rate = config.FPS
        a.gop_len = config.GOP
        return a

    if MODE == "A":
        for chn in range(NCH):
            e = Encoder()
            e.SetOutBufs(chn, 8, W, H)
            e.Create(chn, mkattr(e))
            encs.append(e)
            print("  created Encoder()#%d for chn%d" % (chn, chn))
    else:
        e = Encoder()
        for chn in range(NCH):
            e.SetOutBufs(chn, 8, W, H)
            e.Create(chn, mkattr(e))
            print("  one Encoder() created chn%d" % chn)
        encs.append(e)

    src = sensor.bind_info()["src"]
    for chn in range(NCH):
        links.append(MediaManager.link(src, (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, chn)))
        print("  link chn%d OK" % chn)
    print("LINKS_OK", len(links))

    for e in encs:
        for chn in range(NCH):
            e.Start(chn)
    sensor.run()
    print("STARTED")

    def enc_of(chn):
        return encs[chn] if MODE == "A" else encs[0]

    cnt = [0] * NCH
    byt = [0] * NCH
    pks = [0] * NCH
    first = [None] * NCH
    t0 = time.time()
    while time.time() - t0 < RUN_S:
        for chn in range(NCH):
            e = enc_of(chn)
            ret = e.GetStream(chn, sd[chn], timeout=100)
            if ret == 0:
                cnt[chn] += 1
                for i in range(sd[chn].pack_cnt):
                    byt[chn] += sd[chn].data_size[i]
                    pks[chn] += 1
                if first[chn] is None and sd[chn].pack_cnt > 0:
                    first[chn] = sd[chn].data_size[0]
                e.ReleaseStream(chn, sd[chn])
    el = time.time() - t0
    for chn in range(NCH):
        print("CHN%d frames=%d packs=%d bytes=%d fps=%.1f"
              % (chn, cnt[chn], pks[chn], byt[chn], cnt[chn] / el))
    print("PROBE_RESULT MODE=%s elapsed=%.2f" % (MODE, el))

except BaseException as e:
    print("PROBE_EXC", repr(e))
    sys.print_exception(e)

finally:
    print("CLEANUP_START")
    if sensor:
        try:
            sensor.stop()
        except BaseException as e:
            print("  cleanup sensor EXC", repr(e))
    for lk in links:
        try:
            del lk
        except BaseException as e:
            print("  cleanup link EXC", repr(e))
    links = []
    for e in encs:
        for chn in range(NCH):
            try:
                e.Stop(chn)
            except BaseException as ex:
                print("  cleanup Stop(%d) EXC" % chn, repr(ex))
            try:
                e.Destroy(chn)
            except BaseException as ex:
                print("  cleanup Destroy(%d) EXC" % chn, repr(ex))
    encs = []
    gc.collect()
    print("CLEANUP_DONE MEM", gc.mem_free())
