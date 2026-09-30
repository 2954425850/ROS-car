# /sdcard/k230vision/t_venc_probe2.py
"""Task 4 前置调查 v2：两个 VENC 通道能否共存，Encoder() 该实例化几个。

v1 的 bug：MODE=A 时 Start 循环写成了 encs[i] x 所有 chn 的笛卡尔积，
导致对"只 Create 过 chn0"的编码器调用 Start(1) → mpi venc start failed。
v2 改为 (encoder, chn) 配对，每个编码器只 Start 自己 Create 过的那个通道。

MODE 由调用方注入：A = 每通道一个 Encoder()；B = 单个 Encoder() 带两个通道。
每次跑之前必须 machine.reset()。本脚本带完整 finally 清理。
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

print("PROBE2 MODE=%s NCH=%d" % (MODE, NCH))

sensor = None
pairs = []          # [(encoder, chn), ...]
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
            pairs.append((e, chn))
            print("  Encoder()#%d -> chn%d" % (chn, chn))
    else:
        e = Encoder()
        for chn in range(NCH):
            e.SetOutBufs(chn, 8, W, H)
            e.Create(chn, mkattr(e))
            pairs.append((e, chn))
            print("  shared Encoder() -> chn%d" % chn)

    src = sensor.bind_info()["src"]
    for chn in range(NCH):
        links.append(MediaManager.link(src, (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, chn)))
        print("  link chn%d OK" % chn)
    print("LINKS_OK", len(links))

    enc_by_chn = {}
    for (e, chn) in pairs:
        enc_by_chn[chn] = e
        print("  Start(chn=%d) ..." % chn)
        e.Start(chn)
        print("  Start(chn=%d) OK" % chn)
    sensor.run()
    print("STARTED")

    cnt = [0] * NCH
    byt = [0] * NCH
    pks = [0] * NCH
    t0 = time.time()
    while time.time() - t0 < RUN_S:
        for chn in range(NCH):
            e = enc_by_chn[chn]
            ret = e.GetStream(chn, sd[chn], timeout=100)
            if ret == 0:
                cnt[chn] += 1
                for i in range(sd[chn].pack_cnt):
                    byt[chn] += sd[chn].data_size[i]
                    pks[chn] += 1
                e.ReleaseStream(chn, sd[chn])
    el = time.time() - t0
    for chn in range(NCH):
        print("CHN%d frames=%d packs=%d bytes=%d fps=%.1f"
              % (chn, cnt[chn], pks[chn], byt[chn], cnt[chn] / el))
    print("PROBE2_RESULT MODE=%s elapsed=%.2f" % (MODE, el))

except BaseException as e:
    print("PROBE2_EXC", repr(e))
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
    for (e, chn) in pairs:
        try:
            e.Stop(chn)
        except BaseException as ex:
            print("  cleanup Stop(%d) EXC" % chn, repr(ex))
        try:
            e.Destroy(chn)
        except BaseException as ex:
            print("  cleanup Destroy(%d) EXC" % chn, repr(ex))
    pairs = []
    gc.collect()
    print("CLEANUP_DONE MEM", gc.mem_free())
