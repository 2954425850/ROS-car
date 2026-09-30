# /sdcard/k230vision/t_subchn.py —— 探针：VENC 能不能挂到 **sensor 子通道** 上
#
# ## 它要回答的唯一一个问题
#
# 现在生产代码里**所有 VENC 通道都挂在 sensor 根**上（`bind_info()["src"]`），
# 所以它们**必然同分辨率**。规划书要的「一路 720p + 一路 640x360」因此做不到。
#
# 本探针问：**把 VENC 挂到 sensor chn1（子通道）的 src 上，行不行？**
#
#     root(chn0) 1280x720 YUV420SP -> VENC0   期望 1280x720
#     chn1        640x360 YUV420SP -> VENC1   期望  640x360   <-- 这就是要验的
#
# ## 为什么必须单独验、不能顺手写进生产代码
#
# 全仓库（含所有 t_*.py 探针）**没有一处**用过子通道的 src —— 一直是取根。
# 而媒体层配错的代价是**卡死到必须断电**（README §7.4：`machine.reset()` 有时也救不了）。
# 官方 `libs/PipeLine.py` 只示范了 `bind_info(chn=CAM_CHN_ID_0)` 给 Display 用，
# **没有**把子通道绑给 VENC 的先例。
#
# ## 怎么跑（**必须在干净会话里**）
#
# README §7.1：一个 MicroPython 会话只能建一条管线，拆了再建第二条**必报**
# `MediaManager link failed(9)`。而 `/sdcard/main.py` 自启的 app.py 已经建过一条，
# 所以**不能直接在跑着 app 的 REPL 里 exec 本文件**。做法：
#
#     1. 读卡器：把 /sdcard/main.py 改名成 main.py.off
#     2. 插卡上电 -> 板子停在 REPL，不起 app
#     3. exec(open('/sdcard/k230vision/t_subchn.py').read())
#     4. 收工把 main.py.off 改回 main.py
#
# 或者（不用读卡器）在正常跑的会话里：
#     /sdcard/main.py 临时改名 -> machine.reset() -> 上 REPL -> 跑本文件
#
# ## 结果落在哪
#
# **边跑边写 `/sdcard/subchn_result.txt`**（每步立刻 close，不留缓冲）——
# 这样**即使媒体层把板子卡死，也已经能看到卡在哪一步**，不用猜。
# 跑完把那个文件拷回 PC 即可。
#
# ## 期望与判据
#
#   成功：VENC0 出 1280x720 的帧、**VENC1 出 640x360 的帧**，两边都在动。
#   失败的可能长相（都算"这条路不通"）：
#     - `bind_info(chn=CAM_CHN_ID_1)` 不返回 `src` -> 拿不到子通道的源
#     - `MediaManager.link` 抛异常 / 报错
#     - link 成功但 VENC1 一直 0 帧（源没数据）
#     - VENC1 出帧，但 `sd.data_size` 明显不是 640x360 的量级（说明还是根的分辨率）
#
# ⚠️ 判据**必须看帧数据大小/分辨率**，不能只看"函数没报错" ——
#    这个项目里"没报错但没生效"已经栽过好几次（见 config.py 里 ROTATE_180 那段）。

import os
import sys
import time

sys.path.insert(0, "/sdcard/k230vision")

from media.sensor import *
from media.media import *
from media.vencoder import *

RESULT = "/sdcard/subchn_result.txt"
ROOT_W, ROOT_H = 1280, 720
SUB_W, SUB_H = 640, 360
VENC_ROOT = 0
VENC_SUB = 1
DURATION_MS = 6000          # 取流多久
FPS_TARGET = 30


def log(msg):
    """每步立刻落盘 —— 卡死之后这张纸就是唯一的证据。"""
    line = "[%7d] %s" % (time.ticks_ms() & 0xFFFFFFF, msg)
    print(line)
    try:
        f = open(RESULT, "a")
        f.write(line + "\n")
        f.close()           # 立刻 close：不依赖任何缓冲/flush 语义
    except BaseException as e:
        print("LOG WRITE FAIL", e)


def main():
    try:
        os.remove(RESULT)
    except BaseException:
        pass
    log("=== t_subchn 开始（VENC 能否挂 sensor 子通道）===")

    sensor = None
    encs = {}
    links = {}
    try:
        # ---- 1) sensor 根 + 子通道 -------------------------------------------------
        sensor = Sensor(fps=FPS_TARGET)
        sensor.reset()
        sensor.set_framesize(width=ROOT_W, height=ROOT_H, alignment=12)
        sensor.set_pixformat(Sensor.YUV420SP)
        log("root  chn0 %dx%d YUV420SP 已设" % (ROOT_W, ROOT_H))

        sensor.set_framesize(w=SUB_W, h=SUB_H, chn=CAM_CHN_ID_1)
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)
        log("sub   chn1 %dx%d YUV420SP 已设" % (SUB_W, SUB_H))

        # ---- 2) 先把两个通道的 bind_info 原样打出来（这是最关键的一步）----------
        # 拿不到 src 的话后面全是空谈，所以先把事实记下来。
        for name, chn in (("chn0(root)", CAM_CHN_ID_0), ("chn1(sub)", CAM_CHN_ID_1)):
            try:
                info = sensor.bind_info(chn=chn)
                keys = sorted(info.keys()) if hasattr(info, "keys") else "?"
                log("bind_info(%s) keys=%s  src=%s"
                    % (name, keys, "有" if (hasattr(info, "keys") and "src" in info)
                       else "**没有**"))
            except BaseException as e:
                log("bind_info(%s) 抛异常: %s: %s" % (name, type(e).__name__, e))

        # ---- 3) 两个 VENC，各 new 一个 Encoder（MODE=A，见 cam.py 文件头）------
        specs = ((VENC_ROOT, ROOT_W, ROOT_H, "root"),
                 (VENC_SUB, SUB_W, SUB_H, "sub"))
        for chn, w, h, tag in specs:
            enc = Encoder()
            enc.SetOutBufs(chn, 8, w, h)
            attr = ChnAttrStr(enc.PAYLOAD_TYPE_H264, enc.H264_PROFILE_MAIN,
                              w, h, bit_rate=2048)
            attr.src_frame_rate = FPS_TARGET
            attr.dst_frame_rate = FPS_TARGET
            attr.gop_len = 25
            enc.Create(chn, attr)
            encs[chn] = enc
            log("Encoder chn=%d (%s) Create OK  %dx%d" % (chn, tag, w, h))

        # ---- 4) 链接：VENC0 <- 根 src，VENC1 <- **子通道 src**（本探针的核心）----
        src_root = sensor.bind_info()["src"]
        links[VENC_ROOT] = MediaManager.link(
            src_root, (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, VENC_ROOT))
        log("LINK VENC0 <- root src  OK")

        try:
            info_sub = sensor.bind_info(chn=CAM_CHN_ID_1)
            src_sub = info_sub["src"]
            log("拿到子通道 src OK")
        except BaseException as e:
            log("!!! 拿不到子通道 src: %s: %s" % (type(e).__name__, e))
            log("=== 结论：这条路（VENC 挂子通道）**不通** ===")
            return

        try:
            links[VENC_SUB] = MediaManager.link(
                src_sub, (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, VENC_SUB))
            log("LINK VENC1 <- **子通道 src** OK   <-- 这是全仓库第一次")
        except BaseException as e:
            log("!!! LINK VENC1 <- 子通道 src 失败: %s: %s" % (type(e).__name__, e))
            log("=== 结论：MediaManager.link 不接受子通道 src，这条路**不通** ===")
            return

        for chn in encs:
            encs[chn].Start(chn)
            log("START chn=%d OK" % chn)

        sensor.run()
        log("sensor.run() OK —— 开始取流 %d ms" % DURATION_MS)

        # ---- 5) 取流：两个通道各数帧，并记录**第一帧的数据大小** ---------------
        # 数据大小是判"到底出的是哪个分辨率"的硬证据：
        #   720p 一帧 H.264 明显大于 360p；如果 VENC1 的量级跟 VENC0 一样，
        #   说明它拿到的还是根的画面（= 这次改动没生效）。
        stats = {chn: {"n": 0, "bytes": 0, "first": None} for chn in encs}
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < DURATION_MS:
            for chn in (VENC_ROOT, VENC_SUB):
                sd = StreamData()
                if encs[chn].GetStream(chn, sd, timeout=0) != 0:
                    continue
                try:
                    sz = 0
                    for i in range(sd.pack_cnt):
                        sz += sd.data_size[i]
                    s = stats[chn]
                    s["n"] += 1
                    s["bytes"] += sz
                    if s["first"] is None:
                        s["first"] = sz
                finally:
                    encs[chn].ReleaseStream(chn, sd)

        for chn, tag in ((VENC_ROOT, "root 1280x720"), (VENC_SUB, "sub 640x360")):
            s = stats[chn]
            avg = (s["bytes"] / s["n"]) if s["n"] else 0
            log("VENC%d (%-14s) 帧数=%-5d 首帧=%-6s 平均字节=%.0f"
                % (chn, tag, s["n"], s["first"], avg))

        n0, n1 = stats[VENC_ROOT]["n"], stats[VENC_SUB]["n"]
        if n0 > 0 and n1 > 0:
            ratio = stats[VENC_ROOT]["bytes"] / max(1, stats[VENC_SUB]["bytes"])
            log("两路字节比 root/sub = %.2f" % ratio)
            log("=== 结论：两路**都出流**了 —— 子通道路径可行 ===")
            log("    （还要人看一眼上面两路的平均字节数是否真的不同；")
            log("      差不多就说明 VENC1 拿到的还是根的画面，那就等于没生效）")
        elif n0 > 0 and n1 == 0:
            log("=== 结论：根出流、**子通道 0 帧** —— 这条路不通（或需要额外配置）===")
        else:
            log("=== 结论：连根都没出流，本次实验**无效**，先查根为什么没出 ===")

    except BaseException as e:
        import sys as _s
        _s.print_exception(e)
        log("!!! 主流程抛异常: %s: %s" % (type(e).__name__, e))
    finally:
        # 清理照 cam.py 验证过的顺序：sensor.stop -> del link -> Stop/Destroy。
        # **全部 best-effort**：探针里任何一步失败都不该掩盖真正的结论。
        log("--- cleanup ---")
        try:
            if sensor:
                sensor.stop()
                log("sensor.stop() OK")
        except BaseException as e:
            log("sensor.stop EXC %s: %s" % (type(e).__name__, e))
        for chn, lk in links.items():
            try:
                del lk
                log("del link chn=%d OK" % chn)
            except BaseException as e:
                log("del link chn=%d EXC %s: %s" % (chn, type(e).__name__, e))
        for chn, enc in encs.items():
            try:
                enc.Stop(chn)
                log("Stop chn=%d OK" % chn)
            except BaseException as e:
                log("Stop chn=%d EXC %s: %s" % (chn, type(e).__name__, e))
            try:
                enc.Destroy(chn)
                log("Destroy chn=%d OK" % chn)
            except BaseException as e:
                log("Destroy chn=%d EXC %s: %s" % (chn, type(e).__name__, e))
        log("=== t_subchn 结束；完整记录见 %s ===" % RESULT)
        print("SUBCHN_DONE")


main()
