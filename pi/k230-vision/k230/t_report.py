# /sdcard/k230vision/t_report.py
"""Task 6 验证：**真实的**检测结果经 KPU 推理后上报到 Pi（20 s @ 10 Hz）。

## 与计划草稿的差别（有意，且是本次任务明确要求的）

计划草稿的 t_report.py 发的是写死的 `{"cls":"person","score":0.9,...}`。
那只验了 socket 通道，**验不到**「视觉 → 结构化结果 → 上报」这条真路径。
本脚本把 **chn2 + Detector** 接起来，发**真的** `results.encode(det.detect(img))`。

## v2 修正：**定速必须用 ticks_ms，不能用 time.time**

v1 用 `time.time()` 做 10 Hz 定速，实测跑成 **24.55 Hz**（= 推理的满速），
`STATS frames=491 ... report_hz=24.55`。
根因：**本 build 的 `time.time()` 只有 1 秒分辨率** ——
连续三次调用返回同一个整数：
```
TIME_RES 1789646288 1789646288 1789646288
```
于是 `nxt += 0.1` / `d = nxt - time.time()` 这个节拍器整个退化成了空操作。
（`time.sleep_ms` 与 `time.sleep(float)` 本身是准的：实测 60/100/500 ms 分别
耗时 60/100/500 ms。**问题只在时钟源，不在 sleep。**）
v2 改用 `time.ticks_ms()` / `time.ticks_diff()`，实测 3 s 目标 10 Hz → 30 次，正好 10.0 Hz。

⚠️ `ts` 字段仍是 `int(time.time() * 1000)`，受同一个 1 秒分辨率限制，
**实际只有秒级精度**（尾数恒为 000）。设计文档要的是 epoch_ms，
本 build 拿不到真正的毫秒级 epoch 时间，如实记录。

## 测试协议

一个 MicroPython 会话只能建一条管线 ⇒ 跑之前必须 `machine.reset()`。
本脚本带 `finally` 完整清理。

## 用法

    timeout 150 ~/.local/bin/mpremote connect /dev/ttyACM0 exec \
        "exec(open('/sdcard/k230vision/t_report.py').read())"
"""
import sys
import time
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
import netup
from results import encode
from reporter import Reporter
from media.sensor import *
from media.media import *
from vision import Detector

DUR = 20.0      # 上报秒数
HZ = 10.0       # 上报频率


def main():
    det = None
    sensor = None
    r_pi = None
    r_cloud = None
    try:
        netup.init(config.WIFI_SSID, config.WIFI_PASS)
        if not netup.connect(25):
            print("FAIL wifi 连不上")
            return
        print("WIFI OK %s" % netup.ip())

        r_pi = Reporter(config.RESULT_HOST, config.RESULT_PORT)
        if not r_pi.connect():
            print("FAIL 连不上 %s:%s" % (config.RESULT_HOST, config.RESULT_PORT))
            return
        print("REPORTER OK %s:%s" % (config.RESULT_HOST, config.RESULT_PORT))

        # 云端未就绪（CLOUD_RESULT_HOST 为空串）⇒ 不启用。
        # **本地路径不因此失败** —— 这里的 if 是唯一的开关。
        if config.CLOUD_RESULT_HOST:
            r_cloud = Reporter(config.CLOUD_RESULT_HOST, config.CLOUD_RESULT_PORT)
            print("CLOUD connect=%s %s:%s"
                  % (r_cloud.connect(), config.CLOUD_RESULT_HOST,
                     config.CLOUD_RESULT_PORT))
        else:
            print("CLOUD disabled (CLOUD_RESULT_HOST empty) - local path unaffected")

        # ---------- AI 通道 + 推理器 ----------
        sensor = Sensor(fps=config.SENSOR_FPS)
        sensor.reset()
        sensor.set_framesize(w=config.AI_WIDTH, h=config.AI_HEIGHT, chn=CAM_CHN_ID_2)
        sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
        print("CHN2 OK %dx%d RGBP888" % (config.AI_WIDTH, config.AI_HEIGHT))

        det = Detector(kmodel_path=config.KMODEL_PATH, labels=config.LABELS,
                       model_input_size=config.MODEL_INPUT_SIZE,
                       max_boxes_num=config.MAX_BOXES_NUM,
                       confidence_threshold=config.CONF_THRESHOLD,
                       nms_threshold=config.NMS_THRESHOLD,
                       rgb888p_size=[config.AI_WIDTH, config.AI_HEIGHT])
        det.config_preprocess()
        sensor.run()
        print("PIPELINE OK kmodel=%s" % config.KMODEL_PATH)

        # ---------- 上报主循环（定速用 ticks_ms，见文件头）----------
        t_start = time.ticks_ms()
        nxt = t_start
        period = int(1000.0 / HZ)
        frame = 0
        obj_frames = 0
        last_objs = None
        sample = None
        while True:
            if time.ticks_diff(time.ticks_ms(), t_start) >= int(DUR * 1000):
                break

            img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if img is None:
                print("SNAPSHOT None")
                continue
            objs = det.detect(img.to_numpy_ref())
            del img
            frame += 1
            if objs:
                obj_frames += 1
                last_objs = objs
                if sample is None:
                    sample = objs

            # w/h 用**推流**分辨率：前端拿归一化坐标 × 自己的显示尺寸
            payload = encode(objs, config.WIDTH, config.HEIGHT, frame,
                             int(time.time() * 1000))
            if not r_pi.ok:
                r_pi.connect()
            r_pi.send(payload)
            if r_cloud is not None:
                if not r_cloud.ok:
                    r_cloud.connect()
                r_cloud.send(payload)

            gc.collect()
            nxt += period
            d = time.ticks_diff(nxt, time.ticks_ms())
            if d > 0:
                time.sleep_ms(d)
            elif d < -200:
                nxt = time.ticks_ms()   # 落后太多就重新对齐，不做追赶风暴

        el = time.ticks_diff(time.ticks_ms(), t_start) / 1000.0
        print("STATS frames=%d obj_frames=%d el=%.2f report_hz=%.2f"
              % (frame, obj_frames, el, frame / el))
        print("SAMPLE_OBJS %s" % (sample,))
        print("LAST_OBJS %s" % (last_objs,))
        print("DONE sent=%d dropped=%d" % (r_pi.sent, r_pi.dropped))
        if r_cloud is not None:
            print("CLOUD sent=%d dropped=%d" % (r_cloud.sent, r_cloud.dropped))

    except BaseException as e:
        import sys as _s
        _s.print_exception(e)
    finally:
        print("CLEANUP_START")
        if r_pi is not None:
            r_pi.close()
        if r_cloud is not None:
            r_cloud.close()
        if det is not None:
            try:
                det.deinit()
            except Exception as e:
                print("cleanup det EXC", e)
        if sensor is not None:
            try:
                sensor.stop()
            except Exception as e:
                print("cleanup sensor EXC", e)
        gc.collect()
        print("CLEANUP_DONE MEM %d" % gc.mem_free())


main()
