# /sdcard/k230vision/app.py
"""主循环：WiFi -> 摄像头(3 通道) -> 推流 + RTSP + 推理 + 结果上报。

## 单线程编排

MicroPython 的 _thread 在这块板子上与 MPP 的配合没验证过，**不要用多线程**。
所有"取一帧"的动作都是非阻塞或短超时的（pusher 100 ms / rtsp 0），
在一个循环里轮流推进 —— Task 4 已证单线程 `pump()` 与官方 _thread 版等效。

## 四处按实测修正的地方（照抄计划原版会踩）

1. **不用 `from libs.PipeLine import PipeLine`**，计划里那句是多余的。
   AI 帧来自 sensor 的**第三个通道**：`sensor.snapshot(chn=CAM_CHN_ID_2)`
   （RGBP888）。`PipeLine` 只是把 sensor+Display+OSD 打包的壳，它 `reset()` 后
   **独占 chn0 给 Display**，而 chn0 我们要给推流。`CAM_CHN_ID_MAX = 3`，正好用满。
2. **AI 帧分辨率必须等于模型输入分辨率（320x320）**：官方 `ObjectDetectionApp`
   重写了 `preprocess()`（`return [nn.from_numpy(input_np)]`），ai2d 预处理
   根本没被调用，改尺寸会让推理结果错乱。
3. **`time.time()` 在本平台只有整数秒**，计划里 `now = time.time();
   if now - last >= 0.1` 这种节流**是坏的**（实测一个目标 10 Hz 的循环跑成了
   24.55 Hz，差值恒为 0，节流退化成空操作）。本文件所有定速/计时一律用
   `ticks_ms` / `ticks_diff`（写法照已验证的 t_report.py）。
   注意 `ts` 字段 `int(time.time()*1000)` 因此**只有秒级精度**，这是平台限制。
4. **`cam.request_idr()` 是显式 no-op**（本平台 Encoder 没有请求 IDR 的接口），
   保留调用但不产生效果；重连后的恢复靠 `gop_len=25`（0.83 s）等下一个 IDR。
   计划里的 `_last_objs` 是占位符，这里用真 `det.detect(img)` 的结果补完。

## 与计划的两处有意偏离（都写明理由）

- **推理与上报合并到同一个 10 Hz 节拍。** 计划把它们写成两个各自 `>= 0.1` 的块，
  周期相同却独立推进，会互相漂移（某条 report 可能带上一轮或下一轮的 frame 号）。
  合并后每条消息恰好对应一次推理，`frame` 连续无跳号（与已验证的 t_report.py 同形）。
- **`wd.tick()` 每轮都调。** 计划只在 `rc == 0` 时调；那样在完全断流
  （`rc` 一直 -1/-2）时看门狗**永远不会触发**，恰好丢掉了它唯一该管的场景。
  为了让停滞告警不刷屏，用 `on_stall` 回调（每个停滞期只打一行）。

## Detector 为什么在 Camera 之前建

计划把 `Detector` 建在 sensor 之后。已验证的 `t_vision.py`/`t_report.py` 里
`sensor.run()` 是**最后**一步，而 `cam.start()` 内部就会 `sensor.run()`；
`sensor.run()` 之后再建 Detector 属于未验证路径。`AIBase.__init__` 只做
`kpu.load_kmodel()`，不碰 sensor，所以把 Detector 提到 `Camera` 之前建是安全的。
（本文件已实测：见 NOTES.md「Task 7」。）

## 恢复边界（不要越界）

`pusher.py` **只做 TCP 层重连，不重建 MPP 管线**：同一 MicroPython 会话内拆掉管线
再建第二条**必报** `MediaManager link failed(9)`，重建只有 `machine.reset()` 一条路。
所以这里重连失败就退避重试，**绝不写"推流失败就重建管线"**。

## 用法

    timeout 1900 ~/.local/bin/mpremote connect /dev/ttyACM0 exec \
        "exec(open('/sdcard/k230vision/app.py').read())"
"""
import sys
import time
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
import netup
from media.sensor import CAM_CHN_ID_2
from cam import Camera
from pusher import Pusher
from rtsp_srv import RtspOut
from results import encode
from reporter import Reporter
from vision import Detector
from watchdog import Watchdog

INFER_HZ = 10.0                 # 推理 + 上报节拍（计划两块都是 0.1 s）
STATS_MS = 30000                # 30 s 一行 STATS，给长稳测试留证据


def main():
    netup.init(config.WIFI_SSID, config.WIFI_PASS)
    while not netup.connect():
        print("WIFI FAILED, retry")
        time.sleep(2)
    print("WIFI OK %s" % netup.ip())

    # ---- KPU：在 Camera 之前建（只 load kmodel，不碰 sensor，见文件头）----
    det = Detector(kmodel_path=config.KMODEL_PATH,
                   labels=config.LABELS,
                   model_input_size=config.MODEL_INPUT_SIZE,
                   max_boxes_num=config.MAX_BOXES_NUM,
                   confidence_threshold=config.CONF_THRESHOLD,
                   nms_threshold=config.NMS_THRESHOLD,
                   rgb888p_size=[config.AI_WIDTH, config.AI_HEIGHT])
    det.config_preprocess()
    print("DETECTOR OK kmodel=%s" % config.KMODEL_PATH)

    # ---- 摄像头：chn0 推流 / chn1 RTSP / chn2 AI ----
    cam = Camera(config.WIDTH, config.HEIGHT)
    cam.add_channel(config.PUSH_CHN, config.BITRATE_KBPS)
    cam.add_channel(config.RTSP_CHN, config.BITRATE_KBPS)
    cam.add_ai_channel(config.AI_WIDTH, config.AI_HEIGHT)
    cam.start()
    print("CAM OK chans=%s ai=%sx%s" % (cam.chans, config.AI_WIDTH, config.AI_HEIGHT))

    pusher = Pusher(config.PUSH_HOST, config.PUSH_PORT,
                    cam.encoder(config.PUSH_CHN), config.PUSH_CHN)
    print("PUSH connect=%s %s:%s" % (pusher.connect(),
                                     config.PUSH_HOST, config.PUSH_PORT))

    rtsp = RtspOut(cam.encoder(config.RTSP_CHN), config.RTSP_CHN,
                   config.RTSP_PORT, config.RTSP_SESSION,
                   config.WIDTH, config.HEIGHT)
    rtsp.start()
    print("RTSP %s" % rtsp.get_url())

    r_pi = Reporter(config.RESULT_HOST, config.RESULT_PORT)
    print("REPORTER connect=%s %s:%s" % (r_pi.connect(),
                                         config.RESULT_HOST, config.RESULT_PORT))

    # 云端未就绪（CLOUD_RESULT_HOST 为空串）=> 不启用，**本地路径不因此失败**
    r_cloud = None
    if config.CLOUD_RESULT_HOST:
        r_cloud = Reporter(config.CLOUD_RESULT_HOST, config.CLOUD_RESULT_PORT)
        print("CLOUD connect=%s %s:%s" % (r_cloud.connect(),
                                          config.CLOUD_RESULT_HOST,
                                          config.CLOUD_RESULT_PORT))
    else:
        print("CLOUD disabled (CLOUD_RESULT_HOST empty) - local path unaffected")

    wd = Watchdog(config.PUSH_STALL_WARN_MS)
    # 用回调而不是每轮 print：停滞期每轮打一行会把 30 min 的日志刷爆
    wd.on_stall = lambda: print("WARN push stalled idle_ms=%d sent=%d"
                                % (wd.idle_ms, pusher.frames_sent))

    t_start = time.ticks_ms()
    nxt = t_start
    t_stats = t_start
    period = int(1000.0 / INFER_HZ)
    frame_no = 0
    obj_frames = 0
    last_objs = []
    sample = None

    try:
        while True:
            # 1) 推流：每轮尽量取，不空转
            rc = pusher.pump()
            if rc == -2:
                # 断线：退避后只重连 socket。**不要在这里重建 MPP 管线**（见文件头）
                time.sleep_ms(config.RECONNECT_BACKOFF_MS)
                if pusher.connect():
                    print("PUSH RECONNECTED connects=%d" % pusher.connects)
                    cam.request_idr(config.PUSH_CHN)   # 显式 no-op，见文件头第 4 条
            # 看门狗每轮都喂（计划只在 rc==0 时喂，那样完全断流时它永不触发）
            wd.tick(pusher.frames_sent)

            # 2) RTSP 通道（非阻塞；没有客户端时也要 GetStream->Release，
            #    否则 VENC 的 8 个输出缓冲会被填满把 MPP 顶住）
            rtsp.pump()

            # 3) 10 Hz：推理 + 上报（同一节拍，见文件头"偏离"）
            now = time.ticks_ms()
            if time.ticks_diff(now, nxt) >= 0:
                nxt = now + period
                img = cam.sensor.snapshot(chn=CAM_CHN_ID_2)
                if img is not None:
                    objs = det.detect(img.to_numpy_ref())
                    del img
                    frame_no += 1
                    if objs:
                        obj_frames += 1
                        last_objs = objs
                        if sample is None:
                            sample = objs
                    # w/h 用**推流**分辨率：前端拿归一化坐标 × 自己的显示尺寸
                    payload = encode(last_objs, config.WIDTH, config.HEIGHT,
                                     frame_no, int(time.time() * 1000))
                    if not r_pi.ok:
                        r_pi.connect()
                    r_pi.send(payload)
                    if r_cloud is not None:
                        if not r_cloud.ok:
                            r_cloud.connect()
                        r_cloud.send(payload)
                else:
                    print("SNAPSHOT None")
                gc.collect()

            # 4) 30 s 一行 STATS
            if time.ticks_diff(now, t_stats) >= STATS_MS:
                t_stats = now
                print("STATS t=%ds frame=%d obj=%d sent=%d dropped=%d lost=%d "
                      "conns=%d rtsp=%d rtsp_empty=%d stall=%d report=%d/%d mem=%d"
                      % (time.ticks_diff(now, t_start) // 1000, frame_no, obj_frames,
                         pusher.frames_sent, pusher.frames_dropped,
                         pusher.lost_events, pusher.connects,
                         rtsp.frames, rtsp.empty, wd.stalls,
                         r_pi.sent, r_pi.dropped, gc.mem_free()))

    except BaseException as e:
        import sys as _s
        _s.print_exception(e)
    finally:
        print("APP CLEANUP_START")
        try:
            pusher.close()
        except Exception as e:
            print("app: pusher.close EXC", e)
        try:
            rtsp.stop()
        except Exception as e:
            print("app: rtsp.stop EXC", e)
        try:
            cam.stop()
        except Exception as e:
            print("app: cam.stop EXC", e)
        try:
            det.deinit()
        except Exception as e:
            print("app: det.deinit EXC", e)
        gc.collect()
        print("APP CLEANUP_DONE MEM %d" % gc.mem_free())


if __name__ == "__main__":
    main()
