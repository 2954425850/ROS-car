# /sdcard/k230vision/config.py
"""全部可调参数集中在这里。"""

WIFI_SSID = "TP-LINK_EE82"
WIFI_PASS = "123456789"

# 云端推流目标（先用 Pi 做验证，云端就绪后改成云端公网 IP）
PUSH_HOST = "114.215.188.147"   # 云端（2026-09-18 起，视频去云端）
PUSH_PORT = 8555

# 结构化结果上报目标
RESULT_HOST = "192.168.1.108"
RESULT_PORT = 8556

# 云端结果上报目标。**云端未就绪，留空串表示不启用**。
# 主循环里用 `if config.CLOUD_RESULT_HOST:` 判断 —— 绝不让本地（Pi）路径因此失败。
CLOUD_RESULT_HOST = "114.215.188.147"   # 结果也发云端（Pi 那路保留，两边都收）
CLOUD_RESULT_PORT = 8556

# 视频规格
WIDTH = 1280
HEIGHT = 720
FPS = 25                 # 目标帧率，走 VENC 的 dst_frame_rate
# sensor 档位。**必须配 30**：Sensor(fps=30) 真的选到 1080p30 档，端到端实测
# 输出 ~29.7fps / ~2.04 Mbps（干净板子上复测，见 NOTES.md）。
# 注意：Sensor(fps=25) **不被支持但会静默忽略**，直接回落到 60fps 档（实测 53.6fps），
# 于是 src_frame_rate=25 作除数会把码率放大到 4.2 Mbps —— 这是个静默陷阱。
# 另注：dst_frame_rate 本 build **不丢帧**（试过 25 和 15，输出都还是 ~27~30fps），
# 所以实际输出帧率就等于 sensor 档位，25fps 拿不到。
SENSOR_FPS = 30
BITRATE_KBPS = 2048      # 每帧预算，非硬上限；总码率 ≈ bit_rate × 实际fps / SENSOR_FPS
GOP = 25                 # 1 秒，任何损伤 1 秒内自愈
PUSH_CHN = 0             # chn0 -> 推云端
RTSP_CHN = 1             # chn1 -> 局域网给 Pi
RTSP_PORT = 8554
RTSP_SESSION = "k230"

# 看门狗
PUSH_STALL_WARN_MS = 3000   # 连续这么久没成功发出帧就告警
RECONNECT_BACKOFF_MS = 1000

# ---- Task 5: KPU 推理 ----
# 模型：官方 /sdcard/examples/05-AI-Demo/object_detect_yolov8n.py 用的 COCO 80 类 YOLOv8n。
# 选它的原因：**必须能认出苹果**（COCO 类别 id 47 = "apple"）。验收时镜头前放的是苹果，
# 人脸/人体这类专用模型认不出来，等于无法做坐标正确性核对。
# 板上有 yolov8n_224 和 yolov8n_320 两个同族模型；用 320 是因为苹果在画面里偏小，
# 输入分辨率高一点检出率更好（AI 帧分辨率与推流分辨率无关，见下）。
KMODEL_PATH = "/sdcard/examples/kmodel/yolov8n_320.kmodel"
# COCO 80 类，顺序与官方 object_detect_yolov8n.py 逐字一致；index 47 = "apple"
LABELS = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
MODEL_INPUT_SIZE = [320, 320]
MAX_BOXES_NUM = 30
CONF_THRESHOLD = 0.3
NMS_THRESHOLD = 0.4

# 给 KPU 的那路 sensor 通道分辨率（CAM_CHN_ID_2，RGBP888）。
# **必须等于 MODEL_INPUT_SIZE**：官方的 ObjectDetectionApp 重写了 preprocess()
# 直接 `nn.from_numpy(input_np)` 把 sensor 帧原样喂给模型，ai2d 那套预处理在这条路径上
# 是被绕过的。所以 AI 帧分辨率 = 模型输入分辨率。
# AI 帧和推流帧同源同 sensor，但分辨率可以不同 —— 坐标靠归一化解耦。
AI_WIDTH = 320
AI_HEIGHT = 320
