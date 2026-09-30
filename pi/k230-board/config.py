# /sdcard/k230vision/config.py
"""全部可调参数集中在这里。

## 环境相关的真值**不在**本文件里（2026-10-01 起）

本仓库是**公开**的，所以 WiFi 凭据与局域网地址不入库，拆到同目录的
`config_local.py` 里：

    本文件（入库）     占位符 + 与机器无关的参数（分辨率/阈值/端口号…）
    config_local.py    真值：WiFi SSID/密码、各主机地址 —— **不入库**（见 .gitignore）

覆盖机制就是文件**末尾**那句 `from config_local import *`。

⚠️ **板上必须有 config_local.py**，否则连不上 WiFi。症状是 app.py 一直打
`WIFI FAILED, retry`，**看起来像板子坏了，其实只是配置文件缺了**（见
`5.K230/README.md` §7.5）。所以缺文件时末尾会大声报一句。

**换网络 / 换车 / 换接收端，只改 config_local.py，本文件不用动。**
"""

# ---- 环境相关：这里只是占位符，真值在 config_local.py（文件末尾 import 覆盖）----
WIFI_SSID = "REPLACE_ME"
WIFI_PASS = "REPLACE_ME"

# 云端推流目标（先用 Pi 做验证，云端就绪后改成云端公网 IP）
PUSH_HOST = "REPLACE_ME"
PUSH_PORT = 8555

# 结构化结果上报目标
RESULT_HOST = "REPLACE_ME"
RESULT_PORT = 8556

# 云端结果上报目标。**云端未就绪，留空串表示不启用**。
# 主循环里用 `if config.CLOUD_RESULT_HOST:` 判断 —— 绝不让本地（Pi）路径因此失败。
CLOUD_RESULT_HOST = ""
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

# ---- 整机朝向：相机实物是**倒装**的，画面应当在 sensor 根上翻 180°（2026-09-29 加）----
#
# 实现：cam.py 的 Camera.start() 里 hmirror+vflip 同时开（sensor 根开关，
# chn0(推流)/chn1(RTSP)/chn2(AI) 一起转）。
#
# ⚠️ 别把它当"设计选择"，它是**待实测确认的假设**：判据是抓三路画面落盘、
# 在 PC 侧与 `np.rot90(img, 2)` 逐像素对照（板上探针 t_rot_off.py / t_rot_on.py）。
# 只翻一路 / 压根没翻，都不会抛异常 —— **"函数没报错"不是证据**。
# 实测结论记在 `5.K230/README.md`；改这里之前先读那份实测。
#
# **只改这一处。** 一旦实测通过，app.py / vision.py / 前端 / 下游**任何地方都不许
# 再做 ±180° 补偿**，否则二次翻转（不会报错，只会让画面又倒回去）。
ROTATE_180 = True

# 看门狗
PUSH_STALL_WARN_MS = 3000   # 连续这么久没成功发出帧就告警
RECONNECT_BACKOFF_MS = 1000

# ---- Task 5: KPU 推理 ----
# 模型：官方 /sdcard/examples/05-AI-Demo/object_detect_yolov8n.py 用的 COCO 80 类 YOLOv8n。
# 选它的原因：**必须能认出苹果**（COCO 类别 id 47 = "apple"）。验收时镜头前放的是苹果，
# 人脸/人体这类专用模型认不出来，等于无法做坐标正确性核对。
# 板上有 yolov8n_224 和 yolov8n_320 两个同族模型；用 320 是因为苹果在画面里偏小，
# 输入分辨率高一点检出率更好（AI 帧分辨率与推流分辨率无关，见下）。
KMODEL_PATH = "/sdcard/k230vision/vendor/kmodel/yolov8n_320.kmodel"
# COCO 80 类，顺序与官方 object_detect_yolov8n.py 逐字一致；index 47 = "apple"
LABELS = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
MODEL_INPUT_SIZE = [320, 320]
MAX_BOXES_NUM = 30
CONF_THRESHOLD = 0.3
NMS_THRESHOLD = 0.4

# 给 KPU 的那路 sensor 通道分辨率（CAM_CHN_ID_2，RGBP888）。
#
# **必须是 16:9，不能配成正方形。** sensor 根是 1280x720(16:9)，若 AI 通道配 320x320，
# 硬件会把整幅画面**纵向拉长 1.78 倍**（1280÷4.0 vs 720÷2.25）—— 实测：橘子是球，
# 在 320x320 的 AI 帧里却是一个**竖着的蛋**（见 2026-09-19-task1-evidence.md §3b）。
# YOLO 在不变形的自然图上训练，这是实打实的域偏移，是"检测效果差"的主因。
#
# 而且这个形变**不是均匀降低质量**：椅子本来就是竖向物体，拉长后反而更像椅子、能检对；
# 球被拉成蛋则完全脱离训练分布。所以症状是"有的检得对、有的完全不行"，
# **很容易被误判成"模型不行"** —— 那不是模型的问题。
#
# 320x180 = 1280x720 的**均匀 ÷4**，不变形；再由 vision.py 用 ai2d 做 letterbox
# 补灰到模型的 320x320。对 320x180 -> 320x320 有 ratio=1.0，
# 即那一步 resize 是恒等变换、**不重采样** —— 全程只有 sensor 这一次缩放，路径最短。
#
# 连带关系：vision.py 的 display_size 取自这里，所以改这里坐标会跟着自动对。
# 想改成别的尺寸，先回读设计文档 §6；改完必须用 t_geom.py 重验几何。
AI_WIDTH = 320
AI_HEIGHT = 180

# ---- 跟踪器（NanoTrack）----
# 锁哪个类去跟。
# ⚠️ 这只是一个**临时的框来源** —— 只能覆盖检测器认识的那 80 类。
# 检测器不认识的物体（桃、辣椒……）需要**人给框**，那条路（点屏/语音）**还没做**。
# 见 docs/plans/2026-09-19-tracker-integration.md §5。
TRACK_CLASS = "chair"
TRACK_THRESH = 0.1      # nanotracker_head 的阈值（照抄官方例程）
TRACK_AR_TOL = 0.5      # 长宽比偏离阈值（暂定，只有一轮数据）
TRACK_IOU_MIN = 0.3     # 与检测框的 IoU 下限（暂定）

# ---- 外部目标框入口（targets.py）----
# 让 K230 能被"从外面指定跟哪个东西" —— 检测器不认识的物体（桃、辣椒）唯一的入口。
# 协议是一行 JSON，一次命令一个连接：
#     {"pt": [u, v]}                  # 点（归一化中心点），自动开一个方框
#     {"pt": [u, v], "size": 0.12}    # size = 边长占画面宽度的比例
#     {"box": [l, t, r, b]}           # 归一化框，与 results.normalize 同约定
#     {"cmd": "stop"}                 # 停止跟踪
# 坐标一律**归一化到推流画面**，调用方不需要知道 AI 帧多大、有没有 letterbox。
TARGET_PORT = 8557
TARGET_SIZE = 0.12      # 只给点时，默认方框边长占画面宽度的比例

# 「跟着某个人」（cmd=follow）等正脸等多久。超时就放弃，且**不动 det_auto** ——
# 人不在画面里不该把板子自带的自动锁定废掉。
FOLLOW_TIMEOUT_MS = 8000

# ---- 人脸（faces.py）----
# 检测 -> 对齐(112x112) -> 512 维特征 -> 活体，实测约 17ms/张脸。
# 特征库在 /sdcard/k230vision/facedb/<名字>.bin（每人多帧，比对取最大）。
# **身份字符串就是注册时的名字**（ASCII，如 id1）；映射到"小a/小b"是消费侧的事。
FACE_ENABLE = True
FACE_ANCHORS = "/sdcard/k230vision/vendor/prior_data_320.bin"
FACE_DB_DIR = "/sdcard/k230vision/facedb"
FACE_THRESHOLD = 0.78   # ⚠️ 暂定 —— 依据见 faces.py 的注释（分离窗口只有 0.047 宽，
                          #    且"不同人"那组是在修取景之前测的）

# ---- 增量入库（"边用边长"）----
# 注册**不是一次仪式**：认得出的人每次出现在镜头前，都可能给库里补一帧。
# 于是"注册要多久"这件事不再重要 —— 种子可以很短，库自己会随使用长起来。
#
# ⚠️ **入库门限必须远高于识别门限。** 增量入库会**自我强化错误**：
#    把误认的陌生人写进谁的库，以后就更像他 —— 这是它唯一的真危险。
#    所以只有**非常有把握**的帧才准进（再加：过质量门、画面里只有一张脸、
#    与库里已有的太像也不加）。
FACE_ADMIT = 0.90
FACE_DB_MAX = 40              # 每人最多存多少条（超了丢最早的，旧姿态让位给新姿态）
                              # 40 条 x 2KB(ulab float32) = **80 KB/人**
                              #   -> 3 MB 的堆可装 ~37 人（原来是 Python 浮点列表，
                              #      20KB/条 = 1.6MB/人，装不下两个）
FACE_SAVE_EVERY_MS = 30000    # 落盘间隔（别每帧写 SD）
FACE_SEED_SECONDS = 4.0       # **种子**注册的时间预算（之后靠增量长）


# ============================================================================
# 本地环境覆盖 —— **必须在文件末尾**（前面全是默认值/占位符）
# ============================================================================
#
# 真值放同目录的 `config_local.py`，那个文件**不入库**（仓库是公开的，
# 里面是 WiFi 凭据和局域网地址）。换网络/换接收端只改它，本文件不用动。
#
# MicroPython 的 `from x import *` 会覆盖本模块里已有的同名名字 —— 这正是要的：
# 谁被写进 config_local.py，谁就以那边为准。
#
# ⚠️ 缺文件的后果是**看起来像硬件坏了**：板子连不上 WiFi → app.py 一直打
#    `WIFI FAILED, retry` → 不进主循环 → 没有 RTSP、没有 8556 上报。
#    所以这里不做静默兜底，直接大声报。
try:
    from config_local import *
    _LOCAL_OK = True
except ImportError:
    _LOCAL_OK = False

if (not _LOCAL_OK) or WIFI_SSID == "REPLACE_ME":
    print("!!! ------------------------------------------------------------")
    print("!!! config_local.py 缺失，或没覆盖 WIFI_SSID/WIFI_PASS")
    print("!!! 板子将连不上 WiFi（且看起来像硬件故障）。")
    print("!!! 把 config_local.py 放到 /sdcard/k230vision/ 下再上电。")
    print("!!! 该文件不入库；模板见 pi/k230-board/config_local.py.example")
    print("!!! ------------------------------------------------------------")
