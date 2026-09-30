#!/usr/bin/env python3
# /home/cy/k230-vision/pi/gimbal_track_run.py —— 云台两轴持续跟踪的**运行脚本**
#
# 控制器是 `gimbal_track.GimbalTracker`（Task 2，已验收）。本文件只做三件事：
# 把真实世界接上去、把真相打印出来、把危险关在 `--live` 后面。**不含控制律。**
#
# ---------------------------------------------------------------------------
# 用法
#
#     cd /home/cy/k230-vision/pi
#     python3 gimbal_track_run.py                 # 默认 --dry-run：只看不做
#     python3 gimbal_track_run.py --live          # 真正驱动云台（会动！）
#
# **默认 dry-run，真正驱动必须显式 `--live`。** 这不是风格问题：云台是
# 「最后一条消息赢、没有优先级、没有看门狗」的（见下面"外来发布者"一节），
# 一旦跑起来就没有任何东西能兜住它，所以"给参数才动"必须写在最外层。
# `--dry-run` **完全不 import ROS**，也不建任何 ROS 节点。
#
# ---------------------------------------------------------------------------
# 这个脚本存在的主要理由：判新鲜度只能看 mtime，而只有它看得到
#
# `gimbal_track.GimbalTracker.read_target()` 只有 `(box, frame)` 和 `None`
# 两种返回，所以"文件读不出来"和"这一帧没有 src=='track'"在模块里**分不开**，
# 模块只能用时钟猜（超过 stale_s 就报 stale 而不是 no_target）。
#
# **本适配层是唯一知道真相的一方**（它读得到 mtime），所以：
# 运行脚本打印的原因一律以 `read_snapshot()` 的判定为准，模块那句只作参考。
# 实车调参时看到的会是
#
#     target 缺失但数据新鲜（0.08s） —— 就是跟踪器这一帧没跟到，不是断流
#     数据已 3.41s 没更新（mtime）—— 是断流，去看 k230-resultd / 板子
#
# 这两种情况的处置完全不同，混在一起就没法调。**没有去改 gimbal_track.py 的接口。**
#
# ---------------------------------------------------------------------------
# 反馈源：/tmp/k230/latest-result.json（一行 JSON，6Hz，k230-resultd 写）
#
#     {"w":1280,"h":720,"frame":1396,"ts":1789951147000,
#      "objs":[{"cls":"person","score":0.503,"box":[0.375,0,0.709,0.644]},
#              {"cls":"person","score":1.0,"box":[0.386,0.001,0.715,0.616],
#               "track_id":1,"src":"track"}],
#      "_received_at":"2026-09-21 08:39:08","_peer":"192.168.1.112:64024"}
#
#   · **要跟的是 `src == "track"` 那一条**（跟踪器锁定、帧间连续，实测存在率
#     ~87%）。没有 `src` 的是检测器当帧检出的，会跳会丢，**不要用**。
#   · ⚠️ **`score` 不能用来判断有没有跟丢**（跟到背景上 score 一样 0.999）。
#     所以这里**完全不看 score**，也没有任何"置信度太低就丢目标"的逻辑。
#   · ⚠️ **判新鲜度只能用文件 mtime**，**不能用 `_received_at`** ——
#     它只有秒精度，而且是 resultd 收到那一帧的时刻，不是文件落盘的时刻。
#     `_received_at` 在本脚本里只用来**对照打印**（两者差得远就告警），
#     从不参与任何判断。这条有专门的单测（t_gimbal_run.py 第 5 组）。
#
# ---------------------------------------------------------------------------
# rotator：`CarController.camera_nudge`，为什么不用 `car.ros_bridge.RosBridge`
#
# rotator 用现成的 `CarController.camera_nudge(direction, degrees)` —— 它返回
# 结构化 `(目标脉宽 us, 是否撞限位)`，内部有 800~2200us 的工程安全行程和限位
# 判断，正是 `camcenter` / `gimbal_track` 要求的契约（`t_camcenter_e2e.py`
# 已经这么用过）。
#
# ⚠️ **但节点名必须换掉。** `RosBridge.__init__` 把节点名硬编码成
# `voice_chatbot_car`，而语音助手是 systemd 常驻、**已经在用这个名字**
# —— 2026-09-21 实查（只读）：
#
#     $ ros2 node list
#     /l150pro_driver
#     /rosapi
#     /rosbridge_websocket
#     /voice_chatbot_car        <-- 语音助手，活着
#
# 同名节点在这里是有害的，不是洁癖：`ros2 node list` / `rosapi/nodes` 会把两个
# 进程显示成一个节点，**现场根本分不出跟踪器在没在跑**；将来任何按节点名寻址
# 的东西（/voice_chatbot_car/xxx）都会歧义。
#
# 所以本脚本自己起一个名叫 `gimbal_track_run` 的节点（见 `_ServoBridge`），
# 通过 `CarController(bridge_factory=...)` 注入 —— **只换名字，不换语义**：
# `publish_servo` 的帧格式和长度检查与 RosBridge 逐字一致，`camera_nudge`、
# `_camera_us` 记账、`_servo_frame()` 的「0 = 该路保持不动」约定全部原样保留。
#
# ---------------------------------------------------------------------------
# ⚠️ `/servo_cmd` 外来发布者：只告警，**不抢回来**（刻意的设计选择）
#
# 云台是「最后一条消息赢」：没有优先级、没有看门狗、没有仲裁。语音助手的
# `voice_chatbot_car` 已经在发这个 topic（实查 publisher count = 1），本脚本
# 一起来就是 2 —— 从那一刻起**谁最后发谁生效**，两边都能把对方顶掉。
#
# 运行脚本会在启动后（等 DDS 发现完，3s）和每个状态行周期检查发布者数量，
# `> 1` 就打印醒目告警。
#
# **★ 但绝不去"抢回来"。** 被顶掉时报 "我没有在发" 或干脆什么都不做，
# 不重复发、不加频率、不做看门狗重发。原因：两边各自维护着一套**绝对脉宽
# 假设**（`CarController._camera_us` / teleop 的 `self.gimbal`），互相抢写会让
# 两套假设一起烂掉，而且**从任何一边都看不出来**——表现是云台"自己慢慢跑偏"，
# 谁都不知道该怪谁。宁可让跟踪器被顶掉（肉眼可见、可复现），也不要制造一个
# 看不见的漂移。这条是有意为之，**不要"优化"成抢占式**。
#
# 另外：本脚本**只动云台**（ch5/ch6），底盘电机一概不碰 —— 所以注入的桥的
# `publish_velocity` 是**故意做成空操作**的，见 `_ServoBridge.publish_velocity`。
#
# ---------------------------------------------------------------------------
# /gimbal_track 诊断话题（照 driver_node 的 /yaw_loop 那个模式）
#
# `Float64MultiArray`，**纯观测、不回馈控制路径**：
#
#     [du, dv, outcome码, ch5脉宽us, ch6脉宽us, 目标存在?, 数据年龄s]
#
# du/dv = 0.5 - 框中心（>0 = 目标偏画面左/上）；这一拍没框时是 NaN。
# 脉宽是**本进程以为自己发出去的**值（舵机没有位置反馈，读不回来）。
# 数据年龄出自 mtime。**不想要了，把 `_outcome_code` 和 `_publish_diag`
# 和 `_ServoBridge` 里建 diag 发布者的那两行一起删掉即可，无副作用。**
#
# 代码在 /home/cy/k230-vision/pi，**不用 colcon**（照 ~/teleop_l150pro.py 的习惯）。

import argparse
import json
import os
import sys
import time

sys.path.insert(0, "/home/cy/voice-chatbot")
sys.path.insert(0, "/home/cy/k230-vision/pi")   # pi 在前：本目录模块优先

from gimbal_track import (GimbalTracker, TrackConfig, describe,   # noqa: E402
                          HOLD, TRACKED, NO_TARGET, NO_NEW_FRAME, STALE,
                          HIT_LIMIT, BAD_BOX)

RESULT_PATH = "/tmp/k230/latest-result.json"

# 适配层判"数据新鲜"的 mtime 阈值（秒）。板子 6Hz → 0.167s 一帧，0.5s = 连丢
# 3 帧。**这个值才是权威**：`GimbalTracker.stale_s`（默认 1.0）只是模块内部
# 拿时钟猜的兜底，两者故意不同——见文件头那段说明。
MAX_AGE_S = 0.5

# 状态行周期（秒）。实车调参靠它，别调太密（会盖掉别的输出）。
STATUS_EVERY_S = 5.0

# `/servo_cmd` 发布者数量在启动后多久才开始查 —— DDS 发现需要时间，
# 一上来就查必然报"只有我自己"。driver_node 的 _check_link 也用 3.0s，照抄。
PUB_CHECK_AFTER_S = 3.0

# ---- 适配层判定结果（**权威**）----
_S_OK = "ok"                     # 有 src=="track"，且数据新鲜 → (box, frame)
_S_NO_TRACK = "no_track"         # 数据新鲜，但这一帧没有 src=="track"
_S_STALE = "stale"               # mtime 太旧 —— 数据已 X 秒没更新
_S_MISSING = "missing"           # 文件不存在
_S_UNREADABLE = "unreadable"     # 读不动 / 坏 JSON / 空文件 / 顶层不是对象
_S_NO_FRAME = "no_frame"         # 有目标但顶层没有 frame，没法判"是不是新帧"

# 每档判定的权威原因文本在 explain() 里 —— **那是运行脚本打印的原因**，
# 不是模块拿时钟猜的那句（模块分不开"读不出来"和"没跟到"）。

# 诊断话题用的结果码（顺序即下标，别插队）。
_OUTCOMES = (HOLD, TRACKED, NO_TARGET, NO_NEW_FRAME, STALE, HIT_LIMIT, BAD_BOX)
_OUTCOME_CODE = {oc: i for i, oc in enumerate(_OUTCOMES)}


# ===========================================================================
# 自动加载 ROS 环境：没 source 就自己重跑一遍（照抄 ~/teleop_l150pro.py）
# ===========================================================================
def _ensure_ros():
    """`--dry-run` 直接返回，**完全不 import ROS**。

    真正驱动时才需要 ROS —— 所以判据是"`--live` 在不在 argv 里"，而不是
    teleop 那个"`--dry-run` 在不在"。**默认 dry-run 也走这条早退。**
    """
    if "--live" not in sys.argv:
        return
    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_GIMBAL_TRACK_ROS") == "1":
        sys.stderr.write(
            "错误：加载 ROS 2 环境失败。请手动执行：\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source ~/l150pro_ws/install/setup.bash\n")
        sys.exit(1)
    os.environ["_GIMBAL_TRACK_ROS"] = "1"
    cmd = ('source /opt/ros/jazzy/setup.bash; '
           'source "$HOME/l150pro_ws/install/setup.bash" 2>/dev/null; '
           'exec python3 "$0" "$@"')
    os.execvp("bash", ["bash", "-c", cmd, sys.argv[0]] + sys.argv[1:])


_ensure_ros()


# ===========================================================================
# 适配层：**纯文件 / 纯函数** —— 单测（t_gimbal_run.py）直接 import 这些测
# ===========================================================================

class Snapshot:
    """这一拍到底读到了什么。**权威判定在这里**（它读得到 mtime）。

    模块内部只能拿时钟猜，所以运行脚本打印的原因一律引用本对象。
    """

    __slots__ = ("state", "box", "frame", "age", "note")

    def __init__(self, state, box=None, frame=None, age=0.0, note=""):
        self.state = state
        self.box = box
        self.frame = frame
        self.age = age          # 文件年龄（秒），来自 mtime；-1 表示拿不到
        self.note = note        # 给人看的一句细节

    @property
    def ok(self):
        return self.state == _S_OK

    def __repr__(self):
        return "<Snapshot %s age=%.2f frame=%s %s>" % (
            self.state, self.age, self.frame, self.note)


def explain(snap):
    """**权威原因**一句话。运行脚本打印的就是它。

    刻意不复用 `gimbal_track.describe()` —— 那句是拿时钟猜出来的
    （分不开"文件读不出来"和"这一帧没跟到目标"），作为**原因**是错的。
    两句话都在屏幕上，各标各的来源。
    """
    if snap.state == _S_OK:
        return "数据新鲜（%.2fs），有 src==\"track\" 的目标" % snap.age
    if snap.state == _S_NO_TRACK:
        return ("target 缺失但数据新鲜（%.2fs）—— 是跟踪器这一帧没跟到，"
                "不是断流" % snap.age)
    if snap.state == _S_STALE:
        return ("数据已 %.2fs 没更新（按 mtime，权威）—— 是断流，"
                "去查 k230-resultd / 板子" % snap.age)
    if snap.state == _S_MISSING:
        return "结果文件不存在：%s" % snap.note
    if snap.state == _S_UNREADABLE:
        return "结果文件读不出来：%s" % snap.note
    if snap.state == _S_NO_FRAME:
        return ("有 src==\"track\" 的目标但顶层没有 frame —— 缺帧号就不敢发指令，"
                "否则会拿同一张旧图反复转")
    return "未知判定（%s）" % snap.state


def data_age_s(path, now=None):
    """结果文件的年龄（秒）。**只能用 mtime。**

    ⚠️ **不能用 `_received_at`**：它只有秒精度（同一个文件连续几拍都是同一个
    字符串），而且它是 `k230-resultd` **收到**那一帧的时刻，不是文件**落盘**的
    时刻（写盘可能被拖，也可能先写临时文件再 rename）。判新鲜度只有 mtime
    是权威 —— 这条有专门的单测（t_gimbal_run.py 第 5 组）。
    """
    now = time.time() if now is None else now
    return now - os.stat(path).st_mtime


def received_at_age_s(path, now=None):
    """`_received_at` 字段折算的年龄（秒）；拿不到返回 None。

    **只用来对照打印**（跟 `data_age_s` 差得远就告警，说明有个"看着新鲜其实
    很旧"的坑在发生）—— **从不参与任何判断**。见文件头那段说明。
    """
    now = time.time() if now is None else now
    d, _why = load_json(path)
    if d is None:
        return None
    stamp = d.get("_received_at")
    if not isinstance(stamp, str):
        return None
    try:
        # 板子写的是本地时间 "2026-09-21 08:39:08"，mktime 按本地时区解 —— 一致。
        return now - time.mktime(time.strptime(stamp, "%Y-%m-%d %H:%M:%S"))
    except (ValueError, OverflowError):
        return None


def load_json(path):
    """读出顶层 dict；任何读不动的情形返回 `(None, 一句为什么)`。

    **不抛异常** —— 结果文件是另一个进程（k230-resultd）写的，读到半个文件
    是正常事件，不是本脚本的 bug。
    """
    try:
        with open(path, "r") as f:
            raw = f.read()
    except FileNotFoundError:
        return None, "文件不存在"
    except OSError as exc:
        return None, "读文件失败：%s" % exc
    if not raw.strip():
        return None, "文件是空的"
    try:
        d = json.loads(raw)
    except ValueError as exc:
        return None, "JSON 解不开：%s" % exc
    if not isinstance(d, dict):
        return None, "顶层不是对象（%s）" % type(d).__name__
    return d, ""


def pick_track(objs):
    """挑出 `src == "track"` 那一条。没有就 None。

    板子理论上只写一条。**多条时取 JSON 数组里的第一条** —— 这条规则是刻意
    定的：取第一条在帧与帧之间**稳定**（写序由板子控制），而"取最新的"需要
    一个本脚本看不到的时间戳。**不要用 score 挑**：score 判不出跟丢（跟到
    背景上照样 0.999），这是 2026-09-19 实测踩过的坑。
    """
    if not isinstance(objs, list):
        return None
    for o in objs:
        if isinstance(o, dict) and o.get("src") == "track":
            return o
    return None


def read_snapshot(path=RESULT_PATH, max_age_s=MAX_AGE_S, now=None):
    """这一拍读到了什么。**这是适配层唯一对外的事实来源。**

    先后次序是刻意的：**先看 mtime，再看内容**。旧文件不管内容多漂亮都不能用
    （这正是"`_received_at` 看着新鲜、mtime 很旧"那个坑的解法）。
    """
    now = time.time() if now is None else now

    if not os.path.exists(path):
        return Snapshot(_S_MISSING, age=-1.0, note=path)

    try:
        age = data_age_s(path, now)
    except OSError as exc:
        return Snapshot(_S_MISSING, age=-1.0, note="stat 失败：%s" % exc)

    if age > max_age_s:
        return Snapshot(_S_STALE, age=age,
                        note="文件 %s" % path)

    d, why = load_json(path)
    if d is None:
        return Snapshot(_S_UNREADABLE, age=age, note=why)

    o = pick_track(d.get("objs"))
    if o is None:
        return Snapshot(_S_NO_TRACK, age=age, note="这一帧 objs 里没有 src==\"track\"")

    # ⚠️ `frame` 是**顶层**字段（不在 obj 里）。它缺了就不能返回 (box, frame)：
    # 帧号是模块唯一的"同一帧不重复发指令"保护，缺了它会拿同一张旧图反复转
    # （camcenter 的原话："转飞"）。**宁可不跟这一拍。**
    frame = d.get("frame")
    if frame is None:
        return Snapshot(_S_NO_FRAME, box=o.get("box"), frame=None, age=age,
                        note="顶层没有 frame 字段")

    # box **原样透传**（可能是坏的）—— 坏框由 gimbal_track._bad_box 判，
    # 它的报错比这里能给的更具体，不重复实现。
    return Snapshot(_S_OK, box=o.get("box"), frame=frame, age=age)


def make_read_target(path=RESULT_PATH, max_age_s=MAX_AGE_S, clock=time.time):
    """给 `GimbalTracker` 的 `read_target()` 适配层。

    返回 `(read_target, last)`：`last` 是个单元素列表，每次调用后塞进这一拍的
    最新 `Snapshot`。**运行脚本靠它打印权威原因** —— 模块内部只看得到
    `(box, frame)` / `None`，分不开"读不出来"和"没跟到"。
    """
    last = [None]

    def read_target():
        snap = read_snapshot(path, max_age_s, clock())
        last[0] = snap
        if not snap.ok:
            return None
        return (snap.box, snap.frame)

    return read_target, last


def make_rotator(ctrl):
    """把 `camera_nudge` 包成 rotator，顺手记下**两轴各自**最后的脉宽。

    返回 `(rotator, last_us)`。`last_us = {"pan": us|None, "tilt": us|None}`
    —— 只喂给状态行和 /gimbal_track 诊断话题用；**控制路径不看它**
    （真值在 `CarController._camera_us` 里，本函数不重复记账）。

    阈值判断只在 `ctrl` 那一侧（800~2200us 行程、限位），这里一个字都不加。
    """
    last_us = {"pan": None, "tilt": None}

    def rotator(direction, degrees):
        us, hit_limit = ctrl.camera_nudge(direction, degrees)
        last_us["tilt" if direction in ("up", "down") else "pan"] = us
        return us, hit_limit

    return rotator, last_us


# ===========================================================================
# 桥：dry-run 用假的（不 import ROS），live 用真的（节点名换掉）
# ===========================================================================

class _Cfg:
    """给 `CarLimits.from_config` 的空配置 —— 全部走 dataclass 的默认值。

    照 t_camcenter_e2e.py 的做法。云台行程就是 `CarLimits` 里的
    800~2200us（teleop 的保守值，防堵转扫齿），不是这里另写一遍。
    """

    def get(self, key, default=None):
        return default


class _FakeBridge:
    """dry-run 的假桥：只记账，不 import ROS、不碰硬件。

    它让 `--dry-run` 走的是**和 --live 完全同一条代码路径**（同一个
    `CarController.camera_nudge`），所以 dry-run 打出来的脉宽就是 live 会发出去
    的脉宽 —— 唯一的差别是帧没进 ROS。
    """

    def __init__(self, **_kw):
        self.frames = []            # 每次 publish_servo 的 6 路帧快照

    def publish_servo(self, us):
        if len(us) != 6:
            raise ValueError("舵机帧需要 6 路，收到 %d" % len(us))
        self.frames.append(list(us))

    def publish_velocity(self, vx, wz):
        """**故意什么都不做。** dry-run 本来就不该碰底盘，理由同 _ServoBridge。"""

    def close(self):
        pass


class _ServoBridge:
    """live 用的最小 ROS 桥。**节点名是 `gimbal_track_run`，不是 `voice_chatbot_car`。**

    为什么不用 `car.ros_bridge.RosBridge`：那个类把节点名硬编码成
    `voice_chatbot_car`，而语音助手（systemd 常驻）已经在用这个名字
    （2026-09-21 实查 `ros2 node list`，见文件头）。同名节点会让
    `ros2 node list` / `rosapi/nodes` 把两个进程混成一个，现场没法分辨。

    **只换名字，不换语义**：`publish_servo` 的帧格式、长度检查和 QoS
    （RELIABLE / KEEP_LAST / depth 10）与 `RosBridge` 一致 —— 那套 QoS 是
    对着 driver_node 的订阅端验过的（`ros2 topic info /servo_cmd -v`）。
    """

    _SERVO_CHANNELS = 6

    def __init__(self, *, cmd_rate_hz=25.0, node_name="gimbal_track_run",
                 enable_diag=True):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._rclpy = rclpy
        self._JointState = JointState

        rclpy.init(args=[])          # 传空列表：别让 rclpy 去解析我们自己的参数
        self.node = Node(node_name)
        self.node_name = node_name

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub_servo = self.node.create_publisher(JointState, "servo_cmd", qos)

        # ---- /gimbal_track 诊断话题。纯观测，不回馈控制路径。----
        # **不想要了就把这几行和 Runner._publish_diag() 一起删掉，无副作用。**
        # （照 driver_node.py 里 /yaw_loop 的同一模式。）
        self._pub_diag = None
        self._diag_cls = Float64MultiArray
        if enable_diag:
            self._pub_diag = self.node.create_publisher(
                Float64MultiArray, "gimbal_track", qos)

        self.servo_frames_sent = 0

    # ---- 下行 ----
    def publish_servo(self, us):
        """发一帧舵机脉宽。长度必须为 6；0 = 该路保持不动（固件约定）。"""
        if len(us) != self._SERVO_CHANNELS:
            raise ValueError("舵机帧需要 %d 路，收到 %d"
                             % (self._SERVO_CHANNELS, len(us)))
        msg = self._JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ["ch%d" % i for i in range(1, self._SERVO_CHANNELS + 1)]
        msg.position = [float(x) for x in us]
        self._pub_servo.publish(msg)
        self.servo_frames_sent += 1

    def publish_velocity(self, vx, wz):
        """**故意的空操作 —— 这是安全边界，不是没写完。**

        本脚本**只动云台**（ch5/ch6），底盘电机一概不碰。但 `CarController.close()`
        → `stop_all()` 会调 `publish_velocity(0, 0)` 连发 8 帧；真发出去就是往
        `/cmd_vel` 上插一脚（而云端上位机空闲时也在 20Hz 发零，多一个发布者只会
        让"谁在发"更难查）。所以这里直接吞掉 —— **宁可什么都不发**。

        真需要让这个脚本碰底盘时，**不要**在这里改成 `publish` —— 那是另一个决定
        （底盘有 200ms 丢帧看门狗，发帧断了车会自己停，语义和云台完全不同）。
        """
        pass

    def state(self):
        """只为让 `CarController` 的接口完整 —— 本桥**不做任何上行订阅**。

        所以恒返回 `online=False` 的快照。谁要是拿它去 `status_text()`
        会得到"还读不到小车的状态"，这是**事实**，不是 bug（真值要么从
        `voice_chatbot_car` 读，要么另建桥 —— 都不该在这里悄悄做）。
        """
        from car.types import RobotState
        return RobotState(voltage=None, fault=None, wheel_speeds=None,
                          yaw_rate=None, traveled=0.0, online=False)

    # ---- 诊断 ----
    def publish_diag(self, du, dv, outcome, pan_us, tilt_us, present, age_s):
        """发一帧诊断。**纯观测** —— 没人订阅也不影响任何行为。

        缺失值一律发 NaN（不是 0）：0 是合法值（比如 du=0 表示正中），
        拿它当"没有"会让看曲线的人误判。
        """
        if self._pub_diag is None:
            return
        nan = float("nan")
        msg = self._diag_cls()
        msg.data = [nan if du is None else float(du),
                    nan if dv is None else float(dv),
                    float(_OUTCOME_CODE.get(outcome, -1)),
                    nan if pan_us is None else float(pan_us),
                    nan if tilt_us is None else float(tilt_us),
                    1.0 if present else 0.0,
                    nan if age_s is None or age_s < 0 else float(age_s)]
        self._pub_diag.publish(msg)

    # ---- 只读查询（给状态行和告警用）----
    def servo_publisher_count(self):
        """`/servo_cmd` 上的发布者数量（**含本进程自己**）。"""
        return self._pub_servo.get_publisher_count()

    def close(self):
        """销毁节点。**不调 `rclpy.shutdown()`** —— 它是进程级的（照 RosBridge）。"""
        try:
            self.node.destroy_node()
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write("销毁 ROS 节点失败：%s\n" % exc)


def build_live(enable_diag=True):
    """建真桥 + CarController（rotator 就是它的 camera_nudge）。"""
    from car.controller import CarController

    bridge = _ServoBridge(cmd_rate_hz=25.0, enable_diag=enable_diag)
    ctrl = CarController(_Cfg(), bridge_factory=lambda **_kw: bridge)
    ctrl.start()                 # 失败不抛（照 CarController 的设计），靠状态行兜
    return bridge, ctrl


def build_dry():
    """dry-run：同一个 CarController，换假桥。**完全不 import ROS。**"""
    from car.controller import CarController

    bridge = _FakeBridge()
    ctrl = CarController(_Cfg(), bridge_factory=lambda **_kw: bridge)
    return bridge, ctrl


# ===========================================================================
# 运行
# ===========================================================================

class Runner:
    """一拍一拍地跑，把真相打印出来。**只读地观察，不介入控制路径。**"""

    def __init__(self, tracker, read_last, last_us, args,
                 bridge=None, live=False):
        self.tr = tracker
        self.read_last = read_last
        self.last_us = last_us
        self.a = args
        self.bridge = bridge
        self.live = live
        self.counts = {oc: 0 for oc in _OUTCOMES}
        self.authority = {}         # 适配层的**权威判定**计数（按 mtime）
        self.other = 0
        self.errors = 0
        self.ticks = 0
        self._last_age = None       # (mtime 年龄, _received_at 年龄)，状态行用
        self.t0 = time.time()
        self.next_status = self.t0 + args.status_every
        self._last_shown = None     # 上一次**打印过**的判定（去重用）
        self.pub_warned = False
        self.age_skew_warned = False

    # ---- 每拍 ----
    def on_result(self, res):
        self.ticks += 1
        if res.outcome in self.counts:
            self.counts[res.outcome] += 1
        else:
            self.other += 1

        now = time.time()
        snap = self.read_last[0]
        if snap is not None:
            self.authority[snap.state] = self.authority.get(snap.state, 0) + 1

        # ---- 权威原因：**由适配层按 mtime 判**，不是模块猜的那句 ----
        # 20Hz 的轮询比板子 6Hz 快，"每拍一行"会刷出三倍的重复行。所以只在
        # **真的发了指令**或者**判定变了**的时候打；其余交给状态行和汇总。
        # `--verbose` 可以强制每拍都打（查"为什么不动"的时候用）。
        interesting = res.moved or res.outcome not in _ROUTINE
        if self.a.verbose or (interesting and res.outcome != self._last_shown):
            self._print_tick(res, snap, now)
        if interesting:
            self._last_shown = res.outcome
        self._publish_diag(res, snap)

        if now >= self.next_status:
            self.next_status = now + self.a.status_every
            self._check_age_skew(snap)          # 先算年龄，状态行要打印它
            self._print_status(now)
            self._check_foreign_publishers()

    def _print_tick(self, res, snap, now):
        """一行说清这一拍：**动了就报两轴指令，没动就报权威原因。**

        ⚠️ 未动那行**权威原因在前、模块自报的结果码在后**。原因是模块的判定
        是拿时钟猜的（它看不到 mtime），单独看会跟权威原因**看起来自相矛盾**
        —— 比如文件明明很新鲜、只是这一帧没有 src=="track"，模块会报 `stale`
        （它只是"最近没拿到过数据"）。把模块那个码降级成脚注，读的人才不会
        被它带偏。**没有去改 gimbal_track.py 的接口。**
        """
        tag = "%6.2fs" % (now - self.t0)
        if res.moved:
            parts = []
            for ax in ("pan", "tilt"):
                st = res.axes.get(ax)
                if st is not None and st.direction is not None:
                    ch = 5 if ax == "pan" else 6
                    parts.append("ch%d %s %.1f° -> %sus"
                                 % (ch, st.direction, st.deg, self.last_us[ax]))
            line = "[%s] 动   | %s | 数据 %s" % (tag, "；".join(parts),
                                                _age_txt(snap))
            if res.outcome == HIT_LIMIT:
                line += " | ⚠️ " + _one_line(res.note)
        else:
            line = "[%s] 未动 | 权威原因：%s | 模块自报 %s" % (
                tag,
                explain(snap) if snap is not None else "（这一拍没读到快照）",
                res.outcome)
            if self.a.verbose or res.outcome in (HIT_LIMIT, BAD_BOX):
                line += " | " + _one_line(res.note or describe(res))
        print(line)
        sys.stdout.flush()

    def _publish_diag(self, res, snap):
        if self.bridge is None or not hasattr(self.bridge, "publish_diag"):
            return
        pan = res.axes.get("pan")
        tilt = res.axes.get("tilt")
        present = bool(snap is not None and snap.ok)
        try:
            self.bridge.publish_diag(
                None if pan is None else pan.err,
                None if tilt is None else tilt.err,
                res.outcome, self.last_us["pan"], self.last_us["tilt"],
                present, None if snap is None else snap.age)
        except Exception as exc:  # noqa: BLE001 —— 诊断绝不能拖垮跟踪
            print("⚠️  /gimbal_track 发布失败（不影响跟踪）：%s" % exc)

    # ---- 周期性 ----
    def _authority_row(self):
        """适配层权威判定的计数，一行。

        跟模块自报的那个**分开列**是刻意的：模块只知道"最近有没有拿到过数据"，
        所以"文件一直很新鲜、只是这一帧没有 src==\"track\""这种最常见的情况它
        会报 `stale` —— 看着像断流，其实一点没断。混成一行就会把工具读错。
        """
        names = {_S_OK: "有目标", _S_NO_TRACK: "无目标（数据新鲜）",
                 _S_STALE: "过期（mtime 旧）", _S_MISSING: "文件不存在",
                 _S_UNREADABLE: "读不动", _S_NO_FRAME: "没有帧号"}
        order = (_S_OK, _S_NO_TRACK, _S_STALE, _S_MISSING, _S_UNREADABLE,
                 _S_NO_FRAME)
        parts = ["%s %d" % (names[st], self.authority.get(st, 0))
                 for st in order if self.authority.get(st)]
        return " | ".join(parts) if parts else "（还没有样本）"

    def _print_status(self, now):
        c = self.counts
        pan = self.last_us["pan"]
        tilt = self.last_us["tilt"]
        print("-" * 72)
        print("状态 | 已跑 %.1fs / %d 拍 | 数据年龄 %s" % (
            now - self.t0, self.ticks,
            "n/a" if not self._last_age else _fmt_age(self._last_age)))
        # ⚠️ 这两行**不是一回事**：上面是模块自报的判定，下面是适配层按 mtime
        # 判的权威结果。数据一直很新鲜、只是这一帧没有 src=="track" 时，模块会
        # 报 stale（它只能拿时钟猜），上面那行就会显示成"过期" —— **调参要看下面这行。**
        print("   模块自报 | 跟 %-4d 保持 %-4d 无目标 %-4d 无新帧 %-4d "
              "过期 %-4d 撞限位 %-4d 坏框 %-4d%s"
              % (c[TRACKED], c[HOLD], c[NO_TARGET], c[NO_NEW_FRAME],
                 c[STALE], c[HIT_LIMIT], c[BAD_BOX],
                 (" 其它 %d" % self.other) if self.other else ""))
        print("   适配层判定(权威) | %s" % self._authority_row())
        print("   当前 | ch5(水平) %s  ch6(俯仰) %s   ← **本进程自己的记账**，"
              "舵机没有位置反馈，读不回来" % (
                  "n/a" if pan is None else "%dus" % pan,
                  "n/a" if tilt is None else "%dus" % tilt))
        if self.bridge is not None and hasattr(self.bridge, "servo_frames_sent"):
            print("   已发 %d 帧 servo_cmd" % self.bridge.servo_frames_sent)
        print("-" * 72)
        sys.stdout.flush()

    def _check_foreign_publishers(self):
        """`/servo_cmd` 上除自己以外还有发布者 → 醒目告警。

        **★ 只告警，不去抢回来。** 见文件头"外来发布者"那一节：互相抢写会让
        两边各自维护的绝对脉宽假设一起烂掉，而且看不出来。这条是刻意的。
        """
        if self.bridge is None or not hasattr(self.bridge, "servo_publisher_count"):
            return
        if time.time() - self.t0 < PUB_CHECK_AFTER_S:
            return          # DDS 发现还没完成，这时数出来必然只有自己
        try:
            n = self.bridge.servo_publisher_count()
        except Exception:  # noqa: BLE001
            return
        if n > 1 and not self.pub_warned:
            self.pub_warned = True
            print("!" * 72)
            print("⚠️  /servo_cmd 上有 %d 个发布者（含本进程）—— 除了我还有别人在发。" % n)
            print("    云台是「最后一条消息赢」：没有优先级、没有看门狗、没有仲裁。")
            print("    从这一刻起**谁最后发谁生效**，我随时可能被顶掉；被顶掉的表现")
            print("    是云台自己往别处跑（本进程不会知道我发的被覆盖了）。")
            print("    **本脚本刻意不抢回来**（不去重复发/提频率/加看门狗）：互相抢写")
            print("    会让两边各自维护的绝对脉宽假设一起烂掉，而且从任何一边都")
            print("    看不出来。要独占，请先把语音助手那边的云台路径停下来。")
            print("!" * 72)
            sys.stdout.flush()

    def _check_age_skew(self, snap):
        """mtime 年龄和 `_received_at` 年龄差得远 → 说出来。

        这是那个坑**正在发生**的信号：`_received_at` 看着新鲜、文件其实很旧
        （或反过来）。本脚本一律按 mtime 判，但让操作员看见两个数不一样很重要。
        """
        if snap is None or self.age_skew_warned:
            return
        other = received_at_age_s(self.a.result)
        if other is None or snap.age < 0:
            return
        self._last_age = (snap.age, other)
        if abs(other - snap.age) > 2.0:
            self.age_skew_warned = True
            print("⚠️  mtime 年龄 %.2fs 和 _received_at 年龄 %.2fs **差了 %.2fs** —— "
                  "判新鲜度按 mtime（权威）；_received_at 只作对照。"
                  % (snap.age, other, abs(other - snap.age)))
            sys.stdout.flush()

    # ---- 汇总 ----
    def summary(self, reason="中止"):
        c = self.counts
        span = time.time() - self.t0
        print()
        print("=" * 72)
        print("汇总（%s）：跑了 %.1fs，共 %d 拍" % (reason, span, self.ticks))
        print()
        print("模块自报的判定（GimbalTracker.outcome，**只是参考** —— 它看不到 mtime）:")
        for oc in _OUTCOMES:
            if c[oc]:
                pct = 100.0 * c[oc] / max(self.ticks, 1)
                print("   %-12s %6d  (%5.1f%%)" % (oc, c[oc], pct))
        if self.other:
            print("   %-12s %6d" % ("其它", self.other))
        print()
        print("适配层的判定（按 mtime，**权威** —— 调参看这个）:")
        print("   " + self._authority_row())
        if self.errors:
            print("   云台/ROS 抛异常 %d 次（见上面日志）" % self.errors)
        print("云台最后位置（**本进程自己的记账，不是读回来的**）："
              "ch5(水平) %s  ch6(俯仰) %s" % (
                  "n/a" if self.last_us["pan"] is None else "%dus" % self.last_us["pan"],
                  "n/a" if self.last_us["tilt"] is None else "%dus" % self.last_us["tilt"]))
        if self.live:
            print("退出后云台**保持最后位置**（固件设计如此，没有看门狗、不会回中）。")
        print("=" * 72)


# 这两个判定**每拍都会出现**（板子 6Hz vs 轮询 20Hz），不算"有情况"。
# 其它判定一出现就说明有事情发生，值得打一行。
_ROUTINE = (NO_NEW_FRAME, HOLD)


def _age_txt(snap):
    """数据年龄的一小段文字（拿不到时 n/a）。"""
    if snap is None or snap.age < 0:
        return "n/a"
    return "%.2fs" % snap.age


def _one_line(text):
    """把 describe() 的多句压成一句 —— 每拍一行，别刷屏。"""
    return " ".join(str(text).split())


def _fmt_age(pair):
    return "%.2fs(mtime) / %.2fs(_received_at)" % pair


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="云台两轴持续跟踪（默认 dry-run，真正驱动要 --live）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true",
                   help="**真正驱动云台**（会动！）。不给就是 dry-run：不 import "
                        "ROS、不发任何消息、只把算出来的每一步打出来。")
    p.add_argument("--dry-run", action="store_true",
                   help="不连 ROS、不发消息，只把算出来的每一步打出来"
                        "（**这就是默认行为**；收下这个参数只是为了让习惯显式"
                        "写出来的人不报错，它不做任何额外的事）")
    p.add_argument("--result", default=RESULT_PATH,
                   help="反馈源 JSON，默认 %s" % RESULT_PATH)
    p.add_argument("--max-age", type=float, default=MAX_AGE_S,
                   help="适配层判「数据新鲜」的 mtime 阈值秒数，默认 %g。"
                        "板子 6Hz，%g = 连丢 3 帧。" % (MAX_AGE_S, MAX_AGE_S))
    p.add_argument("--stale-s", type=float, default=None,
                   help="GimbalTracker 自己的 stale_s（模块内部拿时钟猜的兜底，"
                        "默认走模块的 1.0）。**权威判定是 --max-age**，这个只在"
                        "模块内部用。")
    p.add_argument("--poll", type=float, default=0.05,
                   help="轮询周期秒，默认 0.05（< 板子 0.167s，不会漏帧）")
    p.add_argument("--status-every", type=float, default=STATUS_EVERY_S,
                   help="状态行周期秒，默认 %g" % STATUS_EVERY_S)
    p.add_argument("--duration", type=float, default=0.0,
                   help="跑多少秒自动停，默认 0 = 一直跑（Ctrl-C 退）")
    p.add_argument("--verbose", action="store_true",
                   help="每拍都打一行（默认只在有动作或判定变化时打，"
                        "否则 20Hz 轮询会刷出三倍重复行）")
    p.add_argument("--no-diag", action="store_true",
                   help="不发 /gimbal_track 诊断话题")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)

    # ---- 建控制器（rotator 就是 CarController.camera_nudge）----
    if args.live:
        print("=" * 72)
        print("*** --live：**真的会驱动云台**（ch5 水平 / ch6 俯仰）。"
              "底盘电机一概不碰。***")
        print("=" * 72)
        bridge, ctrl = build_live(enable_diag=not args.no_diag)
    else:
        print("=" * 72)
        print("*** dry-run（默认）：不 import ROS、不发任何消息、只打印 ***")
        print("*** 要真正驱动云台，加 --live ***")
        print("=" * 72)
        bridge, ctrl = build_dry()

    print("节点名 %s（**故意不叫 voice_chatbot_car** —— 语音助手在用那个名字）"
          % getattr(bridge, "node_name", "（dry-run 没有 ROS 节点）"))
    print("反馈源 %s   新鲜阈值 %.2fs(mtime)   轮询 %.2fs"
          % (args.result, args.max_age, args.poll))

    # ---- 适配层 + 控制器 ----
    read_target, read_last = make_read_target(args.result, args.max_age)
    rotator, last_us = make_rotator(ctrl)

    cfg_kw = {"poll_s": args.poll}
    if args.stale_s is not None:
        cfg_kw["stale_s"] = args.stale_s
    tracker = GimbalTracker(rotator, read_target, TrackConfig(**cfg_kw))

    runner = Runner(tracker, read_last, last_us, args,
                    bridge=bridge if args.live else None, live=args.live)

    print("开始跟。Ctrl-C 干净退出并打印汇总。")
    sys.stdout.flush()

    deadline = (time.time() + args.duration) if args.duration > 0 else None
    reason = "跑满 --duration"
    try:
        while True:
            if deadline is not None and time.time() >= deadline:
                break
            try:
                res = tracker.update()
            except Exception as exc:  # noqa: BLE001
                # 云台/ROS 出错**不吞**：计数、打印、继续 —— 一次瞬时错误
                # 不该把整条跟踪打断。连着错太多就退出（见下）。
                runner.errors += 1
                print("⚠️  第 %d 拍抛异常：%r" % (tracker.ticks, exc))
                sys.stdout.flush()
                if runner.errors >= 20:
                    reason = "连续异常过多，主动退出"
                    break
                time.sleep(args.poll)
                continue
            runner.on_result(res)
            time.sleep(args.poll)
    except KeyboardInterrupt:
        reason = "Ctrl-C"
    finally:
        runner.summary(reason)
        if args.live:
            # **不调 ctrl.close()** —— 那会走 stop_all() → publish_velocity()。
            # 本脚本不碰底盘是硬边界；直接关桥就够了。
            try:
                bridge.close()
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write("关桥失败：%s\n" % exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
