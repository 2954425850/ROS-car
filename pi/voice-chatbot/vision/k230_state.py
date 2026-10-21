"""K230 实时结果（`/k230/result`）在语音助手里的只读视图。

`look` 是「拍一张、上云问」，几秒一次；本模块是**常驻订阅**板子自己的检测结果
（YOLO 物体 + 人脸身份），随时可查、零成本。两个用途：

1. **谁在跟我说话**（身份 v1）：唤醒那一刻往回看几秒的人脸，挑出"面前那个人"。
2. **眼前有什么**（scene 工具）：直接报检测器看到的物体，不用每次都上云。

## 结构
`FaceWindow` 是纯逻辑（喂记录、按时间窗查），离线可测；`K230State` 才碰 ROS ——
自己建节点 + 自己的执行器 + daemon 线程（同 tof_client 的做法，不碰全局执行器）。
**ROS 起不来只降级**：查询全部返回空，助手照常说话。

## 时间
记录里的 `stamp_mono` 是 k230_bridge 换算好的**拍摄时刻**（Pi monotonic 秒），
没有就退到本进程收到的时刻。窗口都按它算。
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from dataclasses import dataclass

from loguru import logger


@dataclass
class FaceSighting:
    t: float              # 拍摄时刻（monotonic 秒）
    who: str | None       # 板子给的身份 id（ASCII）；None = 库里没有 / 不可信
    match: float | None   # 相似度（板子尺度 0~1）
    live: float | None    # 活体分
    ok: bool              # 质量门：False = 脸不完整/太靠边，身份不可信
    area: float           # 归一化框面积（挑"面前那个人"用）
    box: tuple[float, float, float, float]
    n_faces: int          # 同一帧里一共几张脸


@dataclass
class SceneObject:
    cls: str
    score: float
    box: tuple[float, float, float, float]
    src: str              # 'det' | 'track'


def _box(b) -> tuple[float, float, float, float] | None:
    try:
        l, t, r, bb = (float(v) for v in b)
    except (TypeError, ValueError):
        return None
    return (l, t, r, bb)


class FaceWindow:
    """最近 `keep_s` 秒的人脸与物体（纯逻辑）。"""

    def __init__(self, keep_s: float = 10.0):
        self.keep_s = keep_s
        self._faces: deque[FaceSighting] = deque()
        self._objs: list[SceneObject] = []
        self._objs_t: float | None = None
        self.last_t: float | None = None
        self._lock = threading.Lock()

    def feed(self, rec: dict, now: float) -> None:
        t = rec.get("stamp_mono")
        t = float(t) if isinstance(t, (int, float)) else now
        faces = rec.get("faces") or []
        objs = []
        for o in rec.get("objs") or []:
            b = _box(o.get("box"))
            if b is None:
                continue
            objs.append(SceneObject(str(o.get("cls", "")), float(o.get("score") or 0.0), b,
                                    "track" if o.get("src") == "track" else "det"))
        with self._lock:
            self.last_t = t
            self._objs, self._objs_t = objs, t
            for f in faces:
                b = _box(f.get("box"))
                if b is None:
                    continue
                self._faces.append(FaceSighting(
                    t=t, who=f.get("who") or None,
                    match=_num(f.get("match")), live=_num(f.get("live")),
                    ok=bool(f.get("ok")),
                    area=max(0.0, (b[2] - b[0]) * (b[3] - b[1])), box=b,
                    n_faces=len(faces)))
            while self._faces and t - self._faces[0].t > self.keep_s:
                self._faces.popleft()

    def faces_since(self, t0: float) -> list[FaceSighting]:
        with self._lock:
            return [f for f in self._faces if f.t >= t0]

    def objects(self, now: float, max_age: float = 1.5) -> list[SceneObject]:
        with self._lock:
            if self._objs_t is None or now - self._objs_t > max_age:
                return []
            return list(self._objs)


def _num(v):
    return float(v) if isinstance(v, (int, float)) else None


class K230State:
    """订阅 `/k230/result`，维护一个 FaceWindow。"""

    def __init__(self, topic: str = "/k230/result"):
        self.topic = topic
        self.window = FaceWindow()
        self.enabled = False
        self.reason = "还没 start()"
        self._node = None
        self._executor = None
        self._thread = None

    def start(self) -> bool:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError as e:
            self.reason = f"没有 rclpy（{e}）"
            logger.warning(f"K230 实时结果：{self.reason}，身份/场景功能降级")
            return False
        try:
            try:
                rclpy.init(args=[])
            except Exception:  # noqa: BLE001 —— 小车桥可能已经 init 过
                pass
            self._node = Node("voice_chatbot_k230")
            self._node.create_subscription(String, self.topic, self._on_msg, 10)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._executor.spin, name="k230-state-spin", daemon=True)
            self._thread.start()
        except Exception as e:  # noqa: BLE001
            self.reason = f"建订阅失败：{e!r}"
            logger.error(f"K230 实时结果：{self.reason}")
            return False
        self.enabled = True
        self.reason = "ok"
        logger.info(f"K230 实时结果：已订阅 {self.topic}")
        return True

    def _on_msg(self, msg) -> None:
        try:
            self.window.feed(json.loads(msg.data), time.monotonic())
        except Exception as e:  # noqa: BLE001
            logger.debug(f"K230 实时结果：坏记录 {e}")

    def fresh(self, now: float | None = None, max_age: float = 3.0) -> bool:
        now = time.monotonic() if now is None else now
        t = self.window.last_t
        return t is not None and now - t <= max_age

    def close(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=0.5)
            except Exception:  # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
