"""小车控制器 —— 编排层：闭环运动、云台、急停、状态播报。

## 与语音助手的关系

照抄 tools/music.py 里 MusicController 的已验证模式：

    用户喊唤醒词 → ConversationManager._on_wake_word()
      → self._music.stop()        # 音乐先停
      → self._car.stop_all()      # 小车急停（同构）
      → 才 transition(LISTENING)

唤醒词路径**不经过 ASR / LLM**，几十毫秒就能停住。这是急停。
正常对话里说「停」走 car_stop 工具，要一次 LLM 往返 —— 那是「停一下」，
不是急停。

## 三层里的中间一层

    Tool（car_move 等 5 个）→ CarController ← 未来的 nav_goto / slam_* 也调这里
                                  ↓
                              RosBridge

`set_velocity` 是**给程序用的原语，不是给模型的工具** —— 语音带宽太低，
用户说不出「设 vx 为 0.3」。
"""

from __future__ import annotations

import math
import threading
import time

from loguru import logger

from car.kinematics import (
    camera_target_us,
    clamp_speed,
    clamp_turn_speed,
    direction_names,
    direction_to_velocity,
)
from car.ros_bridge import RosBridge
from car.types import CarLimits, describe_faults

# 云台通道。记忆 l150pro-car 明确写过：ch5 = 水平、ch6 = 俯仰，别搞反。
_PAN_CHANNEL = 5
_TILT_CHANNEL = 6

# 急停连发几帧零速。teleop 的 stop_now() 用 8 帧 / 0.3 秒，照抄。
_STOP_FRAMES = 8
_STOP_FRAME_INTERVAL = 0.04


class CarController:
    """小车的对外接口。线程安全。"""

    def __init__(
        self,
        config,
        *,
        bridge_factory=RosBridge,
        clock=time.monotonic,
        sleeper=time.sleep,
    ) -> None:
        self._config = config
        self._limits = CarLimits.from_config(config)
        self._bridge_factory = bridge_factory
        self._clock = clock
        self._sleep = sleeper

        self._bridge = None
        self._bridge_lock = threading.Lock()   # 保护 _bridge 的懒建
        self._stop_event = threading.Event()   # 置位 = 立刻中止当前运动
        self._motion_thread: threading.Thread | None = None
        self._moving = False
        self._last_outcome = ""                # 上一次运动的结局，car_status 会带上

        # 云台位置。**只在某一路真的被指令过之后才记录**：发帧时没记录的路
        # 一律发 0（固件约定 0 = 保持不动）。若这里预填中位，那么「往右看」
        # 会顺手把俯仰轴也命令到 1500 —— 用户没让它动，等于凭空改了车的姿态。
        self._camera_us: dict[int, int] = {}

    # ------------------------------------------------------------ 生命周期

    def start(self) -> None:
        """提前建立 ROS 连接，让环境问题**在启动日志里就暴露**。

        失败不抛：小车起不来不该拖垮整个语音助手（天气、音乐、闲聊都还得能用）。
        工具调用时会再试一次，所以「先起助手、后起驱动」也能自己恢复。
        """
        try:
            self._bridge_or_raise()
        except Exception as exc:  # noqa: BLE001
            logger.error(f"小车：暂时不可用 —— {exc}")

    def close(self) -> None:
        self.stop_all()
        self.wait_idle(timeout=3.0)
        bridge = self._bridge
        if bridge is not None:
            bridge.close()
            self._bridge = None

    def _bridge_or_raise(self):
        """懒建 ROS 桥。失败抛 RosUnavailable，由工具分发层转成给模型看的错误。"""
        with self._bridge_lock:
            if self._bridge is None:
                self._bridge = self._bridge_factory(
                    cmd_rate_hz=self._limits.cmd_rate_hz
                )
            return self._bridge

    # ------------------------------------------------------------ 原语（给程序用）

    def set_velocity(self, vx: float, wz: float) -> None:
        """发一帧速度。**这是给程序用的原语，不是给模型的工具。**

        未来的 nav_goto / slam_* 直接调它，不需要新增模型可见的工具。
        """
        self._bridge_or_raise().publish_velocity(vx, wz)

    def stop_all(self) -> None:
        """急停。**唤醒词路径会直接调它 —— 这条路径上绝不抛异常。**"""
        self._stop_event.set()
        bridge = self._bridge
        if bridge is None:
            return
        try:
            for _ in range(_STOP_FRAMES):
                bridge.publish_velocity(0.0, 0.0)
                self._sleep(_STOP_FRAME_INTERVAL)
        except Exception as exc:  # noqa: BLE001
            logger.error(f"小车：急停发零速失败 —— {exc}")

    # ------------------------------------------------------------ 运动（工具用）

    # ★ 运动**必须跑在独立线程上**，工具只负责「启动」然后立刻返回。
    #
    # 理由与 tools/music.py 把播放放独立线程完全一样，而后果更严重：
    # 唤醒模块的串口监听和整条对话流水线**共用一个线程** —— 见
    # wakeword/engine.py 的 `_listen_loop → _handle_detection → on_detected`，
    # 那个函数自己的注释就写着「回调里同步跑完了整条流水线」。
    #
    # 所以一旦在工具里阻塞着等车走完，串口就没人读，用户喊唤醒词进不来，
    # **急停失效**。这不是推断 —— 2026-09-17 实车复现：「他在往前走的时候，
    # 喊他没用」。
    #
    # 停下来有三条路，都不经过这个函数：
    #   1. 唤醒词 → ConversationManager._on_wake_word → self._car.stop_all()
    #   2. car_stop 工具 → stop_and_describe()
    #   3. 新指令 → _spawn() 里的 _preempt()（新指令顶掉旧的）

    def move_distance(
        self, direction: str, distance: float, speed: float | None = None
    ) -> str:
        """启动前进/后退，**立刻返回**。走完自动停，也可被急停或新指令打断。"""
        if direction not in direction_names():
            return (
                f"不认识方向 {direction}。可用的有：{'、'.join(direction_names())}。"
                "原地转向请用 car_turn。"
            )
        if distance <= 0:
            return "距离得是正数。"

        requested = self._limits.default_speed if speed is None else speed
        magnitude, clamped = clamp_speed(requested, self._limits)
        vx, wz = direction_to_velocity(
            direction, magnitude, self._limits.default_turn_speed_dps
        )

        bridge = self._bridge_or_raise()
        start = bridge.state().traveled
        timeout = (distance / max(magnitude, 0.01)) * 3.0 + 5.0
        verb = "前进" if vx > 0 else "后退"

        self._spawn(
            vx=vx, wz=wz, timeout=timeout,
            done=lambda: abs(bridge.state().traveled - start) >= distance,
            label=f"{verb} {distance:g} 米",
        )
        note = f"（速度已按上限截到 {magnitude:.2f} 米每秒）" if clamped else ""
        return f"已开始{verb}约 {distance:g} 米，走完自动停。{note}"

    def turn_by(self, angle_deg: float, speed: float | None = None) -> str:
        """启动原地转向，**立刻返回**。正 = 逆时针 = 左转（REP-103）。"""
        requested = self._limits.default_turn_speed_dps if speed is None else speed
        dps, clamped = clamp_turn_speed(requested, self._limits)
        wz = (1.0 if angle_deg >= 0 else -1.0) * dps * math.pi / 180.0

        bridge = self._bridge_or_raise()
        # ★ 全程在【弧度】域累积。`yaw_rate` 是 rad/s，乘秒得弧度 ——
        # 拿它直接跟「度」比会永远到不了目标（转 90 度要积 90 弧度
        # = 5156 度），表现是每次都跑满超时，然后报一个假的
        # 「陀螺仪一直读到 0」故障。只有出口转回度给用户看。
        target_rad = abs(angle_deg) * math.pi / 180.0
        turned_rad = 0.0
        last = self._clock()

        def _done() -> bool:
            nonlocal turned_rad, last
            now = self._clock()
            state = bridge.state()
            if state.yaw_rate is not None:
                turned_rad += abs(state.yaw_rate) * (now - last)
            last = now
            return turned_rad >= target_rad

        timeout = (abs(angle_deg) / max(dps, 1.0)) * 3.0 + 5.0
        direction_word = "左转" if angle_deg >= 0 else "右转"

        self._spawn(
            vx=0.0, wz=wz, timeout=timeout, done=_done,
            label=f"{direction_word} {abs(angle_deg):g} 度",
        )
        note = f"（角速度已按上限截到 {dps:.0f} 度每秒）" if clamped else ""
        return f"已开始{direction_word}约 {abs(angle_deg):g} 度，转完自动停。{note}"

    def stop_and_describe(self) -> str:
        """car_stop 工具用：停下并回报一句。"""
        was_moving = self._moving
        self.stop_all()
        return "好，停了。" if was_moving else "车本来就是停着的。"

    # ------------------------------------------------------------ 云台（工具用）

    def camera_nudge(self, direction: str, degrees: float | None = None):
        """转动云台，返回 (目标脉宽 us, 是否撞行程限位)。

        **结构化版本，不暴露给模型**（同 set_velocity 的约定）——
        给控制回路用：`camera_move` 返回的是给人念的中文串，
        `hit_limit` 埋在文字里，回路没法拿它做判断（2026-09-19 云台居中闭环要用）。

        Raises:
            KeyError: direction 不是 left/right/up/down/center
        """
        channel = _TILT_CHANNEL if direction in ("up", "down") else _PAN_CHANNEL
        # 没指令过就按中位推算（绝对脉宽伺服本来就得有个假设起点，
        # teleop 同样假定上电即中位），但**不会**因此去命令那一路。
        current = self._camera_us.get(channel, self._limits.camera_center_us)
        target, hit_limit = camera_target_us(direction, degrees, current, self._limits)
        self._camera_us[channel] = target
        self._bridge_or_raise().publish_servo(self._servo_frame())
        return target, hit_limit

    def camera_move(self, direction: str, degrees: float | None = None) -> str:
        """转动云台。direction 取 left/right/up/down/center。"""
        try:
            target, hit_limit = self.camera_nudge(direction, degrees)
        except KeyError:
            return (
                f"不认识云台方向 {direction}。"
                "可用：left（往左）、right（往右）、up（抬头）、down（低头）、center（回正）。"
            )

        label = {"left": "往左", "right": "往右", "up": "往上",
                 "down": "往下", "center": "回到中位"}[direction]
        note = "，已经到头了" if hit_limit else ""
        axis = "俯仰" if direction in ("up", "down") else "水平"
        return f"云台{label}了（{axis}轴 {target} 微秒）{note}。"

    def _servo_frame(self) -> list[int]:
        """0 = 该路保持不动 —— 这是不碰底盘上其它舵机的办法（固件约定）。"""
        frame = [0] * 6
        for channel, us in self._camera_us.items():
            frame[channel - 1] = us
        return frame

    # ------------------------------------------------------------ 状态（工具用）

    def status_text(self) -> str:
        state = self._bridge_or_raise().state()
        if not state.online:
            return (
                "还读不到小车的状态 —— 底盘驱动可能没在跑，"
                "或者在跑但还没有数据上来。"
            )
        parts: list[str] = []
        if state.voltage is not None:
            parts.append(f"电池 {state.voltage:.1f} 伏")
        parts.append(f"故障码 {describe_faults(state.fault)}")
        if state.wheel_speeds is not None:
            wheels = "、".join(f"{v:+.2f}" for v in state.wheel_speeds)
            parts.append(f"四轮速度 {wheels} 米每秒")
        parts.append("正在移动" if self._moving else "停着")
        if self._last_outcome:
            parts.append(f"上一次动作：{self._last_outcome}")
        return "，".join(parts) + "。"

    # ------------------------------------------------------------ 内部

    def _preempt(self) -> None:
        """停掉正在跑的运动，为新指令让路。

        新指令顶掉旧指令，而不是回一句「正忙」—— 语音场景下用户改主意是
        常态，回「正忙」只会让人再喊一次。
        """
        thread = self._motion_thread
        if thread is None or not thread.is_alive():
            self._motion_thread = None
            return
        self._stop_event.set()
        thread.join(timeout=2.0)
        self._motion_thread = None

    def _spawn(self, *, vx: float, wz: float, done, timeout: float, label: str) -> None:
        """起一个后台运动线程。**调用方（工具）立刻返回，不等它。**"""
        self._preempt()
        self._stop_event.clear()
        self._motion_thread = threading.Thread(
            target=self._motion_loop,
            args=(vx, wz, done, timeout, label),
            name="car-motion",
            daemon=True,
        )
        self._motion_thread.start()

    def _motion_loop(self, vx: float, wz: float, done, timeout: float, label: str) -> None:
        """后台按 cmd_rate 持续发帧，直到到位 / 超时 / 被急停。"""
        bridge = self._bridge
        if bridge is None:
            return
        period = 1.0 / self._limits.cmd_rate_hz
        deadline = self._clock() + timeout
        self._moving = True
        try:
            while True:
                if self._stop_event.is_set():
                    self._last_outcome = f"{label} 被打断"
                    return
                bridge.publish_velocity(vx, wz)
                if done():
                    self._last_outcome = f"{label} 完成"
                    return
                if self._clock() >= deadline:
                    self._last_outcome = f"{label} 超时（发了 {timeout:.0f} 秒没到位）"
                    logger.warning(
                        f"小车：{label} 超时 —— 底盘驱动没在跑，"
                        "或者里程计 / 陀螺仪没数据上来"
                    )
                    return
                self._sleep(period)
        except Exception as exc:  # noqa: BLE001
            self._last_outcome = f"{label} 出错：{exc}"
            logger.error(f"小车：{self._last_outcome}")
        finally:
            self._moving = False
            self._finish_motion()

    def wait_idle(self, timeout: float = 10.0) -> bool:
        """等当前运动结束。给 close() 和测试用 —— **不要**在唤醒线程上调。"""
        thread = self._motion_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
            return not thread.is_alive()
        return True

    @property
    def last_outcome(self) -> str:
        """上一次运动的结局（完成 / 被打断 / 超时）。car_status 会带上它。"""
        return self._last_outcome

    def _finish_motion(self) -> None:
        """收尾归零，否则车会一直跑。

        **不清 `_stop_event`** —— 那是「谁开下一段运动谁清」的事（见 _spawn）。
        在这里清会让一次刚收到的急停被正在退出的线程悄悄撤销掉。
        """
        bridge = self._bridge
        if bridge is None:
            return
        try:
            for _ in range(3):
                bridge.publish_velocity(0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            logger.error(f"小车：收尾归零失败 —— {exc}")
