"""抓取客户端：调 /arm/grasp（raspot_interfaces/action/Grasp），给语音工具用。

自建节点 + 自己的 executor + daemon 线程（同 vision/k230_state 的做法，不碰全局执行器）。
**ROS / raspot_interfaces / 抓取服务不在都只降级**：`start()` 失败则 enabled=False，
调用时回一句"抓取不可用"。

## 阻塞与取消
`grasp()` 同步阻塞到出结果（抓取全程 aim→descend→close→lift→home，~30~60s）。
这期间 executor 线程在 spin。要取消：唤醒词急停（car.stop_all）不停臂 —— 机械臂的
安全由抓取服务自己兜（/estop、手柄接管、action cancel）。这里只留个 cancel() 备用。
"""

from __future__ import annotations

import threading
import time

from loguru import logger

# 错误码 → 给模型念的人话
_CODE_CN = {
    'OK': '抓好了',
    'BAD_GOAL': '参数不对',
    'BUSY': '机械臂正在忙，等它忙完再说',
    'LOCK_FAILED': '没锁住那个目标',
    'NO_FEEDBACK': '读不到机械臂状态',
    'NO_TARGET': '没找到那个目标（可能跟丢了）',
    'NO_DEPTH': '测不到目标的距离（ToF 没起，或目标太近）',
    'OUT_OF_REACH': '那个东西太远，臂够不着',
    'REFUSED': '这个位置臂摆不出抓取姿态',
    'TIMEOUT': '抓取超时了',
    'CLOSE_TIMEOUT': '合爪超时了',
    'STUCK': '臂好像卡住了',
    'OBS_LOST': '目标跟丢了',
    'CANCELED': '抓取被取消了',
    'ABORTED_BY_OPERATOR': '你动了手柄，抓取让给你了',
    'ESTOP': '急停了',
    'ERROR': '抓取出错了',
}


class GraspClient:

    def __init__(self, goal_timeout: float = 120.0):
        self.goal_timeout = goal_timeout
        self.enabled = False
        self.reason = "还没 start()"
        self._node = None
        self._client = None
        self._executor = None
        self._thread = None
        self._lock = threading.Lock()
        self._gh = None                # 当前 goal handle（cancel 用）

    def start(self) -> bool:
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from raspot_interfaces.action import Grasp
        except ImportError as e:
            self.reason = f"缺依赖（{e}）"
            logger.warning(f"抓取客户端：{self.reason}，抓取工具降级")
            return False
        try:
            try:
                rclpy.init(args=[])
            except Exception:          # noqa: BLE001 —— 小车桥/别的节点可能已 init
                pass
            self._node = Node("voice_grasp")
            self._client = ActionClient(self._node, Grasp, "/arm/grasp")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin,
                                            name="grasp-client-spin", daemon=True)
            self._thread.start()
        except Exception as e:         # noqa: BLE001
            self.reason = f"起客户端失败：{e!r}"
            logger.error(f"抓取客户端：{self.reason}")
            return False
        self.enabled = True
        self.reason = "ok"
        logger.info("抓取客户端：已就绪（/arm/grasp）")
        return True

    def grasp(self, box, alpha0: float = 0.0, timeout: float | None = None) -> str:
        """发一次抓取，阻塞到出结果。返回一句给模型念的话。"""
        if not self.enabled:
            return f"抓取现在不可用 —— {self.reason}"
        from raspot_interfaces.action import Grasp
        goal = Grasp.Goal()
        goal.box = [float(v) for v in box]
        goal.alpha0 = float(alpha0 or 0.0)
        goal.dry_run = False
        timeout = self.goal_timeout if timeout is None else timeout
        code, reason = self._send_and_wait(goal, timeout)
        logger.info(f"抓取：{code} {reason}")
        return _CODE_CN.get(code, reason or code)

    def cancel(self) -> None:
        with self._lock:
            gh = self._gh
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:          # noqa: BLE001
                pass

    def _send_and_wait(self, goal, timeout: float):
        if not self._client.wait_for_server(timeout_sec=2.0):
            return 'ERROR', '抓取服务 /arm/grasp 不在（arm-grasp 服务没跑？）'
        send_fut = self._client.send_goal_async(goal)
        end = time.monotonic() + 10.0
        while not send_fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not send_fut.done():
            return 'ERROR', '抓取 goal 一直没被接受'
        gh = send_fut.result()
        if not gh.accepted:
            return 'BUSY', '机械臂正在忙'
        with self._lock:
            self._gh = gh
        try:
            res_fut = gh.get_result_async()
            end = time.monotonic() + timeout
            while not res_fut.done() and time.monotonic() < end:
                time.sleep(0.05)
            if not res_fut.done():
                gh.cancel_goal_async()
                return 'CANCELED', '抓取太久没结束，取消了'
            res = res_fut.result().result
            return res.code, res.reason
        finally:
            with self._lock:
                self._gh = None

    def close(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=0.5)
            except Exception:          # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:          # noqa: BLE001
                pass
