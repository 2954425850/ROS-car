"""循迹服务客户端：调 /line_follow/enable（std_srvs/SetBool），给 tools/line_follow.py 用。

照 `car/arm_client.py` 的做法：自建节点 + 自己的 SingleThreadedExecutor + daemon 线程，
不碰全局执行器。循迹节点（line-follow.service）不在只降级：`start()` 失败则
enabled=False，调用时回一句原因。

**为什么是服务而不是由语音去起进程**：语音助手不该管进程生命周期（环境、日志、PID、
崩溃重启）。循迹节点常驻待机，服务只切换"跑/不跑"；删掉这个工具不会留下任何
进程管理代码。
"""

from __future__ import annotations

import threading
import time

from loguru import logger


class LineFollowClient:

    def __init__(self, timeout: float = 3.0):
        self.timeout = timeout
        self.enabled = False
        self.reason = "还没 start()"
        self._node = None
        self._cli = None
        self._executor = None
        self._thread = None
        self._SetBool = None

    # ---- 生命周期 ----

    def start(self) -> bool:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_srvs.srv import SetBool
        except ImportError as e:
            self.reason = f"缺依赖（{e}）"
            logger.warning(f"循迹客户端：{self.reason}，循迹工具降级")
            return False
        try:
            try:
                rclpy.init(args=[])
            except Exception:                 # noqa: BLE001 —— 小车桥可能已经 init
                pass
            self._SetBool = SetBool
            self._node = Node("voice_line_follow")
            self._cli = self._node.create_client(SetBool, "/line_follow/enable")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin,
                                            name="line-follow-client-spin", daemon=True)
            self._thread.start()
        except Exception as e:                # noqa: BLE001
            self.reason = f"起客户端失败：{e!r}"
            logger.error(f"循迹客户端：{self.reason}")
            return False
        self.enabled = True
        self.reason = "ok"
        logger.info("循迹客户端：已就绪（/line_follow/enable）")
        return True

    def close(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=0.5)
            except Exception:                 # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:                 # noqa: BLE001
                pass

    # ---- 对外 ----

    def unavailable_text(self) -> str:
        return f"循迹现在用不了 —— {self.reason}"

    def set_enabled(self, enable: bool) -> str:
        """发一次 开始/停止，阻塞到回话（最多 timeout 秒）。返回一句给模型念的话。不抛。"""
        if not self.enabled or self._cli is None:
            return self.unavailable_text()
        try:
            if not self._cli.wait_for_service(timeout_sec=2.0):
                return ("循迹节点没在跑（line-follow 服务不在）。"
                        "先在 Pi 上起 line-follow.service 再来。")
            req = self._SetBool.Request()
            req.data = bool(enable)
            fut = self._cli.call_async(req)
            end = time.monotonic() + self.timeout
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.02)
            if not fut.done():
                return f"循迹服务 {self.timeout:.0f}s 没回话"
            r = fut.result()
            msg = str(r.message or "").strip()
            if r.success:
                return msg or ("已开始循迹" if enable else "已停止循迹")
            return "没做成：" + (msg or "循迹服务拒绝了")
        except Exception as e:                # noqa: BLE001
            return f"调循迹服务出错：{e!r}"
