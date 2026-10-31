"""机械臂语音动作客户端：调 /arm/motion（raspot_interfaces/action/ArmMotion）、
/arm/describe、/arm/go_home（std_srvs/Trigger），给 tools/arm.py 用。

设计 `docs/plans/2026-10-31-arm-voice-motion-design.md` §4.1 §4.2 §9。

照 `grasp_client.py` 的做法：自建节点 + 自己的 SingleThreadedExecutor + daemon 线程，
不碰全局执行器。**ROS / raspot_interfaces / arm-grasp 服务不在都只降级**：`start()` 失败则
enabled=False，调用时回一句原因。

## 返回给模型的话
= 结果人话（`_CODE_CN` 映射 + 服务端 reason）+ 「现在：<服务端回读的状态 text>」。
**状态只用服务端真实回读**（Result.state_json / arm/describe 的 message），客户端不编读数 ——
评测发现假读数会让模型以为没做成而重调（设计 §6 末尾的 ⚠️）。

## 阻塞与取消
`act()` 同步阻塞到出结果。唤醒词走 `cancel()`（监听线程上调，**绝不抛**）：
置取消标志 + `cancel_goal_async()`；`act()` 的等待循环看到标志后最多再等 `cancel_grace` 秒
拿服务端的 CANCELED 结果（带冻结后的真实回读），等不到就自己回 CANCELED 文案。
goal 还没被接受时就喊了唤醒词：标志会在接受后立刻补发 cancel（或干脆不发 goal）。
"""

from __future__ import annotations

import json
import threading
import time

from loguru import logger

# 只给前缀、不接服务端 reason 的码（服务端 reason 和前缀说的是同一件事，接上去是重复）
_CODE_CN = {
    'BUSY': '机械臂正在忙（抓取中或在复位），这次没动',
    'ABORTED_BY_OPERATOR': '你动了手柄，交给你了，动作没做完',
    'CANCELED': '动作被打断了，臂停在当前位置',
    'ESTOP': '急停了，动作中止',
}
# 前缀 + 服务端 reason
_CODE_PREFIX = {
    'BAD_GOAL': '没动 —— 动作描述不对',
    'NO_FEEDBACK': '没动 —— 读不到机械臂状态',
    'TIMEOUT': '动作超时了',
    'ERROR': '机械臂动作出错了',
}
_OK_CODES = ('OK', 'PARTIAL')


def state_text(state_json: str) -> str:
    """服务端 state JSON → 其中的 `text`；解析不了/没有就空串（绝不编）。"""
    if not state_json:
        return ''
    try:
        d = json.loads(state_json)
    except (TypeError, ValueError):
        return ''
    t = d.get('text') if isinstance(d, dict) else None
    return t.strip() if isinstance(t, str) else ''


def format_result(code: str, reason: str, state_json: str = '') -> str:
    """一次动作结果 → 给模型念的话：人话结果 +（若有）「现在：<回读状态>」。"""
    code = (code or '').strip() or 'ERROR'
    reason = (reason or '').strip()
    if code in _OK_CODES:
        head = reason or ('做好了' if code == 'OK' else '做了一部分')
    elif code in _CODE_CN:
        head = _CODE_CN[code]
    else:
        prefix = _CODE_PREFIX.get(code, '机械臂动作没做成（%s）' % code)
        head = prefix + ('：' + reason if reason else '')
    st = state_text(state_json)
    head = head.rstrip('。')
    return head + '。' + ('现在：' + st + '。' if st else '')


class ArmClient:

    def __init__(self, goal_timeout: float = 60.0, cancel_grace: float = 3.0,
                 home_timeout: float = 30.0):
        self.goal_timeout = goal_timeout
        self.cancel_grace = cancel_grace
        self.home_timeout = home_timeout
        self.enabled = False
        self.reason = "还没 start()"
        self._node = None
        self._action = None
        self._describe_cli = None
        self._home_cli = None
        self._executor = None
        self._thread = None
        self._lock = threading.Lock()
        self._gh = None                       # 当前 goal handle（cancel 用）
        self._cancel_evt = threading.Event()  # 唤醒词喊过 ⇒ 当前/正在发的 goal 要取消
        self._ArmMotion = None                # start() 里填（测试可注入假的）
        self._Trigger = None

    # ---- 生命周期 ----

    def start(self) -> bool:
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from raspot_interfaces.action import ArmMotion
            from std_srvs.srv import Trigger
        except ImportError as e:
            self.reason = f"缺依赖（{e}）"
            logger.warning(f"机械臂客户端：{self.reason}，臂动作工具降级")
            return False
        try:
            try:
                rclpy.init(args=[])
            except Exception:                 # noqa: BLE001 —— 小车桥/抓取客户端可能已 init
                pass
            self._ArmMotion, self._Trigger = ArmMotion, Trigger
            self._node = Node("voice_arm")
            self._action = ActionClient(self._node, ArmMotion, "/arm/motion")
            self._describe_cli = self._node.create_client(Trigger, "/arm/describe")
            self._home_cli = self._node.create_client(Trigger, "/arm/go_home")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin,
                                            name="arm-client-spin", daemon=True)
            self._thread.start()
        except Exception as e:                # noqa: BLE001
            self.reason = f"起客户端失败：{e!r}"
            logger.error(f"机械臂客户端：{self.reason}")
            return False
        self.enabled = True
        self.reason = "ok"
        logger.info("机械臂客户端：已就绪（/arm/motion /arm/describe /arm/go_home）")
        return True

    def close(self) -> None:
        self.cancel()
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
        return f"机械臂现在用不了 —— {self.reason}"

    def act(self, spec: dict, timeout: float | None = None) -> str:
        """发一次动作（spec = 设计 §5），阻塞到出结果。返回一句给模型念的话。"""
        if not self.enabled:
            return self.unavailable_text()
        self._cancel_evt.clear()              # 新的一句话 = 新动作，之前的唤醒词不算数
        timeout = self.goal_timeout if timeout is None else timeout
        try:
            spec_json = json.dumps(spec, ensure_ascii=False)
        except (TypeError, ValueError) as e:
            return format_result('BAD_GOAL', f'参数没法序列化：{e}')
        code, reason, state = self._send_and_wait(spec_json, timeout)
        logger.info(f"臂动作：{spec_json} → {code} {reason}")
        return format_result(code, reason, state)

    def describe(self) -> str:
        if not self.enabled:
            return self.unavailable_text()
        ok, msg = self._call_trigger(self._describe_cli, "/arm/describe", 3.0)
        if not ok:
            return f"读不到机械臂状态 —— {msg}"
        st = state_text(msg)
        return f"机械臂现在：{st}。" if st else f"读到的状态没法解析：{msg[:200]}"

    def home(self) -> str:
        if not self.enabled:
            return self.unavailable_text()
        ok, msg = self._call_trigger(self._home_cli, "/arm/go_home", self.home_timeout)
        if not ok:
            return f"没能复位 —— {msg}"
        # 复位服务不带状态 ⇒ 补一次真实回读（读不到就不说读数）
        ok2, msg2 = self._call_trigger(self._describe_cli, "/arm/describe", 3.0)
        st = state_text(msg2) if ok2 else ''
        head = (msg or '已复位').rstrip('。')
        return head + '。' + ('现在：' + st + '。' if st else '')

    def cancel(self) -> None:
        """唤醒词急停路径调（监听线程上）。**绝不抛、不阻塞**。"""
        try:
            self._cancel_evt.set()
            with self._lock:
                gh = self._gh
            if gh is not None:
                gh.cancel_goal_async()
        except Exception:                     # noqa: BLE001
            pass

    # ---- 内部 ----

    def _canceled_fallback(self) -> tuple[str, str, str]:
        """取消后没等到服务端结果：真实回读能读到就带上，读不到就不带。"""
        ok, msg = self._call_trigger(self._describe_cli, "/arm/describe", 1.5)
        return 'CANCELED', '', (msg if ok else '')

    def _send_and_wait(self, spec_json: str, timeout: float) -> tuple[str, str, str]:
        if not self._action.wait_for_server(timeout_sec=2.0):
            return 'ERROR', '动作服务 /arm/motion 不在（arm-grasp 服务没跑？）', ''
        if self._cancel_evt.is_set():
            return 'CANCELED', '', ''
        goal = self._ArmMotion.Goal()
        goal.spec_json = spec_json
        goal.dry_run = False
        send_fut = self._action.send_goal_async(goal)
        end = time.monotonic() + 10.0
        while not send_fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not send_fut.done():
            return 'ERROR', '动作 goal 一直没被接受', ''
        gh = send_fut.result()
        if not gh.accepted:
            return 'BUSY', '', ''
        with self._lock:
            self._gh = gh
        try:
            if self._cancel_evt.is_set():     # 发送途中喊了唤醒词
                gh.cancel_goal_async()
            res_fut = gh.get_result_async()
            end = time.monotonic() + timeout
            cancel_deadline = None
            timed_out = False
            timeout_reason = f'{timeout:.0f}s 还没做完，已取消'
            while not res_fut.done():
                now = time.monotonic()
                if cancel_deadline is None and self._cancel_evt.is_set():
                    cancel_deadline = now + self.cancel_grace
                elif cancel_deadline is None and now > end:
                    gh.cancel_goal_async()    # 超时：同样等服务端冻结后的结果
                    timed_out = True
                    cancel_deadline = now + self.cancel_grace
                if cancel_deadline is not None and now > cancel_deadline:
                    code, _, state = self._canceled_fallback()
                    return ('TIMEOUT', timeout_reason, state) if timed_out else (code, '', state)
                time.sleep(0.02)
            res = res_fut.result().result
            if timed_out and res.code == 'CANCELED':
                return 'TIMEOUT', timeout_reason, res.state_json
            return res.code, res.reason, res.state_json
        finally:
            with self._lock:
                self._gh = None

    def _call_trigger(self, cli, name: str, timeout: float) -> tuple[bool, str]:
        """调一个 Trigger 服务 → (success, message)。不抛。"""
        try:
            if cli is None or not cli.wait_for_service(timeout_sec=2.0):
                return False, f"{name} 不在（arm-grasp 服务没跑？）"
            fut = cli.call_async(self._Trigger.Request())
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.02)
            if not fut.done():
                return False, f"{name} {timeout:.0f}s 没回"
            r = fut.result()
            return bool(r.success), str(r.message or '')
        except Exception as e:                # noqa: BLE001
            return False, f"{name} 调用出错：{e!r}"
