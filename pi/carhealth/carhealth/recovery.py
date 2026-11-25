"""二期恢复层：白名单动作 + 互锁 + 冷却/预算 + 动作后验证 + 失败熔断。

只做三件事：
- `wifi_powersave_off`：关 WiFi 省电（不重启任何东西，忙时也允许）
- `ros_daemon_stop`：停掉绑在旧 IP 上的 ros2 daemon
- `restart_stack` / `restart_voice`：整栈/语音重启（必须在互锁全清时）

特权动作统一走 root-owned 的 `/usr/local/sbin/carhealth-recover`，
sudoers 只放行这一条命令；守护进程本身不需要 root。
"""

from __future__ import annotations

import subprocess
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from .busy import BusyState
from .types import Verdict

RECOVER_CMD = "/usr/local/sbin/carhealth-recover"

# action → 特权脚本子命令。必须显式枚举，不能靠 `replace("_", "-")` 推：
# `restart_stack` 推出来是 `restart-stack`，脚本只认 `restart-ros-stack`，
# 2026-11-18 就这样跑了一次 rc=2 "unknown action"，整栈重启从未真正执行。
ACTION_ARGS = {
    "wifi_powersave_off": "wifi-powersave-off",
    "ros_daemon_stop": "ros-daemon-stop",
    "restart_stack": "restart-ros-stack",
    "restart_voice": "restart-voice",
}

ROS_ROOT_CAUSES = ("ros_discovery", "ros_data_path")


@dataclass
class RecoveryConfig:
    enabled: bool = False
    startup_grace_s: float = 60.0
    settle_s: float = 30.0
    budget_window_s: float = 600.0
    budget_max: int = 3
    fail_escalate: int = 2
    block_after_fail_s: float = 1800.0
    kill_switch: Path | None = None
    cooldowns: dict[str, float] = field(default_factory=lambda: {
        "wifi_powersave_off": 120.0,
        "ros_daemon_stop": 300.0,
        "restart_stack": 600.0,
        "restart_voice": 180.0,
    })


@dataclass(frozen=True)
class RecoveryRequest:
    action: str
    reason: str
    targets: tuple[str, ...] = ()
    deferred: bool = False
    detail: str = ""


class Recovery:
    def __init__(self, config: RecoveryConfig, runner=subprocess.run,
                 clock: Callable[[], float] = time.monotonic,
                 cmd: str = RECOVER_CMD):
        self.config = config
        self._runner = runner
        self._clock = clock
        self._cmd = cmd
        self._start: float | None = None
        self._last: dict[str, float] = {}
        self._blocked_until: dict[str, float] = {}
        self._times: deque[float] = deque()
        self._failures: dict[str, int] = {}
        self._verify: dict | None = None
        self._last_action: dict | None = None

    # ---- 判定 ----

    @staticmethod
    def _has(diag, component, rule_id) -> bool:
        return any(d.id == rule_id for d in (diag.get(component) or ()))

    @staticmethod
    def _blocked_ros(states) -> list[str]:
        return [name for name, st in states.items()
                if st.verdict is Verdict.BLOCKED and st.root_cause in ROS_ROOT_CAUSES]

    def _allowed(self, action: str, now: float) -> bool:
        if now < self._blocked_until.get(action, 0.0):
            return False
        if now - self._last.get(action, -1e9) < self.config.cooldowns.get(action, 300.0):
            return False
        while self._times and now - self._times[0] > self.config.budget_window_s:
            self._times.popleft()
        return len(self._times) < self.config.budget_max

    def _request(self, action: str, reason: str, now: float,
                 targets=(), *, check_budget=True) -> RecoveryRequest:
        if check_budget and not self._allowed(action, now):
            return RecoveryRequest(action, reason, tuple(targets),
                                   deferred=True, detail="cooldown/budget")
        return RecoveryRequest(action, reason, tuple(targets))

    def decide(self, *, states, diag, busy: BusyState,
               network_changed: bool, now: float) -> RecoveryRequest | None:
        if not self.config.enabled:
            return None
        if self.config.kill_switch is not None and Path(self.config.kill_switch).exists():
            return None
        if self._start is None:
            self._start = now
        if now - self._start < self.config.startup_grace_s:
            return None
        if self._verify is not None and now < self._verify["until"]:
            return None

        # 1) WiFi 省电：不重启任何服务，忙时也允许。
        if self._has(diag, "host", "host.wifi_powersave"):
            return self._request("wifi_powersave_off", "WiFi 省电模式开着", now)

        blocked = self._blocked_ros(states)

        # 2) 换网/图故障：整栈重启，必须互锁全清。
        if blocked and (network_changed or len(blocked) >= 3):
            if busy.any:
                return RecoveryRequest("restart_stack", "ROS 图故障，动作中暂缓",
                                       tuple(blocked), deferred=True, detail="interlock")
            return self._request("restart_stack", "ROS 图故障", now, blocked)

        # 3) 只是旧 IP socket 残留、组件都健康：只停 ros2 daemon。
        if self._has(diag, "chassis", "dds.stale_ip") and not blocked:
            return self._request("ros_daemon_stop", "DDS 仍绑旧 IP", now)

        # 4) 唤醒模块持续掉线：空闲时重启 voice。
        voice = states.get("voice")
        if (voice is not None and voice.verdict in (Verdict.STALE, Verdict.ABSENT)
                and self._has(diag, "voice", "voice.wakeword_lost")
                and now - voice.since >= 60.0):
            if busy.voice:
                return RecoveryRequest("restart_voice", "唤醒模块掉线，语音忙暂缓",
                                       ("voice",), deferred=True, detail="interlock")
            return self._request("restart_voice", "唤醒模块持续掉线", now, ("voice",))

        return None

    # ---- 执行与验证 ----

    def execute(self, req: RecoveryRequest, now: float) -> dict:
        self._last[req.action] = now
        self._times.append(now)
        arg = ACTION_ARGS.get(req.action)
        if arg is None:
            # 失败关闭：没登记的动作绝不交给 sudo 脚本去猜。
            rc, output = -1, f"unknown action: {req.action}"
        else:
            cmd = ["sudo", "-n", self._cmd, arg]
            try:
                r = self._runner(cmd, capture_output=True, text=True, timeout=120)
                rc = r.returncode
                output = (r.stdout or "") + (r.stderr or "")
            except Exception as e:                          # noqa: BLE001
                rc = -1
                output = repr(e)
        self._verify = {
            "action": req.action,
            "targets": req.targets,
            "reason": req.reason,
            "until": now + self.config.settle_s,
        }
        self._last_action = {
            "action": req.action, "rc": rc, "output": output[-500:].strip(),
            "reason": req.reason, "targets": list(req.targets),
        }
        return dict(self._last_action)

    def observe(self, *, states, diag, now: float) -> dict | None:
        v = self._verify
        if v is None or now < v["until"]:
            return None
        self._verify = None
        action = v["action"]

        if action == "wifi_powersave_off":
            ok = not self._has(diag, "host", "host.wifi_powersave")
        elif action == "ros_daemon_stop":
            ok = not self._has(diag, "chassis", "dds.stale_ip")
        elif action == "restart_stack":
            ok = not any(st.verdict is Verdict.BLOCKED
                         and st.root_cause in ROS_ROOT_CAUSES
                         for st in states.values())
        elif action == "restart_voice":
            st = states.get("voice")
            ok = bool(st is not None and st.verdict is Verdict.OK)
        else:
            ok = False

        if ok:
            self._failures[action] = 0
        else:
            n = self._failures.get(action, 0) + 1
            self._failures[action] = n
            if n >= self.config.fail_escalate:
                self._blocked_until[action] = now + self.config.block_after_fail_s
        return {"action": action, "ok": ok, "reason": v["reason"],
                "targets": list(v["targets"]),
                "failures": self._failures.get(action, 0)}

    def snapshot(self) -> dict:
        return {
            "enabled": self.config.enabled,
            "last_action": self._last_action,
            "failures": dict(self._failures),
            "blocked_until": {k: round(v, 1) for k, v in self._blocked_until.items()},
        }
