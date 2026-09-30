"""K230 目标命令的客户端 —— 把「跟着谁」翻译成板子能懂的一条 8557 命令。

## 分工（照 k230_look.py 的同一套路子）

- **本层**：调 `k230ctl` 子进程，失败一律抛 `TargetError`，消息是**技术口吻**
  （保持纯粹、好单测）。
- **工具层**（`tools/follow.py`）：把那句技术话翻成人话给模型念。

## 为什么不 import k230pi

`/home/cy/k230-vision/pi` 是**另一个项目**。`k230_look.py` 已经定了跨项目的姿势：
**走 `k230ctl` 子进程**，不 import。这里照做。

## 名字 -> id 的映射在哪

在板子那一侧的 `pi/people.json`（板子上文件名只能是 ASCII，中文名只能存在消费侧）。
本层把用户说的名字原样传下去，`k230ctl` 负责解析。
"""

from __future__ import annotations

import subprocess

DEFAULT_CMD = "/home/cy/k230-vision/pi/k230ctl"
# `follow` 要等对方露正脸（板子最多等 8 秒）才回话，所以超时得比它长。
DEFAULT_TIMEOUT_SEC = 15.0


class TargetError(RuntimeError):
    """给 K230 发目标命令失败（命令跑不起来 / 超时 / 板子回了 err）。"""


class K230Target:
    """三个动作：跟着某人 / 别跟了 / 直接给框。

    `runner` 可注入 —— 测试靠它换假的，不插 K230、不碰串口也能跑。
    """

    def __init__(self, config=None, runner=None):
        get = config.get if config is not None else (lambda k, d=None: d)
        self._cmd = get("follow.cmd", DEFAULT_CMD) or DEFAULT_CMD
        self._timeout = float(get("follow.timeout_sec", DEFAULT_TIMEOUT_SEC)
                              or DEFAULT_TIMEOUT_SEC)
        self._runner = runner or subprocess.run

    # ---- 对外 ----
    def follow(self, name: str) -> str:
        """跟着 `name`（用户说的名字，或直接是 faces 里的 id）。

        ⚠️ 参数名必须和工具的 `name` **一致** —— `_guard` 是把 kwargs 直通给
        客户端的（`getattr(target, m)(**kwargs)`），名字对不上会变成"参数不匹配"。
        """
        return self._run(["follow", str(name)])

    def stop(self) -> str:
        """别跟了。**同时把板子切回"检测器自动锁定"** —— 那才是"恢复正常"。"""
        first = self._run(["stop"])
        try:
            self._run(["auto"])
        except TargetError:
            # 停止已经成功了，auto 失败不该把整件事报成失败。
            return first + "（自动锁定没能切回来，板子可能要重启）"
        return first

    def auto(self) -> str:
        return self._run(["auto"])

    def point(self, u: float, v: float, size: float | None = None) -> str:
        argv = ["pt", repr(float(u)), repr(float(v))]
        if size is not None:
            argv += ["--size", repr(float(size))]
        return self._run(argv)

    # ---- 内部 ----
    def _run(self, argv: list[str]) -> str:
        cmd = [self._cmd] + argv
        try:
            r = self._runner(cmd, capture_output=True, text=True,
                             timeout=self._timeout)
        except subprocess.TimeoutExpired:
            raise TargetError("板子 %.0f 秒没回话（k230ctl %s）" % (self._timeout, argv[0]))
        except OSError as e:
            raise TargetError("跑不起 k230ctl —— %s" % e)
        if r.returncode != 0:
            raise TargetError("k230ctl %s 失败：%s"
                              % (argv[0], (r.stderr or r.stdout or "").strip()[:200]))
        return (r.stdout or "").strip()
