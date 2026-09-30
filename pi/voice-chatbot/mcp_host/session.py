"""单个 MCP server 的客户端会话 —— 同步门面。

项目其余部分是**刻意全同步串行**的（见 core/conversation.py 的模块文档），
而官方 MCP SDK 是 async-only。这里用 anyio 的 BlockingPortal 做桥：一条守护线程
跑事件循环，所有 async 上下文管理器和 RPC 都经 portal 提交，调用方看到的是普通
同步方法。

用 `portal.wrap_async_context_manager()` 而不是自己搓 event loop —— 它是 anyio
为「在同步代码里保住一个 async 上下文管理器的生命周期」专门提供的原语。自己实现
容易在 task group 的 enter/exit 线程亲和性上踩坑（mcp 的 stdio_client 内部就是
task group）。
"""

from __future__ import annotations

import sys
from datetime import timedelta
from typing import Any, TextIO

from loguru import logger
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from mcp_host import projections

_ERRLOG_MAX_BYTES = 2 * 1024 * 1024  # 单文件 2MB，超了就截断重开，防止无界增长


class MCPServerSession:
    """一个 MCP server 子进程的同步客户端。

    Usage:
        session = MCPServerSession("weather", command=["python3", "-m", ...],
                                   callbacks={...})
        session.start(portal)     # 拉起子进程 + initialize + 发现能力
        text = session.call_tool("get_forecast", {"city": "大庆"})
        session.close()
    """

    def __init__(
        self,
        name: str,
        *,
        command: str,
        args: list[str] | None = None,
        env: dict[str, str] | None = None,
        cwd: str | None = None,
        callbacks: dict[str, Any] | None = None,
        read_timeout_sec: float = 30.0,
        errlog_path: str | None = None,
    ) -> None:
        self.name = name
        self._command = command
        self._args = list(args or [])
        self._env = env
        self._cwd = cwd
        self._callbacks = callbacks or {}
        self._read_timeout_sec = read_timeout_sec
        self._errlog_path = errlog_path

        self._portal: Any = None
        self._stdio_cm: Any = None
        self._session_cm: Any = None
        self._session: ClientSession | None = None
        self._errlog_file: TextIO | None = None
        self._init_result: Any = None

        self._tools: list[Any] = []
        self._resources: list[Any] = []
        self._resource_templates: list[Any] = []
        self._prompts: list[Any] = []

    # ------------------------------------------------------------ 生命周期

    def start(self, portal: Any) -> None:
        """拉起子进程并完成 MCP 握手。失败会往上抛，由 MCPHost 决定是否降级。"""
        self._portal = portal

        params = StdioServerParameters(
            command=self._command,
            args=self._args,
            env=self._env,
            cwd=self._cwd,
        )
        errlog = self._open_errlog()

        # 先 __enter__ 成功再赋值：失败时这两个字段保持 None，
        # close() 就不会去 __exit__ 一个根本没进去过的上下文管理器。
        stdio_cm = portal.wrap_async_context_manager(stdio_client(params, errlog=errlog))
        read, write = stdio_cm.__enter__()
        self._stdio_cm = stdio_cm

        # read_timeout_seconds 同时兜住了 initialize —— server 挂住不响应时不会
        # 把助手启动无限期卡死。
        session_cm = portal.wrap_async_context_manager(
            ClientSession(
                read,
                write,
                read_timeout_seconds=timedelta(seconds=self._read_timeout_sec),
                **self._callbacks,
            )
        )
        self._session = session_cm.__enter__()
        self._session_cm = session_cm

        self._init_result = self._call(self._session.initialize)
        logger.info(
            f"MCP[{self.name}]: 已连接 —— {self._init_result.serverInfo.name} "
            f"协议 {self._init_result.protocolVersion}"
        )
        self._discover()

    def close(self) -> None:
        """退出两个 async 上下文管理器（顺序必须是 session 先、stdio 后）。"""
        for attr in ("_session_cm", "_stdio_cm"):
            cm = getattr(self, attr)
            if cm is None:
                continue
            try:
                cm.__exit__(None, None, None)
            except Exception as exc:  # noqa: BLE001 —— 关不干净不该影响助手退出
                logger.debug(f"MCP[{self.name}]: 关闭 {attr} 时出错 —— {exc}")
            setattr(self, attr, None)

        self._session = None
        if self._errlog_file is not None:
            try:
                self._errlog_file.close()
            except Exception:  # noqa: BLE001
                pass
            self._errlog_file = None

    # ------------------------------------------------------------ 能力发现

    @property
    def capabilities(self) -> Any:
        return self._init_result.capabilities if self._init_result else None

    @property
    def server_info(self) -> Any:
        return self._init_result.serverInfo if self._init_result else None

    @property
    def protocol_version(self) -> str | None:
        return self._init_result.protocolVersion if self._init_result else None

    @property
    def tools(self) -> list[Any]:
        return self._tools

    @property
    def resources(self) -> list[Any]:
        return self._resources

    @property
    def resource_templates(self) -> list[Any]:
        """带参数的 resource 在 MCP 里是**模板**，list_resources 拿不到它们。

        这是个很容易漏的点：`@mcp.resource("weather://{city}/current")` 会出现在
        list_resource_templates 而不是 list_resources。只查前者等于整条 resources
        通路都是空的。
        """
        return self._resource_templates

    @property
    def prompts(self) -> list[Any]:
        return self._prompts

    @property
    def supports_subscribe(self) -> bool:
        caps = self.capabilities
        return bool(getattr(getattr(caps, "resources", None), "subscribe", False))

    def _discover(self) -> None:
        """按服务器**声明**的能力去发现内容 —— 没声明的就不问。

        这也是「诚实协商」的另一面：不要向服务器索取它说自己没有的东西。
        """
        caps = self.capabilities

        try:
            self._tools = list(self._call(self._session.list_tools).tools)  # type: ignore[union-attr]
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"MCP[{self.name}]: list_tools 失败 —— {exc}")

        if getattr(caps, "resources", None) is not None:
            try:
                self._resources = list(self._call(self._session.list_resources).resources)  # type: ignore[union-attr]
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"MCP[{self.name}]: list_resources 失败 —— {exc}")
            try:
                result = self._call(self._session.list_resource_templates)  # type: ignore[union-attr]
                self._resource_templates = list(result.resourceTemplates)
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"MCP[{self.name}]: list_resource_templates 失败 —— {exc}")

        if getattr(caps, "prompts", None) is not None:
            try:
                self._prompts = list(self._call(self._session.list_prompts).prompts)  # type: ignore[union-attr]
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"MCP[{self.name}]: list_prompts 失败 —— {exc}")

        logger.info(
            f"MCP[{self.name}]: 发现 {len(self._tools)} 个工具 / "
            f"{len(self._resources)} 个资源(另有 {len(self._resource_templates)} 个模板) / "
            f"{len(self._prompts)} 个提示"
        )

    # ------------------------------------------------------------ RPC

    def call_tool(self, tool_name: str, arguments: dict[str, Any] | None = None) -> str:
        """调用工具，返回拍平后的文本。异常由调用方（MCPHost）处理。"""
        result = self._call(self._session.call_tool, tool_name, arguments or {})  # type: ignore[union-attr]
        text = projections.call_result_to_text(result)
        if getattr(result, "isError", False):
            logger.warning(f"MCP[{self.name}]: 工具 {tool_name} 返回 isError")
        return text

    def read_resource(self, uri: str) -> str:
        result = self._call(self._session.read_resource, uri)  # type: ignore[union-attr]
        return projections.resource_result_to_text(result)

    def get_prompt(self, prompt_name: str, arguments: dict[str, str] | None = None) -> str:
        result = self._call(self._session.get_prompt, prompt_name, arguments or {})  # type: ignore[union-attr]
        return projections.prompt_result_to_text(result)

    def ping(self) -> bool:
        try:
            self._call(self._session.send_ping)  # type: ignore[union-attr]
            return True
        except Exception:  # noqa: BLE001
            return False

    # ------------------------------------------------------------ 内部

    def _call(self, fn: Any, *args: Any) -> Any:
        if self._portal is None:
            raise RuntimeError(f"MCP[{self.name}]: 会话尚未启动")
        return self._portal.call(fn, *args)

    def _open_errlog(self) -> TextIO:
        """server 的 stderr 落到独立日志文件。

        不能让它进 stdout —— 那是协议通道；也不该混进主日志，否则一个话多的
        server 会把助手的日志冲垮。
        """
        if not self._errlog_path:
            return sys.stderr
        try:
            import os

            if os.path.exists(self._errlog_path) and os.path.getsize(self._errlog_path) > _ERRLOG_MAX_BYTES:
                os.remove(self._errlog_path)
            self._errlog_file = open(  # noqa: SIM115 —— 生命周期跟 session 绑定，close() 里关
                self._errlog_path, "a", buffering=1, encoding="utf-8", errors="replace"
            )
            return self._errlog_file
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"MCP[{self.name}]: 打不开 errlog {self._errlog_path} —— {exc}")
            return sys.stderr
