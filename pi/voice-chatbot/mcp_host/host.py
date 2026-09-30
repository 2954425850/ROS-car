"""MCPHost —— 所有 MCP server 的宿主。

职责：
  - 持有一条守护线程里的 anyio BlockingPortal（全体 server 共享一个事件循环）
  - 按 config 拉起各 server 子进程，管理会话生命周期
  - 持有 MCP 工具表（**不进助手的 ToolRegistry**，见下）
  - 把 resources / prompts 也暴露成工具（它们没有 function 对应物，只能这样桥）

为什么 MCP 工具不塞进 tools.ToolRegistry：

    ToolRegistry 的 Tool 是 (name, description, parameters, handler)，没有地方放
    server 归属、annotations(destructiveHint/readOnlyHint)、outputSchema。把 MCP
    压进去等于把协议降级成「远程函数库」。所以 host 自己持有工具表，
    只在组装发给 LLM 的 tools= 数组时，由 projections 做一次薄投影。

降级原则：任何 server 起不来只 warning，绝不阻断助手启动 —— 这是台在用机器。
"""

from __future__ import annotations

import contextlib
import json
import os
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from anyio.from_thread import start_blocking_portal
from loguru import logger

from mcp_host import projections
from mcp_host.callbacks import ElicitationRelay, HostBridge, make_session_callbacks
from mcp_host.session import MCPServerSession

_DEFAULT_READ_TIMEOUT_SEC = 30.0


@dataclass
class ServerSpec:
    """一个 MCP server 的配置。"""

    name: str
    command: list[str]
    prefix: str = ""
    enabled: bool = True
    cwd: str | None = None
    env: dict[str, str] = field(default_factory=dict)
    read_timeout_sec: float = _DEFAULT_READ_TIMEOUT_SEC
    extra: dict[str, Any] = field(default_factory=dict)
    """server 私有配置，通过 MCP_SERVER_CONFIG 环境变量以 JSON 传给它。"""

    expose_tools: list[str] | None = None
    """暴露给模型的工具白名单（None = 全部暴露）。

    只影响**模型看得到什么**，不影响 host 能不能调 —— 内部调用（比如
    MusicController 用 get_song_url）走的是完整工具表。这个区分很要紧：
    早先把两者混在一起，砍白名单会把 play_music 内部链路一起砍断。
    """

    expose_capabilities: bool = False
    """是否把 resources / prompts 也合成成工具暴露给模型。

    默认关：语音场景根本用不上（模型不会自发去 list_prompts），
    开着只是每次请求白占 context。想看这条通路时才打开。
    """

    @classmethod
    def from_config(cls, name: str, raw: dict[str, Any]) -> "ServerSpec":
        command = raw.get("command") or []
        if isinstance(command, str):
            command = [command]
        if not command:
            raise ValueError(f"MCP server {name} 缺少 command")

        # 除已知键外的都当作 server 私有配置传给子进程
        known = {"command", "prefix", "enabled", "cwd", "env", "read_timeout_sec",
                 "expose_tools", "expose_capabilities"}
        extra = {k: v for k, v in raw.items() if k not in known}

        return cls(
            name=name,
            command=[str(c) for c in command],
            prefix=str(raw.get("prefix") or name),
            enabled=bool(raw.get("enabled", True)),
            cwd=raw.get("cwd"),
            env={str(k): str(v) for k, v in (raw.get("env") or {}).items()},
            read_timeout_sec=float(raw.get("read_timeout_sec", _DEFAULT_READ_TIMEOUT_SEC)),
            extra=extra,
            expose_tools=list(raw["expose_tools"]) if raw.get("expose_tools") else None,
            expose_capabilities=bool(raw.get("expose_capabilities", False)),
        )


@dataclass
class ToolEntry:
    """工具表里的一项。

    kind:
        "mcp"      —— 来自服务器 tools/list 的真实工具
        "resource" —— 合成的资源访问工具（分 op: list/read）
        "prompt"   —— 合成的提示访问工具（分 op: list/get）
    """

    server: str
    tool: Any
    kind: str = "mcp"
    op: str | None = None


@dataclass
class _SyntheticTool:
    """伪造一个和 MCP Tool 同形的对象，好让 projections 一视同仁地投影。"""

    name: str
    description: str
    inputSchema: dict[str, Any]
    annotations: Any = None


class MCPHost:
    """所有 MCP server 的宿主。

    Usage:
        host = MCPHost(config, bridge=bridge, project_root=...)
        host.start()
        schemas = host.tool_schemas()          # 合并进发给 LLM 的 tools=
        text = host.call_tool(qualified, args) # 永不抛异常
        host.close()
    """

    def __init__(
        self,
        config: Any,
        *,
        bridge: HostBridge | None = None,
        project_root: Path | str | None = None,
        log_dir: Path | str | None = None,
    ) -> None:
        self._config = config
        self._bridge = bridge or HostBridge()
        self._project_root = Path(project_root or ".")
        self._log_dir = Path(log_dir) if log_dir else self._project_root / "logs"

        self._stack = contextlib.ExitStack()
        self._portal: Any = None
        self._servers: dict[str, MCPServerSession] = {}
        # _tool_index = 全量，供内部调用；_exposed = 子集，供发给模型。
        # 两者必须分开：白名单砍的是「模型看得到什么」，不是「host 能调什么」。
        self._tool_index: dict[str, ToolEntry] = {}
        self._exposed: set[str] = set()

        # elicitation 只有在助手确实提供了语音追问实现时才创建中继。
        # 没有中继 → make_session_callbacks 不传 elicitation_callback →
        # 能力不会被声明。诚实协商。
        self._relay = ElicitationRelay(self._bridge.elicit) if self._bridge.elicit else None

    # ------------------------------------------------------------ 生命周期

    @property
    def elicit_relay(self) -> ElicitationRelay | None:
        """给主线程用的 elicitation 泵。见 ElicitationRelay 的文档。"""
        return self._relay

    def start(self) -> None:
        """拉起所有 enabled 的 server。**任何单个失败都不阻断启动。**"""
        specs = self._read_specs()
        if not specs:
            logger.info("MCP: 配置里没有 enabled 的 server，跳过")
            return

        self._portal = self._stack.enter_context(start_blocking_portal(name="mcp-host"))

        for spec in specs:
            self._start_one(spec)

        if self._servers:
            logger.info(
                f"MCP: 就绪 —— {len(self._servers)}/{len(specs)} 个 server，"
                f"{len(self._exposed)} 个工具暴露给模型"
                f"（另 {len(self._tool_index) - len(self._exposed)} 个仅供内部调用）"
            )
        else:
            logger.warning("MCP: 所有 server 都没起来，助手的原生功能不受影响")

    def close(self) -> None:
        # 顺序很关键：先退各会话的 async 上下文管理器，再停 portal 线程。
        for name, session in self._servers.items():
            try:
                session.close()
            except Exception as exc:  # noqa: BLE001
                logger.debug(f"MCP[{name}]: 关闭会话时出错 —— {exc}")
        self._servers.clear()
        self._tool_index.clear()

        try:
            self._stack.close()  # 停掉 portal 线程
        except Exception as exc:  # noqa: BLE001
            logger.debug(f"MCP: 关闭 portal 时出错 —— {exc}")
        self._portal = None

    # ------------------------------------------------------------ 工具表

    def tool_schemas(self) -> list[dict[str, Any]]:
        """投影成 OpenAI 兼容的 tools 数组 —— **只包含白名单内的**。

        每次请求都随 tools= 发出去，所以这里每多一个工具都是持续的 context 开销
        和注意力稀释。内部要用的工具（如 get_song_url）留在 _tool_index 里不进这里。
        """
        return [
            projections.tool_to_schema(self._tool_index[name].server, self._tool_index[name].tool)
            for name in self._tool_index
            if name in self._exposed
        ]

    @property
    def tool_count(self) -> int:
        """发给模型的工具数。"""
        return len(self._exposed)

    @property
    def internal_tool_count(self) -> int:
        """host 实际持有的工具总数（含只给内部用的）。"""
        return len(self._tool_index)

    def has_tool(self, qualified_name: str) -> bool:
        return qualified_name in self._tool_index

    def is_destructive(self, qualified_name: str) -> bool:
        """MCP 的 destructiveHint。合成工具一律不危险。"""
        entry = self._tool_index.get(qualified_name)
        if entry is None or entry.kind != "mcp":
            return False
        return projections.is_destructive(entry.tool)

    def describe(self, qualified_name: str) -> str | None:
        entry = self._tool_index.get(qualified_name)
        return entry.tool.description if entry else None

    def call_tool(self, qualified_name: str, arguments: dict[str, Any] | None = None) -> str:
        """按全名调用。**永不抛异常** —— 与 ToolRegistry.dispatch() 的约定一致。"""
        arguments = arguments or {}
        entry = self._tool_index.get(qualified_name)
        if entry is None:
            return f"工具调用失败：不存在名为 {qualified_name} 的 MCP 工具。"

        session = self._servers.get(entry.server)
        if session is None:
            return f"工具调用失败：MCP server {entry.server} 当前不可用。"

        try:
            return self._dispatch(session, entry, arguments)
        except Exception as exc:  # noqa: BLE001
            logger.error(f"MCP[{entry.server}]: 调用 {qualified_name} 失败 —— {exc}")
            return f"工具 {qualified_name} 执行失败：{exc}"

    def _dispatch(self, session: MCPServerSession, entry: ToolEntry, arguments: dict[str, Any]) -> str:
        if entry.kind == "mcp":
            return session.call_tool(entry.tool.name, arguments)

        if entry.op == "list_resources":
            return self._format_resources(session)
        if entry.op == "read_resource":
            uri = arguments.get("uri")
            if not uri:
                return "调用失败：缺少 uri 参数。"
            return session.read_resource(str(uri))

        if entry.op == "list_prompts":
            return self._format_prompts(session)
        if entry.op == "get_prompt":
            name = arguments.get("name")
            if not name:
                return "调用失败：缺少 name 参数。"
            raw_args = arguments.get("arguments") or {}
            return session.get_prompt(str(name), {str(k): str(v) for k, v in raw_args.items()})

        return f"调用失败：未知的合成工具 {entry.op}"

    # ------------------------------------------------------------ 内部

    def _read_specs(self) -> list[ServerSpec]:
        raw_all = self._config.get("mcp_servers", {}) or {}
        if not isinstance(raw_all, dict):
            logger.warning("MCP: config.mcp_servers 不是字典，已忽略")
            return []

        specs: list[ServerSpec] = []
        for name, raw in raw_all.items():
            if not isinstance(raw, dict):
                logger.warning(f"MCP: server {name} 的配置不是字典，已忽略")
                continue
            try:
                spec = ServerSpec.from_config(name, raw)
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"MCP: server {name} 配置有误 —— {exc}")
                continue
            if spec.enabled:
                specs.append(spec)
        return specs

    @staticmethod
    def _resolve_command(command: str) -> str | None:
        """定位可执行文件。

        除了 PATH，还要显式找 ~/.local/bin —— systemd 启动时 PATH 里没有它，
        而用 `pip install --user` 装的 MCP server（如 qq-music-mcp）正落在那里。
        实测踩过：表现为「启动失败，已跳过」的**静默降级**，天气照常工作，
        只有 QQ音乐没了，很难第一眼看出问题。
        """
        if os.path.sep in command:
            return command if os.access(command, os.X_OK) else None

        found = shutil.which(command)
        if found:
            return found

        user_bin = Path.home() / ".local" / "bin" / command
        if user_bin.is_file() and os.access(user_bin, os.X_OK):
            return str(user_bin)
        return None

    def _start_one(self, spec: ServerSpec) -> None:
        callbacks = make_session_callbacks(self._bridge, spec.name, self._relay)

        env = {**os.environ, **spec.env}
        if spec.extra:
            # 用环境变量把 server 私有配置(默认城市、API key 等)带过去。
            # 这样 host 不需要知道任何 server 特有的字段名。
            env["MCP_SERVER_CONFIG"] = json.dumps(spec.extra, ensure_ascii=False)

        # 把这几个控制项打出来。它们放错层级（比如写进 env:）会**静默失效** ——
        # 我就在这上面栽过一次：expose_tools 写进 env 后变成了子进程的环境变量，
        # 白名单完全不起作用，而日志里一点异常都没有。
        logger.info(
            f"MCP[{spec.name}]: 启动配置 —— expose_tools={spec.expose_tools} "
            f"expose_capabilities={spec.expose_capabilities} env键={sorted(spec.env)}"
        )

        resolved = self._resolve_command(spec.command[0])
        if resolved is None:
            logger.warning(
                f"MCP[{spec.name}]: 找不到可执行文件 {spec.command[0]!r}"
                f"（PATH 和 ~/.local/bin 都没有），已跳过该 server"
            )
            return

        session = MCPServerSession(
            spec.name,
            command=resolved,
            args=spec.command[1:],
            env=env,
            cwd=spec.cwd,
            callbacks=callbacks,
            read_timeout_sec=spec.read_timeout_sec,
            errlog_path=str(self._log_dir / f"mcp_{spec.name}.log"),
        )

        try:
            session.start(self._portal)
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"MCP[{spec.name}]: 启动失败，已跳过 —— {exc}")
            with contextlib.suppress(Exception):
                session.close()
            return

        self._servers[spec.name] = session
        self._index_tools(spec, session)

    def _index_tools(self, spec: ServerSpec, session: MCPServerSession) -> None:
        whitelist = set(spec.expose_tools) if spec.expose_tools else None
        hidden: list[str] = []

        for tool in session.tools:
            qualified = projections.qualify(spec.name, tool.name)
            # 一律进全量索引（内部调用要用）……
            self._tool_index[qualified] = ToolEntry(server=spec.name, tool=tool, kind="mcp")
            # ……但只有白名单内的才发给模型
            if whitelist is None or tool.name in whitelist:
                self._exposed.add(qualified)
            else:
                hidden.append(tool.name)

        if hidden:
            logger.info(
                f"MCP[{spec.name}]: 白名单外隐藏 {len(hidden)}/{len(session.tools)} 个工具"
                f"（仍可内部调用）：{hidden}"
            )

        # resources / prompts 没有 function 对应物，只能合成工具来暴露。
        # 注意这是「桥」而不是「等价物」—— 真正的资源订阅语义（notifications/
        # resources/updated）走 logging/回调那条路，不在这里。
        #
        # 默认**不暴露**：语音场景用不上（模型不会自发去 list_prompts），
        # 开着纯属每次请求白占 context。要看这条通路就把 expose_capabilities 打开。
        if not spec.expose_capabilities:
            if session.resources or session.resource_templates or session.prompts:
                logger.debug(
                    f"MCP[{spec.name}]: 有 resources/prompts 但未暴露给模型"
                    f"（expose_capabilities=false）"
                )
            return

        if session.resources or session.resource_templates:
            self._add_synthetic(
                spec.name, "list_resources", "列出该服务器提供的所有资源(URI 与说明)。",
                {"type": "object", "properties": {}, "required": []}, "resource", "list_resources",
            )
            self._add_synthetic(
                spec.name, "read_resource", "读取指定 URI 的资源内容。先调 list_resources 拿 URI。",
                {
                    "type": "object",
                    "properties": {"uri": {"type": "string", "description": "资源 URI"}},
                    "required": ["uri"],
                },
                "resource", "read_resource",
            )

        if session.prompts:
            self._add_synthetic(
                spec.name, "list_prompts", "列出该服务器提供的所有提示模板。",
                {"type": "object", "properties": {}, "required": []}, "prompt", "list_prompts",
            )
            self._add_synthetic(
                spec.name, "get_prompt", "按名字取提示模板内容。先调 list_prompts 拿名字。",
                {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string", "description": "提示模板名"},
                        "arguments": {
                            "type": "object",
                            "description": "模板参数(键值都是字符串)",
                            "additionalProperties": {"type": "string"},
                        },
                    },
                    "required": ["name"],
                },
                "prompt", "get_prompt",
            )

    def _add_synthetic(
        self,
        server: str,
        name: str,
        description: str,
        schema: dict[str, Any],
        kind: str,
        op: str,
    ) -> None:
        qualified = projections.qualify(server, name)
        self._tool_index[qualified] = ToolEntry(
            server=server,
            tool=_SyntheticTool(name=name, description=description, inputSchema=schema),
            kind=kind,
            op=op,
        )
        self._exposed.add(qualified)

    @staticmethod
    def _format_resources(session: MCPServerSession) -> str:
        lines = [
            f"- {r.uri}：{getattr(r, 'name', None) or getattr(r, 'description', '') or '(无说明)'}"
            for r in session.resources
        ]
        # 模板要单独列出来，并且明确告诉模型「自己把 {} 换成实际值」——
        # 否则它会把带花括号的原样 URI 交给 read_resource。
        for t in session.resource_templates:
            desc = getattr(t, "description", "") or getattr(t, "name", "") or ""
            lines.append(f"- {t.uriTemplate}（模板，请把 {{}} 处替换成实际值）：{desc}")
        if not lines:
            return "(该服务器没有资源)"
        return "可用资源：\n" + "\n".join(lines)

    @staticmethod
    def _format_prompts(session: MCPServerSession) -> str:
        if not session.prompts:
            return "(该服务器没有提示模板)"
        lines = []
        for p in session.prompts:
            args = getattr(p, "arguments", None) or []
            arg_names = ", ".join(getattr(a, "name", "?") for a in args)
            suffix = f"（参数：{arg_names}）" if arg_names else ""
            lines.append(f"- {p.name}：{getattr(p, 'description', '') or '(无说明)'}{suffix}")
        return "可用提示模板：\n" + "\n".join(lines)
