"""真实 MCP 宿主层 —— 把这个语音助手变成 MCP host。

    - 拉起 MCP server 子进程（stdio 传输），完整双向协议
    - tools / resources / prompts + elicitation / sampling / roots / logging
    - 对外只暴露**同步** API：项目其余部分是刻意全同步串行的
      （见 core/conversation.py 的模块文档）

⚠️ 目录名是 `mcp_host` 而不是 `mcp`，这不是随便起的：

    项目根是 systemd 的 WorkingDirectory。如果根下存在 `mcp/` 目录，
    `import mcp` 会解析到本地那个目录并**遮蔽官方 SDK** —— 结果就是
    qq-music-mcp 和天气服务器一起崩，而且报错信息会指向很莫名的地方。
"""

# projections 必须最先导入：session/host 都用 `from mcp_host import projections`，
# 先让它进 sys.modules，避免包初始化过程中的属性查找走 fallback 路径。
from mcp_host import projections
from mcp_host.callbacks import ElicitationRelay, HostBridge, make_session_callbacks
from mcp_host.session import MCPServerSession
from mcp_host.host import MCPHost, ServerSpec, ToolEntry

__all__ = [
    "ElicitationRelay",
    "HostBridge",
    "MCPHost",
    "MCPServerSession",
    "ServerSpec",
    "ToolEntry",
    "make_session_callbacks",
    "projections",
]
