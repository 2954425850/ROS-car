"""身份相关工具：留言、忘掉某人。

「何时用」照例只写在 description 里，不往 system_prompt 里点名能力。
认人/建档/记偏好**不是工具** —— 那些是 IdentityManager 在后台自动做的。
"""

from __future__ import annotations

from loguru import logger

from tools.registry import Tool, ToolRegistry


def register_identity_tools(registry: ToolRegistry, identity) -> None:
    if identity is None:
        return

    def leave_message(to: str, text: str) -> str:
        pid = identity.profiles.find_by_name(to)
        if pid is None:
            return f"我不认识叫「{to}」的人（档案里没有这个称呼），留不了言。"
        if pid == identity.current_pid:
            return "这就是在跟你说话的人本人，直接告诉 TA 就行。"
        sender = identity.profiles.display_name(identity.current_pid)
        identity.messages.add(pid, text, sender)
        logger.info(f"身份：给 {pid}（{to}）留言，来自 {sender or '未知'}")
        return f"记下了，下次 {to} 来找我说话时转告。"

    def forget_person(name: str) -> str:
        pid = identity.resolve_name(name)
        if pid is None:
            return "我不知道要忘掉的是谁（认不出你，或档案里没有这个称呼）。"
        if pid != identity.current_pid and not identity.is_owner():
            return "只能删除你自己的档案；要删别人的需要主人来说。"
        return identity.forget(pid)

    registry.register(Tool(
        name="leave_message",
        group="identity",
        description=(
            "替用户给另一个人留口信，等那个人下次来跟你说话时转告。"
            "用户说「告诉张三……」「跟李四说一声……」「他回来了让他……」时调用。"
            "to 填对方的称呼，text 填要转告的内容（用第三人称整理好，比如「晚上七点开会」）。"
        ),
        parameters={
            "type": "object",
            "properties": {
                "to": {"type": "string", "description": "收留言的人的称呼。"},
                "text": {"type": "string", "description": "要转告的内容。"},
            },
            "required": ["to", "text"],
        },
        handler=leave_message,
    ))

    registry.register(Tool(
        name="forget_person",
        group="identity",
        description=(
            "删除某人的档案和人脸记录，之后就不再认得 TA。"
            "用户说「忘了我吧」「把我的信息删掉」「别记得我」时调用，name 填「我」；"
            "说「把张三忘了」时 name 填「张三」。"
        ),
        parameters={
            "type": "object",
            "properties": {"name": {"type": "string", "description": "要忘掉的人；说自己就填「我」。"}},
            "required": ["name"],
        },
        handler=forget_person,
    ))


def restrict_to_owner(registry: ToolRegistry, identity, groups: set[str]) -> int:
    """把这些 group 的工具改成「只有认出主人才执行」。返回包了几个。

    默认不开（config identity.restrict_to_owner=false）：认人有失败率，
    开了之后主人背对摄像头就叫不动车。真要开，先确认 identity.owners 配对了。
    """
    n = 0
    for name, tool in list(registry._tools.items()):  # noqa: SLF001 —— 同包内的受控改写
        if tool.group not in groups:
            continue
        inner = tool.handler

        def _guarded(_inner=inner, _name=name, **kwargs):
            if not identity.is_owner():
                logger.info(f"身份：{_name} 被拦（当前说话人 {identity.current_pid} 不是主人）")
                return "这个操作只听主人的；我现在没认出你是主人（可以正对摄像头再说一次）。"
            return _inner(**kwargs)

        registry._tools[name] = Tool(name=tool.name, description=tool.description,
                                     parameters=tool.parameters, handler=_guarded,
                                     group=tool.group, speaks=tool.speaks)
        n += 1
    return n
