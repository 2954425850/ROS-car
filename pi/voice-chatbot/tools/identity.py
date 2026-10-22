"""身份相关工具：记住我、查档案、留言、忘掉某人。

「何时用」照例只写在 description 里，不往 system_prompt 里点名能力。
自动认人/记偏好仍然是 IdentityManager 在后台做的；`remember_person` 是
**用户明确要求时**的显式入口 —— 它会当场录脸并如实报告成功/失败。
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

    def remember_person(name: str = "") -> str:
        logger.info(f"身份：用户要求记住当前的人（称呼={name!r}）")
        return identity.remember(name)

    def list_known_people() -> str:
        return identity.people_summary()

    def forget_person(name: str) -> str:
        pid = identity.resolve_name(name)
        if pid is None:
            return "我不知道要忘掉的是谁（认不出你，或档案里没有这个称呼）。"
        if pid != identity.current_pid and not identity.is_owner():
            return "只能删除你自己的档案；要删别人的需要主人来说。"
        return identity.forget(pid)

    registry.register(Tool(
        name="remember_person",
        group="identity",
        description=(
            "把**现在正对着摄像头说话的人**记下来（人脸 + 称呼），以后就能认出来。"
            "用户说「记住我」「记住我，我叫高岩」「以后你要认得我」「把我存进档案」时调用。"
            "name 填用户自报的称呼（如「高岩」）；用户没说名字就留空。"
            "⚠️ 会让对方对着镜头采集几秒，成功/失败都会如实返回。"
        ),
        parameters={
            "type": "object",
            "properties": {
                "name": {"type": "string",
                         "description": "用户自报的称呼；没说就留空。"},
            },
            "required": [],
        },
        handler=remember_person,
    ))

    registry.register(Tool(
        name="list_known_people",
        group="identity",
        description=(
            "查看本机保存了哪些人的档案（称呼、见过几次、记下的事）。"
            "用户问「你都记得谁」「都有谁的档案」「你记得我什么」「有没有我的档案」时调用。"
        ),
        parameters={"type": "object", "properties": {}, "required": []},
        handler=list_known_people,
    ))

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
