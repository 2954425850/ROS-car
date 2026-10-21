"""一轮对话结束后，从对话里提取「关于说话人本人」的信息写进档案。

不让主对话模型边聊边调工具记东西：那样每轮都多一次工具往返、还会让它
"表演记忆"（"好的我记住了！"）。这里是**后台**、**不出声**的一次补全：
拿这一轮的问答 + 已有档案，让模型只回一段 JSON。

    {"name": "张三" | null, "facts": {"喜欢的歌手": "周杰伦"}}

## 规矩（写进提示词，也在代码里兜底）
- 只记**说话人自己**的事；他说"我同学叫李四"不能把李四当他的名字。
- 名字只在他**明确自称**时给（"我叫张三""叫我小张就行"）。
- 已有的不重复；没有新信息就 `{"name": null, "facts": {}}`。
- 不记密码、证件号、住址门牌这类敏感内容。
"""

from __future__ import annotations

import json
import re

from loguru import logger

_SYSTEM = """你是对话记录员。根据【最近一轮对话】，提取关于【说话人本人】的新信息，只输出一个 JSON 对象，不要任何别的文字。
格式：{"name": 称呼或null, "facts": {"简短的键": "简短的值"}}
规则：
1. 只记说话人自己的情况（偏好、职业、习惯、正在做的事、对助手的称呼要求等）；他提到的其他人的事不记。
2. name 只在说话人明确自称时给出（如"我叫张三""叫我小张"），否则为 null。
3. 已在【已知档案】里的信息不要重复；没有新信息就输出 {"name": null, "facts": {}}。
4. 不记录密码、证件号、银行卡、详细住址等敏感信息。
5. 键不超过 8 个字，值不超过 30 个字，最多 3 条。"""

_SENSITIVE = re.compile(r"密码|身份证|银行卡|卡号|验证码|住址|门牌")


def build_messages(user_text: str, reply: str, profile: dict | None) -> list[dict]:
    known = {}
    if profile:
        if profile.get("name"):
            known["称呼"] = profile["name"]
        known.update(profile.get("facts") or {})
    content = (f"【已知档案】{json.dumps(known, ensure_ascii=False) if known else '（空）'}\n"
               f"【最近一轮对话】\n说话人：{user_text}\n助手：{reply}")
    return [{"role": "system", "content": _SYSTEM}, {"role": "user", "content": content}]


def parse_extraction(text: str) -> tuple[str | None, dict]:
    """模型输出 → (name, facts)。容忍代码块包裹/前后废话；坏了就当没提取到。"""
    m = re.search(r"\{.*\}", text or "", re.S)
    if not m:
        return None, {}
    try:
        d = json.loads(m.group(0))
    except ValueError:
        return None, {}
    if not isinstance(d, dict):
        return None, {}
    name = d.get("name")
    name = name.strip()[:16] if isinstance(name, str) and name.strip() else None
    if name and (name in ("我", "你", "助手", "null", "None") or _SENSITIVE.search(name)):
        name = None
    facts = {}
    raw = d.get("facts")
    if isinstance(raw, dict):
        for k, v in list(raw.items())[:3]:
            k, v = str(k).strip()[:8], str(v).strip()[:30]
            if k and v and not _SENSITIVE.search(k + v):
                facts[k] = v
    return name, facts


def extract(llm, user_text: str, reply: str, profile: dict | None) -> tuple[str | None, dict]:
    """调一次无工具补全。任何异常都吞掉返回空 —— 记忆是锦上添花，不能影响对话。"""
    try:
        out = llm.complete(build_messages(user_text, reply, profile), max_tokens=200)
    except Exception as e:  # noqa: BLE001
        logger.warning(f"记忆提取：调模型失败 —— {e}")
        return None, {}
    return parse_extraction(out)
