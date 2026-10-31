"""机械臂语音工具的 schema（纯数据，不 import 任何运行时依赖）。

两种候选形状，由 Task 1 的评测（`scripts/eval_arm_tools.py`）定稿：

- **A（拆分）**：arm_look（toward/turn/speed）+ arm_move（shift/approach/gripper/oscillate/steps/speed）
  + arm_state + arm_home
- **B（合一）**：arm_act（设计 §5 的 spec；单步时字段直接平铺在顶层，先后多步才用 steps）
  + arm_state + arm_home

字段与枚举严格按设计 `docs/plans/2026-10-31-arm-voice-motion-design.md` §5/§5.1。
「何时调用 / 何时不要」全部写在 description 里 —— **不改 system_prompt**
（记忆 prompt-no-capability-emphasis）。
"""

from __future__ import annotations

import copy
from typing import Any

# ---- 枚举（设计 §5 / §5.1）----

TOWARD = ["前方", "左边", "右边", "左前", "右前", "上方", "地上", "脚下", "地上远处", "后方", "上一个姿态"]
TURN_DIRS = ["上", "下", "左", "右"]
SHIFT_DIRS = ["前", "后", "左", "右", "上", "下"]
APPROACH_DIRS = ["靠近", "远离"]
AMOUNTS = ["一点", "一些", "很多", "到底"]
OSC_AMOUNTS = ["一点", "一些", "很多"]          # oscillate 没有「到底」（§5.1 表里是 —）
GRIPPER = ["张开", "合上", "半开"]
OSC_AXES = ["上下", "左右", "夹爪"]
SPEEDS = ["慢", "正常", "快"]

LOOK_FIELDS = ("toward", "turn")
MOVE_FIELDS = ("shift", "approach", "gripper", "oscillate")
STEP_FIELDS = LOOK_FIELDS + MOVE_FIELDS

# ---- 子 schema ----

_AMOUNT = {
    "type": "string",
    "enum": AMOUNTS,
    "description": "程度，照用户原话挑：「一点/稍微/一下/一丢丢」=一点，「一些/点儿/看看」=一些，"
                   "「很多/大幅」=很多，「到底/到头/最低/最高」=到底。用户说了具体数字就不填它。",
}

_TOWARD = {
    "type": "string",
    "enum": TOWARD,
    "description": "整体看向某个方向（绝对方向，不管现在朝哪）。「左边看看」=左边，「看看地上」=地上，"
                   "「看脚下」=脚下，「看前面」=前方，「抬头看天/看上面」=上方，「往后看」=后方。"
                   "「回到刚才那样/回去刚才的位置」=上一个姿态。",
}

_TURN = {
    "type": "array",
    "description": "在**当前**视线基础上再转一点（相对量）。「低下头」「抬头」「往右瞄一下」「再低点」"
                   "「再往左一点」都用它。往左下看就放两项：左 + 下。",
    "items": {
        "type": "object",
        "properties": {
            "dir": {"type": "string", "enum": TURN_DIRS, "description": "往哪转：上=抬头，下=低头。"},
            "amount": _AMOUNT,
            "degrees": {"type": "number", "description": "用户说了度数才填（「左转 30 度看看」填 30），不要自己换算。"},
        },
        "required": ["dir"],
    },
}

_SHIFT = {
    "type": "array",
    "description": "整只爪子平移，视线角度不变。「往前伸」=前，「收回来/往回缩」=后，"
                   "「举高/抬高」=上，「放低/往下放」=下，「往左挪」=左。",
    "items": {
        "type": "object",
        "properties": {
            "dir": {"type": "string", "enum": SHIFT_DIRS},
            "amount": _AMOUNT,
            "cm": {"type": "number", "description": "用户说了厘米数才填（「往前伸 5 厘米」填 5）。"},
        },
        "required": ["dir"],
    },
}

_APPROACH = {
    "type": "object",
    "description": "沿视线方向凑近 / 退远（「凑近点看」「靠近看看」=靠近，「离远点」=远离）。",
    "properties": {
        "dir": {"type": "string", "enum": APPROACH_DIRS},
        "amount": _AMOUNT,
        "cm": {"type": "number", "description": "用户说了厘米数才填。"},
    },
    "required": ["dir"],
}

_GRIPPER = {
    "type": "string",
    "enum": GRIPPER,
    "description": "夹爪：「张开/松开/打开爪子」=张开，「合上/闭上/夹住」=合上，「张一半」=半开。",
}

_OSCILLATE = {
    "type": "object",
    "description": "来回动几下再回原位：「点点头」=上下，「摇摇头」=左右，「爪子开合几下」=夹爪。"
                   "**不能和其它字段放在同一步**。",
    "properties": {
        "axis": {"type": "string", "enum": OSC_AXES},
        "amount": {"type": "string", "enum": OSC_AMOUNTS, "description": "幅度，没说就不填。"},
        "times": {"type": "integer", "minimum": 1, "maximum": 10,
                  "description": "来回几次。用户说了次数就填，没说不填（默认 3）。"},
    },
    "required": ["axis"],
}

_SPEED = {
    "type": "string",
    "enum": SPEEDS,
    "description": "用户说「慢慢地/轻一点」=慢，「快点」=快；没提就不填。",
}

_REPEAT = {"type": "integer", "minimum": 1, "maximum": 10,
           "description": "整串动作重复几遍，没说不填（默认 1）。"}


def _step_schema() -> dict[str, Any]:
    return {
        "type": "object",
        "description": "一步。键只能是 toward/turn/shift/approach/gripper/oscillate；dir、amount、degrees、cm "
                       "要写在 turn/shift/approach **里面**，不能直接写在步上。同一步里的字段合成一个姿态；"
                       "oscillate 必须单独成一步。",
        "properties": {
            "toward": _TOWARD,
            "turn": _TURN,
            "shift": _SHIFT,
            "approach": _APPROACH,
            "gripper": _GRIPPER,
            "oscillate": _OSCILLATE,
        },
    }


# ---- 共用的分界说明（何时**不要**调）----

_NOT_CAR = (
    "⚠️ 车身不动。单说「左转」「右转」「掉个头」「转个圈」「往前走」「后退」是让**车**动，用 car_turn / car_move，"
    "不是本工具；话里带「看/瞄/瞅/头/伸/爪子」才是机械臂（「左转 30 度看看」是臂，「左转 30 度」是车）。"
)
_NOT_OTHERS = (
    "要把东西抓起来用 grasp_object；要一直盯着/跟着某个人或东西用 follow_person / track_object；"
    "只想知道画面里有什么用 look。"
)


def _fn(name: str, description: str, properties: dict[str, Any], required: list[str]) -> dict[str, Any]:
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": {"type": "object", "properties": properties, "required": required},
        },
    }


ARM_STATE = _fn(
    "arm_state",
    "查机械臂现在的姿势：镜头朝哪看、爪子在哪、夹爪开着还是合着。"
    "用户问「手臂现在什么姿势」「你在看哪」「爪子开着吗」时调用。"
    "动作工具的返回里已经带了动完的状态，动完不用再查。",
    {}, [],
)

ARM_HOME = _fn(
    "arm_home",
    "机械臂回到初始收好的位置（复位，夹爪保持原样）。用户说「复位」「回原位」「手收好」「归位」时调用。"
    "只是「回到刚才那样」不是复位，那是看向「上一个姿态」。",
    {}, [],
)

# ---- A：拆分 ----

ARM_LOOK = _fn(
    "arm_look",
    "转动机械臂上的摄像头**换个方向看**（像转头）。用户说「左边看看」「往右瞄一眼」「低下头」「抬头」"
    "「看看地上」「看前面」「往左下看」「再低点」「再往左一点」「回到刚才那样」时调用。"
    "**低头、抬头、往哪边看一律用本工具**（arm_move 没有 turn/toward 参数）。"
    "toward = 整体看向某个方向；turn = 在当前视线上再转一点（相对量，「再…一点」用它）。"
    "程度照用户原话选 amount；说了度数才填 degrees，不要自己换算。"
    + _NOT_CAR + _NOT_OTHERS,
    {"toward": _TOWARD, "turn": _TURN, "speed": _SPEED},
    [],
)

ARM_MOVE = _fn(
    "arm_move",
    "让机械臂**伸、缩、抬、放、开合夹爪、点头摇头**。用户说「往前伸点」「收回来」「举高点」「放低点」"
    "「凑近点看」「张开爪子」「合上」「点点头」「摇摇头」时调用。"
    "shift = 整只爪子平移；approach = 沿视线凑近/退远；gripper = 夹爪；oscillate = 来回几下再回原位。"
    "单纯低头、抬头、往哪边看用 arm_look，本工具顶层**没有** turn/toward。"
    "只有一句话里要**先后**做好几件事（「先低头再张开爪子」）才用 steps，每件一步，"
    "步里可以写 toward/turn（看向）。"
    + _NOT_CAR + _NOT_OTHERS,
    {
        "shift": _SHIFT,
        "approach": _APPROACH,
        "gripper": _GRIPPER,
        "oscillate": _OSCILLATE,
        "steps": {"type": "array", "description": "按顺序执行的多步；只有一件事时不用它。",
                  "items": _step_schema()},
        "repeat": _REPEAT,
        "speed": _SPEED,
    },
    [],
)

# ---- B：合一 ----

ARM_ACT = _fn(
    "arm_act",
    "动小车上的**机械臂**（摄像头装在爪子上，臂指哪镜头就看哪）："
    "换方向看（「左边看看」「往右瞄一眼」「低下头」「抬头」「看看地上」「看前面」「再低点」「回到刚才那样」）、"
    "伸缩抬放（「往前伸点」「收回来」「举高点」「凑近点看」）、"
    "开合夹爪（「张开爪子」「合上」）、来回动（「点点头」「摇摇头」）时调用。"
    "toward = 整体看向某方向；turn = 在当前视线上再转一点（相对量，「再…一点」用它）；"
    "shift = 爪子平移；approach = 沿视线凑近/退远；gripper = 夹爪；oscillate = 来回几下再回原位。"
    "一件事（或几项同时做，如「低头凑近点看」）直接填顶层字段；"
    "一句话里要**先后**做几件事（「先低头再张开爪子」）才用 steps，每件一步，steps 的每步里写同样这些字段。"
    "oscillate 不能和别的字段同时填。"
    "程度照用户原话选 amount；说了度数/厘米才填 degrees/cm，不要自己换算。"
    + _NOT_CAR + _NOT_OTHERS,
    {
        "toward": _TOWARD,
        "turn": _TURN,
        "shift": _SHIFT,
        "approach": _APPROACH,
        "gripper": _GRIPPER,
        "oscillate": _OSCILLATE,
        "steps": {"type": "array", "description": "有先后顺序的多步；只有一件事时不用它。",
                  "items": _step_schema()},
        "repeat": _REPEAT,
        "speed": _SPEED,
    },
    [],
)

SCHEMAS_A: list[dict[str, Any]] = [ARM_LOOK, ARM_MOVE, ARM_STATE, ARM_HOME]
SCHEMAS_B: list[dict[str, Any]] = [ARM_ACT, ARM_STATE, ARM_HOME]

#: 会让臂动起来的工具名（评测里算"车/臂混淆"用）
MOTION_TOOLS = {"arm_look", "arm_move", "arm_act", "arm_home"}


def schemas(variant: str) -> list[dict[str, Any]]:
    """取一份候选形状（深拷贝，调用方随便改）。"""
    v = variant.upper()
    if v == "A":
        return copy.deepcopy(SCHEMAS_A)
    if v == "B":
        return copy.deepcopy(SCHEMAS_B)
    raise ValueError(f"未知变体: {variant}")


def to_spec(name: str, args: dict[str, Any]) -> dict[str, Any] | None:
    """把一次臂动作工具调用还原成设计 §5 的 spec（{steps, repeat?, speed?}）。

    非动作工具（arm_state 等）返回 None。参数形状不合法时抛 ValueError。
    """
    args = dict(args or {})
    spec: dict[str, Any] = {}
    if args.get("speed") is not None:
        spec["speed"] = args["speed"]
    if args.get("repeat") is not None:
        spec["repeat"] = args["repeat"]

    if name in ("arm_act", "arm_move"):
        fields = STEP_FIELDS if name == "arm_act" else MOVE_FIELDS
        step = {k: args[k] for k in fields if args.get(k) not in (None, [], {}, "")}
        steps = args.get("steps") or []
        if not isinstance(steps, list):
            raise ValueError(f"{name} 的 steps 应为数组")
        spec["steps"] = ([step] if step else []) + list(steps)
        return spec
    if name == "arm_look":
        step = {k: args[k] for k in LOOK_FIELDS if args.get(k) not in (None, [], {}, "")}
        spec["steps"] = [step]
        return spec
    return None


def from_spec(variant: str, spec: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    """spec → 该变体下的工具调用列表（评测里把多轮历史渲染成变体自己的形状用）。"""
    v = variant.upper()
    extra = {k: spec[k] for k in ("speed", "repeat") if k in spec}
    steps = spec["steps"]
    if v == "B":
        if len(steps) == 1:
            return [("arm_act", {**steps[0], **extra})]
        return [("arm_act", {"steps": steps, **extra})]
    if len(steps) == 1:
        step = steps[0]
        if set(step) <= set(LOOK_FIELDS) and "repeat" not in extra:
            return [("arm_look", {**step, **extra})]
        if set(step) <= set(MOVE_FIELDS):
            return [("arm_move", {**step, **extra})]
    return [("arm_move", {"steps": steps, **extra})]
