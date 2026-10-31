# -*- coding: utf-8 -*-
"""机械臂语音动作：配置（档位表/速度档/命名方向）与 spec 解析。纯数据 + 校验，不 import rclpy。

spec 形状见设计 `pi/voice-chatbot/docs/plans/2026-10-31-arm-voice-motion-design.md` §5，
由语音侧 `tools/arm_schema.to_spec` 产出：

    {"steps": [{toward?, turn?, shift?, approach?, gripper?, oscillate?}, ...],
     "repeat"?: int, "speed"?: "慢|正常|快"}

这里只做**结构与枚举校验**，不量化（量化在 `motion_plan`，用 `resolve_amount` 查档位表）。
不合法一律抛 `SpecError(中文原因)`，由服务端回 `BAD_GOAL`。
"""
import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ---- 枚举（与 voice-chatbot/tools/arm_schema.py 逐字一致）----
TOWARD_PREV = '上一个姿态'
TOWARD_NAMES = ('前方', '左边', '右边', '左前', '右前', '上方', '地上', '脚下', '地上远处', '后方')
TURN_DIRS = ('上', '下', '左', '右')
SHIFT_DIRS = ('前', '后', '左', '右', '上', '下')
APPROACH_DIRS = ('靠近', '远离')
AMOUNTS = ('一点', '一些', '很多', '到底')
OSC_AMOUNTS = ('一点', '一些', '很多')
GRIPPER_STATES = ('张开', '合上', '半开')
OSC_AXES = ('上下', '左右', '夹爪')
SPEEDS = ('慢', '正常', '快')

STEP_KEYS = ('toward', 'turn', 'shift', 'approach', 'gripper', 'oscillate')
SPEC_KEYS = ('steps', 'repeat', 'speed')
MAX_TIMES = 10

MAX = 'max'     # resolve_amount 对「到底」的返回值


class SpecError(ValueError):
    """spec 不合法；str(e) 是给人看的中文原因。"""


# --------------------------------------------------------------------------- 配置

@dataclass
class MotionConfig:
    base_limit_deg: float
    el_range_deg: Tuple[float, float]
    min_frame_ms: int
    speeds: Dict[str, float]
    default_speed: str
    oscillate_speed: str
    amounts: Dict[str, Dict[str, float]]
    default_amount: str
    oscillate_times: int
    max_travel_cm: float
    directions: Dict[str, Tuple[float, float]]
    gripper: Dict[str, float]


def _src_config_path():
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'motion.yaml')


def default_config_path():
    """装好的 share/arm_grasp/config/motion.yaml；找不到（没装/没 ROS）就退回源码树 config/。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('arm_grasp'), 'config', 'motion.yaml')
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return _src_config_path()


def load_config(path=None):
    """读 YAML → MotionConfig，并校验它覆盖了 spec 里所有枚举（缺了直接报错，别等到运行时）。"""
    import yaml
    path = path or default_config_path()
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    try:
        cfg = MotionConfig(
            base_limit_deg=float(raw['base_limit_deg']),
            el_range_deg=(float(raw['el_range_deg'][0]), float(raw['el_range_deg'][1])),
            min_frame_ms=int(raw['min_frame_ms']),
            speeds={k: float(v) for k, v in raw['speeds'].items()},
            default_speed=str(raw['default_speed']),
            oscillate_speed=str(raw['oscillate_speed']),
            amounts={kind: {k: float(v) for k, v in tbl.items()}
                     for kind, tbl in raw['amounts'].items()},
            default_amount=str(raw['default_amount']),
            oscillate_times=int(raw['oscillate_times']),
            max_travel_cm=float(raw['max_travel_cm']),
            directions={k: (float(v[0]), float(v[1])) for k, v in raw['directions'].items()},
            gripper={k: float(v) for k, v in raw['gripper'].items()},
        )
    except (KeyError, TypeError, IndexError, ValueError) as e:
        raise ValueError('motion.yaml 格式不对（%s）: %r' % (path, e))
    missing = [n for n in TOWARD_NAMES if n not in cfg.directions]
    missing += [s for s in SPEEDS if s not in cfg.speeds]
    missing += [g for g in GRIPPER_STATES if g not in cfg.gripper]
    for kind, names in (('turn', AMOUNTS[:3]), ('shift', AMOUNTS[:3]),
                        ('approach', AMOUNTS[:3]), ('oscillate', OSC_AMOUNTS)):
        missing += ['%s.%s' % (kind, a) for a in names if a not in cfg.amounts.get(kind, {})]
    if cfg.default_speed not in SPEEDS or cfg.oscillate_speed not in SPEEDS:
        missing.append('default_speed/oscillate_speed')
    if cfg.default_amount not in OSC_AMOUNTS:
        missing.append('default_amount')
    if missing:
        raise ValueError('motion.yaml 缺少条目（%s）: %s' % (path, ', '.join(missing)))
    return cfg


def resolve_amount(kind, amount, number, cfg):
    """程度 → 数值。kind ∈ turn/shift/approach/oscillate。

    给了数（degrees/cm）就原样用；「到底」返回 'max'；都没给用 cfg.default_amount。
    """
    if number is not None:
        return float(number)
    a = amount if amount is not None else cfg.default_amount
    if a == '到底':
        if kind == 'oscillate':
            raise SpecError('来回动的幅度不能是「到底」')
        return MAX
    try:
        return cfg.amounts[kind][a]
    except KeyError:
        raise SpecError('未知程度「%s」（%s）' % (a, kind))


# --------------------------------------------------------------------------- spec

@dataclass
class Move:
    dir: str
    amount: Optional[str] = None
    number: Optional[float] = None      # turn 的 degrees / shift、approach 的 cm


@dataclass
class Oscillate:
    axis: str
    amount: Optional[str] = None
    times: Optional[int] = None


@dataclass
class Step:
    toward: Optional[str] = None
    turn: List[Move] = field(default_factory=list)
    shift: List[Move] = field(default_factory=list)
    approach: Optional[Move] = None
    gripper: Optional[str] = None
    oscillate: Optional[Oscillate] = None


@dataclass
class Spec:
    steps: List[Step]
    repeat: int = 1
    speed: Optional[str] = None


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _enum(v, allowed, what):
    if v not in allowed:
        raise SpecError('未知%s「%s」，可选：%s' % (what, v, '/'.join(allowed)))
    return v


def _move(obj, dirs, num_key, what):
    if not isinstance(obj, dict):
        raise SpecError('%s 的每一项应是对象，收到 %r' % (what, obj))
    extra = set(obj) - {'dir', 'amount', num_key}
    if extra:
        raise SpecError('%s 里有不认识的键：%s' % (what, ', '.join(sorted(extra))))
    if 'dir' not in obj:
        raise SpecError('%s 缺少 dir' % what)
    d = _enum(obj['dir'], dirs, '%s方向' % what)
    amount = obj.get('amount')
    if amount is not None:
        _enum(amount, AMOUNTS, '程度')
    number = obj.get(num_key)
    if number is not None:
        if not _is_num(number) or number <= 0:
            raise SpecError('%s 的 %s 应是正数，收到 %r' % (what, num_key, number))
        number = float(number)
    return Move(d, amount, number)


def _move_list(v, dirs, num_key, what):
    if isinstance(v, dict):            # 宽松：单个对象当成一项
        v = [v]
    if not isinstance(v, list) or not v:
        raise SpecError('%s 应是非空数组' % what)
    return [_move(o, dirs, num_key, what) for o in v]


def _oscillate(v):
    if not isinstance(v, dict):
        raise SpecError('oscillate 应是对象')
    extra = set(v) - {'axis', 'amount', 'times'}
    if extra:
        raise SpecError('oscillate 里有不认识的键：%s' % ', '.join(sorted(extra)))
    if 'axis' not in v:
        raise SpecError('oscillate 缺少 axis')
    axis = _enum(v['axis'], OSC_AXES, '来回方向')
    amount = v.get('amount')
    if amount is not None:
        _enum(amount, OSC_AMOUNTS, '来回幅度')
    times = v.get('times')
    if times is not None and (not _is_int(times) or not 1 <= times <= MAX_TIMES):
        raise SpecError('oscillate.times 应是 1~%d 的整数，收到 %r' % (MAX_TIMES, times))
    return Oscillate(axis, amount, times)


def _step(obj, i):
    if not isinstance(obj, dict):
        raise SpecError('第 %d 步应是对象' % (i + 1))
    extra = set(obj) - set(STEP_KEYS)
    if extra:
        raise SpecError('第 %d 步有不认识的键：%s（dir/amount/cm 要写在 turn/shift/approach 里面）'
                        % (i + 1, ', '.join(sorted(extra))))
    present = [k for k in STEP_KEYS if obj.get(k) not in (None, [], {}, '')]
    if not present:
        raise SpecError('第 %d 步是空的' % (i + 1))
    if 'oscillate' in present and len(present) > 1:
        raise SpecError('第 %d 步：oscillate（来回动）不能和其它动作放在同一步' % (i + 1))
    st = Step()
    if 'toward' in present:
        st.toward = _enum(obj['toward'], TOWARD_NAMES + (TOWARD_PREV,), '方向')
    if 'turn' in present:
        st.turn = _move_list(obj['turn'], TURN_DIRS, 'degrees', 'turn')
    if 'shift' in present:
        st.shift = _move_list(obj['shift'], SHIFT_DIRS, 'cm', 'shift')
    if 'approach' in present:
        st.approach = _move(obj['approach'], APPROACH_DIRS, 'cm', 'approach')
    if 'gripper' in present:
        st.gripper = _enum(obj['gripper'], GRIPPER_STATES, '夹爪动作')
    if 'oscillate' in present:
        st.oscillate = _oscillate(obj['oscillate'])
    return st


def parse_spec(src):
    """spec（JSON 字符串或已解析的 dict）→ Spec。不合法抛 SpecError。"""
    if isinstance(src, (str, bytes)):
        try:
            obj = json.loads(src)
        except ValueError as e:
            raise SpecError('spec 不是合法 JSON：%s' % e)
    else:
        obj = src
    if not isinstance(obj, dict):
        raise SpecError('spec 应是 JSON 对象')
    extra = set(obj) - set(SPEC_KEYS)
    if extra:
        raise SpecError('spec 顶层有不认识的键：%s' % ', '.join(sorted(extra)))
    steps = obj.get('steps')
    if not isinstance(steps, list) or not steps:
        raise SpecError('spec 没有任何动作（steps 为空）')
    repeat = obj.get('repeat', 1)
    if repeat is None:
        repeat = 1
    if not _is_int(repeat) or not 1 <= repeat <= MAX_TIMES:
        raise SpecError('repeat 应是 1~%d 的整数，收到 %r' % (MAX_TIMES, repeat))
    speed = obj.get('speed')
    if speed is not None:
        _enum(speed, SPEEDS, '速度')
    return Spec([_step(s, i) for i, s in enumerate(steps)], repeat, speed)
