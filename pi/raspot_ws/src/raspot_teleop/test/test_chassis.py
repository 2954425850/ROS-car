# -*- coding: utf-8 -*-
"""chassis.py（云端 `Gv`/轮盘的逐行移植）的对拍 + 接线测试。

## 这套测试里哪条才算证据
* 对拍（`test_*_matches_js_*`）比的是**我从 bundle 里抄回来的 JS**，两边都是我
  抄的 —— 它只能证明"两段代码行为一致"，**证明不了我抄的那段就是云端在跑的
  那份**。所以必须配着手算锚点（`test_anchor_*`）一起看：锚点是从 JS 原文按
  0.6 / 0.12 / 0.02 这些数**手推**出来的，不经过任何一侧的实现。
* 全绿不构成证据 —— 变异锚点写在文件末尾，改一行必须能变红。

不需要 ROS（`chassis.py` 是纯函数），跑法：
    cd pi/raspot_ws/src && python -m pytest raspot_teleop/test/test_chassis.py -q
"""
import ast
import math
import pathlib

import pytest

from raspot_teleop import chassis

# 与节点 CFG 里那几项一致（test_node_cfg_is_wired_to_chassis 会核对）
CFG = {
    'VX_MAX': 0.50, 'WZ_MAX': 1.50, 'R_MIN': 0.6, 'V_MIN': 0.12,
    'LAT_EPS': 0.02, 'STICK_DEADZONE': 0.12,
    'GEAR_MIN': 0.15, 'GEAR_MAX': 0.50, 'GEAR_STEP': 0.05, 'GEAR_DEFAULT': 0.15,
}
GEARS = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
GRID = [-1.0, -0.9, -0.5, -0.12, -0.03, -0.02, -0.0199,
        0.0, 0.0199, 0.02, 0.03, 0.12, 0.5, 0.9, 1.0]


# --------------------------------------------------------------------------
# JS 原样抄一遍（只供对拍用；原文见 chassis.py 的 docstring）
# --------------------------------------------------------------------------

def _et(e, t, n):
    return max(t, min(n, e))


class _WE:
    """云端 `we` 里跟底盘有关的那几个（值是死的，抄自 bundle）。"""
    latEpsilon = 0.02
    vMin = 0.12
    rMin = 0.6
    maxWz = 1.5
    joystickDeadzone = 0.12


def js_gv(e, t, n):
    s = _et(e, -1, 1)
    o = _et(t, -1, 1)
    if s == 0 and o == 0:
        return {'vx': 0, 'wz': 0}
    r = n * abs(s)
    if abs(o) > _WE.latEpsilon:
        r = max(r, _WE.vMin)
    i = -r if s < 0 else r
    a = _et(i * o / _WE.rMin, -1.5, _WE.maxWz)
    return {'vx': i, 'wz': 0 if a == 0 else a}


def js_pad(x, y, speed):
    """前端 `h(v)` 的数学部分 + store 里的 `Gv(analog.vx, -analog.wz, speed)`。

    两块隔着好几个组件，这里是**接起来**抄的；这个"接法"本身由手算锚点背书。
    """
    m = math.hypot(x, y)
    o = 1 / m if m > 1 else 1
    L, T = x * o, y * o
    if math.hypot(L, T) < _WE.joystickDeadzone:
        L = T = 0
    analog = {'vx': _et(-T, -1, 1), 'wz': _et(L, -1, 1)}
    return js_gv(analog['vx'], -analog['wz'], speed)


# --------------------------------------------------------------------------
# 对拍
# --------------------------------------------------------------------------

@pytest.mark.parametrize('gear', GEARS)
def test_gv_matches_js(gear):
    for fwd in GRID:
        for turn in GRID:
            want = js_gv(fwd, turn, gear)
            assert chassis.gv(fwd, turn, gear, CFG) == pytest.approx(
                (want['vx'], want['wz']), abs=1e-12), f'fwd={fwd} turn={turn} gear={gear}'


@pytest.mark.parametrize('gear', GEARS)
def test_pad_matches_js(gear):
    xy = [(x, y) for x in GRID for y in GRID]
    xy += [(2.0, 0.0), (0.0, -2.0), (0.0, 2.0), (3.0, 4.0), (-0.6, -0.8),
           (0.08, -0.08), (0.05, 0.05), (0.0849, 0.0849), (0.085, 0.085)]
    for x, y in xy:
        want = js_pad(x, y, gear)
        assert chassis.stick_to_drive(x, y, gear, CFG) == pytest.approx(
            (want['vx'], want['wz']), abs=1e-12), f'x={x} y={y} gear={gear}'


# --------------------------------------------------------------------------
# 手算锚点：直接从 JS 原文推，不经过任一实现
# --------------------------------------------------------------------------

def test_anchor_forward_no_turn():
    """s=1,o=0,gear=.5：r=.5，|0|>0.02 不成立，i=+.5，a=.5*0/.6=0 ⇒ wz 归零。"""
    assert chassis.gv(1.0, 0.0, 0.50, CFG) == pytest.approx((0.50, 0.0))


def test_anchor_reverse_no_turn():
    assert chassis.gv(-1.0, 0.0, 0.50, CFG) == pytest.approx((-0.50, 0.0))


def test_anchor_turn_only_creeps_at_vmin():
    """s=0,o=1,gear=.15：r=.15*0=0 → |1|>.02 ⇒ r=max(0,.12)=.12 ⇒ i=+.12；
    a=.12*1/.6=+.2。**只推左右会以 0.12 m/s 前蠕画弧**（旧 PS2 的守卫挡掉了它）。"""
    assert chassis.gv(0.0, 1.0, 0.15, CFG) == pytest.approx((0.12, 0.2))


def test_anchor_forward_arc():
    assert chassis.gv(1.0, 0.5, 0.30, CFG) == pytest.approx((0.30, 0.25))


def test_anchor_reverse_arc_sign():
    """s=-1,o=.5,gear=.4：i=-.4，a=-.4*.5/.6=-1/3。
    倒车时"左"对应 **wz<0**（ROS 里 wz<0 是右转）—— 就是记忆 `l150pro-turn-decisions`
    ② 那条"后退转向反"：云端缺原厂 `if(Move_X<0) Move_Z=-Move_Z`，这里是照抄它。"""
    assert chassis.gv(-1.0, 0.5, 0.40, CFG) == pytest.approx((-0.4, -0.4 * 0.5 / 0.6))


def test_anchor_turn_inside_lateps_never_raises_to_vmin():
    """s=0,o=.02：`|o|>.02` 取严 ⇒ r 仍是 0 ⇒ (0,0)（不是 vMin）。"""
    assert chassis.gv(0.0, 0.02, 0.50, CFG) == (0.0, 0.0)


def test_anchor_wz_clamp_is_unreachable_within_gear_range():
    """档位到顶 0.5 时 |a| 最大 = .5/.6 = .8333 < WZ_MAX=1.5 ⇒ 那个 ±1.5 限幅
    在合法档位区间里**够不着**（云端同理）。写下来是为了以后有人调 WZ_MAX 时
    能一眼看出它是否真的在起作用。"""
    assert chassis.gv(1.0, 1.0, 0.50, CFG) == pytest.approx((0.50, 0.50 / 0.6))
    assert CFG['GEAR_MAX'] / CFG['R_MIN'] < CFG['WZ_MAX']


def test_anchor_pad_center_and_radial_deadzone():
    """死区是**径向**的（判矢量模长），不是逐轴。
    边界是**严格小于**才算死区（`Math.hypot(L,$) < dz`）：模长正好 .12 是活的。"""
    assert chassis.stick_to_drive(0.0, 0.0, 0.15, CFG) == (0.0, 0.0)
    assert chassis.stick_to_drive(0.09, -0.07, 0.50, CFG) == (0.0, 0.0)   # 模长 .114
    assert chassis.stick_to_drive(0.1199, 0.0, 0.50, CFG) == (0.0, 0.0)
    assert chassis.stick_to_drive(0.12, 0.0, 0.50, CFG) != (0.0, 0.0)     # 正好在阈值上 → 活
    # 逐轴判会漏掉的那一类：两个轴各 0.1，逐轴都小于阈值、径向模长 .141 却活着
    assert chassis.stick_to_drive(0.1, 0.1, 0.50, CFG) != (0.0, 0.0)


def test_anchor_pad_up_is_forward_down_is_reverse():
    """轮盘 y **下**为正：上推 y=-1 ⇒ analog.vx=+1；下推 ⇒ -1。"""
    assert chassis.stick_to_drive(0.0, -1.0, 0.50, CFG) == pytest.approx((0.50, 0.0))
    assert chassis.stick_to_drive(0.0, 1.0, 0.50, CFG) == pytest.approx((-0.50, 0.0))


def test_anchor_pad_right_turns_right_left_turns_left():
    """x 右为正 ⇒ `Gv` 的 turn=-x ⇒ x>0 时 wz<0。
    ROS 里 wz>0=逆时针=左，所以**右推 → wz<0 → 右转**。
    ← 这一位就是本次相对旧 PS2（右推 → wz>0）翻过来的地方。"""
    assert chassis.stick_to_drive(1.0, 0.0, 0.15, CFG) == pytest.approx((0.12, -0.2))
    assert chassis.stick_to_drive(-1.0, 0.0, 0.15, CFG) == pytest.approx((0.12, 0.2))


def test_anchor_pad_outside_circle_clipped_by_inverse_radius():
    """圆外只按 `1/|v|` 缩到圆上（`M>1?1/M:1`）：(3,4) 模长 5 ⇒ ×0.2 ⇒ (0.6,0.8)，
    所以它必须与"直接推 (0.6,0.8)"逐位相等。"""
    for gear in GEARS:
        assert (chassis.stick_to_drive(3.0, 4.0, gear, CFG)
                == chassis.stick_to_drive(0.6, 0.8, gear, CFG))


def test_anchor_pad_inside_circle_is_untouched():
    """圆**内**原样传下去 —— 不是"归一化成单位长"：(0.3,0.4) 模长 0.5，必须等于
    直接调 `gv(-0.4, -0.3, ...)`。误写成归一化的实现会把它放大成 (0.6,0.8) ⇒ 红。"""
    for gear in GEARS:
        assert (chassis.stick_to_drive(0.3, 0.4, gear, CFG)
                == chassis.gv(-0.4, -0.3, gear, CFG))
        assert (chassis.stick_to_drive(0.3, 0.4, gear, CFG)
                != chassis.stick_to_drive(0.6, 0.8, gear, CFG))


# --------------------------------------------------------------------------
# PS2 专用律（2026-10-22）：R_MIN=1.0，默认前进，左推量决定半径
# --------------------------------------------------------------------------

CFG_PS2 = dict(CFG, R_MIN=1.0)


@pytest.mark.parametrize('gear', GEARS)
def test_ps2_centered_is_stop(gear):
    """摇杆完全回中 → 停。不能因为"默认向前"就让车一直自己跑。"""
    assert chassis.ps2_drive(0.0, 0.0, gear, CFG_PS2) == (0.0, 0.0)


@pytest.mark.parametrize('gear', GEARS)
def test_ps2_forward_push_defaults_to_gear_without_scaling(gear):
    """前推不改变速度（档位已是上限）；只推左/不推前后时也默认档位速度。"""
    assert chassis.ps2_drive(0.0, -1.0, gear, CFG_PS2) == pytest.approx((gear, 0.0))
    vx, wz = chassis.ps2_drive(-1.0, 0.0, gear, CFG_PS2)
    assert vx == pytest.approx(gear)


@pytest.mark.parametrize('gear', GEARS)
def test_ps2_full_left_is_one_metre_radius(gear):
    """满左推 + 不推前后：vx=gear, wz=gear ⇒ R=vx/wz=1.0 m。"""
    vx, wz = chassis.ps2_drive(-1.0, 0.0, gear, CFG_PS2)
    assert (vx, wz) == pytest.approx((gear, gear))
    assert abs(vx / wz) == pytest.approx(1.0)


@pytest.mark.parametrize('a', [0.25, 0.5, 0.75, 1.0])
def test_ps2_radius_is_stepless_and_inverse(a):
    """R = R_MIN / a：左推多少，半径就连续地变成 1/a；速度不随转向放大。"""
    vx, wz = chassis.ps2_drive(-a, 0.0, 0.40, CFG_PS2)
    assert vx == pytest.approx(0.40)
    assert abs(vx / wz) == pytest.approx(1.0 / a)
    assert abs(wz) == pytest.approx(0.40 * a)


def test_ps2_left_right_sign():
    """左推 → wz>0（左转/逆时针）；右推 → wz<0（右转）。"""
    assert chassis.ps2_drive(-1.0, 0.0, 0.15, CFG_PS2) == pytest.approx((0.15, 0.15))
    assert chassis.ps2_drive(1.0, 0.0, 0.15, CFG_PS2) == pytest.approx((0.15, -0.15))


def test_ps2_lateral_deadzone():
    """左推量逐轴死区：小于 0.12 视为没有转向输入（回中 = 停）；正好 0.12 生效。"""
    assert chassis.ps2_drive(0.1199, 0.0, 0.5, CFG_PS2) == (0.0, 0.0)
    assert chassis.ps2_drive(-0.1199, 0.0, 0.5, CFG_PS2) == (0.0, 0.0)
    # 0.12 刚过死区：按档位前进，右推 → wz<0
    assert chassis.ps2_drive(0.12, 0.0, 0.5, CFG_PS2) == pytest.approx((0.5, -0.06))


def test_ps2_reverse_is_proportional_after_deadzone():
    """后拉：行程中点（y≈0.56）为 0，拉满为 -gear；倒车时转向方向与前进一致。"""
    dz = CFG_PS2['STICK_DEADZONE']
    mid = dz + 0.5 * (1.0 - dz)
    assert chassis.ps2_drive(0.0, mid, 0.50, CFG_PS2) == pytest.approx((0.0, 0.0))
    assert chassis.ps2_drive(0.0, 1.0, 0.50, CFG_PS2) == pytest.approx((-0.50, 0.0))
    # 满左推 + 满倒车：wz 仍 >0（左转），不因倒车翻向
    assert chassis.ps2_drive(-1.0, 1.0, 0.50, CFG_PS2) == pytest.approx((-0.50, 0.50))


@pytest.mark.parametrize('gear', GEARS)
def test_ps2_wheel_speeds_at_min_radius(gear):
    """固件按 ±(W+A)/2=±0.5 m 拆左右；R=1 m、wz=vx ⇒ 左=0.5vx，右=1.5vx。"""
    vx, wz = chassis.ps2_drive(-1.0, 0.0, gear, CFG_PS2)
    v_left = vx - 0.5 * wz
    v_right = vx + 0.5 * wz
    assert (v_left, v_right) == pytest.approx((0.5 * gear, 1.5 * gear))


# --------------------------------------------------------------------------
# 接线：节点那几行常量真的接到了 chassis 上
# --------------------------------------------------------------------------

def _node_cfg():
    """把 `ps2_teleop_node.py` 里的 `CFG = {...}` 抠出来（不 import，避开 rclpy）。

    用 `ast.literal_eval`，所以 CFG 里**不能出现表达式** —— 这也顺带守住
    "CFG 只放常量"这条约定。
    """
    tree = _node_tree()
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'CFG' for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('ps2_teleop_node.py 里没找到 CFG = {...}')


def _node_tree():
    path = (pathlib.Path(__file__).resolve().parents[1]
            / 'raspot_teleop' / 'ps2_teleop_node.py')
    return ast.parse(path.read_text(encoding='utf-8'))


def _self_attrs(node):
    """函数体里被赋值/自增的 `self.xxx` 名字集合。"""
    out = set()
    for n in ast.walk(node):
        if isinstance(n, ast.Assign):
            targets = n.targets
        elif isinstance(n, ast.AugAssign):
            targets = [n.target]
        else:
            continue
        for t in targets:
            if (isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                    and t.value.id == 'self'):
                out.add(t.attr)
    return out


def test_node_cfg_is_wired_to_chassis():
    """钉住"哪两根轴、什么方向、用哪个死区、档位范围"—— 数学对了但接线错了
    一样是错的车。"""
    c = _node_cfg()
    assert (c['AX_PAD_X'], c['PAD_X_DIR']) == (0, 1)     # 左摇杆X；轴正方向=右推
    assert (c['AX_PAD_Y'], c['PAD_Y_DIR']) == (1, 1)     # 左摇杆Y；轴负方向=上推
    assert c['STICK_DEADZONE'] == 0.12                   # 左推量/后拉逐轴死区
    assert c['DEADZONE'] == 0.15                         # 臂的逐轴死区，别被上面那条覆盖
    # ★ 2026-10-22：PS2 改用自有律，最小半径 1.0 m（云端是 0.6，已不用于 PS2）。
    #   V_MIN / LAT_EPS 只留给 chassis.py 里的云端移植版做对拍。
    assert (c['R_MIN'], c['V_MIN'], c['LAT_EPS']) == (1.0, 0.12, 0.02)
    assert (c['VX_MAX'], c['WZ_MAX']) == (0.50, 1.50)
    assert (c['GEAR_MIN'], c['GEAR_MAX'], c['GEAR_STEP'],
            c['GEAR_DEFAULT']) == (0.15, 0.50, 0.05, 0.15)
    # 旧的逐轴模型已经退役，别留半套
    assert 'AX_THROTTLE' not in c and 'AX_STEER' not in c
    assert 'THROTTLE_FLIP' not in c and 'STEER_FLIP' not in c
    # 本文件对拍用的 CFG 必须跟节点一致（R_MIN 除外：云端移植版是 0.6，
    # PS2 现行律是 1.0，两套模型分开测）
    for k in ('VX_MAX', 'WZ_MAX', 'V_MIN', 'LAT_EPS', 'STICK_DEADZONE',
              'GEAR_MIN', 'GEAR_MAX', 'GEAR_STEP', 'GEAR_DEFAULT'):
        assert CFG[k] == c[k], k


def test_node_keeps_arm_state_and_drops_chassis_state():
    """照抄云端只该扫掉**底盘**那套 LPF（`sm_th/sm_st`）；臂的 `sm_sh/sm_bs`
    在那 14 行冻结清单里，必须原样留着 —— 两边都在同一段代码里，很容易一起删。
    节点 import 不了 rclpy，所以用 AST 判"谁被赋值了"，不用字符串。
    """
    cls = next(n for n in ast.walk(_node_tree()) if isinstance(n, ast.ClassDef))
    fns = {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef)}
    init, tick = fns['__init__'], fns['tick']

    assert not ({'sm_th', 'sm_st'} & _self_attrs(init))   # 底盘 LPF 状态已删
    assert not ({'sm_th', 'sm_st'} & _self_attrs(tick))
    assert {'sm_sh', 'sm_bs', 'arm_vel', 'targets', 'base_t'} <= _self_attrs(init)
    assert {'sm_sh', 'sm_bs'} <= _self_attrs(tick)        # 臂的平滑还在跑


def test_node_tick_calls_the_port_exactly_once():
    """底盘必须走 `ps2_drive`，不是又在 tick 里手写一遍公式
    （手写一遍 = 以后改 chassis.py 不再影响车）。"""
    tick = next(f for f in ast.walk(_node_tree())
                if isinstance(f, ast.FunctionDef) and f.name == 'tick')
    calls = [n for n in ast.walk(tick)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
             and n.func.id == 'ps2_drive']
    assert len(calls) == 1
    x, y, gear, c = calls[0].args                        # 形参就该是 ps2_drive(x, y, self.gear, c)
    assert (x.id, y.id, c.id) == ('x', 'y', 'c')
    assert isinstance(gear, ast.Attribute) and gear.attr == 'gear'


# --------------------------------------------------------------------------
# 接线：驱动的底盘限幅（冻结行，2026-10-12 改成读参数）
# --------------------------------------------------------------------------

def _driver_tree():
    path = (pathlib.Path(__file__).resolve().parents[1]
            / 'raspot_teleop' / 'l150pro_driver_node.py')
    return ast.parse(path.read_text(encoding='utf-8'))


def test_driver_clamp_max_vx_is_a_param_with_the_frozen_defaults():
    """`chassis_tick` 里那两行 ±0.5/±1.5 在冻结名单里，2026-10-12 改成读 ROS 参数
    （为了台架上量传动上限）。**默认值必须还是 0.5/1.5**，否则冻结的行为就被改了：
      · `chassis_tick` 里不许再出现写死的 0.5 / 1.5，而且必须真的去读参数
      · `__init__` 里声明 `max_vx=0.5` / `max_wz=1.5`"""
    fns = {f.name: f for f in
           next(n for n in ast.walk(_driver_tree()) if isinstance(n, ast.ClassDef)
                and any(isinstance(b, ast.FunctionDef) and b.name == 'chassis_tick'
                        for b in n.body)).body
           if isinstance(f, ast.FunctionDef)}
    tick = fns['chassis_tick']
    lits = {n.value for n in ast.walk(tick)
            if isinstance(n, ast.Constant) and isinstance(n.value, float)}
    assert not (lits & {0.5, 1.5}), f'chassis_tick 里还有写死的限幅：{lits}'
    assert any(isinstance(n, ast.Attribute) and n.attr == 'get_parameter'
               for n in ast.walk(tick)), 'chassis_tick 没在读限幅参数'
    declared = {}
    for n in ast.walk(fns['__init__']):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter' and len(n.args) == 2
                and all(isinstance(a, ast.Constant) for a in n.args)):
            declared[n.args[0].value] = n.args[1].value
    assert declared.get('max_vx') == 0.5, declared.get('max_vx')
    assert declared.get('max_wz') == 1.5, declared.get('max_wz')


# ==========================================================================
# ★ 变异锚点（2026-10-12 逐条改过、确认能红。全绿不算证据）
#   改 chassis.py / ps2_teleop_node.py 里的**一处**，跑本文件，看它红在哪：
#   1. gv：`if abs(o) > c['LAT_EPS']` 改成 `>=`
#        → test_anchor_turn_inside_lateps_never_raises_to_vmin + test_gv_matches_js
#   2. gv：`r = max(r, c['V_MIN'])` 改成 `pass`（不抬到 vMin 了）
#        → test_anchor_turn_only_creeps_at_vmin + *_turns_right_left_*
#   3. gv：`i = -r if s < 0.0 else r` 的两个符号互换
#        → test_anchor_reverse_no_turn
#   4. gv：`a = clamp(i * o / c['R_MIN'], ...)` 里的 `/` 改 `*`
#        → test_anchor_forward_arc
#   5. stick_to_drive：`clamp(-y, ...)` 与 `-clamp(x, ...)` 里的负号各删一个（左右/前后互换）
#        → test_anchor_pad_up_is_forward_down_is_reverse / *_turns_right_left_*
#   6. stick_to_drive：径向死区判据改成逐轴（`abs(x)<dz and abs(y)<dz` 才算死）
#        → test_anchor_pad_center_and_radial_deadzone 的 (0.1,0.1) 那条
#   7. stick_to_drive：单位圆**内**也归一化（去掉 `if m > 1.0` 这个门）
#        → test_anchor_pad_inside_circle_is_untouched
#   8. 节点 CFG：PAD_X_DIR 改 -1
#   9. 节点 CFG：把 STICK_DEADZONE 又写成 0.15（并让底盘读 DEADZONE）
#        → 8/9 都红在 test_node_cfg_is_wired_to_chassis
#  10. 节点 tick 里又加一行 `self.sm_th = 0.0`（底盘 LPF 复活）
#  11. 节点 `__init__` 里删掉 `self.sm_sh = self.sm_bs = 0.0`（顺手清"没用到的"）
#        → 10/11 都红在 test_node_keeps_arm_state_and_drops_chassis_state
#
#  12. 驱动 `chassis_tick`：限幅改回写死的 `min(max(x, -0.5), 0.5)`
#  13. 驱动 `__init__`：`declare_parameter('max_vx', 0.5)` 的默认值改成 1.8
#        → 12/13 都红在 test_driver_clamp_max_vx_is_a_param_with_the_frozen_defaults
#
#   ★ 一条**等价变异**（故意不上榜）：把 gv 开头 `if s == 0.0 and o == 0.0: return
#     0.0, 0.0` 整段删掉，**测不出来** —— 走主路径时 r = gear*|0| = 0、
#     |o|>latEps 不成立 ⇒ i = 0、a = 0，输出还是 (0.0, 0.0)。它只是省算，
#     不是行为。写在这里免得下次又当作"漏测"去补一条测不出东西的用例。
# ==========================================================================
