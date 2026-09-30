# 小车语音控制（car tools）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 nepu 上的语音助手 JARVIS 加上用嘴控制 L150Pro 小车的 5 个工具，并建立未来导航/建图能复用的底层。

**Architecture:** 三层。模型只看到 5 个粗粒度口语化工具；它们调 `CarController` 的 Python API（未来的导航/建图节点也调同一层，不再新增模型可见工具）；`CarController` 通过唯一的 ROS 适配层 `RosBridge` 收发 `cmd_vel` / `servo_cmd`。纯逻辑（方向合成、限幅、云台换算）拆到 `car/kinematics.py`，可以用假桥在不接车的机器上测。

**Tech Stack:** Python 3.12、ROS 2 Jazzy (`rclpy`)、pytest 7.4.4、loguru、PyYAML

## Global Constraints

- **不改 `system_prompt`。** 加能力只加工具，工具何时该被调用由 `description` 承担。改完必须 `diff` 确认提示词逐字未变。
- **不改 `mcp_host/` 下任何文件。** 小车走原生 `ToolRegistry`，不走 MCP。
- **`car/` 下只有 `ros_bridge.py` 可以 `import rclpy`。** 其余模块必须能在没装 ROS 的机器上 import。
- **`.sh` 文件必须 LF 行尾。** 在树莓派上用 heredoc 直接生成，不要从 Windows 拷回（CRLF 会让 shebang 变成 `bad interpreter: /usr/bin/env bash^M`）。
- 项目根是树莓派上的 `/home/cy/voice-chatbot`，主控机是 `192.168.1.108`（用户 `cy`）。所有操作走 ssh-mcp 工具。
- 驱动的 `max_vx = 0.5` / `max_wz = 1.5`，看门狗 200ms（`l150pro_driver` 默认参数）。
- 云台 **ch5 = 水平、ch6 = 俯仰**，脉宽单位微秒，`1500` 中位，**`0` = 该路保持不动**。
- 参数名 `speed` 在 `car_move` 是 m/s、在 `car_turn` 是度/秒，单位由各自 description 说明。
- **`fs_write` 在本环境无法覆盖已有文件**（返回 `EFS Failure`；新建文件正常）。
  改已有文件用 `proc_exec` 的 heredoc 落盘（`cat > 文件 <<'EOF' ... EOF`），或用 `patch_apply`。
  **不要**因为 `fs_write` 失败就以为目标文件被写坏了 —— 实测失败时原文件完好无损。
- 测试跑法一律用 `cd /home/cy/voice-chatbot && python3 -m pytest -v`。
- **`car/` 里凡是要自旋的地方，必须用自己的 `SingleThreadedExecutor`**，不许裸调 `rclpy.spin_once(node)` —— 那会落到进程级全局 executor，同进程再有一处裸调就撞 `Executor is already spinning`。

---

## Task 0: 开工快照与测试脚手架

项目当前**没有版本控制、也没有测试目录**。后面每个任务都要跑 pytest；而这个计划要动一个正在跑的服务和一个 ROS 驱动包，所以要有一份能退回去的快照。

**Files:**
- Create: `/home/cy/voice-chatbot/pytest.ini`
- Create: `/home/cy/voice-chatbot/tests/test_smoke.py`
- Modify: `/home/cy/voice-chatbot/requirements.txt`

**Interfaces:**
- Consumes: 无
- Produces: 可用的 `pytest` 命令，以及一份可回滚的开工快照

- [ ] **Step 1: 打一份开工快照**

没有版本控制，快照就是唯一的安全网。这个计划动的是**正在跑的服务**和**一个 ROS 驱动包**，
出问题时得有东西能退回去。

```bash
cd /home/cy && tar czf "voice-chatbot.bak-$(date +%Y%m%d-%H%M%S).tar.gz" \
    --exclude='voice-chatbot/logs' --exclude='__pycache__' \
    --exclude='.pytest_cache' voice-chatbot
ls -lh /home/cy/voice-chatbot.bak-*.tar.gz | tail -3
```
Expected: 出现一个带今天时间戳的 `voice-chatbot.bak-YYYYmmdd-HHMMSS.tar.gz`

回滚办法（记下来，别等到需要时才现想）：

```bash
cd /home/cy && rm -rf voice-chatbot && tar xzf voice-chatbot.bak-<那个时间戳>.tar.gz
```

`config.yaml` 刻意**不**排除——配置也要能一起退回去。

- [ ] **Step 2: 写 `pytest.ini`**

```ini
[pytest]
testpaths = tests
python_files = test_*.py
addopts = -q
# 把项目根放进 sys.path，测试里才能 `from car.xxx` / `from tools.xxx`。
# 不加这行也能跑 —— 但前提是必须用 `python3 -m pytest`（-m 会把 cwd 塞进
# sys.path）。加了之后裸 `pytest` 也行，少一个隐式前提。
pythonpath = .
```

- [ ] **Step 3: 写冒烟测试 `tests/test_smoke.py`**

```python
"""脚手架自检：确认 pytest 能发现测试、项目根在 sys.path 上。

注意：**不能**写成 `assert str(_ROOT) in sys.path or _ROOT.exists()` ——
`_ROOT` 是本文件的父目录的父目录，必然存在，`or` 那半边恒真，测试永远不会
失败，等于没测。第一版就是这么写的，被审查抓出来了。
"""

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent


def test_project_root_is_on_sys_path():
    """后续每个测试都靠「从项目根 import」，这里把它钉住。"""
    assert str(_ROOT) in sys.path


def test_can_import_project_modules():
    """光看 sys.path 还不够 —— 真的 import 到项目模块，才说明路走通了。"""
    import tools.registry  # noqa: F401
    import utils.config  # noqa: F401
```

写完**自检这个测试确实会失败**（否则就是又一个空洞断言）：

Run: `cd /home/cy/voice-chatbot && pytest -o pythonpath= -v`
Expected: FAIL —— `ModuleNotFoundError: No module named 'tools'`

- [ ] **Step 4: 把 pytest 加进 `requirements.txt`**

在文件末尾追加：

```
# 测试
pytest
```

- [ ] **Step 5: 跑测试确认脚手架可用**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest -v`
Expected: PASS，2 passed

---

## Task 1: 纯领域层 —— `car/types.py` 与 `car/kinematics.py`

方向合成、限幅、云台角度换算全是纯函数，**不碰 ROS**，所以能在任何机器上快速测。把这一层先做出来，`CarController` 就只剩「循环发帧」这点事了。

**Files:**
- Create: `/home/cy/voice-chatbot/car/__init__.py`
- Create: `/home/cy/voice-chatbot/car/types.py`
- Create: `/home/cy/voice-chatbot/car/kinematics.py`
- Create: `/home/cy/voice-chatbot/tests/test_kinematics.py`

**Interfaces:**
- Consumes: `utils.config.Config`（已有，`config.get("a.b", default)` 点号取值）
- Produces:
  - `car.types.CarLimits` —— frozen dataclass，字段 `default_speed: float`、`max_speed: float`、`default_turn_speed_dps: float`、`max_turn_speed_dps: float`、`camera_step_deg: float`、`camera_min_us: int`、`camera_max_us: int`、`camera_center_us: int`、`cmd_rate_hz: float`；类方法 `CarLimits.from_config(config) -> CarLimits`
  - `car.types.RobotState` —— frozen dataclass，字段 `voltage: float | None`、`fault: int | None`、`wheel_speeds: tuple[float, float, float, float] | None`、`yaw_rate: float | None`、`traveled: float`、`online: bool`
  - `car.types.describe_faults(fault: int | None) -> str`
  - `car.kinematics.DIRECTIONS: dict[str, tuple[int, int]]`（6 个键）
  - `car.kinematics.direction_names() -> list[str]`
  - `car.kinematics.clamp_speed(speed, limits) -> tuple[float, bool]`
  - `car.kinematics.clamp_turn_speed(dps, limits) -> tuple[float, bool]`
  - `car.kinematics.direction_to_velocity(direction, speed, turn_speed_dps) -> tuple[float, float]`
  - `car.kinematics.camera_target_us(direction, degrees, current_us, limits) -> tuple[int, bool]`

- [ ] **Step 1: 写失败的测试 `tests/test_kinematics.py`**

```python
"""纯运动学测试 —— 不碰 ROS、不碰硬件。"""

import math

import pytest

from car.kinematics import (
    DIRECTIONS,
    camera_target_us,
    clamp_speed,
    clamp_turn_speed,
    direction_names,
    direction_to_velocity,
)
from car.types import CarLimits, RobotState, describe_faults

LIMITS = CarLimits()


# ---------------- 方向表 ----------------

def test_move_directions_have_no_pure_turn():
    """car_move 只管带前进分量的方向；纯转向归 car_turn。"""
    assert "left" not in DIRECTIONS
    assert "right" not in DIRECTIONS
    assert set(direction_names()) == {
        "forward", "backward",
        "forward_left", "forward_right",
        "backward_left", "backward_right",
    }


def test_forward_is_positive_vx():
    vx, wz = direction_to_velocity("forward", 0.3, 60.0)
    assert vx == pytest.approx(0.3)
    assert wz == pytest.approx(0.0)


def test_backward_is_negative_vx():
    vx, _ = direction_to_velocity("backward", 0.3, 60.0)
    assert vx == pytest.approx(-0.3)


def test_left_turn_is_positive_wz():
    """REP-103：角速度为正 = 逆时针 = 左转。"""
    _, wz = direction_to_velocity("forward_left", 0.3, 90.0)
    assert wz > 0


def test_arc_turn_is_half_of_plain_turn():
    """弧线的转向速率是原地转向的一半 —— 与 teleop 的 arc_gain 一致。"""
    _, wz_arc = direction_to_velocity("forward_left", 0.3, 90.0)
    expected = 90.0 * 0.5 * math.pi / 180.0
    assert wz_arc == pytest.approx(expected)


def test_direction_to_velocity_rejects_unknown_direction():
    with pytest.raises(KeyError):
        direction_to_velocity("sideways", 0.3, 60.0)


# ---------------- 限幅 ----------------

def test_clamp_speed_passes_through_when_within_limit():
    value, clamped = clamp_speed(0.3, LIMITS)
    assert value == pytest.approx(0.3)
    assert clamped is False


def test_clamp_speed_reports_clamping():
    """超限必须报出来，不能静默截断 —— 否则模型以为车真按 0.8 在跑。"""
    value, clamped = clamp_speed(0.8, LIMITS)
    assert value == pytest.approx(0.5)
    assert clamped is True


def test_clamp_speed_uses_magnitude():
    value, _ = clamp_speed(-0.3, LIMITS)
    assert value == pytest.approx(0.3)


def test_clamp_turn_speed_reports_clamping():
    value, clamped = clamp_turn_speed(999.0, LIMITS)
    assert value == pytest.approx(120.0)
    assert clamped is True


# ---------------- 云台 ----------------

def test_camera_center_goes_to_center_regardless_of_degrees():
    target, hit = camera_target_us("center", 45.0, 2000, LIMITS)
    assert target == 1500
    assert hit is False


def test_camera_right_increases_pulse_width():
    target, _ = camera_target_us("right", None, 1500, LIMITS)
    assert target > 1500


def test_camera_left_decreases_pulse_width():
    target, _ = camera_target_us("left", None, 1500, LIMITS)
    assert target < 1500


def test_camera_default_step_is_symmetric():
    right, _ = camera_target_us("right", None, 1500, LIMITS)
    left, _ = camera_target_us("left", None, 1500, LIMITS)
    assert right - 1500 == 1500 - left


def test_camera_explicit_degrees_overrides_step():
    """说了角度就按说的做：30 度 ≈ 333us，与默认 15 度的步进不同。"""
    explicit, _ = camera_target_us("right", 30.0, 1500, LIMITS)
    default, _ = camera_target_us("right", None, 1500, LIMITS)
    assert explicit != default
    assert explicit - 1500 == pytest.approx(30.0 * 1000.0 / 90.0, abs=1)


def test_camera_up_and_down_map_to_tilt():
    up, _ = camera_target_us("up", None, 1500, LIMITS)
    down, _ = camera_target_us("down", None, 1500, LIMITS)
    assert up > 1500 > down


def test_camera_reports_hitting_the_limit():
    target, hit = camera_target_us("right", 900.0, 1500, LIMITS)
    assert target == LIMITS.camera_max_us
    assert hit is True


def test_camera_rejects_unknown_direction():
    with pytest.raises(KeyError):
        camera_target_us("diagonal", None, 1500, LIMITS)


# ---------------- 配置与故障码 ----------------

def test_car_limits_read_from_config():
    class _FakeConfig:
        def get(self, key, default=None):
            return {"car.max_speed": 0.42}.get(key, default)

    limits = CarLimits.from_config(_FakeConfig())
    assert limits.max_speed == pytest.approx(0.42)
    assert limits.camera_center_us == 1500


def test_describe_faults_normal():
    assert describe_faults(0) == "正常"


def test_describe_faults_names_the_bits():
    assert describe_faults(1 << 0) == "欠压"
    assert "欠压" in describe_faults((1 << 0) | (1 << 4))
    assert "下行超时" in describe_faults((1 << 0) | (1 << 4))


def test_describe_faults_handles_missing_data():
    assert describe_faults(None) == "读不到故障码"


def test_robot_state_is_frozen():
    state = RobotState(voltage=12.0, fault=0, wheel_speeds=None,
                       yaw_rate=0.0, traveled=0.0, online=True)
    with pytest.raises(Exception):
        state.voltage = 11.0
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/test_kinematics.py -v`
Expected: FAIL —— `ModuleNotFoundError: No module named 'car'`

- [ ] **Step 3: 写 `car/__init__.py`**

```python
"""小车控制。见 docs/plans/2026-09-17-car-tools-design.md。

模块边界：
  types.py       —— 纯数据类型与配置解析，无依赖
  kinematics.py  —— 纯函数（方向合成、限幅、云台换算），无依赖
  ros_bridge.py  —— **唯一** import rclpy 的地方
  controller.py  —— 编排：闭环运动、急停、状态播报
"""
```

- [ ] **Step 4: 写 `car/types.py`**

```python
"""小车领域类型。**不 import ROS** —— 这样纯逻辑可以在不接硬件的机器上测。"""

from __future__ import annotations

from dataclasses import dataclass

# 与 l150pro_driver/protocol.py 的 FAULT_* 逐位一致。
# 刻意复制而不是 import：驱动是 ROS 包，import 它会把语音助手
# 绑死在 workspace 的环境上，而这里只需要 6 个整数。
FAULT_NAMES: dict[int, str] = {
    1 << 0: "欠压",
    1 << 1: "过压",
    1 << 2: "急停",
    1 << 3: "IMU故障",
    1 << 4: "下行超时",
    1 << 5: "编码器异常",
}


def describe_faults(fault: int | None) -> str:
    """把 fault 位掩码翻译成能念出来的中文。"""
    if fault is None:
        return "读不到故障码"
    if fault == 0:
        return "正常"
    names = [name for bit, name in FAULT_NAMES.items() if fault & bit]
    return "、".join(names) if names else f"未知({fault:#x})"


@dataclass(frozen=True)
class CarLimits:
    """限幅与默认值。全部来自 config.yaml 的 car: 段，不写死在代码里 ——
    将来标定完成或供电改善，只需要改 yaml。"""

    default_speed: float = 0.3          # m/s，car_move 未指定 speed 时用
    max_speed: float = 0.5              # m/s，与驱动 max_vx 一致
    default_turn_speed_dps: float = 60  # 度/秒，car_turn 未指定 speed 时用
    max_turn_speed_dps: float = 120     # 度/秒
    camera_step_deg: float = 15         # car_camera 未指定 degrees 时的步进
    camera_min_us: int = 800            # 云台行程下限（teleop 的保守值，防堵转扫齿）
    camera_max_us: int = 2200           # 云台行程上限
    camera_center_us: int = 1500        # 中位
    cmd_rate_hz: float = 25.0           # 发帧频率（驱动 cmd_rate_hz=20，留余量）

    @classmethod
    def from_config(cls, config) -> "CarLimits":
        get = config.get
        return cls(
            default_speed=float(get("car.default_speed", 0.3)),
            max_speed=float(get("car.max_speed", 0.5)),
            default_turn_speed_dps=float(get("car.default_turn_speed_dps", 60)),
            max_turn_speed_dps=float(get("car.max_turn_speed_dps", 120)),
            camera_step_deg=float(get("car.camera_step_deg", 15)),
            camera_min_us=int(get("car.camera_min_us", 800)),
            camera_max_us=int(get("car.camera_max_us", 2200)),
            camera_center_us=int(get("car.camera_center_us", 1500)),
            cmd_rate_hz=float(get("car.cmd_rate_hz", 25.0)),
        )


@dataclass(frozen=True)
class RobotState:
    """一帧机器人状态快照。RosBridge 与测试里的 FakeBridge 都产出它。"""

    voltage: float | None                              # V；驱动未发 battery_voltage 时为 None
    fault: int | None                                  # fault 位，见 FAULT_NAMES
    wheel_speeds: tuple[float, float, float, float] | None   # A/B/C/D，m/s
    yaw_rate: float | None                             # rad/s，IMU gz（不积分就不会漂）
    traveled: float                                    # 累计行进距离，m
    online: bool                                       # 是否收到过任何上行数据
```

- [ ] **Step 5: 写 `car/kinematics.py`**

```python
"""小车运动学：方向→速度、云台角度→脉宽。**纯函数，不 import ROS。**

刻意与 CarController 分开：这里每个决策（斜向怎么合成、超限怎么截、云台到没到
限位）都能在没接车的机器上测，而 CarController 要碰 ROS。
"""

from __future__ import annotations

import math

from car.types import CarLimits

# 每个方向 = (前进分量, 转向分量)，各取 -1 / 0 / +1。
# 与 teleop_l150pro.py 的 MOTION_KEYS 一致。
# 角速度为正 = 逆时针 = 左转（REP-103）。
#
# **没有 left / right。** 差速车没有横移，「往左」要么是原地转（car_turn），
# 要么是弧线（forward_left）。「往左走两米」里的「两米」对原地转毫无意义。
DIRECTIONS: dict[str, tuple[int, int]] = {
    "forward": (+1, 0),
    "backward": (-1, 0),
    "forward_left": (+1, +1),
    "forward_right": (+1, -1),
    "backward_left": (-1, +1),
    "backward_right": (-1, -1),
}

# 弧线（同时有前进和转向）时，转向速率相对原地转向的倍数。teleop 用 0.5，照抄。
ARC_TURN_RATIO = 0.5

# 标准舵机：中位 1500us，±90° 对应 ±1000us。
_US_PER_DEG = 1000.0 / 90.0


def direction_names() -> list[str]:
    """给工具 schema 用的方向枚举。

    工具的 JSON Schema 直接用它，**不要再在 tools/ 里抄一份** —— 抄一份会
    静默漂移：schema 是发给模型的，漂了就变成「模型传一个控制器不认的方向」。
    有没有漂，靠 tests/test_car_tools.py 里的同步断言兜。
    """
    return list(DIRECTIONS)


# 云台动作。注意 `center` 是回中位，不是方向。
CAMERA_DIRECTIONS: tuple[str, ...] = ("left", "right", "up", "down", "center")


def camera_direction_names() -> list[str]:
    """给工具 schema 用的云台方向枚举。理由同 direction_names()。"""
    return list(CAMERA_DIRECTIONS)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def clamp_speed(speed: float, limits: CarLimits) -> tuple[float, bool]:
    """返回 (生效速度, 是否被截断)。取绝对值 —— 方向由 direction 决定。"""
    magnitude = abs(float(speed))
    if magnitude > limits.max_speed:
        return limits.max_speed, True
    return magnitude, False


def clamp_turn_speed(dps: float, limits: CarLimits) -> tuple[float, bool]:
    """返回 (生效角速度 度/秒, 是否被截断)。"""
    magnitude = abs(float(dps))
    if magnitude > limits.max_turn_speed_dps:
        return limits.max_turn_speed_dps, True
    return magnitude, False


def direction_to_velocity(
    direction: str, speed: float, turn_speed_dps: float
) -> tuple[float, float]:
    """方向 + 线速度 → (vx m/s, wz rad/s)。未知方向抛 KeyError。"""
    fwd, turn = DIRECTIONS[direction]
    arc = fwd != 0 and turn != 0
    angular_dps = turn_speed_dps * (ARC_TURN_RATIO if arc else 1.0)
    vx = fwd * speed
    wz = turn * angular_dps * math.pi / 180.0
    return vx, wz


def camera_target_us(
    direction: str,
    degrees: float | None,
    current_us: int,
    limits: CarLimits,
) -> tuple[int, bool]:
    """算出云台动作后的目标脉宽。

    direction == "center" 时忽略 degrees，直接回中位。

    Returns:
        (目标脉宽 us, 是否撞到行程限位)

    Raises:
        KeyError: direction 不是 left/right/up/down/center
    """
    if direction == "center":
        return limits.camera_center_us, False

    if direction in ("left", "right"):
        sign = +1 if direction == "right" else -1
    elif direction in ("up", "down"):
        sign = +1 if direction == "up" else -1
    else:
        raise KeyError(direction)

    # 方向是权威：degrees 只取大小，符号由 direction 决定。
    # 这样模型把「左转 30 度」写成 (left, -30) 也不会转到反方向去。
    delta_deg = limits.camera_step_deg if degrees is None else abs(float(degrees))
    target = current_us + int(round(sign * delta_deg * _US_PER_DEG))
    clamped = int(clamp(target, limits.camera_min_us, limits.camera_max_us))
    return clamped, clamped != target
```

- [ ] **Step 6: 跑测试确认通过**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/test_kinematics.py -v`
Expected: PASS，约 22 passed

- [ ] **Step 7: 回归全量测试 + 打快照**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest -v`
Expected: PASS —— 本任务新增用例全绿，前面任务的用例**没有回归**

测试过了再打快照。没有版本控制，这就是唯一的回滚点：

```bash
cd /home/cy && tar czf "voice-chatbot.bak-T1-$(date +%Y%m%d-%H%M%S).tar.gz" \
    --exclude='voice-chatbot/logs' --exclude='__pycache__' \
    --exclude='.pytest_cache' voice-chatbot
```
（Task 0 那份开工快照是总退路，但它退不回「上一个任务刚做完」的状态。）

---

## Task 2: 驱动发布电压

电压已经通到解析层了（`protocol.py` 的 `Uplink.voltage`），只差 `driver_node.py` 没往外发。**不碰固件、不碰协议。**

**Files:**
- Modify: `/home/cy/l150pro_ws/src/l150pro_driver/l150pro_driver/driver_node.py`
- Modify: `/home/cy/l150pro_ws/src/l150pro_driver/README.md`

**Interfaces:**
- Consumes: `Uplink.voltage`（`protocol.py` 已提供）
- Produces: ROS 话题 `/battery_voltage`，类型 `std_msgs/msg/Float32`，单位伏特

- [ ] **Step 1: 先备份，再确认电压已到解析层**

**备份必须在任何编辑之前**（原计划把备份写成了 Step 7，那是错的 —— 编辑都做完了再备份等于没备）：

```bash
cd /home/cy/l150pro_ws/src/l150pro_driver/l150pro_driver && \
cp -p driver_node.py "driver_node.py.bak-volt-$(date +%m%d-%H%M%S)" && \
ls -l driver_node.py.bak-volt-*
```

回滚 = 把 `driver_node.py.bak-volt-*` 拷回 `driver_node.py`，再 `colcon build` 一次。

然后确认电压确实已经通到解析层了：

Run:
```bash
cd /home/cy/l150pro_ws/src/l150pro_driver && \
grep -n "voltage" l150pro_driver/protocol.py | head -5
```
Expected: 看到 `voltage: float          # V` 和 `voltage=voltage / 1000.0`

- [ ] **Step 2: 加发布器**

`driver_node.py` 顶部**已有** `from std_msgs.msg import Int16MultiArray`。
**合并成一行**，别再单开一行 —— 这个文件其它 import 都是合并写法
（`from sensor_msgs.msg import Imu, JointState`）：

```python
from std_msgs.msg import Float32, Int16MultiArray
```

在 `__init__` 里创建发布器（紧挨着 `self.pub_fault` 那几行）：

```python
        self.pub_odom = self.create_publisher(Odometry, "wheel_odom", qos)
        self.pub_imu = self.create_publisher(Imu, "imu/data_raw", qos)
        self.pub_fault = self.create_publisher(Int16MultiArray, "driver_fault", qos)
        # 电压**不受 publish_diagnostics 控制**：它是判断供电是否够用的核心指标，
        # 欠压是这台车最常踩的故障（见 README 的供电一节），应当始终发布。
        self.pub_battery = self.create_publisher(Float32, "battery_voltage", qos)
```

- [ ] **Step 3: 在状态发布里带出去**

把 `_publish_state` 改成：

```python
    def _publish_state(self, up: Uplink):
        # 用【接收时刻】做时间戳，不用固件的 tick——两者时钟不同源。
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(up, stamp)
        self._publish_imu(up, stamp)

        battery = Float32()
        battery.data = float(up.voltage)
        self.pub_battery.publish(battery)

        if self.get_parameter("publish_diagnostics").value:
            self._publish_fault(up)
```

- [ ] **Step 4: 重新编译**

Run:
```bash
cd /home/cy/l150pro_ws && source /opt/ros/jazzy/setup.bash && \
colcon build --packages-select l150pro_driver
```
Expected: `Summary: 1 package finished`，无 error

- [ ] **Step 5: 验证话题存在**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source /home/cy/l150pro_ws/install/setup.bash && \
timeout 12 ros2 launch l150pro_driver l150pro.launch.py port:=/dev/l150pro use_ekf:=false &
sleep 6
source /opt/ros/jazzy/setup.bash && source /home/cy/l150pro_ws/install/setup.bash && \
timeout 5 ros2 topic list | grep battery_voltage
```
Expected: 出现 `/battery_voltage`

（若 `/dev/l150pro` 不存在——板子没插——这一步会失败。此时退而求其次：确认节点能起来且 `ros2 topic list` 里有 `/battery_voltage`，把它记为「待接车验证」。）

- [ ] **Step 6: 记录到驱动 README**

在 `/home/cy/l150pro_ws/src/l150pro_driver/README.md` 的话题表里加一行：

```markdown
| `battery_voltage` | `std_msgs/Float32` | 上 | 电池电压 (V)，每个状态帧发一次 |
```

- [ ] **Step 7: 确认驱动侧改动范围**

驱动包**不在这个项目里**，也不受任何版本控制。确认这次只动了两个文件：

```bash
cd /home/cy/l150pro_ws/src/l150pro_driver && \
ls -l l150pro_driver/driver_node.py l150pro_driver/driver_node.py.bak-volt-* README.md
```

除 `driver_node.py`（已改）和 `README.md`（加了话题表一行）外，**不应该有别的文件被动过**。
若发现别的文件 mtime 也变了，停下来报告。

---

## Task 3: ROS 适配层 —— `car/ros_bridge.py`

全项目**唯一** import rclpy 的地方。拆出来的理由：`CarController` 的闭环逻辑能在假桥上测；将来导航/建图如果只想要「发速度、读里程计」，可以只用这一层。

**Files:**
- Create: `/home/cy/voice-chatbot/car/ros_bridge.py`
- Create: `/home/cy/voice-chatbot/tests/test_ros_bridge.py`

**Interfaces:**
- Consumes: `car.types.RobotState`
- Produces:
  - `car.ros_bridge.RosUnavailable(RuntimeError)`
  - `car.ros_bridge.ensure_ros_available() -> None`
  - `car.ros_bridge.RosBridge`，方法：
    - `__init__(self, *, cmd_rate_hz: float = 25.0)`
    - `publish_velocity(self, vx: float, wz: float) -> None`
    - `publish_servo(self, us: list[int]) -> None`（长度必须为 6）
    - `state(self) -> RobotState`
    - `close(self) -> None`

- [ ] **Step 1: 写失败的测试 `tests/test_ros_bridge.py`**

```python
"""RosBridge 测试。

不接车也能测的部分：环境检查的报错、发布的帧真的能被子节点收到。
需要 rclpy 的用例在没有 ROS 的机器上自动跳过。

**桥是 module 级共享的** —— rclpy.init() 是进程级的，只能调一次，
每个用例各建一次桥会从第二个开始报错。
"""

import builtins
import sys
import threading
import time

import pytest


def test_ensure_ros_available_gives_actionable_error(monkeypatch):
    """rclpy 不在时，报错必须告诉使用者怎么办，不能只说 ModuleNotFoundError。"""
    from car import ros_bridge

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "rclpy":
            raise ImportError("No module named 'rclpy'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    with pytest.raises(ros_bridge.RosUnavailable) as excinfo:
        ros_bridge.ensure_ros_available()
    assert "source /opt/ros/jazzy/setup.bash" in str(excinfo.value)
    assert "run.sh" in str(excinfo.value)


@pytest.fixture(scope="module")
def bridge():
    """整个模块共用一个桥 —— rclpy.init() 进程内只能调一次。

    `importorskip` 放在 fixture 里，**不放在模块级**：模块级的 Skipped 会在
    collection 阶段把整个文件吞掉，连不依赖 ROS 的
    test_ensure_ros_available_gives_actionable_error 都跑不了。
    """
    pytest.importorskip("rclpy", reason="需要 ROS 2 环境")
    from car.ros_bridge import RosBridge

    b = RosBridge(cmd_rate_hz=25.0)
    yield b
    b.close()


def test_bridge_publishes_velocity_that_others_can_hear(bridge):
    """自己发的帧，同一 DDS 域内的另一个节点应当收得到。"""
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    listener = Node("test_car_listener")
    received: list[tuple[float, float]] = []
    done = threading.Event()

    def _on_msg(msg):
        received.append((msg.linear.x, msg.angular.z))
        done.set()

    listener.create_subscription(Twist, "cmd_vel", _on_msg, qos)

    # 给 DDS 发现留时间：反复发，同时自旋收
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not done.is_set():
        bridge.publish_velocity(0.25, 0.0)
        rclpy.spin_once(listener, timeout_sec=0.1)

    listener.destroy_node()
    assert received, "5 秒内没收到自己发的 cmd_vel —— DDS 发现或发布有问题"
    assert received[-1] == pytest.approx((0.25, 0.0))


def test_publish_servo_rejects_wrong_length(bridge):
    with pytest.raises(ValueError):
        bridge.publish_servo([1500, 1500])


def test_state_reports_offline_before_any_uplink(bridge):
    state = bridge.state()
    assert state.online is False
    assert state.traveled == 0.0
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/test_ros_bridge.py -v`
Expected: `1 failed, 3 skipped` —— 失败的是
`test_ensure_ros_available_gives_actionable_error`（`ModuleNotFoundError: No module named 'car.ros_bridge'`），
另 3 个 ROS 用例因为 fixture 里 importorskip 而跳过。
**若看到「1 skipped」把整个文件吞掉**，说明 importorskip 又跑到模块级去了。

- [ ] **Step 3: 写 `car/ros_bridge.py`**

```python
"""ROS 2 适配层 —— 全项目唯一 import rclpy 的地方。

单独拆出来的三个理由：
  1. CarController 的闭环逻辑（走两米、转九十度）能在假桥上测，不需要插着车。
  2. 将来导航/建图节点如果只想要「发速度、读里程计」，可以只用这一层。
  3. ROS 环境加载失败是**静默**的（systemd 不会 source setup.bash），
     集中在一处才好做出声的检查。
"""

from __future__ import annotations

import threading
import time

from loguru import logger

from car.types import RobotState

_SERVO_CHANNELS = 6


class RosUnavailable(RuntimeError):
    """ROS 2 环境没准备好。故意做成吵闹的异常，不要静默降级。"""


def ensure_ros_available() -> None:
    """在 import rclpy 之前给出可操作的报错。

    systemd 用户单元不会 source ROS 的 setup.bash，失败表现为
    「ModuleNotFoundError: No module named 'rclpy'」——那句话对使用者
    毫无指导意义。这里换成能照着做的提示。
    """
    try:
        import rclpy  # noqa: F401
    except ImportError as exc:
        raise RosUnavailable(
            "加载 rclpy 失败 —— ROS 2 环境没有准备好。\n"
            "  由 systemd 启动时：确认单元里的 ExecStart 指向 voice-chatbot/run.sh"
            "（它会先 source ROS 再跑 main.py）。\n"
            "  手工运行请先执行：\n"
            "    source /opt/ros/jazzy/setup.bash\n"
            "    source ~/l150pro_ws/install/setup.bash"
        ) from exc


class RosBridge:
    """对 ROS 2 的薄封装：发 cmd_vel / servo_cmd，收上行状态。

    线程安全：publish_* 只做加锁后调 publisher.publish()；state() 返回不可变
    快照。所以发帧线程和闭环逻辑可以同时用它。
    """

    def __init__(self, *, cmd_rate_hz: float = 25.0) -> None:
        ensure_ros_available()

        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Float32, Int16MultiArray

        self._rclpy = rclpy
        self._Twist = Twist
        self._JointState = JointState

        rclpy.init(args=[])
        self._node = Node("voice_chatbot_car")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # ---- 下行 ----
        self._pub_vel = self._node.create_publisher(Twist, "cmd_vel", qos)
        self._pub_servo = self._node.create_publisher(JointState, "servo_cmd", qos)

        # ---- 上行（全部只读，供闭环与 car_status 用）----
        self._lock = threading.Lock()
        self._voltage: float | None = None
        self._fault: int | None = None
        self._wheels: tuple[float, float, float, float] | None = None
        self._yaw_rate: float | None = None
        self._traveled = 0.0
        self._last_xy: tuple[float, float] | None = None
        self._online = False

        self._node.create_subscription(Odometry, "wheel_odom", self._on_odom, qos)
        self._node.create_subscription(Imu, "imu/data_raw", self._on_imu, qos)
        self._node.create_subscription(
            Int16MultiArray, "driver_fault", self._on_fault, qos
        )
        self._node.create_subscription(
            Float32, "battery_voltage", self._on_voltage, qos
        )

        # ---- 自旋 ----
        # 用**自己的** executor，不要碰 rclpy 的进程级全局 executor。
        # 裸调 `rclpy.spin_once(node)` 会落到 `get_global_executor()`，
        # 而那个 executor **整个进程只有一份**。桥的自旋线程占住它之后，
        # 同进程里任何别处再裸调一次就会撞
        # `RuntimeError: Executor is already spinning`（已实测复现）。
        # 将来导航/建图与助手同进程时也会自旋 —— 这条是给那时候留的路。
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spin, name="car-ros-spin", daemon=True
        )
        self._spin_thread.start()

        # 3 秒后查一次 cmd_vel 有没有订阅者（同 teleop 的排错提示）。
        # 用定时器而不是「第一帧就查」：DDS 发现需要时间，第一帧查必然误报。
        self._link_timer = self._node.create_timer(3.0, self._check_link)
        logger.info("小车：ROS 桥已就绪（节点 voice_chatbot_car）")

    # ---------------------------------------------------------------- 发布

    def publish_velocity(self, vx: float, wz: float) -> None:
        """发一帧速度。[只取 linear.x 和 angular.z —— 差速车没有横移]"""
        msg = self._Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        with self._lock:
            self._pub_vel.publish(msg)

    def publish_servo(self, us: list[int]) -> None:
        """发一帧舵机脉宽。长度必须为 6；0 表示该路保持不动。"""
        if len(us) != _SERVO_CHANNELS:
            raise ValueError(
                f"舵机帧需要 {_SERVO_CHANNELS} 路，收到 {len(us)}"
            )
        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = [f"ch{i}" for i in range(1, _SERVO_CHANNELS + 1)]
        msg.position = [float(x) for x in us]
        with self._lock:
            self._pub_servo.publish(msg)

    # ---------------------------------------------------------------- 状态

    def state(self) -> RobotState:
        with self._lock:
            return RobotState(
                voltage=self._voltage,
                fault=self._fault,
                wheel_speeds=self._wheels,
                yaw_rate=self._yaw_rate,
                traveled=self._traveled,
                online=self._online,
            )

    # ---------------------------------------------------------------- 内部

    def _spin(self) -> None:
        while not self._spin_stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as exc:  # noqa: BLE001 —— 自旋不能因为一次异常就死
                logger.error(f"小车：ROS 自旋异常 —— {exc}")
                time.sleep(0.1)

    def _check_link(self) -> None:
        """只跑一次：cmd_vel 上没有订阅者 = 底盘驱动没在跑。"""
        self._node.destroy_timer(self._link_timer)
        if self._pub_vel.get_subscription_count() == 0:
            logger.warning(
                "小车：cmd_vel 上没有订阅者 —— 底盘驱动没在跑？"
                "另开终端执行：roslaunch 见 ~/l150pro_start_driver.sh"
            )

    def _on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        with self._lock:
            if self._last_xy is not None:
                dx = p.x - self._last_xy[0]
                dy = p.y - self._last_xy[1]
                self._traveled += (dx * dx + dy * dy) ** 0.5
            self._last_xy = (p.x, p.y)
            self._online = True

    def _on_imu(self, msg) -> None:
        with self._lock:
            self._yaw_rate = float(msg.angular_velocity.z)
            self._online = True

    def _on_fault(self, msg) -> None:
        data = list(msg.data)
        with self._lock:
            if data:
                self._fault = int(data[0])
            self._online = True

    def _on_voltage(self, msg) -> None:
        with self._lock:
            self._voltage = float(msg.data)
            self._online = True

    # ---------------------------------------------------------------- 收尾

    def close(self) -> None:
        """关自旋线程并销毁节点。**不调 rclpy.shutdown()** ——
        它是进程级的，将来同进程还有别的 ROS 消费者时会把它们一起关掉。"""
        self._spin_stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self._node)
            self._executor.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"小车：关闭 executor 失败 —— {exc}")
        try:
            self._node.destroy_node()
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"小车：销毁 ROS 节点失败 —— {exc}")
```

> ⚠️ **已知残留问题**：`rclpy.init()` 与 `destroy_node()` 在多实例测试下会互相干扰（同一进程里 init 两次会报错）。上面每个用例都 `close()` 了，但 `test_ros_bridge.py` 里三个用例各建一次桥，第二个就会炸。实现时改成 **module 级 fixture 共用一个 bridge**：
> ```python
> @pytest.fixture(scope="module")
> def bridge():
>     from car.ros_bridge import RosBridge
>     b = RosBridge()
>     yield b
>     b.close()
> ```
> 三个用例改成接收 `bridge` 参数。`test_ensure_ros_available_gives_actionable_error` 不建桥，不受影响。

- [ ] **Step 4: 跑测试确认通过**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/test_ros_bridge.py -v`
Expected: PASS，**4 passed**（带 ROS 环境跑）
不带 ROS 环境跑应当是 **1 passed, 3 skipped**（不是 1 skipped —— 那说明整个文件被吞了）。

- [ ] **Step 5: 回归全量测试 + 打快照**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest -v`
Expected: PASS —— 本任务新增用例全绿，前面任务的用例**没有回归**

测试过了再打快照。没有版本控制，这就是唯一的回滚点：

```bash
cd /home/cy && tar czf "voice-chatbot.bak-T3-$(date +%Y%m%d-%H%M%S).tar.gz" \
    --exclude='voice-chatbot/logs' --exclude='__pycache__' \
    --exclude='.pytest_cache' voice-chatbot
```
（Task 0 那份开工快照是总退路，但它退不回「上一个任务刚做完」的状态。）

---

## Task 4: `CarController` —— 闭环运动与急停

**Files:**
- Create: `/home/cy/voice-chatbot/car/controller.py`
- Create: `/home/cy/voice-chatbot/tests/fakes.py`
- Create: `/home/cy/voice-chatbot/tests/test_car_controller.py`

**Interfaces:**
- Consumes: `CarLimits`、`RobotState`（Task 1）；`RosUnavailable`（Task 3）；`clamp_speed`、`clamp_turn_speed`、`direction_to_velocity`、`camera_target_us`（Task 1）
- Produces: `car.controller.CarController`
  - `__init__(self, config, *, bridge_factory=RosBridge, clock=time.monotonic, sleeper=time.sleep)`
  - `start(self) -> None`
  - `stop_all(self) -> None`
  - `set_velocity(self, vx: float, wz: float) -> None` ← 原语，程序内部用，**不是工具**
  - `move_distance(self, direction: str, distance: float, speed: float | None = None) -> str`
  - `turn_by(self, angle_deg: float, speed: float | None = None) -> str`
  - `stop_and_describe(self) -> str`
  - `camera_move(self, direction: str, degrees: float | None = None) -> str`
  - `status_text(self) -> str`
  - `close(self) -> None`

- [ ] **Step 1: 写测试替身 `tests/fakes.py`**

```python
"""测试替身。

FakeClock 让「按时间推进」的闭环逻辑在测试里瞬间跑完且完全确定 ——
不用真的 sleep，也不会因为机器慢而 flaky。
"""

from __future__ import annotations

from car.types import RobotState


class FakeClock:
    """手动推进的时钟。sleep() 只把时间往前拨，不真的等。"""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds


class FakeBridge:
    """按「速度 × 时间」积分的假 ROS 桥。

    不做物理仿真 —— 只要能驱动闭环逻辑就够了。
    """

    def __init__(
        self,
        clock: FakeClock,
        *,
        speed_scale: float = 1.0,
        voltage: float | None = 12.0,
        fault: int | None = 0,
        online: bool = True,
        dead: bool = False,
    ) -> None:
        self.velocity_calls: list[tuple[float, float]] = []
        self.servo_calls: list[list[int]] = []
        self.closed = False
        self._clock = clock
        self._speed_scale = speed_scale
        self._voltage = voltage
        self._fault = fault
        self._online = online
        self._dead = dead              # True = 永远收不到上行（模拟驱动没跑）
        self._traveled = 0.0
        self._yaw = 0.0
        self._last_t = clock()

    def publish_velocity(self, vx: float, wz: float) -> None:
        self.velocity_calls.append((vx, wz))
        now = self._clock()
        dt = max(0.0, now - self._last_t)
        self._last_t = now
        self._traveled += abs(vx) * self._speed_scale * dt
        self._yaw += wz * dt

    def publish_servo(self, us: list[int]) -> None:
        self.servo_calls.append(list(us))

    def state(self) -> RobotState:
        if self._dead:
            return RobotState(voltage=None, fault=None, wheel_speeds=None,
                              yaw_rate=None, traveled=0.0, online=False)
        # yaw_rate 如实回报「上一帧指令」，让 controller 的积分跑得起来
        last_wz = self.velocity_calls[-1][1] if self.velocity_calls else 0.0
        return RobotState(
            voltage=self._voltage,
            fault=self._fault,
            wheel_speeds=(0.0, 0.0, 0.0, 0.0),
            yaw_rate=last_wz,
            traveled=self._traveled,
            online=self._online,
        )

    def close(self) -> None:
        self.closed = True
```

- [ ] **Step 2: 写失败的测试 `tests/test_car_controller.py`**

```python
"""CarController 的闭环逻辑测试 —— 用假桥，不接车、不碰 ROS。"""

import re

import pytest

from car.controller import CarController
from car.types import CarLimits
from tests.fakes import FakeBridge, FakeClock

LIMITS = CarLimits()


class _FakeConfig:
    def get(self, key, default=None):
        return default


def _make(**bridge_kwargs):
    clock = FakeClock()
    bridge = FakeBridge(clock, **bridge_kwargs)
    controller = CarController(
        _FakeConfig(), bridge_factory=lambda **_: bridge,
        clock=clock, sleeper=clock.sleep,
    )
    # 先把桥建起来 —— 模拟「车被用过」。stop_all() 刻意**不**建桥
    # （它跑在唤醒词路径上，不该为了发几帧零速去 rclpy.init()），
    # 所以测「急停连发 8 帧」必须先有一次真实使用。
    controller.start()
    return controller, bridge, clock


# ---------------- 走距离 ----------------

def test_move_distance_stops_after_covering_the_distance():
    controller, bridge, _ = _make()
    controller.move_distance("forward", 1.0, speed=0.5)
    # 前进总里程应当略大于等于 1.0（最后一次采样会过冲一点）
    assert bridge.state().traveled >= 1.0


def test_move_distance_ends_with_zero_velocity():
    """走完必须收尾归零，否则车会一直跑。"""
    controller, bridge, _ = _make()
    controller.move_distance("forward", 0.5, speed=0.5)
    assert bridge.velocity_calls[-1] == (0.0, 0.0)


def test_move_distance_publishes_at_the_configured_rate():
    """0.5 米 @ 0.5 m/s = 1 秒；25Hz 下应当约 25 帧，而不是一帧。"""
    controller, bridge, _ = _make()
    controller.move_distance("forward", 0.5, speed=0.5)
    moving = [c for c in bridge.velocity_calls if c != (0.0, 0.0)]
    assert len(moving) >= 20


def test_move_distance_uses_default_speed_when_omitted():
    controller, bridge, _ = _make()
    controller.move_distance("forward", 0.3)
    first = bridge.velocity_calls[0]
    assert first[0] == pytest.approx(LIMITS.default_speed)


def test_move_distance_clamps_and_says_so():
    """超限要报出来，不能静默截断。"""
    controller, bridge, _ = _make()
    text = controller.move_distance("forward", 0.2, speed=0.8)
    assert "0.5" in text
    assert bridge.velocity_calls[0][0] == pytest.approx(LIMITS.max_speed)


def test_backward_is_negative():
    controller, bridge, _ = _make()
    controller.move_distance("backward", 0.3, speed=0.3)
    assert bridge.velocity_calls[0][0] < 0


def test_move_distance_gives_up_when_odom_never_moves():
    """驱动没在跑时里程计恒为 0，必须超时退出并说清原因。"""
    controller, bridge, _ = _make(dead=True)
    text = controller.move_distance("forward", 1.0, speed=0.5)
    assert "里程计" in text
    assert bridge.velocity_calls[-1] == (0.0, 0.0)


def test_move_distance_rejects_unknown_direction():
    controller, _, _ = _make()
    text = controller.move_distance("sideways", 1.0)
    assert "方向" in text or "sideways" in text


# ---------------- 转向 ----------------

def test_turn_by_integrates_yaw_to_the_target_angle():
    """90 度 @ 60 度/秒 ≈ 1.5 秒。

    ★ 下面两条断言是**必须的**：只查首帧符号和末帧归零的话，「度/弧度混用」
    那个 bug 会照样通过 —— 它的表现是永远跑满超时，再报一个假的
    「陀螺仪一直读到 0 —— IMU 数据没上来」。测试必须验到**走的是完成分支**。
    """
    controller, bridge, _ = _make()
    text = controller.turn_by(90.0)
    assert bridge.velocity_calls[-1] == (0.0, 0.0)
    assert bridge.velocity_calls[0][1] > 0        # 正角 = 逆时针 = 左转
    assert "已左转" in text, f"没走完成分支，实际返回：{text}"
    # 落在目标的一帧之内即可 —— 控制环是「先发帧、再判断是否到位」，
    # 25Hz 下 60 度/秒每帧走 2.4 度，必然有一帧过冲。别去"修"成精确等于 90。
    matched = re.search(r"约 (\d+) 度", text)
    assert matched, f"返回里读不到角度：{text}"
    assert 88 <= int(matched.group(1)) <= 93, f"角度偏差超过一帧：{text}"


def test_turn_by_finishes_in_about_the_expected_time():
    """90 度 @ 60 度/秒 = 1.5 秒；25Hz 下约 37 帧。

    这是给「度当弧度用」上的第二道锁：那个 bug 会让帧数变成几十上百倍。
    """
    controller, bridge, _ = _make()
    controller.turn_by(90.0)
    moving = [c for c in bridge.velocity_calls if c != (0.0, 0.0)]
    assert 30 <= len(moving) <= 45, f"发了 {len(moving)} 帧，与 1.5 秒的预期差太远"


def test_turn_by_360_degrees_finishes():
    """整整一圈也该正常收尾（角度大了更容易暴露单位问题）。"""
    controller, bridge, _ = _make()
    text = controller.turn_by(360.0)
    assert "已左转" in text, f"没走完成分支，实际返回：{text}"
    matched = re.search(r"约 (\d+) 度", text)
    assert matched, f"返回里读不到角度：{text}"
    assert 358 <= int(matched.group(1)) <= 363, f"角度偏差超过一帧：{text}"


def test_turn_by_negative_angle_turns_the_other_way():
    controller, bridge, _ = _make()
    controller.turn_by(-90.0)
    assert bridge.velocity_calls[0][1] < 0


def test_turn_by_clamps_turn_speed():
    controller, bridge, _ = _make()
    text = controller.turn_by(360.0, speed=999.0)
    assert "120" in text


# ---------------- 急停 ----------------

def test_stop_all_publishes_zero_frames_repeatedly():
    """急停要连发几帧，确保驱动真收到（teleop 同款做法）。"""
    controller, bridge, _ = _make()
    controller.stop_all()
    assert len(bridge.velocity_calls) >= 8
    assert all(c == (0.0, 0.0) for c in bridge.velocity_calls)


def test_stop_all_is_safe_before_bridge_exists():
    """唤醒词路径会直接调它 —— 那条路径上绝不能抛异常。"""
    clock = FakeClock()
    controller = CarController(
        _FakeConfig(), bridge_factory=lambda **_: FakeBridge(clock),
        clock=clock, sleeper=clock.sleep,
    )
    controller.stop_all()          # 不该抛


def test_stop_and_describe_reports_stopping():
    controller, _, _ = _make()
    assert "停" in controller.stop_and_describe()


# ---------------- 云台 ----------------

def test_camera_move_sends_six_channel_frame():
    controller, bridge, _ = _make()
    controller.camera_move("right")
    frame = bridge.servo_calls[-1]
    assert len(frame) == 6
    assert frame[4] > 1500          # ch5 = 水平
    assert frame[5] == 0            # ch6 没动 → 0 = 保持不动


def test_camera_center_returns_to_1500():
    controller, bridge, _ = _make()
    controller.camera_move("right")
    controller.camera_move("center")
    assert bridge.servo_calls[-1][4] == 1500


def test_camera_up_moves_tilt_channel():
    controller, bridge, _ = _make()
    controller.camera_move("up")
    frame = bridge.servo_calls[-1]
    assert frame[4] == 0            # ch5 没动
    assert frame[5] > 1500          # ch6 = 俯仰


def test_camera_reports_hitting_the_limit():
    controller, _, _ = _make()
    text = controller.camera_move("right", degrees=900.0)
    assert "限位" in text or "到头" in text


# ---------------- 状态 ----------------

def test_status_text_includes_voltage_and_fault():
    controller, _, _ = _make(voltage=11.6, fault=0)
    text = controller.status_text()
    assert "11.6" in text
    assert "正常" in text


def test_status_text_says_so_when_offline():
    controller, _, _ = _make(dead=True)
    assert "读不到" in controller.status_text() or "没在跑" in controller.status_text()
```

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/test_car_controller.py -v`
Expected: FAIL —— `ModuleNotFoundError: No module named 'car.controller'`

- [ ] **Step 4: 写 `car/controller.py`**

```python
"""小车控制器 —— 编排层：闭环运动、云台、急停、状态播报。

## 与语音助手的关系

照抄 tools/music.py 里 MusicController 的已验证模式：

    用户喊唤醒词 → ConversationManager._on_wake_word()
      → self._music.stop()        # 音乐先停
      → self._car.stop_all()      # 小车急停（同构）
      → 才 transition(LISTENING)

唤醒词路径**不经过 ASR / LLM**，几十毫秒就能停住。这是急停。
正常对话里说「停」走 car_stop 工具，要一次 LLM 往返 —— 那是「停一下」，
不是急停。

## 三层里的中间一层

    Tool（car_move 等 5 个）→ CarController ← 未来的 nav_goto / slam_* 也调这里
                                  ↓
                              RosBridge

`set_velocity` 是**给程序用的原语，不是给模型的工具** —— 语音带宽太低，
用户说不出「设 vx 为 0.3」。
"""

from __future__ import annotations

import math
import threading
import time

from loguru import logger

from car.kinematics import (
    camera_target_us,
    clamp_speed,
    clamp_turn_speed,
    direction_names,
    direction_to_velocity,
)
from car.ros_bridge import RosBridge
from car.types import CarLimits, describe_faults

# 云台通道。记忆 l150pro-car 明确写过：ch5 = 水平、ch6 = 俯仰，别搞反。
_PAN_CHANNEL = 5
_TILT_CHANNEL = 6

# 急停连发几帧零速。teleop 的 stop_now() 用 8 帧 / 0.3 秒，照抄。
_STOP_FRAMES = 8
_STOP_FRAME_INTERVAL = 0.04


class CarController:
    """小车的对外接口。线程安全。"""

    def __init__(
        self,
        config,
        *,
        bridge_factory=RosBridge,
        clock=time.monotonic,
        sleeper=time.sleep,
    ) -> None:
        self._config = config
        self._limits = CarLimits.from_config(config)
        self._bridge_factory = bridge_factory
        self._clock = clock
        self._sleep = sleeper

        self._bridge = None
        self._bridge_lock = threading.Lock()   # 保护 _bridge 的懒建
        self._motion_lock = threading.Lock()   # 同一时刻只允许一个运动指令
        self._stop_event = threading.Event()   # 置位 = 立刻中止当前运动
        self._moving = False

        # 云台位置。**只在某一路真的被指令过之后才记录**：发帧时没记录的路
        # 一律发 0（固件约定 0 = 保持不动）。若这里预填中位，那么「往右看」
        # 会顺手把俯仰轴也命令到 1500 —— 用户没让它动，等于凭空改了车的姿态。
        self._camera_us: dict[int, int] = {}

    # ------------------------------------------------------------ 生命周期

    def start(self) -> None:
        """提前建立 ROS 连接，让环境问题**在启动日志里就暴露**。

        失败不抛：小车起不来不该拖垮整个语音助手（天气、音乐、闲聊都还得能用）。
        工具调用时会再试一次，所以「先起助手、后起驱动」也能自己恢复。
        """
        try:
            self._bridge_or_raise()
        except Exception as exc:  # noqa: BLE001
            logger.error(f"小车：暂时不可用 —— {exc}")

    def close(self) -> None:
        self.stop_all()
        bridge = self._bridge
        if bridge is not None:
            bridge.close()
            self._bridge = None

    def _bridge_or_raise(self):
        """懒建 ROS 桥。失败抛 RosUnavailable，由工具分发层转成给模型看的错误。"""
        with self._bridge_lock:
            if self._bridge is None:
                self._bridge = self._bridge_factory(
                    cmd_rate_hz=self._limits.cmd_rate_hz
                )
            return self._bridge

    # ------------------------------------------------------------ 原语（给程序用）

    def set_velocity(self, vx: float, wz: float) -> None:
        """发一帧速度。**这是给程序用的原语，不是给模型的工具。**

        未来的 nav_goto / slam_* 直接调它，不需要新增模型可见的工具。
        """
        self._bridge_or_raise().publish_velocity(vx, wz)

    def stop_all(self) -> None:
        """急停。**唤醒词路径会直接调它 —— 这条路径上绝不抛异常。**"""
        self._stop_event.set()
        bridge = self._bridge
        if bridge is None:
            return
        try:
            for _ in range(_STOP_FRAMES):
                bridge.publish_velocity(0.0, 0.0)
                self._sleep(_STOP_FRAME_INTERVAL)
        except Exception as exc:  # noqa: BLE001
            logger.error(f"小车：急停发零速失败 —— {exc}")

    # ------------------------------------------------------------ 运动（工具用）

    def move_distance(
        self, direction: str, distance: float, speed: float | None = None
    ) -> str:
        """按方向走一段距离，走完停。阻塞直到到位 / 超时 / 被急停。"""
        if direction not in direction_names():
            return (
                f"不认识方向 {direction}。可用的有：{'、'.join(direction_names())}。"
                "原地转向请用 car_turn。"
            )
        if distance <= 0:
            return "距离得是正数。"

        if not self._motion_lock.acquire(blocking=False):
            return "正忙，先等上一个动作做完。"
        try:
            requested = self._limits.default_speed if speed is None else speed
            magnitude, clamped = clamp_speed(requested, self._limits)
            vx, wz = direction_to_velocity(
                direction, magnitude, self._limits.default_turn_speed_dps
            )

            bridge = self._bridge_or_raise()
            start = bridge.state().traveled
            timeout = (distance / max(magnitude, 0.01)) * 3.0 + 5.0
            finished = self._pump_until(
                vx=vx, wz=wz,
                done=lambda: abs(bridge.state().traveled - start) >= distance,
                timeout=timeout,
            )
            self._finish_motion()

            moved = abs(bridge.state().traveled - start)
            verb = "前进" if vx > 0 else "后退"
            note = f"（速度已按上限截到 {magnitude:.2f} 米每秒）" if clamped else ""
            if finished:
                return f"已{verb}约 {moved:.2f} 米{note}。"
            if self._stop_event.is_set():
                return f"{verb}被打断，停在约 {moved:.2f} 米处。"
            return (
                f"发了 {timeout:.0f} 秒速度但里程计几乎没动（{moved:.2f} 米）—— "
                "底盘驱动可能没在跑，车没有真的走。"
            )
        finally:
            self._motion_lock.release()

    def turn_by(self, angle_deg: float, speed: float | None = None) -> str:
        """原地转一个角度，转完停。正 = 逆时针 = 左转（REP-103）。"""
        if not self._motion_lock.acquire(blocking=False):
            return "正忙，先等上一个动作做完。"
        try:
            requested = (
                self._limits.default_turn_speed_dps if speed is None else speed
            )
            dps, clamped = clamp_turn_speed(requested, self._limits)
            wz = (1.0 if angle_deg >= 0 else -1.0) * dps * math.pi / 180.0

            bridge = self._bridge_or_raise()
            # ★ 全程在【弧度】域累积。`yaw_rate` 是 rad/s，乘秒得弧度 ——
            # 拿它直接跟「度」比会永远到不了目标（转 90 度要积 90 弧度
            # = 5156 度），表现是每次都跑满超时，再报一个假的
            # 「陀螺仪一直读到 0」故障。只有出口转回度给用户看。
            target_rad = abs(angle_deg) * math.pi / 180.0
            turned_rad = 0.0
            last = self._clock()

            def _done() -> bool:
                nonlocal turned_rad, last
                now = self._clock()
                state = bridge.state()
                if state.yaw_rate is not None:
                    turned_rad += abs(state.yaw_rate) * (now - last)
                last = now
                return turned_rad >= target_rad

            timeout = (abs(angle_deg) / max(dps, 1.0)) * 3.0 + 5.0
            finished = self._pump_until(wz=wz, vx=0.0, done=_done, timeout=timeout)
            self._finish_motion()

            turned_deg = turned_rad * 180.0 / math.pi
            direction_word = "左转" if angle_deg >= 0 else "右转"
            note = f"（角速度已按上限截到 {dps:.0f} 度每秒）" if clamped else ""
            if finished:
                return f"已{direction_word}约 {turned_deg:.0f} 度{note}。"
            if self._stop_event.is_set():
                return f"{direction_word}被打断，停在约 {turned_deg:.0f} 度。"
            return (
                f"发了 {timeout:.0f} 秒转速但陀螺仪一直读到 0 —— "
                f"IMU 数据没上来，车可能没真的转。{note}"
            )
        finally:
            self._motion_lock.release()

    def stop_and_describe(self) -> str:
        """car_stop 工具用：停下并回报一句。"""
        was_moving = self._moving
        self.stop_all()
        return "好，停了。" if was_moving else "车本来就是停着的。"

    # ------------------------------------------------------------ 云台（工具用）

    def camera_move(self, direction: str, degrees: float | None = None) -> str:
        """转动云台。direction 取 left/right/up/down/center。"""
        try:
            channel = _TILT_CHANNEL if direction in ("up", "down") else _PAN_CHANNEL
            # 没指令过就按中位推算（绝对脉宽伺服本来就得有个假设起点，
            # teleop 同样假定上电即中位），但**不会**因此去命令那一路。
            current = self._camera_us.get(channel, self._limits.camera_center_us)
            target, hit_limit = camera_target_us(
                direction, degrees, current, self._limits
            )
        except KeyError:
            return (
                f"不认识云台方向 {direction}。"
                "可用：left（往左）、right（往右）、up（抬头）、down（低头）、center（回正）。"
            )

        self._camera_us[channel] = target
        self._bridge_or_raise().publish_servo(self._servo_frame())

        label = {"left": "往左", "right": "往右", "up": "往上",
                 "down": "往下", "center": "回到中位"}[direction]
        note = "，已经到头了" if hit_limit else ""
        axis = "俯仰" if channel == _TILT_CHANNEL else "水平"
        return f"云台{label}了（{axis}轴 {target} 微秒）{note}。"

    def _servo_frame(self) -> list[int]:
        """0 = 该路保持不动 —— 这是不碰底盘上其它舵机的办法（固件约定）。"""
        frame = [0] * 6
        for channel, us in self._camera_us.items():
            frame[channel - 1] = us
        return frame

    # ------------------------------------------------------------ 状态（工具用）

    def status_text(self) -> str:
        state = self._bridge_or_raise().state()
        if not state.online:
            return (
                "还读不到小车的状态 —— 底盘驱动可能没在跑，"
                "或者在跑但还没有数据上来。"
            )
        parts: list[str] = []
        if state.voltage is not None:
            parts.append(f"电池 {state.voltage:.1f} 伏")
        parts.append(f"故障码 {describe_faults(state.fault)}")
        if state.wheel_speeds is not None:
            wheels = "、".join(f"{v:+.2f}" for v in state.wheel_speeds)
            parts.append(f"四轮速度 {wheels} 米每秒")
        parts.append("正在移动" if self._moving else "停着")
        return "，".join(parts) + "。"

    # ------------------------------------------------------------ 内部

    def _pump_until(self, *, vx: float, wz: float, done, timeout: float) -> bool:
        """按 cmd_rate 持续发帧，直到 done() 为真 / 超时 / 被急停。

        Returns:
            True = done() 达成；False = 超时或被急停打断
        """
        self._stop_event.clear()
        bridge = self._bridge_or_raise()
        period = 1.0 / self._limits.cmd_rate_hz
        deadline = self._clock() + timeout

        self._moving = True
        try:
            while True:
                if self._stop_event.is_set():
                    return False
                bridge.publish_velocity(vx, wz)
                if done():
                    return True
                if self._clock() >= deadline:
                    return False
                self._sleep(period)
        finally:
            self._moving = False

    def _finish_motion(self) -> None:
        """任何一条退出路径都要收尾归零，否则车会一直跑。"""
        self._stop_event.clear()
        bridge = self._bridge
        if bridge is None:
            return
        try:
            for _ in range(3):
                bridge.publish_velocity(0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            logger.error(f"小车：收尾归零失败 —— {exc}")
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/test_car_controller.py -v`
Expected: PASS，**22 passed**。
**不带 ROS 环境也必须全绿** —— 这一层是纯逻辑，用 FakeClock + FakeBridge 注入，
瞬间跑完、完全确定。需要 ROS 才能跑说明分层错了。

- [ ] **Step 6: 回归全量测试 + 打快照**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest -v`
Expected: PASS —— 本任务新增用例全绿，前面任务的用例**没有回归**

测试过了再打快照。没有版本控制，这就是唯一的回滚点：

```bash
cd /home/cy && tar czf "voice-chatbot.bak-T4-$(date +%Y%m%d-%H%M%S).tar.gz" \
    --exclude='voice-chatbot/logs' --exclude='__pycache__' \
    --exclude='.pytest_cache' voice-chatbot
```
（Task 0 那份开工快照是总退路，但它退不回「上一个任务刚做完」的状态。）

---

## Task 5: 工具层 —— 注册表分组预留 + 5 个小车工具

**Files:**
- Modify: `/home/cy/voice-chatbot/tools/registry.py`
- Create: `/home/cy/voice-chatbot/tools/car.py`
- Create: `/home/cy/voice-chatbot/tests/test_car_tools.py`
- Modify: `/home/cy/voice-chatbot/tools/__init__.py`

**Interfaces:**
- Consumes: `Tool`、`ToolRegistry`（`tools/registry.py`）；`CarController`（Task 4）
- Produces:
  - `Tool` 新增字段 `group: str = "core"`
  - `ToolRegistry.schemas(self, groups: Iterable[str] | None = None) -> list[dict]`
  - `tools.car.register_car_tools(registry: ToolRegistry, controller: CarController | None) -> None`

- [ ] **Step 1: 写失败的测试 `tests/test_car_tools.py`**

```python
"""工具注册测试 —— 只查 schema 形状与接线，不碰 ROS。"""

import pytest

from tools.car import register_car_tools
from tools.registry import Tool, ToolRegistry


class _StubController:
    def __init__(self):
        self.calls = []

    def move_distance(self, direction, distance, speed=None):
        self.calls.append(("move", direction, distance, speed))
        return "已前进约 2.00 米。"

    def turn_by(self, angle_deg, speed=None):
        self.calls.append(("turn", angle_deg, speed))
        return "已左转约 90 度。"

    def stop_and_describe(self):
        self.calls.append(("stop",))
        return "好，停了。"

    def camera_move(self, direction, degrees=None):
        self.calls.append(("camera", direction, degrees))
        return "云台往左了（水平轴 1333 微秒）。"

    def status_text(self):
        self.calls.append(("status",))
        return "电池 11.6 伏，故障码 正常，停着。"


EXPECTED_TOOLS = {"car_move", "car_turn", "car_stop", "car_camera", "car_status"}


def _registry():
    registry = ToolRegistry()
    register_car_tools(registry, _StubController())
    return registry


def test_registers_exactly_five_tools():
    assert set(_registry()._tools) == EXPECTED_TOOLS


def test_all_tools_are_in_the_car_group():
    for tool in _registry()._tools.values():
        assert tool.group == "car"


def test_schemas_are_openai_shaped():
    for schema in _registry().schemas():
        assert schema["type"] == "function"
        assert schema["function"]["name"]
        assert "description" in schema["function"]
        assert schema["function"]["parameters"]["type"] == "object"


def test_schemas_enum_is_not_a_hand_copy():
    """schema 的 enum 必须**就是** kinematics 的表，不能是抄的一份。

    抄一份会静默漂移：schema 发给模型，漂了就变成「模型传一个控制器不认的
    方向」，而且没有任何东西会报错。
    """
    from car.kinematics import camera_direction_names, direction_names

    schemas = {s["function"]["name"]: s for s in _registry().schemas()}
    move_enum = schemas["car_move"]["function"]["parameters"]["properties"]["direction"]["enum"]
    camera_enum = schemas["car_camera"]["function"]["parameters"]["properties"]["direction"]["enum"]
    assert move_enum == direction_names()
    assert camera_enum == camera_direction_names()


def test_car_move_direction_enum_has_six_values():
    schema = {s["function"]["name"]: s for s in _registry().schemas()}["car_move"]
    enum = schema["function"]["parameters"]["properties"]["direction"]["enum"]
    assert len(enum) == 6
    assert "left" not in enum          # 纯转向归 car_turn
    assert "forward_left" in enum


def test_car_move_distance_is_required():
    schema = {s["function"]["name"]: s for s in _registry().schemas()}["car_move"]
    assert "distance" in schema["function"]["parameters"]["required"]


def test_optional_params_are_not_required():
    schemas = {s["function"]["name"]: s for s in _registry().schemas()}
    assert "speed" not in schemas["car_move"]["function"]["parameters"]["required"]
    assert "speed" not in schemas["car_turn"]["function"]["parameters"]["required"]
    assert "degrees" not in schemas["car_camera"]["function"]["parameters"]["required"]


def test_car_stop_takes_no_parameters():
    schema = {s["function"]["name"]: s for s in _registry().schemas()}["car_stop"]
    assert schema["function"]["parameters"]["properties"] == {}


# ---------------- 分发 ----------------

def test_dispatch_calls_the_controller():
    controller = _StubController()
    registry = ToolRegistry()
    register_car_tools(registry, controller)
    outcome = registry.dispatch("car_move", {"direction": "forward", "distance": 2.0})
    assert controller.calls == [("move", "forward", 2.0, None)]
    assert "2.00" in outcome.result


def test_dispatch_passes_optional_speed():
    controller = _StubController()
    registry = ToolRegistry()
    register_car_tools(registry, controller)
    registry.dispatch("car_move", {"direction": "forward", "distance": 2.0,
                                   "speed": 0.4})
    assert controller.calls == [("move", "forward", 2.0, 0.4)]


def test_dispatch_turn_passes_angle():
    controller = _StubController()
    registry = ToolRegistry()
    register_car_tools(registry, controller)
    registry.dispatch("car_turn", {"angle_deg": -90})
    assert controller.calls == [("turn", -90, None)]


# ---------------- 控制器不可用时 ----------------

def test_registers_even_when_controller_is_none():
    """小车起不来也要注册 —— 让模型调了以后得到一句解释，而不是「没有这个工具」。"""
    registry = ToolRegistry()
    register_car_tools(registry, None)
    assert set(registry._tools) == EXPECTED_TOOLS


def test_tools_say_so_when_controller_is_none():
    registry = ToolRegistry()
    register_car_tools(registry, None)
    outcome = registry.dispatch("car_move", {"direction": "forward", "distance": 1.0})
    assert "不可用" in outcome.result


# ---------------- 分组过滤（为将来工具过 20 个预留） ----------------

def test_schemas_can_filter_by_group():
    registry = ToolRegistry()
    registry.register(Tool(name="get_current_time", description="d",
                           parameters={"type": "object", "properties": {}},
                           handler=lambda: "x"))
    register_car_tools(registry, _StubController())
    names = {s["function"]["name"] for s in registry.schemas(groups=["car"])}
    assert names == EXPECTED_TOOLS


def test_schemas_without_groups_returns_everything():
    registry = ToolRegistry()
    register_car_tools(registry, _StubController())
    assert len(registry.schemas()) == 5


def test_duplicate_registration_still_raises():
    registry = _registry()
    with pytest.raises(ValueError):
        register_car_tools(registry, _StubController())
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/test_car_tools.py -v`
Expected: FAIL —— `ModuleNotFoundError: No module named 'tools.car'`

- [ ] **Step 3: 给 `tools/registry.py` 加分组**

把 `Tool` dataclass 改成（新增最后一行字段）：

```python
@dataclass(frozen=True)
class Tool:
    """一个可供 LLM 调用的工具。

    Attributes:
        name: 工具名，全局唯一。
        description: 给模型看的说明，决定模型何时会调用它 —— 写得越具体越好。
        parameters: 参数的 JSON Schema（type/properties/required）。
        handler: 实际执行函数，接收关键字参数，返回 str 或 ToolOutcome。
        group: 能力分组。现在只有 "core" / "car"，全部都会发给模型；
               只是**预留**——将来工具过 20 个、开始稀释模型注意力时，
               按组过滤即可（见 schemas() 的 groups 参数），不用改数据结构。
    """

    name: str
    description: str
    parameters: dict[str, Any]
    handler: Callable[..., Any]
    group: str = "core"

    def schema(self) -> dict[str, Any]:
        """转换成 OpenAI 兼容的 tools 数组元素。"""
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.parameters,
            },
        }
```

（`schema()` 里**不放 group** —— 它不是协议的一部分，发过去只是噪音。）

给 `ToolRegistry.schemas` 加过滤：

```python
    def schemas(self, groups: Iterable[str] | None = None) -> list[dict[str, Any]]:
        """返回可直接塞进 `chat.completions.create(tools=...)` 的列表。

        Args:
            groups: 只要这些分组的工具。
                **None = 全都要；空集合 = 一个都不要。** 两者语义不同，
                别拿空 list 当「没配置」传进来 —— 那会静默发出空的 tools 数组。
        """
        tools = self._tools.values()
        if groups is not None:
            wanted = set(groups)
            tools = [t for t in tools if t.group in wanted]
        return [tool.schema() for tool in tools]
```

顶部 import 加 `Iterable`：

```python
from typing import Any, Callable, Iterable
```

- [ ] **Step 4: 写 `tools/car.py`**

```python
"""小车控制工具。

## 为什么是原生工具而不是 MCP

`cmd_vel` 要 25Hz 持续发（底盘有 200ms 丢帧看门狗），而 MCP server 天生是
请求-响应；急停更不该经过一次 CallTool 往返。小车是**本机常驻硬件能力**，
不是远程服务 —— 与 tools/music.py 里「服务器给链接、宿主负责放」同一个道理。

## 描述怎么写

「何时该调用」全部由 description 承担，**不要往 system_prompt 里加针对小车
的提示行** —— 在 persona 里点名一批能力，会给那批 token 加权，小模型于是主动
推销（2026-09-13 踩过两次，见记忆 prompt-no-capability-emphasis）。
"""

from __future__ import annotations

from loguru import logger

from car.kinematics import camera_direction_names, direction_names
from tools.registry import Tool, ToolRegistry

# 方向表直接从 car.kinematics 取，**不要在这里抄一份**。
# （第一版抄了一份并声称「Schema 的 enum 必须是字面量」—— 那是假的：
#   list(DIRECTIONS) 运行时就是普通 list，序列化毫无问题，而 kinematics
#   是纯模块不 import rclpy，哪里都能 import。）

_UNAVAILABLE = (
    "小车当前不可用 —— ROS 2 环境没准备好，或者底盘驱动没在跑。"
    "请检查服务日志。"
)


def _guard(controller):
    """controller 为 None 时统一回一句解释，而不是让模型看到「没有这个工具」。"""

    def _wrap(method_name: str):
        def _handler(**kwargs):
            if controller is None:
                return _UNAVAILABLE
            return getattr(controller, method_name)(**kwargs)

        return _handler

    return _wrap


def register_car_tools(registry: ToolRegistry, controller) -> None:
    """把小车能力登记成原生工具。controller 可以是 None（ROS 起不来时）。"""
    if controller is None:
        logger.warning("小车：控制器不可用，仍注册工具（调用时会说明原因）")

    guarded = _guard(controller)

    registry.register(
        Tool(
            name="car_move",
            group="car",
            description=(
                "让小车朝一个方向走一段距离，走完自动停。"
                "用户说「往前走」「后退」「往左前方走」「退两米」这类"
                "**带方向、也带一段路**的话时调用。"
                "原地转向请用 car_turn，本工具不接受纯左右。"
                "distance 单位是米；用户没提速度就不要传 speed，"
                "只在说了「快点」「慢点」「用 0.3 米每秒」时才传。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": direction_names(),
                        "description": "行进方向，六选一。",
                    },
                    "distance": {
                        "type": "number",
                        "description": "走多远，单位米。用户说「一点」时按 0.5 估。",
                    },
                    "speed": {
                        "type": "number",
                        "description": "线速度，单位米每秒。留空用默认值。",
                    },
                },
                "required": ["direction", "distance"],
            },
            handler=guarded("move_distance"),
        )
    )

    registry.register(
        Tool(
            name="car_turn",
            group="car",
            description=(
                "让小车原地转向，转完自动停。"
                "用户说「左转」「右转」「转九十度」「掉个头」时调用。"
                "angle_deg 左转为正、右转为负（如左转 90 度传 90，右转传 -90）。"
                "用户没提转速就不要传 speed。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "angle_deg": {
                        "type": "number",
                        "description": "要转的角度，左正右负。掉头按 180 算。",
                    },
                    "speed": {
                        "type": "number",
                        "description": "转向角速度，单位度每秒。留空用默认值。",
                    },
                },
                "required": ["angle_deg"],
            },
            handler=guarded("turn_by"),
        )
    )

    registry.register(
        Tool(
            name="car_stop",
            group="car",
            description=(
                "让小车立刻停下。用户说「停」「别走了」「停下吧」时调用。"
                "底盘正在走或正在转、用户想中止时就用它。"
            ),
            parameters={"type": "object", "properties": {}, "required": []},
            handler=guarded("stop_and_describe"),
        )
    )

    registry.register(
        Tool(
            name="car_camera",
            group="car",
            description=(
                "转动小车上的云台摄像头。"
                "用户说「往左看看」「往右看看」「抬头」「低头」「云台回正」时调用。"
                "用户没给具体角度就不要传 degrees（会转一个固定小步）；"
                "只有明确说了「转 30 度」这类角度时才传。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": camera_direction_names(),
                        "description": (
                            "left=往左转，right=往右转，up=抬头，"
                            "down=低头，center=回中位。"
                        ),
                    },
                    "degrees": {
                        "type": "number",
                        "description": "转动角度。留空 = 固定小步。",
                    },
                },
                "required": ["direction"],
            },
            handler=guarded("camera_move"),
        )
    )

    registry.register(
        Tool(
            name="car_status",
            group="car",
            description=(
                "查询小车的状态：电池电压、故障码、四轮速度、在不在动。"
                "用户问「还有多少电」「车怎么了」「车什么情况」时调用。"
            ),
            parameters={"type": "object", "properties": {}, "required": []},
            handler=guarded("status_text"),
        )
    )
```

- [ ] **Step 5: 更新 `tools/__init__.py` 的模块说明**

把模块 docstring 里的「小车控制工具后续加在这里」改成实际情况：

```python
"""LLM 工具调用层。

registry.py —— 工具注册表与白名单分发
builtin.py  —— 内置工具（ask_user / get_current_time）
car.py      —— 小车控制工具（car_move / car_turn / car_stop / car_camera / car_status）
"""
```

- [ ] **Step 6: 跑测试确认通过**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest tests/ -v`
Expected: PASS，全部通过

- [ ] **Step 7: 回归全量测试 + 打快照**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest -v`
Expected: PASS —— 本任务新增用例全绿，前面任务的用例**没有回归**

测试过了再打快照。没有版本控制，这就是唯一的回滚点：

```bash
cd /home/cy && tar czf "voice-chatbot.bak-T5-$(date +%Y%m%d-%H%M%S).tar.gz" \
    --exclude='voice-chatbot/logs' --exclude='__pycache__' \
    --exclude='.pytest_cache' voice-chatbot
```
（Task 0 那份开工快照是总退路，但它退不回「上一个任务刚做完」的状态。）

---

## Task 6: 接线 —— 配置、包装脚本、systemd、对话层

**Files:**
- Modify: `/home/cy/voice-chatbot/config.yaml`
- Modify: `/home/cy/voice-chatbot/core/conversation.py`
- Create: `/home/cy/voice-chatbot/run.sh`
- Modify: `/home/cy/.config/systemd/user/voice-chatbot.service`
- Create: `/home/cy/voice-chatbot/scripts/setup_car_env.sh`

**Interfaces:**
- Consumes: `CarController`（Task 4）、`register_car_tools`（Task 5）
- Produces: 一个会加载 ROS 环境并运行助手可执行包装脚本 `run.sh`

- [ ] **Step 1: 给 `config.yaml` 加 `car:` 段**

先备份（沿用项目现有的 `config.yaml.bak-*` 习惯）：

```bash
cd /home/cy/voice-chatbot && cp -p config.yaml "config.yaml.bak-car-$(date +%m%d-%H%M%S)"
```

插在 `music:` 段之后、`conversation:` 段之前：

```yaml
# ---- 小车（L150Pro）----
# 限幅与默认值放这里而不是写死在代码里 —— 将来标定完成或供电改善，
# 只改这个文件即可，不用改代码重部署。
# 驱动侧的真实限幅：max_vx=0.5 / max_wz=1.5（l150pro_driver 默认参数）。
car:
  default_speed: 0.3          # m/s，car_move 未指定 speed 时用
  max_speed: 0.5              # m/s，与驱动 max_vx 一致
  default_turn_speed_dps: 60  # 度/秒，car_turn 未指定 speed 时用
  max_turn_speed_dps: 120     # 度/秒
  camera_step_deg: 15         # car_camera 未指定 degrees 时的步进
  camera_min_us: 800          # 云台行程下限（teleop 的保守值，防堵转扫齿）
  camera_max_us: 2200
  camera_center_us: 1500
  cmd_rate_hz: 25             # 发帧频率（驱动 cmd_rate_hz=20，留余量）
```

- [ ] **Step 2: 改 `core/conversation.py` —— import**

在 `from tools.music import MusicController, register_music_tools` 下面加：

```python
from tools.car import register_car_tools
from car.controller import CarController
```

- [ ] **Step 3: 改 `core/conversation.py` —— 构造**

在 `register_music_tools(self._registry, self._music)` 之后加：

```python
        # 小车。放置位置同 music —— 它也会起线程，也要在 _on_wake_word 里被叫停。
        # ROS 起不来**不该**拖垮语音助手（天气、音乐、闲聊都还得能用），
        # 所以 start() 只记错误不抛；工具调用时会再试一次，
        # 「先起助手、后起驱动」也能自己恢复。
        self._car = CarController(config)
        register_car_tools(self._registry, self._car)
        self._car.start()
```

- [ ] **Step 4: 改 `core/conversation.py` —— 唤醒词急停**

在 `_on_wake_word` 里，把音乐那行后面补上小车：

```python
        # 喊唤醒词就是「停」的开关：先停音乐并等它真正放开 Speaker，再开麦。
        # 小车同理 —— 这条路径不经过 ASR / LLM，几十毫秒就能停住，是急停。
        # 两者都必须在开麦之前做完。
        self._music.stop()
        self._car.stop_all()
```

- [ ] **Step 5: 改 `core/conversation.py` —— 收尾**

在 `shutdown()` 里 `self._music.stop()` 之后加：

```python
        self._car.close()
```

- [ ] **Step 6: 写 `run.sh`**

**必须在树莓派上用 heredoc 生成，保证 LF 行尾**（从 Windows 拷回会变 CRLF，shebang 就废了）：

```bash
cat > /home/cy/voice-chatbot/run.sh <<'EOF'
#!/usr/bin/env bash
# 语音助手启动包装。
#
# 为什么要包装：systemd 用户单元不会 source ROS 的 setup.bash，
# 而 CarController 需要 rclpy。不写这一层，症状是
# 「ModuleNotFoundError: No module named 'rclpy'」——
# 而且因为 MCP 那边有先例（PATH 不含 ~/.local/bin 导致静默降级），
# 这种失败很容易被当成「小车功能没写对」而查错方向。
set -e

# **ROS 缺失不致命**：小车功能降级（CarController 会在日志里大声报错），
# 但天气、音乐、闲聊都还得能用。所以这里只警告，不退出。
#
# 不能直接 `source` 了事：`set -e` 下 source 一个不存在的文件会立刻退出，
# 而 systemd 单元是 Restart=on-failure —— 那就变成每 10 秒重启一次的
# 崩溃循环，且症状看起来像「加了小车之后助手起不来了」。
for setup in /opt/ros/jazzy/setup.bash "$HOME/l150pro_ws/install/setup.bash"; do
    if [ -f "$setup" ]; then
        # shellcheck disable=SC1090
        source "$setup"
    else
        echo "警告：找不到 $setup —— 小车功能将不可用，其余功能照常" >&2
    fi
done

cd "$HOME/voice-chatbot"
exec /usr/bin/python3 main.py "$@"
EOF
chmod +x /home/cy/voice-chatbot/run.sh
```

- [ ] **Step 7: 验证 run.sh 行尾与可执行**

Run:
```bash
file /home/cy/voice-chatbot/run.sh && \
head -c 200 /home/cy/voice-chatbot/run.sh | od -c | grep -c '\\r' || echo "无 CR，行尾正确"
```
Expected: `Bourne-Again shell script, ASCII text executable` 且无 `\r`

- [ ] **Step 8: 手动验证 run.sh 能加载 ROS 并看到小车工具**

先停掉服务，再手动跑 8 秒看启动日志：

```bash
systemctl --user stop voice-chatbot
timeout 8 /home/cy/voice-chatbot/run.sh 2>&1 | grep -E "小车|car|Error|error" | head -20
```
Expected: 看到 `小车：ROS 桥已就绪`；若驱动没跑，看到 `cmd_vel 上没有订阅者` 的警告（这是正常的）；**不应**看到 `ModuleNotFoundError: No module named 'rclpy'`

- [ ] **Step 9: 改 systemd 单元指向 run.sh**

先备份单元（它是唯一能让助手起来的东西，写坏了就得手工恢复）：

```bash
cp -p ~/.config/systemd/user/voice-chatbot.service \
      ~/.config/systemd/user/voice-chatbot.service.bak-car-$(date +%m%d-%H%M%S)
ls -l ~/.config/systemd/user/voice-chatbot.service*
```

把 `~/.config/systemd/user/voice-chatbot.service` 里的：

```ini
ExecStart=/usr/bin/python3 main.py
```

改成：

```ini
# 用 run.sh 而不是直接跑 python3 —— 它会先 source ROS 2 的 setup.bash。
# systemd 不加载 ROS 环境，而 CarController 需要 rclpy。
ExecStart=/home/cy/voice-chatbot/run.sh
```

在 `Environment=PATH=...` 那行下面加一条注释说明：

```ini
# ROS 2 的环境由 run.sh 负责 source，不在这里写 Environment=
# （setup.bash 是 shell 脚本，Environment= 没法 source 它）。
```

- [ ] **Step 10: 重新加载并重启服务**

```bash
systemctl --user daemon-reload
systemctl --user restart voice-chatbot
sleep 12
systemctl --user status voice-chatbot --no-pager | head -12
```

Expected: `Active: active (running)`

**如果起不来**（`Active: activating (auto-restart)` 或 `failed`），先看日志：

```bash
journalctl --user -u voice-chatbot -n 40 --no-pager
```

回退（三样都要退，只退一样可能留下不一致状态）：

```bash
cp ~/.config/systemd/user/voice-chatbot.service.bak-car-<时间戳> \
   ~/.config/systemd/user/voice-chatbot.service
cd /home/cy/voice-chatbot && cp config.yaml.bak-car-<时间戳> config.yaml
systemctl --user daemon-reload && systemctl --user restart voice-chatbot
```

代码层面的退路是快照 `voice-chatbot.bak-T5-*.tar.gz`。

- [ ] **Step 11: 确认工具数量从 9 变成 14**

```bash
journalctl --user -u voice-chatbot -n 60 --no-pager | grep -E "tools=|MCP: 就绪"
```

Expected: 看到 `LLM: sending ... (tools=14)`（第一次对话后），或至少启动无错误

- [ ] **Step 12: 确认 system_prompt 逐字未改**

```bash
cd /home/cy/voice-chatbot && \
diff <(sed -n '/system_prompt/,/^$/p' "$(ls -t config.yaml.bak-car-* | head -1)") \
     <(sed -n '/system_prompt/,/^$/p' config.yaml) && echo "system_prompt 未改动 ✅"
```
Expected: 无差异输出 + `system_prompt 未改动 ✅`

再整体确认差异只有一个段落：

```bash
cd /home/cy/voice-chatbot && diff "$(ls -t config.yaml.bak-car-* | head -1)" config.yaml
```
Expected: 差异**只有**新增的 `car:` 段，不含任何 `system_prompt` 相关行。

- [ ] **Step 13: 回归全量测试 + 打快照**

Run: `cd /home/cy/voice-chatbot && python3 -m pytest -v`
Expected: PASS —— 本任务新增用例全绿，前面任务的用例**没有回归**

测试过了再打快照。没有版本控制，这就是唯一的回滚点：

```bash
cd /home/cy && tar czf "voice-chatbot.bak-T6-$(date +%Y%m%d-%H%M%S).tar.gz" \
    --exclude='voice-chatbot/logs' --exclude='__pycache__' \
    --exclude='.pytest_cache' voice-chatbot
```
（Task 0 那份开工快照是总退路，但它退不回「上一个任务刚做完」的状态。）

---

## Task 7: 实车验收

前置：电池充满、底盘驱动已启动、车架起来（轮子悬空）先跑一遍方向检查。

**Files:** 无（纯验证）

**Interfaces:**
- Consumes: 前面全部任务
- Produces: 验收结论

- [ ] **Step 1: 启动底盘驱动**

```bash
/home/cy/l150pro_start_driver.sh
```
Expected: `✅ 串口 /dev/l150pro -> ttyACM0` 且无 CRC 报错

- [ ] **Step 2: 另开终端确认电压话题在发**

```bash
source /opt/ros/jazzy/setup.bash && source /home/cy/l150pro_ws/install/setup.bash && \
timeout 5 ros2 topic echo /battery_voltage --once
```
Expected: 出现一个与万用表读数接近的电压值（如 `data: 11.6`）

- [ ] **Step 3: 确认车静止时助手不发帧（安静 pump）**

```bash
source /opt/ros/jazzy/setup.bash && source /home/cy/l150pro_ws/install/setup.bash && \
timeout 6 ros2 topic hz /cmd_vel
```
Expected: `no new messages`（助手没在动时应当完全安静，不干扰 teleop 和前后端）

- [ ] **Step 4: 云台方向核对（车架空状态下）**

喊唤醒词，依次说：
- 「云台往左看看」→ 云台水平轴应当往**左**转
- 「云台往右」→ 往右
- 「抬头」→ 俯仰轴往上
- 「云台回正」→ 回到中位

若方向相反：改 `teleop_l150pro.py` 的 `--pan-invert` / `--tilt-invert` 对应的
`car/kinematics.py` 里 `camera_target_us` 的 sign，改完重跑 Task 1 的测试。

- [ ] **Step 5: 转向核对**

喊唤醒词说「左转九十度」。车应当**逆时针**转约 90°。
若转反：`car/kinematics.py` 的 `_US_PER_DEG` 与此无关——检查 `turn_by` 里
`angle_deg >= 0` 的符号，以及驱动 `gz` 的极性。

- [ ] **Step 6: 走距离核对**

喊唤醒词说「往前走一米」。用卷尺量实际走的直线距离。
偏差大属预期（等效轮距 `1.000` 是外推初值，见记忆 `l150pro-car`）——
记下实测比例，回头用 `track_eff` 标定，**不要**在代码里加魔数补偿。

- [ ] **Step 7: 急停核对**

喊唤醒词启动「一直往前走」（`car_move(forward, 100)`），车走起来后**喊唤醒词**。
Expected: **几十毫秒内**停住（不是 1~2 秒——那是走 LLM 的路径）。

- [ ] **Step 8: 状态核对**

问「还有多少电」「车什么情况」。
Expected: 报出的电压与 Step 2 的读数吻合。

- [ ] **Step 9: 记录验收结果**

把 Step 4/5/6 的实测结论补进 `docs/plans/2026-09-17-car-tools-design.md` 末尾的
「验收记录」小节。

## 执行后订正（2026-09-17 全部做完后补记）

这份计划在**执行过程中**被订正过多处。以代码为准，下面是差异清单：

| 位置 | 计划原本 | 实际改成 | 为什么 |
|---|---|---|---|
| Task 3 `ros_bridge.py` | 裸调 `rclpy.spin_once(node)` | 用**自己的** `SingleThreadedExecutor` | 裸调会落到进程级全局 executor，同进程再有别处裸调就撞 `Executor is already spinning` |
| Task 3 测试 | 模块级 `pytest.importorskip` | 移进 fixture | 模块级 Skipped 会吞掉整个文件，连不依赖 ROS 的用例都跑不了 |
| Task 4 `_pump_until` | 位置传参 `(vx, wz, ...)` | 关键字 `vx=vx, wz=wz` | 签名是 keyword-only，`move_distance` 所有路径 `TypeError` |
| Task 4 `turn_by` | `target = abs(angle_deg)` 直接比弧度 | 全程弧度域累积，出口转回度 | 度/弧度混用 → 永远转不到位，跑满超时后报**假故障** |
| Task 4 `_servo_frame` | 两路都发绝对脉宽 | 只发**被指令过**的路，其余 0 | 否则「往右看」会顺手把俯仰轴命令到中位 |
| Task 4 测试 `_make` | 不建桥 | 先 `controller.start()` | `stop_all` 刻意不建桥，测「发 8 帧」得先有真实使用 |
| Task 5 方向枚举 | `tools/car.py` 手抄一份 | 从 `car.kinematics` 取 | 手抄会静默漂移，schema 发给模型的枚举和控制器不一致 |
| **Task 4 运动模型** | **阻塞直到到位** | **独立线程，工具立刻返回** | ★ 阻塞会占住唤醒线程 → 串口没人读 → **急停失效**。实车复现 |
| Task 6 `run.sh` | `set -e` + 直接 source ROS | 缺文件只警告不退出 | 否则崩成 `Restart=on-failure` 的 10 秒重启循环 |
| Task 6 Step 9 | 直接改 unit | 先备份 unit | 它是唯一能让助手起来的东西 |

另外两条流程上的：项目**没有版本控制**，回滚靠 `voice-chatbot.bak-*.tar.gz` 快照
（开工 + 每任务一份）；`fs_write` **无法覆盖已有文件**，改文件一律用
`proc_exec` heredoc 或精确字符串替换 + 匹配数断言。

---

## 自检清单（写完后逐条核对）

- [ ] 设计文档里每个验收标准都有对应任务：电压话题→Task 2；5 个工具→Task 5；唤醒词急停→Task 6；安静 pump→Task 7 Step 3；system_prompt 未改→Task 6 Step 12
- [ ] `car_move` 方向从 8 个收窄到 6 个（去掉 left/right），设计文档需同步修改
- [ ] 所有类型名跨任务一致：`CarLimits` / `RobotState` / `RosUnavailable` / `CarController`
- [ ] 无 `TODO` / `TBD` / 「同 Task N」这类占位
- [ ] 测试断言不是恒真的（smoke test 第一版就是 `or _ROOT.exists()` 这种空洞断言，已修）
