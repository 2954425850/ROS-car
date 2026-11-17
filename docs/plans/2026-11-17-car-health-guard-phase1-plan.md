# 整车健康守护（carhealth）第一期实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不执行任何恢复动作的前提下，把整车各组件判成 `OK/DEGRADED/STALE/ABSENT/BLOCKED/UNKNOWN`，把每次状态变化与证据写进结构化故障日志，并提供 `status/doctor/history/why` 四个命令。

**Architecture:** 新增独立 Python 包 `pi/carhealth/`。声明式契约（`contracts.py`）描述每个组件的探针与判据；只读探针采集 systemd / ROS 话题 / status 文件 / 网络 / udev / 主机资源；`classify.py` 做状态机与根因挂载；`journal.py` 写 JSONL；`engine.py` 由 `carhealthd` 常驻调用，也被 `carhealthctl doctor` 一次性调用。第一期没有 `recovery.py`，代码里不存在任何执行恢复的路径。

**Tech Stack:** Python 3.12（Ubuntu 24.04 / Pi）、仅标准库、pytest；ROS 2 Jazzy 仅在 `probes/ros_topic.py` 的 rclpy 后端里按需导入。

**设计文档：** `docs/plans/2026-11-17-car-health-guard-design.md`（在分支 `docs/car-health-guard` 上）。

## Global Constraints

- 只允许 Python 标准库；不新增运行时依赖，测试只用 pytest。
- **一期绝不能有任何自动动作**：不重启单元、不开关设备、不发 ROS 指令、不写配置文件。
- 所有模块导入时不得 import rclpy / 不得执行 `systemctl` / 不得访问硬件；一切副作用发生在 `poll()` 或显式 `start()`。
- 所有测试必须离线可跑（Windows 开发机也可以），靠注入 fake runner / 临时文件 / 假时钟。
- 状态文件与日志写入必须原子（`tmp` + `os.replace`）；日志 append-only，绝不重写。
- **state.json 写 tmpfs**（`/run/carhealth`），journal 才落盘；不得每秒写 SD 卡。
- 生产路径必须可注入（构造参数），测试不得写 `~/.local/state` 或真实 `/dev/shm`。
- 阈值只允许写在 `pi/carhealth/carhealth/contracts.py` 一处；故障库规则只写 `faults.py`。
- 面向用户的输出用中文；代码注释可中文，与仓库现有风格一致。
- 不打开串口、不拉视频流、不跑 `ros2 topic echo` 子进程。
- 时间：进程内年龄/间隔用 `time.monotonic()`；cross-process 新鲜度用 mtime/墙钟；journal 每条必须带 `boot_id` 与 `mono`。
- ROS 订阅中 Image / Imu / JointState 一律 `raw=True`（只记时间戳），不反序列化大消息。
- 任何地址都不得写死回退 IP；拿不到就把状态判 `UNKNOWN`。
- 关键判据（字段缺失、BLOCKED 优先级、迟滞）必须做**变异自检**：故意改坏实现，确认测试变红，再改回。


## File Map

```
pi/carhealth/
  pytest.ini
  README.md
  carhealthctl                       # bash 包装，exec python3 -m carhealth.cli
  carhealth/
    __init__.py
    types.py                         # Verdict / Check / ComponentSpec / ProbeResult / ComponentState / Diagnosis / compare
    classify.py                      # evaluate / attach_graph_root_cause / fingerprint
    journal.py                       # JSONL 追加、按天滚动、读取、清理（boot_id/mono）
    contracts.py                     # 全部组件契约（唯一阈值来源）
    faults.py                        # 已知故障库：证据 → 结论 + 动作类别 + 指令
    diagnose.py                      # 规则求值，产出 Diagnosis
    registry.py                      # build_specs / build_probes / build_engine
    engine.py                        # 轮询、迟滞、判定、状态迁移、写 state.json
    daemon.py                        # 常驻入口
    cli.py                           # status / doctor / history / why
    busy.py                          # 互锁信号汇总
    probes/
      __init__.py
      host.py                        # 内存/磁盘/温度/load/wifi
      power.py                       # vcgencmd get_throttled
      systemd.py                     # 批量 systemctl show + 每单元 UnitProbe
      udev.py                        # 稳定名 + udevadm 接口号 + by-id 兜底
      file_status.py                 # status JSON + mtime 年龄
      net.py                         # TCP/ping 探活、K230 地址解析（未知则 UNKNOWN）
      ros_topic.py                   # 只订阅 rclpy 后端 + 纯存储 + 发布者计数
      service_health.py              # NRestarts 增量 / 内存占用
      usb_events.py                  # 内核日志 USB disconnect 计数
      dds.py                         # ss -uan 旧 IP 绑定
      config_drift.py                # ordering cycle / enabled-dead / udev / git / build
  tests/
    test_types.py
    test_classify.py
    test_journal.py
    test_contracts.py
    test_probe_host.py
    test_probe_power.py
    test_probe_systemd.py
    test_probe_udev.py
    test_probe_file_status.py
    test_probe_net.py
    test_probe_service_health.py
    test_probe_usb_events.py
    test_probe_dds.py
    test_probe_config_drift.py
    test_ros_topic.py
    test_busy.py
    test_faults.py
    test_engine.py
    test_registry.py
    test_daemon.py
    test_cli.py
    test_doctor.py
  systemd/carhealthd.service
  tools/inject_fault.sh
```

同时修改：

- `pi/voice-chatbot/utils/heartbeat.py`（新建）、`pi/voice-chatbot/core/conversation.py`、`pi/voice-chatbot/tests/test_heartbeat.py`（新建）
- `pi/raspot_ws/src/arm_grasp/arm_grasp/health.py`（新建）、`grasp_server.py`、`test/test_health.py`（新建）
- `pi/k230-vision/pi/k230pi.py`、`pi/k230-vision/pi/pytest.ini`（新建）、`pi/k230-vision/pi/tests/test_framed_status.py`（新建）
- `pi/systemd/system/carhealthd.service`（新建）
- `pi/deploy.sh`

---

## Task 1: 包骨架与公共类型

**Files:**
- Create: `pi/carhealth/pytest.ini`
- Create: `pi/carhealth/carhealth/__init__.py`
- Create: `pi/carhealth/carhealth/types.py`
- Test: `pi/carhealth/tests/test_types.py`

**Interfaces:**
- Consumes: 无
- Produces: `Verdict`、`CHECK_SEVERITY`、`ProbeResult(ok, evidence, error)`、`Probe`（Protocol）、`Check(probe, op, value, field, level, reason)`、`ComponentSpec(name, display, checks, blocked_by)`、`Diagnosis(id, title, action_class, hint)`、`ComponentState(name, display, verdict, reason, evidence, root_cause, failing, diagnoses, since, updated)`、`compare(actual, op, expected)`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_types.py`：

```python
import pytest

from carhealth.types import (
    CHECK_SEVERITY, Check, ComponentSpec, ComponentState, Diagnosis,
    ProbeResult, Verdict, compare,
)


def test_compare_numeric_operators():
    assert compare(3, "gt", 2)
    assert compare(2, "ge", 2)
    assert compare(1, "lt", 2)
    assert compare(2, "le", 2)
    assert not compare(None, "lt", 2)


def test_compare_equality_and_specials():
    assert compare(True, "eq", True)
    assert compare(True, "truthy", None)
    assert compare(None, "is_none", None)
    assert compare("00", "eq", "00")
    assert not compare(None, "eq", False)     # 探针没报 = 不知道，不是 false


def test_compare_rejects_unknown_op():
    with pytest.raises(ValueError):
        compare(1, "approximately", 1)


def test_verdict_strings_are_stable():
    """日志里存的是 verdict.value，改名等于破坏历史日志兼容性。"""
    assert [v.value for v in Verdict] == [
        "OK", "DEGRADED", "STALE", "ABSENT", "BLOCKED", "UNKNOWN",
    ]


def test_check_severity_only_contains_failure_levels():
    assert CHECK_SEVERITY[Verdict.ABSENT] > CHECK_SEVERITY[Verdict.STALE]
    assert CHECK_SEVERITY[Verdict.STALE] > CHECK_SEVERITY[Verdict.DEGRADED]
    assert CHECK_SEVERITY[Verdict.BLOCKED] > CHECK_SEVERITY[Verdict.ABSENT]


def test_component_state_is_constructible():
    d = Diagnosis(id="usb.disconnect_storm", title="USB 在反复重枚举",
                  action_class="human", hint="换线/换口")
    st = ComponentState(
        name="tof", display="ToF", verdict=Verdict.OK, reason="正常",
        evidence={"topic:/tof/status": {"age_s": 0.1}}, root_cause=None,
        failing=(), diagnoses=(d,), since=1.0, updated=1.0,
    )
    assert st.verdict is Verdict.OK
    assert st.diagnoses[0].action_class == "human"
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_types.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth'`

- [ ] **Step 3: 写最小实现**

`pi/carhealth/pytest.ini`：

```ini
[pytest]
testpaths = tests
python_files = test_*.py
addopts = -q
pythonpath = .
```

`pi/carhealth/carhealth/__init__.py`：

```python
"""整车健康守护。一期只观察、不执行任何恢复动作。"""

__version__ = "0.1.0"
```

`pi/carhealth/carhealth/types.py`：

```python
"""健康判定公共类型。纯标准库，不依赖 ROS / systemd。"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Any, Protocol


class Verdict(enum.Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    STALE = "STALE"
    ABSENT = "ABSENT"
    BLOCKED = "BLOCKED"
    UNKNOWN = "UNKNOWN"


# 只在「多个 check 失败时挑最严重的那个」时使用。
# BLOCKED 代表环境根因（欠压 / 板子离线），比任何单点症状都优先。
CHECK_SEVERITY = {
    Verdict.DEGRADED: 1,
    Verdict.STALE: 2,
    Verdict.ABSENT: 3,
    Verdict.BLOCKED: 4,
}


@dataclass
class ProbeResult:
    ok: bool | None = None
    evidence: dict[str, Any] = field(default_factory=dict)
    error: str | None = None


class Probe(Protocol):
    name: str
    interval_s: float

    def poll(self) -> ProbeResult: ...


@dataclass(frozen=True)
class Check:
    probe: str
    op: str
    value: Any = None
    field: str | None = None
    level: Verdict = Verdict.STALE
    reason: str = ""


@dataclass(frozen=True)
class ComponentSpec:
    name: str
    display: str
    checks: tuple[Check, ...]
    blocked_by: tuple[str, ...] = ()


@dataclass(frozen=True)
class Diagnosis:
    """已知故障库的命中结果。只给结论与指令，不改变六态判定。"""
    id: str
    title: str
    action_class: str      # wait / restart_unit / restart_stack / human / fix_config
    hint: str


@dataclass
class ComponentState:
    name: str
    display: str
    verdict: Verdict
    reason: str
    evidence: dict[str, dict]
    root_cause: str | None
    failing: tuple[str, ...]
    diagnoses: tuple[Diagnosis, ...]
    since: float
    updated: float


def compare(actual: Any, op: str, expected: Any) -> bool:
    """判据比较。字段缺省由调用方处理成「不知道」，这里只处理已取到的值。"""
    if op == "exists":
        return actual is not None
    if op == "truthy":
        return bool(actual)
    if op == "falsy":
        return not bool(actual)
    if op == "is_none":
        return actual is None
    if op == "eq":
        return actual == expected
    if op == "ne":
        return actual != expected
    if op in ("lt", "le", "gt", "ge"):
        try:
            a, b = float(actual), float(expected)
        except (TypeError, ValueError):
            return False
        if op == "lt":
            return a < b
        if op == "le":
            return a <= b
        if op == "gt":
            return a > b
        return a >= b
    raise ValueError(f"未知比较操作: {op}")
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_types.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/pytest.ini pi/carhealth/carhealth/__init__.py pi/carhealth/carhealth/types.py pi/carhealth/tests/test_types.py
git commit -m "feat(health): carhealth 包骨架与公共类型"
```

---

## Task 2: 判据引擎（分类器、根因挂载、指纹）

**Files:**
- Create: `pi/carhealth/carhealth/classify.py`
- Test: `pi/carhealth/tests/test_classify.py`

**Interfaces:**
- Consumes: `types.py` 全部类型
- Produces: `Evaluation(verdict, reason, evidence, failing_reasons, failing_probes, probes_ok, root_cause)`、`evaluate(spec, results, states)`、`attach_graph_root_cause(evals, specs)`、`fingerprint(component, verdict, root_cause, failing)`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_classify.py`：

```python
from dataclasses import replace

from carhealth.classify import (
    Evaluation, attach_graph_root_cause, evaluate, fingerprint,
)
from carhealth.types import Check, ComponentSpec, ProbeResult, Verdict


def spec(*checks, blocked_by=()):
    return ComponentSpec(name="x", display="X", checks=tuple(checks), blocked_by=blocked_by)


def ok_probe(**evidence):
    return ProbeResult(ok=True, evidence=evidence)


def test_all_checks_pass_is_ok():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="太旧"))
    ev = evaluate(s, {"p": ok_probe(age_s=0.2)}, {})
    assert ev.verdict is Verdict.OK
    assert ev.failing_reasons == ()


def test_stale_check_reports_level_and_reason():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="太旧"))
    ev = evaluate(s, {"p": ok_probe(age_s=3.0)}, {})
    assert ev.verdict is Verdict.STALE
    assert ev.reason == "太旧"
    assert ev.failing_probes == ("p",)


def test_worst_level_wins_but_all_reasons_are_kept():
    s = spec(
        Check("p", "gt", 0, field="fps", level=Verdict.DEGRADED, reason="fps 掉了"),
        Check("p", "lt", 2.0, field="age_s", level=Verdict.STALE, reason="没数据"),
    )
    ev = evaluate(s, {"p": ok_probe(fps=0, age_s=5)}, {})
    assert ev.verdict is Verdict.STALE
    assert "fps 掉了" in ev.failing_reasons
    assert "没数据" in ev.failing_reasons


def test_missing_field_is_unknown_not_ok():
    """探针没报这个字段 ≠ 这个字段合格。这条是防假绿的关键。"""
    s = spec(Check("p", "eq", True, field="connected", level=Verdict.STALE, reason="没连上"))
    ev = evaluate(s, {"p": ok_probe()}, {})
    assert ev.verdict is Verdict.UNKNOWN
    assert "探针不可用" in ev.reason


def test_probe_itself_failed_is_unknown():
    s = spec(Check("p", "eq", True, field="x", level=Verdict.STALE, reason="坏"))
    ev = evaluate(s, {"p": ProbeResult(ok=None, error="systemctl 挂了")}, {})
    assert ev.verdict is Verdict.UNKNOWN


def test_blocked_dependency_wins_over_local_symptoms():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.BLOCKED, "欠压", {}, (), (), {}, root_cause="power")
    ev = evaluate(s, {"p": ok_probe(age_s=9)}, {"power": power})
    assert ev.verdict is Verdict.BLOCKED
    assert ev.root_cause == "power"
    assert "欠压" in ev.reason


def test_healthy_component_is_not_masked_by_blocked_dependency():
    """欠压期间底盘完全正常时不能显示 BLOCKED——否则真故障会被掩盖。"""
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.BLOCKED, "欠压", {}, (), (), {}, root_cause="power")
    ev = evaluate(s, {"p": ok_probe(age_s=0.1)}, {"power": power})
    assert ev.verdict is Verdict.OK
    assert ev.root_cause is None


def test_unknown_component_with_blocked_dependency_is_blocked():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.BLOCKED, "欠压", {}, (), (), {}, root_cause="power")
    ev = evaluate(s, {"p": ProbeResult(ok=None, error="失明")}, {"power": power})
    assert ev.verdict is Verdict.BLOCKED
    assert ev.root_cause == "power"


def test_blocked_dependency_not_blocked_falls_through():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.OK, "正常", {}, (), (), {})
    ev = evaluate(s, {"p": ok_probe(age_s=0.1)}, {"power": power})
    assert ev.verdict is Verdict.OK


def test_is_none_operator():
    s = spec(Check("p", "is_none", None, field="last_error", level=Verdict.DEGRADED, reason="有错误"))
    assert evaluate(s, {"p": ok_probe(last_error=None)}, {}).verdict is Verdict.OK
    assert evaluate(s, {"p": ok_probe(last_error="boom")}, {}).verdict is Verdict.DEGRADED


def _graph_case(publishers, n=3, unit_active=True):
    specs = []
    results = {"unit": ProbeResult(ok=True, evidence={"active": unit_active})}
    for name in ("a", "b", "c")[:n]:
        specs.append(ComponentSpec(name, name, (
            Check("unit", "eq", True, field="active", level=Verdict.ABSENT, reason="unit 不在"),
            Check("topic:" + name, "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
        )))
        results["topic:" + name] = ok_probe(age_s=9, publishers=publishers)
    evals = {s.name: evaluate(s, results, {}) for s in specs}
    return specs, evals


def test_zero_publishers_means_discovery_fault():
    specs, evals = _graph_case(publishers=0)
    fixed = attach_graph_root_cause(evals, specs)
    assert all(fixed[n].verdict is Verdict.BLOCKED for n in ("a", "b", "c"))
    assert all(fixed[n].root_cause == "ros_discovery" for n in ("a", "b", "c"))


def test_publishers_alive_but_no_data_means_data_path_fault():
    specs, evals = _graph_case(publishers=2)
    fixed = attach_graph_root_cause(evals, specs)
    assert all(fixed[n].root_cause == "ros_data_path" for n in ("a", "b", "c"))


def test_unknown_publisher_count_does_not_attribute():
    """没有 rclpy 后端时 publishers 缺失；不能靠症状数量硬猜。"""
    specs, evals = _graph_case(publishers=None)
    assert attach_graph_root_cause(evals, specs) == evals


def test_graph_fault_needs_three_candidates():
    specs, evals = _graph_case(publishers=0, n=2)
    assert attach_graph_root_cause(evals, specs) == evals


def test_graph_fault_ignores_components_whose_unit_is_down():
    specs, evals = _graph_case(publishers=0, unit_active=False)
    fixed = attach_graph_root_cause(evals, specs)
    assert fixed["a"].verdict is Verdict.ABSENT


def test_fingerprint_is_stable_and_sensitive():
    a = fingerprint("tof", Verdict.STALE, None, ["没数据"])
    b = fingerprint("tof", Verdict.STALE, None, ["没数据"])
    c = fingerprint("tof", Verdict.STALE, None, ["连不上"])
    assert a == b
    assert a != c
    assert len(a) == 12
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_classify.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.classify'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/classify.py`：

```python
"""把探针结果判成组件状态，并挂载环境根因。纯函数，方便单测。"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, replace

from .types import (
    CHECK_SEVERITY, Check, ComponentSpec, ProbeResult, Verdict, compare,
)


@dataclass(frozen=True)
class Evaluation:
    verdict: Verdict
    reason: str
    evidence: dict[str, dict]          # probe 名 -> 该探针的 evidence（避免 age_s 互相覆盖）
    failing_reasons: tuple[str, ...]
    failing_probes: tuple[str, ...]
    probes_ok: dict[str, bool | None]
    root_cause: str | None = None


def _dedupe(items):
    return tuple(dict.fromkeys(items))


def evaluate(spec: ComponentSpec, results: dict[str, ProbeResult],
             states: dict[str, Evaluation]) -> Evaluation:
    probes_ok: dict[str, bool | None] = {}
    failing: list[tuple[Check, str]] = []
    unknown: list[str] = []

    for check in spec.checks:
        r = results.get(check.probe)
        probes_ok[check.probe] = None if r is None else r.ok
        if r is None or r.ok is not True:
            unknown.append(check.probe)
            continue
        if check.field is not None:
            if check.field not in r.evidence:
                unknown.append(check.probe)
                continue
            actual = r.evidence[check.field]
        else:
            actual = r.evidence
        if not compare(actual, check.op, check.value):
            failing.append((check, check.probe))

    evidence = {}
    for probe_name in _dedupe(c.probe for c in spec.checks):
        r = results.get(probe_name)
        if r is not None:
            evidence[probe_name] = dict(r.evidence)

    blocked_dep = None
    blocked_reason = ""
    for dep in spec.blocked_by:
        dep_ev = states.get(dep)
        if dep_ev is not None and dep_ev.verdict is Verdict.BLOCKED:
            blocked_dep, blocked_reason = dep, dep_ev.reason
            break

    if failing:
        worst = max(CHECK_SEVERITY[c.level] for c, _ in failing)
        top = [c for c, _ in failing if CHECK_SEVERITY[c.level] == worst]
        base = Evaluation(
            verdict=top[0].level,
            reason=top[0].reason or ("%s 判据不满足" % top[0].probe),
            evidence=evidence,
            failing_reasons=_dedupe(c.reason or c.probe for c, _ in failing),
            failing_probes=_dedupe(p for _, p in failing),
            probes_ok=probes_ok,
        )
        if blocked_dep is None:
            return base
        # 只有自己确实失败/失明时，才把根因挂上去；健康的组件不许被 BLOCKED 掩盖。
        return replace(
            base,
            verdict=Verdict.BLOCKED,
            root_cause=blocked_dep,
            reason="被 %s 压住（%s）；本组件判据：%s" % (
                blocked_dep, blocked_reason, base.reason),
        )

    if unknown:
        reason = "探针不可用：%s" % "、".join(_dedupe(unknown))
        if blocked_dep is not None:
            return Evaluation(
                verdict=Verdict.BLOCKED,
                reason="被 %s 压住（%s）；%s" % (blocked_dep, blocked_reason, reason),
                evidence=evidence, failing_reasons=(), failing_probes=(),
                probes_ok=probes_ok, root_cause=blocked_dep,
            )
        return Evaluation(
            verdict=Verdict.UNKNOWN, reason=reason, evidence=evidence,
            failing_reasons=(), failing_probes=(), probes_ok=probes_ok,
        )

    return Evaluation(Verdict.OK, "正常", evidence, (), (), probes_ok)


def attach_graph_root_cause(evals: dict[str, Evaluation],
                            specs: tuple[ComponentSpec, ...]) -> dict[str, Evaluation]:
    """≥3 个「单元健康但话题全停」的组件同时异常时，用发布者数量区分根因。

    - 所有失败话题 `count_publishers == 0` → `ros_discovery`（DDS 发现/网络）
    - 任一失败话题有发布者但没数据   → `ros_data_path`（QoS/MTU/网络）
    - 发布者数量未知（没有 rclpy 后端）→ **不归因**，宁可 UNKNOWN，不靠症状数量猜
    """
    by_name = {s.name: s for s in specs}
    candidates: list[str] = []
    pub_counts: list[int] = []

    for name, ev in evals.items():
        if ev.verdict not in (Verdict.STALE, Verdict.DEGRADED):
            continue
        spec = by_name[name]
        has_topic_check = any(c.probe.startswith("topic:") for c in spec.checks)
        if not has_topic_check:
            continue
        other_probes = {c.probe for c in spec.checks if not c.probe.startswith("topic:")}
        if any(ev.probes_ok.get(p) is not True for p in other_probes):
            continue
        if not ev.failing_probes or not all(p.startswith("topic:") for p in ev.failing_probes):
            continue
        for probe_name in ev.failing_probes:
            publishers = ev.evidence.get(probe_name, {}).get("publishers")
            if not isinstance(publishers, int):
                return evals          # 证据不足，不归因
            pub_counts.append(publishers)
        candidates.append(name)

    if len(candidates) < 3:
        return evals

    if any(count > 0 for count in pub_counts):
        cause = "ros_data_path"
        reason = "有发布者但收不到数据：查 QoS 兼容、MTU 分片、WiFi 质量"
    else:
        cause = "ros_discovery"
        reason = "多个话题 0 发布者：判 DDS 发现/网络故障，通常要整条 ROS 栈重启"

    out = dict(evals)
    for name in candidates:
        out[name] = replace(
            evals[name], verdict=Verdict.BLOCKED, root_cause=cause, reason=reason,
        )
    return out


def fingerprint(component: str, verdict: Verdict, root_cause: str | None,
                failing: list[str] | tuple[str, ...]) -> str:
    """同一组件、同一状态、同一组判据失败原因 -> 同一指纹。用于二期熔断。"""
    raw = "|".join([component, verdict.value, root_cause or "", *sorted(failing)])
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()[:12]
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_classify.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/classify.py pi/carhealth/tests/test_classify.py
git commit -m "feat(health): 判据引擎与根因挂载"
```

---

## Task 3: JSONL 故障日志

**Files:**
- Create: `pi/carhealth/carhealth/journal.py`
- Test: `pi/carhealth/tests/test_journal.py`

**Interfaces:**
- Consumes: 无
- Produces: `Journal(dir_path, clock=time.time, keep_days=30)`，方法 `record(event) -> dict`、`read(since_ts=None) -> list[dict]`、`prune()`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_journal.py`：

```python
import json
import os

from carhealth.journal import Journal


def test_record_appends_timestamped_json_line(tmp_path):
    j = Journal(tmp_path, clock=lambda: 1763388000.0)
    event = j.record({"event": "incident_open", "component": "tof"})
    assert event["ts"] == 1763388000.0
    path = next(tmp_path.glob("journal-*.jsonl"))
    lines = path.read_text().splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["component"] == "tof"


def test_record_keeps_explicit_ts(tmp_path):
    j = Journal(tmp_path, clock=lambda: 200.0)
    assert j.record({"ts": 123.0, "event": "x"})["ts"] == 123.0


def test_record_adds_boot_id_and_mono(tmp_path):
    """Pi 无 RTC：墙钟在对时前不可信，排序/过滤要靠 boot_id + mono。"""
    j = Journal(tmp_path, clock=lambda: 100.0,
                boot_id_fn=lambda: "boot-a", mono_fn=lambda: 12.5)
    event = j.record({"event": "x"})
    assert event["boot_id"] == "boot-a"
    assert event["mono"] == 12.5


def test_read_filters_by_since(tmp_path):
    times = iter([100.0, 200.0, 300.0])
    j = Journal(tmp_path, clock=lambda: next(times))
    for i in range(3):
        j.record({"event": "e", "n": i})
    got = j.read(since_ts=150.0)
    assert [e["n"] for e in got] == [1, 2]


def test_read_skips_bad_lines(tmp_path):
    j = Journal(tmp_path)
    j.record({"event": "ok"})
    path = next(tmp_path.glob("journal-*.jsonl"))
    with open(path, "a", encoding="utf-8") as f:
        f.write("not json\n")
    assert [e["event"] for e in j.read()] == ["ok"]


def test_read_since_uses_mono_for_current_boot(tmp_path):
    """NTP 对时前墙钟可能差几小时；当前 boot 的事件必须按 mono 过滤。"""
    j = Journal(tmp_path, clock=lambda: 1_000_000.0,
                boot_id_fn=lambda: "b1", mono_fn=lambda: 900.0)
    j.record({"event": "new", "mono": 850.0})
    j.record({"event": "old", "mono": 100.0})
    j.record({"event": "previous_boot", "ts": 999_990.0, "boot_id": "b0", "mono": 5.0})
    got = j.read_since(120.0, boot_id="b1", now_mono=900.0)
    assert [e["event"] for e in got] == ["new", "previous_boot"]


def test_prune_removes_old_files_only(tmp_path):
    old = tmp_path / "journal-20200101.jsonl"
    old.write_text("{}\n")
    new = tmp_path / "journal-20990101.jsonl"
    new.write_text("{}\n")
    Journal(tmp_path, clock=lambda: 1763388000.0).prune()
    assert not old.exists()
    assert new.exists()
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_journal.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.journal'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/journal.py`：

```python
"""按天滚动的 JSONL 故障日志。append-only，永不重写。"""

from __future__ import annotations

import json
import os
import re
import time
from pathlib import Path

_NAME_RE = re.compile(r"^journal-(\d{8})\.jsonl$")


def read_boot_id(path: str = "/proc/sys/kernel/random/boot_id") -> str:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read().strip() or "unknown"
    except OSError:
        return "unknown"


class Journal:
    def __init__(self, dir_path, clock=time.time, keep_days: int = 30,
                 boot_id_fn=read_boot_id, mono_fn=time.monotonic):
        self.dir = Path(dir_path)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._clock = clock
        self._boot_id = boot_id_fn
        self._mono = mono_fn
        self.keep_days = keep_days

    def _path_for(self, ts: float) -> Path:
        return self.dir / ("journal-%s.jsonl" % time.strftime("%Y%m%d", time.localtime(ts)))

    def record(self, event: dict) -> dict:
        event = dict(event)
        event.setdefault("ts", self._clock())
        event.setdefault("boot_id", self._boot_id())
        event.setdefault("mono", round(self._mono(), 3))
        line = json.dumps(event, ensure_ascii=False, sort_keys=True, default=str)
        with open(self._path_for(event["ts"]), "a", encoding="utf-8") as f:
            f.write(line + "\n")
            f.flush()
            os.fsync(f.fileno())
        return event

    def read(self, since_ts: float | None = None) -> list[dict]:
        out: list[dict] = []
        for path in sorted(self.dir.glob("journal-*.jsonl")):
            try:
                lines = path.read_text(encoding="utf-8").splitlines()
            except OSError:
                continue
            for line in lines:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if since_ts is not None and event.get("ts", 0) < since_ts:
                    continue
                out.append(event)
        out.sort(key=lambda e: e.get("ts", 0))
        return out

    def read_since(self, seconds: float, boot_id: str | None = None,
                   now_mono: float | None = None) -> list[dict]:
        """当前 boot 用 mono 过滤（墙钟可能被 NTP 校正），其它 boot 用墙钟。"""
        boot = self._boot_id() if boot_id is None else boot_id
        now_mono = self._mono() if now_mono is None else now_mono
        cutoff_wall = self._clock() - seconds
        anchor_mono = now_mono - seconds
        out = []
        for event in self.read():
            mono = event.get("mono")
            if event.get("boot_id") == boot and isinstance(mono, (int, float)):
                if mono >= anchor_mono:
                    out.append(event)
            elif event.get("ts", 0) >= cutoff_wall:
                out.append(event)
        out.sort(key=lambda e: (
            e.get("boot_id") != boot, e.get("mono", 0), e.get("ts", 0)))
        return out

    def prune(self) -> None:
        cutoff = self._clock() - self.keep_days * 86400
        cutoff_day = time.strftime("%Y%m%d", time.localtime(cutoff))
        for path in self.dir.glob("journal-*.jsonl"):
            m = _NAME_RE.match(path.name)
            if m and m.group(1) < cutoff_day:
                try:
                    path.unlink()
                except OSError:
                    pass
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_journal.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/journal.py pi/carhealth/tests/test_journal.py
git commit -m "feat(health): JSONL 故障日志"
```

---

## Task 4: 组件契约

**Files:**
- Create: `pi/carhealth/carhealth/contracts.py`
- Test: `pi/carhealth/tests/test_contracts.py`

**Interfaces:**
- Consumes: `types.py`
- Produces: `build_specs() -> tuple[ComponentSpec, ...]`；常量 `VOICE_HEARTBEAT_PATHS`、`RESULTD_STATUS`、`FRAMED_STATUS`、`LATEST_IDR`、`ROSBRIDGE_ADDR`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_contracts.py`：

```python
from carhealth.contracts import build_specs
from carhealth.types import CHECK_SEVERITY


def test_component_names_unique_and_ordered_for_blocking():
    specs = build_specs()
    names = [s.name for s in specs]
    assert len(names) == len(set(names))
    seen = set()
    for s in specs:
        for dep in s.blocked_by:
            assert dep in seen, f"{s.name} 的根因 {dep} 必须排在它前面"
        seen.add(s.name)


def test_every_check_has_known_level_and_probe_name():
    known_probes = {
        "power", "host",
        "unit:ps2-teleop", "unit:tof-a010", "unit:k230-resultd",
        "unit:k230-bridge", "unit:k230-framed", "unit:rosbridge",
        "unit:arm-grasp",
        "udev:l150pro", "udev:a010", "udev:js0",
        "file:resultd-status", "file:framed-status", "file:latest-idr",
        "file:voice-heartbeat",
        "net:board", "net:rosbridge",
        "topic:/tof/status", "topic:/tof/depth", "topic:/wheel_odom",
        "topic:/imu/data_raw", "topic:/arm/feedback", "topic:/arm/online",
        "topic:/k230/result", "topic:/health/arm_grasp",
        "topic:/battery_voltage", "topic:/chassis_faults",
    }
    for spec in build_specs():
        assert spec.display
        assert spec.checks
        for c in spec.checks:
            assert c.probe in known_probes, f"{spec.name} 引用了未知探针 {c.probe}"
            assert c.level in CHECK_SEVERITY
            assert c.reason, f"{spec.name} 的 {c.probe} 判据缺中文原因"


def test_power_and_board_are_root_causes_with_no_dependencies():
    specs = {s.name: s for s in build_specs()}
    assert specs["power"].blocked_by == ()
    assert specs["board"].blocked_by == ()
    assert specs["arm"].blocked_by == ("power",)
    assert specs["k230_link"].blocked_by == ("board",)
    assert specs["k230_video"].blocked_by == ("board",)


def test_tof_has_interface_number_check():
    """同颗 FT2232 的通道 B 也能被打开、但一个字节都收不到（2026-10-14 实测）。"""
    tof = {s.name: s for s in build_specs()}["tof"]
    fields = {(c.probe, c.field, c.op, c.value) for c in tof.checks}
    assert ("udev:a010", "interface", "eq", "00") in fields


def test_tof_does_not_check_last_error():
    """tof_a010 重连成功后不清 last_error（serial_dev._disconnect 只写不清），
    拿它当判据会永久误报。"""
    tof = {s.name: s for s in build_specs()}["tof"]
    assert all(c.field != "last_error" for c in tof.checks)


def test_tof_reports_by_id_fallback_as_degraded():
    tof = {s.name: s for s in build_specs()}["tof"]
    fields = {(c.probe, c.field, c.op, c.value, c.level) for c in tof.checks}
    assert ("udev:a010", "resolved_by", "eq", "short", Verdict.DEGRADED) in fields


def test_battery_component_warns_below_threshold():
    battery = {s.name: s for s in build_specs()}["battery"]
    fields = {(c.probe, c.field, c.op, c.value) for c in battery.checks}
    assert ("topic:/battery_voltage", "data", "gt", 11.5) in fields
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_contracts.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.contracts'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/contracts.py`：

```python
"""组件契约：唯一阈值来源。改判据只改这里。

判据设计见 docs/plans/2026-11-17-car-health-guard-design.md §5。
"""

from __future__ import annotations

from pathlib import Path

from .types import Check, ComponentSpec, Verdict

VOICE_HEARTBEAT_PATHS = (
    Path("/dev/shm/voice-heartbeat.json"),
    Path.home() / ".local/state/voice-chatbot/heartbeat.json",
)
RESULTD_STATUS = Path("/tmp/k230/resultd-status.json")
FRAMED_STATUS = Path("/tmp/k230/framed-status.json")
LATEST_IDR = Path("/dev/shm/k230/latest-idr.h264")
ROSBRIDGE_ADDR = ("127.0.0.1", 9090)


def _unit(unit: str, name: str | None = None) -> Check:
    return Check(
        probe="unit:" + (name or unit),
        op="eq", value=True, field="active",
        level=Verdict.ABSENT,
        reason="%s 单元不在运行（systemd ActiveState != active）" % unit,
    )


def build_specs() -> tuple[ComponentSpec, ...]:
    return (
        ComponentSpec("power", "供电", (
            Check("power", "eq", False, field="uv_now", level=Verdict.BLOCKED,
                  reason="当前欠压（vcgencmd bit0）"),
            Check("power", "eq", False, field="uv_recent", level=Verdict.BLOCKED,
                  reason="进程观察到 10 分钟内出现过欠压（USB 设备可能已掉）"),
            Check("power", "eq", False, field="uv_since_boot", level=Verdict.DEGRADED,
                  reason="本次开机出现过欠压（历史位，时间未知）"),
            Check("power", "eq", False, field="freq_capped", level=Verdict.DEGRADED,
                  reason="CPU 正在降频（throttled bit1）"),
            Check("power", "eq", False, field="soft_temp", level=Verdict.DEGRADED,
                  reason="软温度上限触发（throttled bit3）"),
        )),

        ComponentSpec("board", "K230 板", (
            Check("net:board", "eq", True, field="ping_ok", level=Verdict.BLOCKED,
                  reason="K230 板 ping 不通（掉电 / 掉 WiFi / IP 变了）"),
        )),

        ComponentSpec("host", "主机资源", (
            Check("host", "gt", 10, field="disk_free_pct", level=Verdict.DEGRADED,
                  reason="根分区剩余 <10%"),
            Check("host", "gt", 150, field="mem_available_mb", level=Verdict.DEGRADED,
                  reason="可用内存 <150MB"),
            Check("host", "lt", 80, field="soc_temp_c", level=Verdict.DEGRADED,
                  reason="SoC 温度 ≥80°C"),
            Check("host", "lt", 6.0, field="load1", level=Verdict.DEGRADED,
                  reason="1 分钟负载 ≥6（实测出现过 17.8，会放大抖动）"),
        )),

        ComponentSpec("battery", "电池", (
            Check("topic:/battery_voltage", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="/battery_voltage 超过 3s 没有数据"),
            Check("topic:/battery_voltage", "gt", 11.5, field="data", level=Verdict.DEGRADED,
                  reason="电池电压 <11.5V"),
        )),

        ComponentSpec("chassis", "底盘", (
            _unit("ps2-teleop"),
            Check("udev:l150pro", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="底盘板串口 /dev/l150pro 不存在"),
            Check("udev:js0", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="PS2 接收器 /dev/input/js0 不存在"),
            Check("topic:/wheel_odom", "lt", 1.0, field="age_s", level=Verdict.STALE,
                  reason="/wheel_odom 超过 1s 没有数据"),
            Check("topic:/imu/data_raw", "lt", 1.5, field="age_s", level=Verdict.STALE,
                  reason="/imu/data_raw 超过 1.5s 没有数据"),
            Check("topic:/battery_voltage", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="/battery_voltage 超过 3s 没有数据"),
            Check("topic:/chassis_faults", "eq", 0, field="data", level=Verdict.DEGRADED,
                  reason="底盘固件报告故障位（check /chassis_faults 与驱动日志）"),
        ), blocked_by=("power",)),

        ComponentSpec("arm", "机械臂", (
            _unit("ps2-teleop"),
            Check("udev:l150pro", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="底盘/臂共用串口 /dev/l150pro 不存在"),
            Check("topic:/arm/feedback", "lt", 0.5, field="age_s", level=Verdict.STALE,
                  reason="/arm/feedback 超过 0.5s 没有数据"),
            Check("topic:/arm/online", "eq", True, field="data", level=Verdict.STALE,
                  reason="机械臂未 READY（/arm/online=false）"),
        ), blocked_by=("power",)),

        ComponentSpec("tof", "ToF 深度", (
            _unit("tof-a010"),
            Check("udev:a010", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="ToF 设备节点不存在（/dev/a010 与 by-id if00 都没有）"),
            Check("udev:a010", "eq", "short", field="resolved_by", level=Verdict.DEGRADED,
                  reason="短名 /dev/a010 不存在，走了 by-id（99-tof-a010.rules 可能未装）"),
            Check("udev:a010", "eq", "00", field="interface", level=Verdict.ABSENT,
                  reason="ToF 接到了 FT2232 的通道 B（接口号应为 00）"),
            Check("topic:/tof/status", "lt", 2.0, field="age_s", level=Verdict.STALE,
                  reason="/tof/status 超过 2s 没有更新"),
            Check("topic:/tof/status", "eq", True, field="connected", level=Verdict.DEGRADED,
                  reason="ToF 串口未连接"),
            Check("topic:/tof/status", "gt", 0, field="fps", level=Verdict.DEGRADED,
                  reason="ToF 帧率为 0"),
            Check("topic:/tof/depth", "lt", 2.0, field="age_s", level=Verdict.STALE,
                  reason="/tof/depth 超过 2s 没有数据"),
        ), blocked_by=("power",)),

        ComponentSpec("grasp", "抓取服务", (
            _unit("arm-grasp"),
            Check("topic:/health/arm_grasp", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="/health/arm_grasp 心跳超过 3s"),
            Check("topic:/health/arm_grasp", "lt", 1.0, field="feedback_age_s",
                  level=Verdict.STALE,
                  reason="抓取服务收不到 /arm/feedback（臂驱动可能已停）"),
        ), blocked_by=("power",)),

        ComponentSpec("k230_link", "K230 结果链路", (
            _unit("k230-resultd"),
            _unit("k230-bridge"),
            Check("file:resultd-status", "lt", 7.0, field="age_s", level=Verdict.STALE,
                  reason="resultd 状态文件超过 7s 未更新"),
            Check("topic:/k230/result", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="/k230/result 超过 3s 没有数据"),
        ), blocked_by=("board",)),

        ComponentSpec("k230_video", "K230 视频链路", (
            _unit("k230-framed"),
            Check("file:framed-status", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="framed 状态文件超过 3s 未更新"),
            Check("file:framed-status", "eq", True, field="connected", level=Verdict.DEGRADED,
                  reason="板子没有连上 8555 推流口"),
            Check("file:latest-idr", "lt", 5.0, field="age_s", level=Verdict.STALE,
                  reason="最新 IDR 关键帧超过 5s（GOP 间隔需现场确认）"),
        ), blocked_by=("board",)),

        ComponentSpec("rosbridge", "rosbridge", (
            _unit("rosbridge"),
            Check("net:rosbridge", "eq", True, field="tcp_ok", level=Verdict.DEGRADED,
                  reason="127.0.0.1:9090 连不上"),
        )),

        ComponentSpec("voice", "语音助手", (
            Check("file:voice-heartbeat", "lt", 5.0, field="age_s", level=Verdict.STALE,
                  reason="语音助手心跳超过 5s（进程卡死或退出）"),
            Check("file:voice-heartbeat", "lt", 180.0, field="busy_for_s", level=Verdict.STALE,
                  reason="语音助手在忙状态超过 180s（疑似 API 卡死）"),
            Check("file:voice-heartbeat", "eq", True, field="wakeword_ready",
                  level=Verdict.STALE,
                  reason="唤醒模块串口不可用（USB 重枚举后喊它没反应）"),
            Check("file:voice-heartbeat", "eq", True, field="wakeword_thread_alive",
                  level=Verdict.STALE,
                  reason="唤醒词监听线程不在"),
            Check("file:voice-heartbeat", "eq", True, field="mic_open",
                  level=Verdict.DEGRADED,
                  reason="麦克风句柄未打开"),
        )),
    )
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_contracts.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/contracts.py pi/carhealth/tests/test_contracts.py
git commit -m "feat(health): 组件契约与阈值"
```

---

## Task 5: 主机资源与供电探针

**Files:**
- Create: `pi/carhealth/carhealth/probes/__init__.py`
- Create: `pi/carhealth/carhealth/probes/host.py`
- Create: `pi/carhealth/carhealth/probes/power.py`
- Test: `pi/carhealth/tests/test_probe_host.py`
- Test: `pi/carhealth/tests/test_probe_power.py`

**Interfaces:**
- Consumes: `ProbeResult`
- Produces: `HostProbe(name="host", interval_s=5.0)`、`PowerProbe(name="power", interval_s=2.0, fake_output=None, clock=time.monotonic, recent_s=600.0)`、`parse_throttled(text) -> int | None`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_probe_host.py`：

```python
from carhealth.probes.host import HostProbe, parse_cpu_stat


def test_parse_cpu_stat():
    total, idle = parse_cpu_stat("cpu 100 0 100 800 0 0 0 0 0 0\ncpu0 ...\n")
    assert total == 1000
    assert idle == 800


def test_cpu_pct_needs_two_samples(tmp_path):
    stat = tmp_path / "stat"
    stat.write_text("cpu 100 0 100 800 0 0 0 0 0 0\n")
    now = [100.0]
    p = HostProbe(meminfo=str(tmp_path / "nope"), thermal=str(tmp_path / "nope"),
                  disk_path=str(tmp_path), wireless=str(tmp_path / "nope"),
                  stat_path=str(stat), getloadavg=lambda: (0.0, 0.0, 0.0),
                  clock=lambda: now[0])
    assert "cpu_pct" not in p.poll().evidence
    stat.write_text("cpu 200 0 200 1200 0 0 0 0 0 0\n")
    now[0] = 101.0
    assert p.poll().evidence["cpu_pct"] == 33.3


def test_host_evidence_from_fake_files(tmp_path):
    mem = tmp_path / "meminfo"
    mem.write_text(
        "MemTotal:       8000000 kB\n"
        "MemAvailable:   1048576 kB\n"
        "SwapTotal:      1000000 kB\n"
        "SwapFree:        500000 kB\n"
    )
    thermal = tmp_path / "thermal"
    thermal.write_text("55000\n")
    wireless = tmp_path / "wireless"
    wireless.write_text(
        "Inter-| sta-|   Quality        |   Discarded packets\n"
        " face | tus | link level noise |  nwid  crypt   frag  retry   misc\n"
        " wlan0: 0000   45.  -55.  -256        0      0      0      0      0\n"
    )
    r = HostProbe(meminfo=str(mem), thermal=str(thermal), disk_path=str(tmp_path),
                  wireless=str(wireless), getloadavg=lambda: (1.5, 1.2, 1.0)).poll()
    assert r.ok is True
    assert r.evidence["mem_available_mb"] == 1024.0
    assert r.evidence["soc_temp_c"] == 55.0
    assert r.evidence["load1"] == 1.5
    assert r.evidence["wifi_link"] == 45.0
    assert r.evidence["wifi_signal_dbm"] == -55.0
    assert 0 < r.evidence["disk_free_pct"] <= 100


def test_missing_files_leave_fields_absent(tmp_path):
    r = HostProbe(meminfo=str(tmp_path / "nope"), thermal=str(tmp_path / "nope2"),
                  disk_path=str(tmp_path), wireless=str(tmp_path / "nope3"),
                  getloadavg=lambda: (_ for _ in ()).throw(OSError("n/a"))).poll()
    assert "mem_available_mb" not in r.evidence
    assert "soc_temp_c" not in r.evidence
    assert "wifi_link" not in r.evidence
    assert "disk_free_pct" in r.evidence
```

`pi/carhealth/tests/test_probe_power.py`：

```python
from carhealth.probes.power import PowerProbe, parse_throttled


def test_parse_throttled():
    assert parse_throttled("throttled=0x50000\n") == 0x50000
    assert parse_throttled("garbage") is None


def test_undervoltage_now_blocks():
    r = PowerProbe(fake_output="throttled=0x1").poll()
    assert r.evidence["uv_now"] is True
    assert r.evidence["uv_recent"] is True


def test_history_bit_alone_is_not_recent():
    """历史位只在开机后置位、时间未知；doctor 每次新建探针，
    绝不能因此永远判 FAIL。"""
    r = PowerProbe(fake_output="throttled=0x50000").poll()
    assert r.evidence["uv_now"] is False
    assert r.evidence["uv_recent"] is False
    assert r.evidence["uv_since_boot"] is True


def test_observed_undervoltage_stays_recent_for_window():
    now = [1000.0]
    outputs = iter(["throttled=0x1", "throttled=0x10000", "throttled=0x10000"])
    p = PowerProbe(fake_output=None, runner=lambda *a, **k: _cp(str(next(outputs))),
                   clock=lambda: now[0], recent_s=600)
    assert p.poll().evidence["uv_recent"] is True
    now[0] += 100
    assert p.poll().evidence["uv_recent"] is True
    now[0] += 601
    assert p.poll().evidence["uv_recent"] is False


def test_other_throttle_bits_are_exposed():
    r = PowerProbe(fake_output="throttled=0xe").poll()
    assert r.evidence["freq_capped"] is True
    assert r.evidence["throttled_now"] is True
    assert r.evidence["soft_temp"] is True


def test_no_vcgencmd_is_unknown_not_blocked():
    r = PowerProbe(fake_output=None, runner=lambda *a, **k: (_ for _ in ()).throw(OSError("no vcgencmd"))).poll()
    assert r.ok is None
    assert "vcgencmd" in r.error


def _cp(stdout, rc=0):
    import subprocess
    return subprocess.CompletedProcess([], rc, stdout, "")
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_host.py tests/test_probe_power.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.probes'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/probes/__init__.py`：

```python
"""只读探针。"""
```

`pi/carhealth/carhealth/probes/host.py`：

```python
"""内存 / 磁盘 / 温度 / load / WiFi。只读 /proc、statvfs、thermal。"""

from __future__ import annotations

import os
import time

from ..types import ProbeResult


def parse_cpu_stat(text: str) -> tuple[int, int]:
    """返回 (总 jiffies, 空闲 jiffies)。只看 aggregate 的 `cpu` 行。"""
    for line in (text or "").splitlines():
        if line.startswith("cpu "):
            values = [int(v) for v in line.split()[1:9]]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            return total, idle
    raise ValueError("没有 cpu 行")


def parse_wireless(text: str) -> dict:
    for line in (text or "").splitlines():
        if ":" not in line or "|" in line:
            continue
        name, _, rest = line.partition(":")
        parts = rest.split()
        if len(parts) < 3:
            continue
        try:
            return {
                "wifi_iface": name.strip(),
                "wifi_link": float(parts[1]),
                "wifi_signal_dbm": float(parts[2]),
            }
        except ValueError:
            continue
    return {}


class HostProbe:
    name = "host"
    interval_s = 5.0

    def __init__(self, meminfo: str = "/proc/meminfo", disk_path: str = "/",
                 thermal: str = "/sys/class/thermal/thermal_zone0/temp",
                 wireless: str = "/proc/net/wireless", stat_path: str = "/proc/stat",
                 getloadavg=os.getloadavg, clock=time.monotonic):
        self.meminfo = meminfo
        self.disk_path = disk_path
        self.thermal = thermal
        self.wireless = wireless
        self.stat_path = stat_path
        self._getloadavg = getloadavg
        self._clock = clock
        self._prev_cpu = None

    def poll(self) -> ProbeResult:
        ev: dict = {}
        try:
            values = {}
            with open(self.meminfo, "r", encoding="utf-8") as f:
                for line in f:
                    key, _, rest = line.partition(":")
                    values[key.strip()] = int(rest.split()[0])
            ev["mem_available_mb"] = round(values["MemAvailable"] / 1024.0, 1)
            ev["swap_free_mb"] = round(values.get("SwapFree", 0) / 1024.0, 1)
        except (OSError, KeyError, ValueError, IndexError):
            pass
        try:
            st = os.statvfs(self.disk_path)
            ev["disk_free_pct"] = round(100.0 * st.f_bavail / st.f_blocks, 1)
        except OSError:
            pass
        try:
            with open(self.thermal, "r", encoding="utf-8") as f:
                ev["soc_temp_c"] = round(int(f.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            pass
        try:
            ev["load1"] = round(float(self._getloadavg()[0]), 2)
        except (OSError, ValueError, IndexError):
            pass
        try:
            with open(self.wireless, "r", encoding="utf-8") as f:
                ev.update(parse_wireless(f.read()))
        except OSError:
            pass
        try:
            with open(self.stat_path, "r", encoding="utf-8") as f:
                total, idle = parse_cpu_stat(f.read())
            if self._prev_cpu is not None:
                d_total = total - self._prev_cpu[0]
                d_idle = idle - self._prev_cpu[1]
                if d_total > 0:
                    ev["cpu_pct"] = round(100.0 * (1 - d_idle / d_total), 1)
            self._prev_cpu = (total, idle)
        except (OSError, ValueError, IndexError):
            pass
        return ProbeResult(ok=True, evidence=ev)
```

`pi/carhealth/carhealth/probes/power.py`：

```python
"""Pi 供电状态。掉压会打掉 USB 设备（2026-10-14 实测：连臂串口一起掉）。"""

from __future__ import annotations

import re
import subprocess
import time

from ..types import ProbeResult

_THROTTLED_RE = re.compile(r"throttled=(0x[0-9a-fA-F]+|\d+)")


def parse_throttled(text: str) -> int | None:
    m = _THROTTLED_RE.search(text or "")
    if not m:
        return None
    try:
        return int(m.group(1), 0)
    except ValueError:
        return None


class PowerProbe:
    name = "power"
    interval_s = 2.0

    def __init__(self, runner=subprocess.run, fake_output: str | None = None,
                 clock=time.monotonic, recent_s: float = 600.0):
        self._runner = runner
        self._fake = fake_output
        self._clock = clock
        self._recent_s = recent_s
        self._uv_seen: float | None = None

    def _read(self) -> str | None:
        if self._fake is not None:
            return self._fake
        try:
            r = self._runner(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return r.stdout if r.returncode == 0 else None

    def poll(self) -> ProbeResult:
        text = self._read()
        bits = parse_throttled(text) if text is not None else None
        if bits is None:
            return ProbeResult(ok=None, error="vcgencmd get_throttled 不可用")
        now = self._clock()
        uv_now = bool(bits & 0x1)
        uv_since_boot = bool(bits & 0x10000)
        # 只有**亲眼看到**当前欠压才更新 recent；历史位时间未知，只算 Degraded。
        if uv_now:
            self._uv_seen = now
        uv_recent = uv_now or (
            self._uv_seen is not None and now - self._uv_seen < self._recent_s
        )
        return ProbeResult(ok=True, evidence={
            "throttled": hex(bits),
            "uv_now": uv_now,
            "uv_recent": uv_recent,
            "uv_since_boot": uv_since_boot,
            "freq_capped": bool(bits & 0x2),
            "throttled_now": bool(bits & 0x4),
            "soft_temp": bool(bits & 0x8),
        })
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_host.py tests/test_probe_power.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/probes pi/carhealth/tests/test_probe_host.py pi/carhealth/tests/test_probe_power.py
git commit -m "feat(health): 主机资源与供电探针"
```

---

## Task 6: systemd 与 udev 探针

**Files:**
- Create: `pi/carhealth/carhealth/probes/systemd.py`
- Create: `pi/carhealth/carhealth/probes/udev.py`
- Test: `pi/carhealth/tests/test_probe_systemd.py`
- Test: `pi/carhealth/tests/test_probe_udev.py`

**Interfaces:**
- Consumes: `ProbeResult`
- Produces: `SystemdCache(units, runner=subprocess.run, interval_s=1.0, clock=time.monotonic)`、`UnitProbe(name, unit, cache)`、`UdevProbe(name, path, expect_interface=None, realpath=os.path.realpath, exists=os.path.exists, read_text=None)`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_probe_systemd.py`：

```python
import subprocess

from carhealth.probes.systemd import SystemdCache, UnitProbe

SHOW_OUTPUT = """\
Id=ps2-teleop.service
ActiveState=active
SubState=running
NRestarts=2
MemoryCurrent=125829120
CPUUsageNSec=9876543210

Id=tof-a010.service
ActiveState=failed
SubState=failed
NRestarts=7
MemoryCurrent=[not set]
CPUUsageNSec=[not set]
"""


class FakeRunner:
    def __init__(self, out=SHOW_OUTPUT, rc=0):
        self.out = out
        self.rc = rc
        self.calls = 0

    def __call__(self, cmd, **kw):
        self.calls += 1
        return subprocess.CompletedProcess(cmd, self.rc, self.out, "")


def test_batches_one_call_for_all_units():
    runner = FakeRunner()
    cache = SystemdCache(["ps2-teleop", "tof-a010"], runner=runner)
    a = UnitProbe("unit:ps2-teleop", "ps2-teleop", cache).poll()
    b = UnitProbe("unit:tof-a010", "tof-a010", cache).poll()
    assert runner.calls == 1
    assert a.evidence["active"] is True
    assert a.evidence["n_restarts"] == 2
    assert a.evidence["memory_mb"] == 120.0
    assert a.evidence["cpu_usage_ns"] == 9876543210
    assert b.evidence["active"] is False
    assert b.evidence["active_state"] == "failed"
    assert b.evidence["memory_mb"] is None


def test_systemctl_failure_is_unknown():
    runner = FakeRunner(out="", rc=1)
    cache = SystemdCache(["ps2-teleop"], runner=runner)
    r = UnitProbe("unit:ps2-teleop", "ps2-teleop", cache).poll()
    assert r.ok is None
    assert "systemctl" in r.error


def test_refresh_interval_is_respected():
    now = [0.0]
    runner = FakeRunner()
    cache = SystemdCache(["ps2-teleop"], runner=runner, interval_s=1.0, clock=lambda: now[0])
    probe = UnitProbe("unit:ps2-teleop", "ps2-teleop", cache)
    probe.poll()
    now[0] = 0.5
    probe.poll()
    assert runner.calls == 1
    now[0] = 1.1
    probe.poll()
    assert runner.calls == 2
```

`pi/carhealth/tests/test_probe_udev.py`：

```python
import subprocess

from carhealth.probes.udev import UdevProbe


class FakeRunner:
    def __init__(self, out="", rc=0):
        self.out, self.rc = out, rc

    def __call__(self, cmd, **kw):
        return subprocess.CompletedProcess(cmd, self.rc, self.out, "")


def test_missing_node_and_no_by_id():
    r = UdevProbe("udev:a010", ["/dev/a010", "/dev/serial/by-id/*SIPEED*"],
                  exists=lambda p: False, glob_fn=lambda p: []).poll()
    assert r.evidence["exists"] is False
    assert "interface" not in r.evidence


def test_short_name_and_interface_from_udevadm():
    r = UdevProbe(
        "udev:a010", ["/dev/a010"], expect_interface="00",
        exists=lambda p: True, realpath=lambda p: "/dev/ttyUSB0",
        runner=FakeRunner("DEVPATH=/x\nID_USB_INTERFACE_NUM=00\n"),
    ).poll()
    assert r.evidence["exists"] is True
    assert r.evidence["resolved_by"] == "short"
    assert r.evidence["real"] == "/dev/ttyUSB0"
    assert r.evidence["interface"] == "00"


def test_by_id_fallback_is_reported():
    by_id = "/dev/serial/by-id/usb-SIPEED_Meta_Sense_Lite_123-if00-port0"
    r = UdevProbe(
        "udev:a010", ["/dev/a010", "/dev/serial/by-id/*SIPEED*if00-port0"],
        expect_interface="00",
        exists=lambda p: False,
        glob_fn=lambda p: [by_id],
        realpath=lambda p: "/dev/ttyUSB0",
        runner=FakeRunner("ID_USB_INTERFACE_NUM=00\n"),
    ).poll()
    assert r.evidence["exists"] is True
    assert r.evidence["resolved_by"] == "by_id"
    assert r.evidence["path"] == by_id


def test_udevadm_failure_omits_interface_instead_of_guessing():
    r = UdevProbe(
        "udev:a010", ["/dev/a010"], expect_interface="00",
        exists=lambda p: True, realpath=lambda p: "/dev/ttyUSB0",
        runner=FakeRunner("", rc=1),
    ).poll()
    assert "interface" not in r.evidence
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_systemd.py tests/test_probe_udev.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.probes.systemd'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/probes/systemd.py`：

```python
"""批量 systemctl show，避免每个单元每秒起一个子进程。"""

from __future__ import annotations

import subprocess
import time

from ..types import ProbeResult


def _parse_show(text: str) -> dict[str, dict]:
    units: dict[str, dict] = {}
    block: dict[str, str] = {}
    for line in (text or "").splitlines() + [""]:
        if not line.strip():
            if "Id" in block:
                units[block["Id"]] = block
            block = {}
            continue
        key, _, value = line.partition("=")
        block[key.strip()] = value.strip()
    return units


class SystemdCache:
    def __init__(self, units, runner=subprocess.run, interval_s: float = 1.0,
                 clock=time.monotonic):
        self.units = tuple(units)
        self._runner = runner
        self._interval = interval_s
        self._clock = clock
        self._last: float | None = None
        self._data: dict[str, dict] = {}
        self._error: str | None = None

    def _refresh(self) -> None:
        self._last = self._clock()
        cmd = ["systemctl", "show", "-p", "Id", "-p", "ActiveState",
               "-p", "SubState", "-p", "NRestarts", "-p", "MemoryCurrent",
               "-p", "CPUUsageNSec", *self.units]
        try:
            r = self._runner(cmd, capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as e:
            self._data, self._error = {}, "systemctl 执行失败: %r" % (e,)
            return
        if r.returncode != 0:
            self._data, self._error = {}, "systemctl 返回 %d" % r.returncode
            return
        self._data = _parse_show(r.stdout)
        self._error = None

    @staticmethod
    def _int(value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def get(self, unit: str, full_unit: str) -> ProbeResult:
        if self._last is None or self._clock() - self._last >= self._interval:
            self._refresh()
        if self._error:
            return ProbeResult(ok=None, error=self._error)
        data = self._data.get(full_unit)
        if data is None:
            return ProbeResult(ok=None, error="systemctl 没有返回 %s" % full_unit)
        mem_bytes = self._int(data.get("MemoryCurrent"))
        return ProbeResult(ok=True, evidence={
            "active": data.get("ActiveState") == "active",
            "active_state": data.get("ActiveState"),
            "sub_state": data.get("SubState"),
            "n_restarts": self._int(data.get("NRestarts")) or 0,
            "memory_mb": None if mem_bytes is None else round(mem_bytes / 1048576.0, 1),
            "cpu_usage_ns": self._int(data.get("CPUUsageNSec")),
        })


class UnitProbe:
    def __init__(self, name: str, unit: str, cache: SystemdCache, interval_s: float = 1.0):
        self.name = name
        self.interval_s = interval_s
        self._unit = unit
        self._full = unit if unit.endswith(".service") else unit + ".service"
        self._cache = cache

    def poll(self) -> ProbeResult:
        return self._cache.get(self._unit, self._full)
```

`pi/carhealth/carhealth/probes/udev.py`：

```python
"""udev 稳定名、by-id 兜底与 USB 接口号。

同一个 FT2232 出两个 tty；只按 vid/pid 匹配会抢同一个名字，
接到通道 B 时「节点存在、能打开、但一个字节都收不到」（2026-10-14 实测）。
接口号用 `udevadm info -q property` 读 `ID_USB_INTERFACE_NUM`——
从 sysfs 读对 FTDI 的 ttyUSB 要经过 usb-serial 端口再上一级，容易读错。
"""

from __future__ import annotations

import glob as _glob
import os
import subprocess

from ..types import ProbeResult


class UdevProbe:
    def __init__(self, name: str, paths, expect_interface: str | None = None,
                 realpath=os.path.realpath, exists=os.path.exists,
                 glob_fn=_glob.glob, runner=subprocess.run,
                 interval_s: float = 5.0):
        self.name = name
        self.interval_s = interval_s
        self.paths = (paths,) if isinstance(paths, str) else tuple(paths)
        self.expect_interface = expect_interface
        self._realpath = realpath
        self._exists = exists
        self._glob = glob_fn
        self._runner = runner

    def _resolve(self):
        for index, pattern in enumerate(self.paths):
            kind = "short" if index == 0 else "by_id"
            if any(ch in pattern for ch in "*?["):
                matches = self._glob(pattern)
                if matches:
                    return matches[0], kind
            elif self._exists(pattern):
                return pattern, kind
        return None, None

    def _interface(self, real: str) -> str | None:
        try:
            r = self._runner(["udevadm", "info", "-q", "property", "-n", real],
                             capture_output=True, text=True, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if r.returncode != 0:
            return None
        for line in r.stdout.splitlines():
            if line.startswith("ID_USB_INTERFACE_NUM="):
                return line.split("=", 1)[1].strip()
        return None

    def poll(self) -> ProbeResult:
        path, resolved_by = self._resolve()
        if path is None:
            return ProbeResult(ok=True, evidence={"exists": False, "real": None})
        real = self._realpath(path)
        ev: dict = {"exists": True, "real": real, "path": path,
                    "resolved_by": resolved_by}
        if self.expect_interface is not None:
            iface = self._interface(real)
            if iface is not None:
                ev["interface"] = iface
        return ProbeResult(ok=True, evidence=ev)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_systemd.py tests/test_probe_udev.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/probes/systemd.py pi/carhealth/carhealth/probes/udev.py pi/carhealth/tests/test_probe_systemd.py pi/carhealth/tests/test_probe_udev.py
git commit -m "feat(health): systemd 与 udev 探针"
```

---

## Task 7: 状态文件与网络探针

**Files:**
- Create: `pi/carhealth/carhealth/probes/file_status.py`
- Create: `pi/carhealth/carhealth/probes/net.py`
- Test: `pi/carhealth/tests/test_probe_file_status.py`
- Test: `pi/carhealth/tests/test_probe_net.py`

**Interfaces:**
- Consumes: `ProbeResult`
- Produces: `FileStatusProbe(name, paths, parse_json=True, clock=time.monotonic, wall=time.time)`、`TcpProbe(name, host, port, timeout=1.0)`、`PingProbe(name, host_fn, runner=subprocess.run)`、`resolve_board_host(env=None, status_path=...) -> str | None`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_probe_file_status.py`：

```python
import json
import os
import time

from carhealth.probes.file_status import FileStatusProbe


def test_reads_json_and_age(tmp_path):
    path = tmp_path / "status.json"
    path.write_text(json.dumps({"connected": True, "fps": 12.5}))
    r = FileStatusProbe("file:x", [path], wall=lambda: os.path.getmtime(path) + 1.5).poll()
    assert r.evidence["exists"] is True
    assert r.evidence["connected"] is True
    assert r.evidence["fps"] == 12.5
    assert r.evidence["age_s"] == 1.5


def test_missing_file_has_none_age_so_checks_fail(tmp_path):
    r = FileStatusProbe("file:x", [tmp_path / "nope.json"]).poll()
    assert r.evidence["exists"] is False
    assert r.evidence["age_s"] is None


def test_malformed_json_keeps_age_and_marks_error(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json")
    r = FileStatusProbe("file:x", [path]).poll()
    assert "parse_error" in r.evidence
    assert r.evidence["age_s"] is not None


def test_binary_file_without_json(tmp_path):
    path = tmp_path / "idr.h264"
    path.write_bytes(b"\x00\x00\x00\x01")
    r = FileStatusProbe("file:x", [path], parse_json=False).poll()
    assert r.evidence["exists"] is True
    assert "parse_error" not in r.evidence


def test_picks_newest_of_candidate_paths(tmp_path):
    old = tmp_path / "old.json"
    new = tmp_path / "new.json"
    old.write_text("{}")
    new.write_text("{}")
    os.utime(old, (1000, 1000))
    r = FileStatusProbe("file:x", [old, new], wall=lambda: os.path.getmtime(new) + 2).poll()
    assert r.evidence["path"] == str(new)
    assert r.evidence["age_s"] == 2.0
```

`pi/carhealth/tests/test_probe_net.py`：

```python
import json
import subprocess

from carhealth.probes.net import PingProbe, TcpProbe, resolve_board_host


def test_tcp_ok():
    r = TcpProbe("net:x", "127.0.0.1", 9090, connect=lambda addr, timeout: _Closer()).poll()
    assert r.evidence["tcp_ok"] is True


def test_tcp_failure_is_measured_not_raised():
    def boom(addr, timeout):
        raise OSError("refused")

    r = TcpProbe("net:x", "127.0.0.1", 9090, connect=boom).poll()
    assert r.ok is True
    assert r.evidence["tcp_ok"] is False
    assert "refused" in r.evidence["error"]


class _Closer:
    def close(self):
        pass


def test_ping_uses_host_fn_each_poll():
    hosts = iter(["10.0.0.1", "10.0.0.2"])
    seen = []

    def runner(cmd, **kw):
        seen.append(cmd[-1])
        return subprocess.CompletedProcess(cmd, 0, "ok", "")

    p = PingProbe("net:board", host_fn=lambda: next(hosts), runner=runner)
    assert p.poll().evidence["ping_ok"] is True
    assert p.poll().evidence["ping_ok"] is True
    assert seen == ["10.0.0.1", "10.0.0.2"]


def test_ping_exception_is_false():
    def runner(cmd, **kw):
        raise OSError("no ping")

    r = PingProbe("net:board", host_fn=lambda: "10.0.0.1", runner=runner).poll()
    assert r.evidence["ping_ok"] is False


def test_ping_without_known_host_is_unknown_not_offline():
    """Pi 重启后 /tmp 清空、resultd 还没有 peer；不能拿写死的旧 IP 判板子离线。"""
    r = PingProbe("net:board", host_fn=lambda: None,
                  runner=lambda *a, **k: (_ for _ in ()).throw(AssertionError("不应 ping"))).poll()
    assert r.ok is None
    assert "地址未知" in r.error


def test_resolve_board_host_prefers_env_then_peer_then_none(tmp_path):
    status = tmp_path / "status.json"
    status.write_text(json.dumps({"peer": "10.64.35.77:54321"}))
    assert resolve_board_host(env={"K230_HOST": "1.2.3.4"}, status_path=status) == "1.2.3.4"
    assert resolve_board_host(env={}, status_path=status) == "10.64.35.77"
    assert resolve_board_host(env={}, status_path=tmp_path / "nope.json") is None
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_file_status.py tests/test_probe_net.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.probes.file_status'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/probes/file_status.py`：

```python
"""status 文件的 mtime 新鲜度 + JSON 内容。沿用 k230ctl status 的语义。"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

from ..types import ProbeResult


class FileStatusProbe:
    def __init__(self, name: str, paths, parse_json: bool = True,
                 clock=time.monotonic, wall=time.time, interval_s: float = 1.0):
        self.name = name
        self.interval_s = interval_s
        if isinstance(paths, (str, Path)):
            paths = [paths]
        self.paths = tuple(Path(p) for p in paths)
        self.parse_json = parse_json
        self._clock = clock
        self._wall = wall

    def poll(self) -> ProbeResult:
        best: tuple[Path, float] | None = None
        for path in self.paths:
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if best is None or mtime > best[1]:
                best = (path, mtime)

        if best is None:
            return ProbeResult(ok=True, evidence={
                "exists": False, "age_s": None, "path": None,
            })

        path, mtime = best
        age = max(0.0, self._wall() - mtime)
        ev: dict = {"exists": True}
        if self.parse_json:
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    ev.update(data)
            except (OSError, ValueError) as e:
                ev["parse_error"] = str(e)
        ev["age_s"] = round(age, 3)
        ev["path"] = str(path)
        return ProbeResult(ok=True, evidence=ev)
```

`pi/carhealth/carhealth/probes/net.py`：

```python
"""TCP / ping 探活。不做任何协议交互，连上就关。"""

from __future__ import annotations

import json
import os
import socket
import subprocess

from ..types import ProbeResult


def resolve_board_host(env=None, status_path="/tmp/k230/resultd-status.json") -> str | None:
    """返回 None = 地址未知。**绝不写死回退 IP**：网段变过（192.168.1.x → 10.198.146.x），
    写死的旧地址只会把「不知道」伪装成「板子离线」。"""
    env = os.environ if env is None else env
    if "K230_HOST" in env:
        return env["K230_HOST"]
    try:
        with open(status_path, "r", encoding="utf-8") as f:
            peer = json.load(f).get("peer")
        if isinstance(peer, str) and ":" in peer:
            return peer.rsplit(":", 1)[0]
    except (OSError, ValueError, AttributeError):
        pass
    return None


class TcpProbe:
    def __init__(self, name: str, host: str, port: int, timeout: float = 1.0,
                 connect=socket.create_connection, interval_s: float = 5.0):
        self.name = name
        self.interval_s = interval_s
        self.host = host
        self.port = port
        self.timeout = timeout
        self._connect = connect

    def poll(self) -> ProbeResult:
        try:
            sock = self._connect((self.host, self.port), timeout=self.timeout)
            sock.close()
            return ProbeResult(ok=True, evidence={"tcp_ok": True, "error": None})
        except OSError as e:
            return ProbeResult(ok=True, evidence={"tcp_ok": False, "error": str(e)})


class PingProbe:
    def __init__(self, name: str, host_fn, runner=subprocess.run,
                 timeout: float = 2.0, interval_s: float = 5.0):
        self.name = name
        self.interval_s = interval_s
        self._host_fn = host_fn
        self._runner = runner
        self._timeout = timeout

    def poll(self) -> ProbeResult:
        host = self._host_fn()
        if not host:
            return ProbeResult(
                ok=None,
                error="K230 地址未知（resultd 无 peer 且未设 K230_HOST）",
                evidence={"ping_ok": None, "host": None},
            )
        try:
            r = self._runner(["ping", "-c", "1", "-W", "1", host],
                             capture_output=True, text=True, timeout=self._timeout)
            ok = r.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            ok = False
        return ProbeResult(ok=True, evidence={"ping_ok": ok, "host": host})
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_file_status.py tests/test_probe_net.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/probes/file_status.py pi/carhealth/carhealth/probes/net.py pi/carhealth/tests/test_probe_file_status.py pi/carhealth/tests/test_probe_net.py
git commit -m "feat(health): 状态文件与网络探针"
```

---

## Task 8: ROS 话题采集

**Files:**
- Create: `pi/carhealth/carhealth/probes/ros_topic.py`
- Test: `pi/carhealth/tests/test_ros_topic.py`

**Interfaces:**
- Consumes: `ProbeResult`
- Produces: `RosTopicStore(clock=time.monotonic)`（`record(topic, payload)`、`snapshot(topic)`、`set_publisher_counter(fn)`）、`TopicProbe(name, topic, store)`、`RosNodeBackend(store)`（`start() -> bool`、`stop()`、`reason`）、转换函数 `tof_status_evidence`、`arm_health_evidence`、`twist_evidence`、`odom_evidence`、`bool_evidence`、`float_evidence`、`int_evidence`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_ros_topic.py`：

```python
from carhealth.probes.ros_topic import (
    RosTopicStore, TopicProbe, arm_health_evidence, bool_evidence, float_evidence,
    int_evidence, odom_evidence, tof_status_evidence, twist_evidence,
)


def test_snapshot_age_and_payload():
    now = [100.0]
    store = RosTopicStore(clock=lambda: now[0])
    store.record("/tof/status", {"connected": True, "fps": 12.0})
    now[0] = 101.5
    r = TopicProbe("topic:/tof/status", "/tof/status", store).poll()
    assert r.evidence["age_s"] == 1.5
    assert r.evidence["count"] == 1
    assert r.evidence["connected"] is True


def test_never_seen_topic_has_none_age():
    store = RosTopicStore(clock=lambda: 1.0)
    r = TopicProbe("topic:/x", "/x", store).poll()
    assert r.evidence["age_s"] is None
    assert r.evidence["count"] == 0


def test_publisher_count_is_included():
    store = RosTopicStore(clock=lambda: 1.0)
    store.set_publisher_counter(lambda topic: 0 if topic == "/x" else 3)
    store.record("/x", {})
    assert store.snapshot("/x").evidence["publishers"] == 0
    assert store.snapshot("/never").evidence["publishers"] == 3


def test_publisher_counter_failure_is_none_not_zero():
    def boom(topic):
        raise RuntimeError("graph api down")

    store = RosTopicStore(clock=lambda: 1.0)
    store.set_publisher_counter(boom)
    assert store.snapshot("/x").evidence["publishers"] is None


def test_store_is_thread_safe_enough_for_counter():
    store = RosTopicStore(clock=lambda: 1.0)
    for _ in range(3):
        store.record("/x", {})
    assert store.snapshot("/x").evidence["count"] == 3


def test_negative_age_is_clamped():
    now = [100.0]
    store = RosTopicStore(clock=lambda: now[0])
    store.record("/x", {})
    now[0] = 99.0
    assert store.snapshot("/x").evidence["age_s"] == 0.0


def test_tof_status_keeps_only_contract_fields():
    ev = tof_status_evidence({"connected": True, "fps": 9.0, "reconnects": 2,
                              "age_s": 0.2, "last_error": None, "huge": "x" * 1000})
    assert ev["fps"] == 9.0
    assert "huge" not in ev


def test_arm_health_and_simple_converters():
    assert arm_health_evidence({"busy": True, "pid": 1, "feedback_age_s": 0.2})["busy"] is True
    assert bool_evidence(True)["data"] is True
    assert float_evidence(12.3)["data"] == 12.3
    assert int_evidence(5)["data"] == 5
    assert twist_evidence(0.2, 0.0)["linear_x"] == 0.2
    assert odom_evidence(0.5, 0.1)["linear_x"] == 0.5
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_ros_topic.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.probes.ros_topic'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/probes/ros_topic.py`：

```python
"""ROS 话题新鲜度。

纯存储部分可单测；rclpy 后端只在 start() 里导入，ROS 不可用时只影响话题类探针。
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass

from ..types import ProbeResult

_TOF_FIELDS = ("connected", "connecting", "frames", "fps", "reconnects",
               "age_s", "device", "port", "last_error", "frame_id")
_ARM_HEALTH_FIELDS = ("busy", "pid", "uptime_s", "state", "feedback_age_s",
                      "last_goal_wall")


def _pick(data: dict, fields) -> dict:
    return {k: data[k] for k in fields if k in data}


def tof_status_evidence(data: dict) -> dict:
    return _pick(data, _TOF_FIELDS)


def arm_health_evidence(data: dict) -> dict:
    return _pick(data, _ARM_HEALTH_FIELDS)


def bool_evidence(value: bool) -> dict:
    return {"data": bool(value)}


def float_evidence(value: float) -> dict:
    return {"data": round(float(value), 3)}


def int_evidence(value: int) -> dict:
    return {"data": int(value)}


def twist_evidence(linear_x: float, angular_z: float) -> dict:
    return {"linear_x": round(float(linear_x), 3),
            "angular_z": round(float(angular_z), 3)}


def odom_evidence(linear_x: float, angular_z: float = 0.0) -> dict:
    return twist_evidence(linear_x, angular_z)


@dataclass
class _Record:
    last_mono: float | None = None
    count: int = 0
    payload: dict | None = None


class RosTopicStore:
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._recs: dict[str, _Record] = {}
        self._publisher_count = None

    def set_publisher_counter(self, fn) -> None:
        """rclpy 后端注入 node.count_publishers；没有后端时为 None。"""
        self._publisher_count = fn

    def record(self, topic: str, payload: dict | None = None) -> None:
        with self._lock:
            rec = self._recs.setdefault(topic, _Record())
            rec.last_mono = self._clock()
            rec.count += 1
            rec.payload = dict(payload or {})

    def _publishers(self, topic: str):
        if self._publisher_count is None:
            return None
        try:
            return int(self._publisher_count(topic))
        except Exception:                                   # noqa: BLE001
            return None

    def snapshot(self, topic: str) -> ProbeResult:
        publishers = self._publishers(topic)
        with self._lock:
            rec = self._recs.get(topic)
            if rec is None or rec.last_mono is None:
                return ProbeResult(ok=True, evidence={
                    "age_s": None, "count": 0, "publishers": publishers,
                })
            age = max(0.0, self._clock() - rec.last_mono)
            ev = dict(rec.payload or {})
            ev["age_s"] = round(age, 3)
            ev["count"] = rec.count
            ev["publishers"] = publishers
            return ProbeResult(ok=True, evidence=ev)


class TopicProbe:
    def __init__(self, name: str, topic: str, store: RosTopicStore, interval_s: float = 0.5):
        self.name = name
        self.topic = topic
        self.interval_s = interval_s
        self._store = store

    def poll(self) -> ProbeResult:
        return self._store.snapshot(self.topic)


class RosNodeBackend:
    """单订阅节点 + 独立 spin 线程。"""

    def __init__(self, store: RosTopicStore):
        self._store = store
        self._node = None
        self._executor = None
        self._thread = None
        self.reason: str | None = None

    def start(self) -> bool:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image, Imu, JointState
            from std_msgs.msg import Bool, Float32, Int32, String
        except Exception as e:                              # noqa: BLE001
            self.reason = "rclpy/消息包不可用: %s" % e
            return False

        try:
            rclpy.init(args=None)
            node = rclpy.create_node("carhealth_probe")
            self._node = node

            def sub_json(topic, convert):
                def cb(msg):
                    try:
                        data = json.loads(msg.data)
                    except ValueError:
                        data = {}
                    if isinstance(data, dict):
                        self._store.record(topic, convert(data))
                node.create_subscription(String, topic, cb, 10)

            self._store.set_publisher_counter(node.count_publishers)

            sub_json("/tof/status", tof_status_evidence)
            sub_json("/health/arm_grasp", arm_health_evidence)

            node.create_subscription(String, "/k230/result",
                                     lambda m: self._store.record("/k230/result", {}), 10)
            # 高频/大消息只记到达时间，不反序列化：Pi 本来就容易过载。
            node.create_subscription(
                Image, "/tof/depth",
                lambda m: self._store.record("/tof/depth", {}),
                qos_profile_sensor_data, raw=True)
            node.create_subscription(
                Imu, "/imu/data_raw",
                lambda m: self._store.record("/imu/data_raw", {}), 10, raw=True)
            node.create_subscription(
                JointState, "/arm/feedback",
                lambda m: self._store.record("/arm/feedback", {}), 10, raw=True)
            node.create_subscription(
                JointState, "/arm/command",
                lambda m: self._store.record("/arm/command", {}), 10, raw=True)

            node.create_subscription(
                Bool, "/arm/online",
                lambda m: self._store.record("/arm/online", bool_evidence(m.data)), 10)
            node.create_subscription(
                Float32, "/battery_voltage",
                lambda m: self._store.record(
                    "/battery_voltage", float_evidence(m.data)), 10)
            node.create_subscription(
                Int32, "/chassis_faults",
                lambda m: self._store.record(
                    "/chassis_faults", int_evidence(m.data)), 10)
            node.create_subscription(
                Int32, "/arm/status",
                lambda m: self._store.record("/arm/status", int_evidence(m.data)), 10)
            node.create_subscription(
                Odometry, "/wheel_odom",
                lambda m: self._store.record(
                    "/wheel_odom",
                    odom_evidence(m.twist.twist.linear.x, m.twist.twist.angular.z)), 10)
            node.create_subscription(
                Twist, "/cmd_vel_mux",
                lambda m: self._store.record(
                    "/cmd_vel_mux",
                    twist_evidence(m.linear.x, m.angular.z)), 10)

            self._executor = SingleThreadedExecutor()
            self._executor.add_node(node)
            self._thread = threading.Thread(target=self._executor.spin,
                                            name="carhealth-ros", daemon=True)
            self._thread.start()
            return True
        except Exception as e:                              # noqa: BLE001
            self.reason = "ROS 节点启动失败: %s" % e
            return False

    def stop(self) -> None:
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:                                   # noqa: BLE001
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:                                   # noqa: BLE001
            pass
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:                                   # noqa: BLE001
            pass
```

> 提交前跑一遍 `python3 -m py_compile carhealth/probes/ros_topic.py`，确认没有未定义名。

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_ros_topic.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/probes/ros_topic.py pi/carhealth/tests/test_ros_topic.py
git commit -m "feat(health): ROS 话题新鲜度探针"
```

---

## Task 8A: 服务资源与 USB 事件探针

**Files:**
- Create: `pi/carhealth/carhealth/probes/service_health.py`
- Create: `pi/carhealth/carhealth/probes/usb_events.py`
- Test: `pi/carhealth/tests/test_probe_service_health.py`
- Test: `pi/carhealth/tests/test_probe_usb_events.py`

**Interfaces:**
- Consumes: `UnitProbe`（Task 6）、`ProbeResult`
- Produces: `ServiceHealthProbe(unit_probes, clock, window_s)`；`UsbEventsProbe(runner=subprocess.run, clock)`；`parse_usb_disconnects(text) -> dict[str,int]`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_probe_service_health.py`：

```python
from carhealth.probes.service_health import ServiceHealthProbe
from carhealth.types import ProbeResult


class FakeUnit:
    def __init__(self):
        self.n = 0
        self.mem = 10.0
        self.cpu_ns = 0

    def poll(self):
        return ProbeResult(ok=True, evidence={"n_restarts": self.n,
                                              "memory_mb": self.mem,
                                              "cpu_usage_ns": self.cpu_ns})


def test_crash_loop_detected_and_baseline_resets():
    now = [0.0]
    a, b = FakeUnit(), FakeUnit()
    p = ServiceHealthProbe({"a": a, "b": b}, clock=lambda: now[0], window_s=60)
    p.poll()
    a.n = 2
    now[0] = 10.0
    r = p.poll()
    assert r.evidence["crash_loop_units"] == ["a"]
    assert r.evidence["memory_mb"]["a"] == 10.0
    assert r.evidence["n_restarts"]["a"] == 2


def test_crash_loop_window_expires():
    now = [0.0]
    a = FakeUnit()
    p = ServiceHealthProbe({"a": a}, clock=lambda: now[0], window_s=60)
    p.poll()
    now[0] = 120.0
    a.n = 2
    assert p.poll().evidence["crash_loop_units"] == []


def test_unavailable_unit_is_skipped():
    class Bad:
        def poll(self):
            return ProbeResult(ok=None, error="no systemctl")

    r = ServiceHealthProbe({"x": Bad()}, clock=lambda: 0.0).poll()
    assert r.evidence["n_restarts"] == {}


def test_cpu_pct_from_cpu_usage_delta():
    now = [0.0]
    a = FakeUnit()
    p = ServiceHealthProbe({"a": a}, clock=lambda: now[0])
    p.poll()
    a.cpu_ns = 2_000_000_000          # 2 秒 CPU 时间
    now[0] = 2.0                      # 墙钟 2 秒 -> 100%
    r = p.poll()
    assert r.evidence["cpu_pct"]["a"] == 100.0
```

`pi/carhealth/tests/test_probe_usb_events.py`：

```python
import subprocess

from carhealth.probes.usb_events import UsbEventsProbe, parse_usb_disconnects

FIXTURE = """\
usb 1-1: new full-speed USB device number 3 using xhci_hcd
usb 1-1: New USB device found, idVendor=0403, idProduct=6010
usb 1-2: New USB device found, idVendor=1209, idProduct=abd1
usb 1-1: USB disconnect, device number 3
usb 1-1: New USB device found, idVendor=0403, idProduct=6010
usb 1-1: USB disconnect, device number 4
"""


def test_parse_counts_by_vid_pid():
    assert parse_usb_disconnects(FIXTURE) == {"0403:6010": 2}


def test_unknown_bus_counts_separately():
    text = "usb 9-9: USB disconnect, device number 7\n"
    assert parse_usb_disconnects(text) == {"unknown": 1}


def test_probe_permission_failure_is_unknown():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 1, "", "no permission")

    r = UsbEventsProbe(runner=runner).poll()
    assert r.ok is None
    assert "USB 事件" in r.error


def test_probe_counts_from_journal():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 0, FIXTURE, "")

    r = UsbEventsProbe(runner=runner).poll()
    assert r.evidence["disconnects"] == {"0403:6010": 2}
    assert r.evidence["total"] == 2
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_service_health.py tests/test_probe_usb_events.py -q`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/probes/service_health.py`：

```python
"""服务资源与崩溃循环。Restart=always 会把崩溃循环伪装成 active。"""

from __future__ import annotations

import time

from ..types import ProbeResult


class ServiceHealthProbe:
    name = "service"
    interval_s = 5.0

    def __init__(self, unit_probes: dict, clock=time.monotonic, window_s: float = 60.0):
        self._units = dict(unit_probes)
        self._clock = clock
        self._window = window_s
        self._baseline: dict[str, tuple[int, float]] = {}
        self._cpu_prev: dict[str, tuple[int, float]] = {}

    def poll(self) -> ProbeResult:
        now = self._clock()
        crash, memory, restarts, cpu_pct = [], {}, {}, {}
        for name, probe in self._units.items():
            result = probe.poll()
            if not result.ok:
                continue
            n = int(result.evidence.get("n_restarts") or 0)
            memory[name] = result.evidence.get("memory_mb")
            restarts[name] = n

            cpu_ns = result.evidence.get("cpu_usage_ns")
            if isinstance(cpu_ns, int):
                prev = self._cpu_prev.get(name)
                if prev is not None and now > prev[1]:
                    cpu_pct[name] = round(
                        (cpu_ns - prev[0]) / 1e9 / (now - prev[1]) * 100.0, 1)
                self._cpu_prev[name] = (cpu_ns, now)

            base = self._baseline.get(name)
            if base is None or now - base[1] > self._window:
                self._baseline[name] = (n, now)
                continue
            if n - base[0] >= 2:
                crash.append(name)
                self._baseline[name] = (n, now)
        return ProbeResult(ok=True, evidence={
            "crash_loop_units": crash,
            "memory_mb": memory,
            "n_restarts": restarts,
            "cpu_pct": cpu_pct,
        })
```

`pi/carhealth/carhealth/probes/usb_events.py`：

```python
"""USB disconnect 计数。K230 24 小时 13 次重枚举这类问题在这里现形。"""

from __future__ import annotations

import re
import subprocess
import time

from ..types import ProbeResult

_NEW_RE = re.compile(
    r"usb (\S+): New USB device found, idVendor=([0-9a-fA-F]{4}), idProduct=([0-9a-fA-F]{4})")
_DISC_RE = re.compile(r"usb (\S+): USB disconnect")


def parse_usb_disconnects(text: str) -> dict[str, int]:
    known: dict[str, str] = {}
    counts: dict[str, int] = {}
    for line in (text or "").splitlines():
        m = _NEW_RE.search(line)
        if m:
            known[m.group(1)] = "%s:%s" % (m.group(2).lower(), m.group(3).lower())
            continue
        m = _DISC_RE.search(line)
        if m:
            key = known.get(m.group(1), "unknown")
            counts[key] = counts.get(key, 0) + 1
    return counts


class UsbEventsProbe:
    name = "usb"
    interval_s = 10.0

    def __init__(self, runner=subprocess.run, clock=time.monotonic):
        self._runner = runner
        self._clock = clock

    def poll(self) -> ProbeResult:
        cmd = ["journalctl", "-k", "-b", "--since", "-10min",
               "--no-pager", "-o", "cat"]
        try:
            r = self._runner(cmd, capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as e:
            return ProbeResult(ok=None, error="USB 事件读取失败: %r" % (e,))
        if r.returncode != 0:
            return ProbeResult(ok=None, error="USB 事件读取失败: journalctl rc=%d" % r.returncode)
        counts = parse_usb_disconnects(r.stdout)
        return ProbeResult(ok=True, evidence={
            "disconnects": counts,
            "total": sum(counts.values()),
        })
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_service_health.py tests/test_probe_usb_events.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/probes/service_health.py pi/carhealth/carhealth/probes/usb_events.py pi/carhealth/tests/test_probe_service_health.py pi/carhealth/tests/test_probe_usb_events.py
git commit -m "feat(health): 服务资源与 USB 事件探针"
```

---

## Task 8B: DDS 旧 IP 与配置漂移探针

**Files:**
- Create: `pi/carhealth/carhealth/probes/dds.py`
- Create: `pi/carhealth/carhealth/probes/config_drift.py`
- Test: `pi/carhealth/tests/test_probe_dds.py`
- Test: `pi/carhealth/tests/test_probe_config_drift.py`

**Interfaces:**
- Consumes: `ProbeResult`
- Produces: `DdsProbe(runner, clock)`、`parse_ss_local_addrs(text)`；`ConfigDriftProbe(repo, rules_dir, packages, units, runner, clock)`、`parse_ordering_cycles(text)`、`build_stale_packages(...)`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_probe_dds.py`：

```python
import subprocess

from carhealth.probes.dds import DdsProbe, parse_ss_local_addrs

SS = """\
UNCONN 0 0 10.0.0.5:7400 0.0.0.0:*
UNCONN 0 0 192.168.1.108:7401 0.0.0.0:*
UNCONN 0 0 127.0.0.1:7402 0.0.0.0:*
UNCONN 0 0 0.0.0.0:7403 0.0.0.0:*
"""


def test_parse_ss_skips_wildcard_and_loopback():
    assert parse_ss_local_addrs(SS) == {"10.0.0.5", "192.168.1.108"}


def test_probe_detects_stale_ip():
    def runner(cmd, **kw):
        out = "10.0.0.5 \n" if cmd[0] == "hostname" else SS
        return subprocess.CompletedProcess(cmd, 0, out, "")

    r = DdsProbe(runner=runner).poll()
    assert r.evidence["current_ips"] == ["10.0.0.5"]
    assert r.evidence["stale_binds"] == ["192.168.1.108"]


def test_probe_without_ss_is_unknown():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 1, "", "no ss")

    assert DdsProbe(runner=runner).poll().ok is None
```

`pi/carhealth/tests/test_probe_config_drift.py`：

```python
import os
import subprocess

from carhealth.probes.config_drift import (
    ConfigDriftProbe, build_stale_packages, parse_ordering_cycles,
)


def test_parse_ordering_cycles():
    text = "systemd[1]: Job foo.service/start deleted to break ordering cycle\n"
    assert parse_ordering_cycles(text) == 1
    assert parse_ordering_cycles("nothing") == 0


def test_build_stale_packages_uses_source_vs_install_mtime():
    mtimes = {
        "/repo/pi/raspot_ws/src/arm_grasp/setup.py": 200.0,
        "/repo/pi/raspot_ws/install/arm_grasp/share/arm_grasp/package.xml": 100.0,
        "/repo/pi/raspot_ws/src/tof_a010/package.xml": 100.0,
        "/repo/pi/raspot_ws/install/tof_a010/share/tof_a010/package.xml": 200.0,
    }
    stale = build_stale_packages(
        "/repo", ["arm_grasp", "tof_a010"], exists=lambda p: p in mtimes,
        mtime=lambda p: mtimes[p])
    assert stale == ["arm_grasp"]


def test_probe_collects_drift(tmp_path):
    repo = tmp_path / "repo"
    (repo / "pi/raspot_ws/src/arm_grasp").mkdir(parents=True)
    (repo / "pi/raspot_ws/src/arm_grasp/setup.py").write_text("x")
    (repo / "pi/raspot_ws/install/arm_grasp/share/arm_grasp").mkdir(parents=True)
    install_marker = repo / "pi/raspot_ws/install/arm_grasp/share/arm_grasp/package.xml"
    install_marker.write_text("x")
    os.utime(repo / "pi/raspot_ws/src/arm_grasp/setup.py", (200, 200))
    os.utime(install_marker, (100, 100))

    rules = tmp_path / "rules"
    rules.mkdir()          # 故意什么规则都不装

    def runner(cmd, **kw):
        if cmd[0] == "journalctl":
            return subprocess.CompletedProcess(cmd, 0, "ordering cycle here\n", "")
        if cmd[0] == "systemctl":
            return subprocess.CompletedProcess(
                cmd, 0, "ActiveState=inactive\nUnitFileState=enabled\n", "")
        if cmd[0] == "git":
            out = "main\n" if "rev-parse" in cmd and "--abbrev-ref" in cmd else "abc123\n"
            return subprocess.CompletedProcess(cmd, 0, out, "")
        return subprocess.CompletedProcess(cmd, 0, "", "")

    r = ConfigDriftProbe(repo=str(repo), rules_dir=str(rules),
                         packages=["arm_grasp"], units=["arm-grasp"],
                         runner=runner).poll()
    assert r.evidence["ordering_cycles"] == 1
    assert r.evidence["enabled_but_dead"] == ["arm-grasp"]
    assert "99-tof-a010.rules" in r.evidence["missing_udev_rules"]
    assert r.evidence["git_branch"] == "main"
    assert r.evidence["stale_packages"] == ["arm_grasp"]
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_dds.py tests/test_probe_config_drift.py -q`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/probes/dds.py`：

```python
"""DDS 是否还绑着旧网段的地址。换网段后只重启一个节点修不好，必须整条 ROS 栈重启。"""

from __future__ import annotations

import subprocess
import time

from ..types import ProbeResult


def parse_ss_local_addrs(text: str) -> set[str]:
    addrs = set()
    for line in (text or "").splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        local = parts[3]
        if ":" not in local:          # 表头 / 非地址列
            continue
        host = local.rsplit(":", 1)[0].strip("[]")
        if host in ("*", "0.0.0.0", "::", "::1", "127.0.0.1"):
            continue
        if host.startswith("127."):
            continue
        addrs.add(host.split("%", 1)[0])
    return addrs


class DdsProbe:
    name = "dds"
    interval_s = 10.0

    def __init__(self, runner=subprocess.run, clock=time.monotonic):
        self._runner = runner
        self._clock = clock

    def poll(self) -> ProbeResult:
        try:
            ip = self._runner(["hostname", "-I"], capture_output=True,
                              text=True, timeout=3)
            ss = self._runner(["ss", "-uan"], capture_output=True,
                              text=True, timeout=3)
        except (OSError, subprocess.TimeoutExpired) as e:
            return ProbeResult(ok=None, error="ss/hostname 不可用: %r" % (e,))
        if ip.returncode != 0 or ss.returncode != 0:
            return ProbeResult(ok=None, error="ss/hostname 返回非零")
        current = [a for a in ip.stdout.split() if a]
        stale = sorted(parse_ss_local_addrs(ss.stdout) - set(current))
        return ProbeResult(ok=True, evidence={
            "current_ips": current,
            "stale_binds": stale,
        })
```

`pi/carhealth/carhealth/probes/config_drift.py`：

```python
"""配置漂移：ordering cycle、enabled 但 inactive、udev 规则、git HEAD、build 过期。"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

from ..types import ProbeResult

EXPECTED_RULES = ("99-k230.rules", "99-tof-a010.rules", "99-l150pro.rules",
                  "99-myspeech.rules")


def parse_ordering_cycles(text: str) -> int:
    return sum(1 for line in (text or "").splitlines() if "ordering cycle" in line)


def build_stale_packages(repo: str, packages, exists=os.path.exists,
                         mtime=os.path.getmtime) -> list[str]:
    stale = []
    for pkg in packages:
        src = Path(repo) / "pi/raspot_ws/src" / pkg
        install = Path(repo) / "pi/raspot_ws/install" / pkg / "share" / pkg / "package.xml"
        if not exists(str(install)):
            continue
        sources = [src / "package.xml", src / "setup.py", src / "setup.cfg"]
        latest = max((mtime(str(p)) for p in sources if exists(str(p))), default=0.0)
        if latest > mtime(str(install)):
            stale.append(pkg)
    return stale


class ConfigDriftProbe:
    name = "config_drift"
    interval_s = 60.0

    def __init__(self, repo: str, rules_dir: str = "/etc/udev/rules.d",
                 packages=("arm_grasp", "k230_ros", "tof_a010"),
                 units=("ps2-teleop", "tof-a010", "k230-resultd", "k230-bridge",
                        "k230-framed", "rosbridge", "arm-grasp"),
                 runner=subprocess.run, clock=time.monotonic):
        self.repo = repo
        self.rules_dir = Path(rules_dir)
        self.packages = tuple(packages)
        self.units = tuple(units)
        self._runner = runner
        self._clock = clock

    def _run(self, cmd, timeout=5):
        try:
            return self._runner(cmd, capture_output=True, text=True, timeout=timeout)
        except (OSError, subprocess.TimeoutExpired):
            return None

    def poll(self) -> ProbeResult:
        ev: dict = {"errors": []}

        r = self._run(["journalctl", "-b", "--no-pager", "-g", "ordering cycle",
                       "-o", "cat"])
        if r is not None and r.returncode == 0:
            ev["ordering_cycles"] = parse_ordering_cycles(r.stdout)
        else:
            ev["errors"].append("journalctl 不可读")

        dead = []
        for unit in self.units:
            r = self._run(["systemctl", "show", "-p", "ActiveState",
                           "-p", "UnitFileState", unit + ".service"])
            if r is None or r.returncode != 0:
                continue
            values = dict(
                line.split("=", 1) for line in r.stdout.splitlines() if "=" in line)
            if (values.get("UnitFileState") == "enabled"
                    and values.get("ActiveState") == "inactive"):
                dead.append(unit)
        ev["enabled_but_dead"] = dead

        ev["missing_udev_rules"] = [
            name for name in EXPECTED_RULES
            if not (self.rules_dir / name).exists()
        ]

        branch = self._run(["git", "-C", self.repo, "rev-parse", "--abbrev-ref", "HEAD"])
        head = self._run(["git", "-C", self.repo, "rev-parse", "--short", "HEAD"])
        ev["git_branch"] = branch.stdout.strip() if branch and branch.returncode == 0 else None
        ev["git_head"] = head.stdout.strip() if head and head.returncode == 0 else None

        ev["stale_packages"] = build_stale_packages(self.repo, self.packages)
        return ProbeResult(ok=True, evidence=ev)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_probe_dds.py tests/test_probe_config_drift.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/probes/dds.py pi/carhealth/carhealth/probes/config_drift.py pi/carhealth/tests/test_probe_dds.py pi/carhealth/tests/test_probe_config_drift.py
git commit -m "feat(health): DDS 旧 IP 与配置漂移探针"
```

---

## Task 9: 互锁信号

**Files:**
- Create: `pi/carhealth/carhealth/busy.py`
- Test: `pi/carhealth/tests/test_busy.py`

**Interfaces:**
- Consumes: `ProbeResult`
- Produces: `BusyState(arm, chassis, grasp, voice)`（`.any`）、`compute(results, voice_evidence) -> BusyState`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_busy.py`：

```python
from carhealth.busy import BusyState, compute
from carhealth.types import ProbeResult


def r(**ev):
    return ProbeResult(ok=True, evidence=ev)


def test_idle_everything():
    assert compute({}, {}).any is False


def test_arm_command_makes_arm_busy():
    st = compute({"topic:/arm/command": r(age_s=0.2)}, {})
    assert st.arm is True


def test_stale_arm_command_does_not_count():
    st = compute({"topic:/arm/command": r(age_s=5.0)}, {})
    assert st.arm is False


def test_chassis_from_cmd_vel_or_odom():
    assert compute({"topic:/cmd_vel_mux": r(age_s=0.1, linear_x=0.2, angular_z=0.0)}, {}).chassis
    assert compute({"topic:/wheel_odom": r(age_s=0.1, linear_x=0.0, angular_z=0.6)}, {}).chassis
    # 车被推着直线走：只看角速度会判成空闲，二期这就是自动重启的开关。
    assert compute({"topic:/wheel_odom": r(age_s=0.1, linear_x=0.3, angular_z=0.0)}, {}).chassis
    assert not compute({"topic:/cmd_vel_mux": r(age_s=0.1, linear_x=0.0, angular_z=0.0)}, {}).chassis


def test_grasp_busy_from_health_topic():
    st = compute({"topic:/health/arm_grasp": r(age_s=0.5, busy=True)}, {})
    assert st.grasp is True
    assert st.arm is True          # 抓取也是臂在动


def test_voice_busy_from_file_evidence():
    assert compute({}, {"busy": True}).voice is True
    assert compute({}, {"busy": False}).voice is False


def test_any_combines():
    st = BusyState(arm=False, chassis=False, grasp=False, voice=True)
    assert st.any is True
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_busy.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.busy'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/busy.py`：

```python
"""动作互锁。二期恢复动作的唯一开关；一期只写进日志。"""

from __future__ import annotations

from dataclasses import dataclass

from .types import ProbeResult


@dataclass(frozen=True)
class BusyState:
    arm: bool
    chassis: bool
    grasp: bool
    voice: bool

    @property
    def any(self) -> bool:
        return self.arm or self.chassis or self.grasp or self.voice


def _fresh(ev: dict, max_age: float) -> bool:
    age = ev.get("age_s")
    return isinstance(age, (int, float)) and age <= max_age


def _num(ev: dict, key: str) -> float:
    try:
        return float(ev.get(key) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def compute(results: dict[str, ProbeResult], voice_evidence: dict | None) -> BusyState:
    arm_cmd = results.get("topic:/arm/command")
    arm = bool(arm_cmd and arm_cmd.ok and _fresh(arm_cmd.evidence, 1.5))

    chassis = False
    cmd = results.get("topic:/cmd_vel_mux")
    if cmd and cmd.ok and _fresh(cmd.evidence, 1.0):
        chassis = abs(_num(cmd.evidence, "linear_x")) > 0.02 or \
                  abs(_num(cmd.evidence, "angular_z")) > 0.05
    odom = results.get("topic:/wheel_odom")
    if odom and odom.ok and _fresh(odom.evidence, 1.0):
        chassis = chassis or abs(_num(odom.evidence, "linear_x")) > 0.02
        chassis = chassis or abs(_num(odom.evidence, "angular_z")) > 0.05

    grasp = False
    health = results.get("topic:/health/arm_grasp")
    if health and health.ok and _fresh(health.evidence, 3.0):
        grasp = bool(health.evidence.get("busy"))
    arm = arm or grasp

    voice = bool((voice_evidence or {}).get("busy"))
    return BusyState(arm=arm, chassis=chassis, grasp=grasp, voice=voice)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_busy.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/busy.py pi/carhealth/tests/test_busy.py
git commit -m "feat(health): 动作互锁信号"
```

---

## Task 9A: 已知故障库与诊断

**Files:**
- Create: `pi/carhealth/carhealth/faults.py`
- Create: `pi/carhealth/carhealth/diagnose.py`
- Test: `pi/carhealth/tests/test_faults.py`

**Interfaces:**
- Consumes: `types.Diagnosis`、`ProbeResult`、`classify.Evaluation`
- Produces: `FaultContext(results, evals, now_mono)`、`RULES`、`RULES_BY_ID`、`diagnose(ctx) -> dict[str | None, list[Diagnosis]]`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_faults.py`：

```python
from carhealth.classify import Evaluation
from carhealth.diagnose import diagnose
from carhealth.faults import FaultContext
from carhealth.types import ProbeResult, Verdict


def ctx(evals=None, **probes):
    results = {k: ProbeResult(ok=True, evidence=v) for k, v in probes.items()}
    return FaultContext(results=results, evals=evals or {}, now_mono=100.0)


def hits(context):
    return {d.id for ds in diagnose(context).values() for d in ds}


def root_eval(root):
    return Evaluation(Verdict.BLOCKED, "x", {}, (), (), {}, root_cause=root)


def test_undervoltage():
    assert "power.undervoltage" in hits(ctx(power={"uv_now": True}))
    assert "power.undervoltage" in hits(ctx(power={"uv_recent": True}))
    assert "power.undervoltage" not in hits(ctx(power={"uv_since_boot": True}))


def test_usb_disconnect_storm():
    assert "usb.disconnect_storm" in hits(
        ctx(usb={"disconnects": {"0403:6010": 3}}))
    assert "usb.disconnect_storm" not in hits(
        ctx(usb={"disconnects": {"0403:6010": 2}}))


def test_dds_stale_ip():
    assert "dds.stale_ip" in hits(ctx(dds={"stale_binds": ["192.168.1.108"]}))


def test_crash_loop_and_config_drift():
    assert "service.crash_loop" in hits(
        ctx(service={"crash_loop_units": ["tof-a010"]}))
    assert "service.ordering_cycle" in hits(
        ctx(config_drift={"ordering_cycles": 1}))
    assert "udev.rules_missing" in hits(
        ctx(config_drift={"missing_udev_rules": ["99-tof-a010.rules"]}))
    assert "build.stale" in hits(
        ctx(config_drift={"stale_packages": ["arm_grasp"]}))


def test_k230_host_unknown():
    assert "k230.host_unknown" in hits(ctx(**{"net:board": {"host": None}}))


def test_battery_low():
    assert "battery.low" in hits(ctx(**{"topic:/battery_voltage": {"data": 11.1}}))
    assert "battery.low" not in hits(ctx(**{"topic:/battery_voltage": {"data": 12.0}}))


def test_cpu_rules():
    assert "host.cpu_high" in hits(ctx(host={"cpu_pct": 90.0}))
    assert "service.cpu_high" in hits(ctx(service={"cpu_pct": {"x": 250.0}}))
    assert "host.cpu_high" not in hits(ctx(host={"cpu_pct": 40.0}))


def test_wakeword_lost_and_api_stuck():
    assert "voice.wakeword_lost" in hits(
        ctx(**{"file:voice-heartbeat": {"wakeword_ready": False}}))
    assert "voice.api_stuck" in hits(
        ctx(**{"file:voice-heartbeat": {"busy_for_s": 200.0}}))
    assert "voice.mic_error" in hits(
        ctx(**{"file:voice-heartbeat": {"mic_last_error": "Input overflow"}}))


def test_tof_channel_b_and_by_id_only():
    assert "udev.tof_channel_b" in hits(ctx(**{"udev:a010": {"interface": "01"}}))
    assert "udev.tof_byid_only" in hits(
        ctx(**{"udev:a010": {"resolved_by": "by_id"}}))


def test_ros_root_causes():
    evals = {"a": root_eval("ros_discovery")}
    assert "ros.discovery" in hits(ctx(evals=evals))
    evals = {"a": root_eval("ros_data_path")}
    assert "ros.data_path" in hits(ctx(evals=evals))


def test_diagnosis_targets_component():
    out = diagnose(ctx(power={"uv_now": True}))
    assert out["power"][0].id == "power.undervoltage"
    assert out["power"][0].action_class == "wait"


def test_clean_context_has_no_diagnosis():
    assert diagnose(ctx(power={"uv_now": False, "uv_recent": False},
                        host={"load1": 0.5, "wifi_link": 60.0},
                        dds={"stale_binds": []})) == {}


def test_rule_exception_does_not_break_diagnose():
    class Boom:
        id = "x"
        title = "x"
        action_class = "none"
        hint = "x"
        components = ()
        def when(self, c):
            raise RuntimeError("bug")

    import carhealth.diagnose as d
    old = d.RULES
    d.RULES = (Boom(),)
    try:
        assert d.diagnose(ctx()) == {}
    finally:
        d.RULES = old
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_faults.py -q`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/faults.py`：

```python
"""已知故障库。每条规则只做「特征 → 结论 + 动作类别 + 指令」。

规则不改变六态判定；它们让 status/doctor/why 直接给出人话结论。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .types import Diagnosis, ProbeResult, Verdict


@dataclass
class FaultContext:
    results: dict[str, ProbeResult]
    evals: dict
    now_mono: float


def ev(ctx: FaultContext, probe: str) -> dict:
    r = ctx.results.get(probe)
    return r.evidence if r is not None and r.ok is True else {}


def raw_ev(ctx: FaultContext, probe: str) -> dict:
    """连 ok=None 的探针证据也要看（例如「地址未知」本身就是结论）。"""
    r = ctx.results.get(probe)
    return dict(r.evidence or {}) if r is not None else {}


def _root(ctx: FaultContext, component: str):
    return getattr(ctx.evals.get(component), "root_cause", None)


def _any_root(ctx: FaultContext, cause: str) -> bool:
    return any(_root(ctx, name) == cause for name in ctx.evals)


def _count_at_least(ctx: FaultContext, probe: str, key: str, threshold: int) -> bool:
    counts = ev(ctx, probe).get(key) or {}
    return any(int(value) >= threshold for value in counts.values())


def _number(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class Rule:
    id: str
    title: str
    action_class: str          # wait / restart_unit / restart_stack / human / fix_config / none
    hint: str
    components: tuple[str, ...]  # 空 = 全局
    when: Callable[[FaultContext], bool]


RULES = (
    Rule("power.undervoltage", "Pi 供电欠压", "wait",
         "检查电源线、接插件与驱动板压降；欠压会连带打掉 USB 设备",
         ("power",),
         lambda c: bool(ev(c, "power").get("uv_now") or ev(c, "power").get("uv_recent"))),

    Rule("usb.disconnect_storm", "USB 设备在反复重枚举", "human",
         "10 分钟内同一设备断开 ≥3 次：换线、换口、检查供电",
         ("host",),
         lambda c: _count_at_least(c, "usb", "disconnects", 3)),

    Rule("udev.l150pro_missing", "底盘/臂串口稳定名不存在", "fix_config",
         "检查 99-l150pro.rules 是否安装、serial 是否匹配",
         ("chassis", "arm"),
         lambda c: ev(c, "udev:l150pro").get("exists") is False),

    Rule("udev.tof_byid_only", "ToF 短名丢失，只走了 by-id", "fix_config",
         "装 99-tof-a010.rules；功能可能仍可用，但节点名不再稳定",
         ("tof",),
         lambda c: ev(c, "udev:a010").get("resolved_by") == "by_id"),

    Rule("udev.tof_channel_b", "ToF 接到了 FT2232 的通道 B", "human",
         "插回通道 A；通道 B 能打开但一个字节都收不到",
         ("tof",),
         lambda c: (ev(c, "udev:a010").get("interface") not in (None, "00"))),

    Rule("udev.rules_missing", "udev 规则未安装", "fix_config",
         "按 pi/systemd/*.rules 安装并 udevadm reload/trigger",
         (),
         lambda c: bool(ev(c, "config_drift").get("missing_udev_rules"))),

    Rule("dds.stale_ip", "DDS 仍绑着旧网段地址", "restart_stack",
         "换网段后只重启一个单元修不好：整条 ROS 栈一起重启",
         ("chassis",),
         lambda c: bool(ev(c, "dds").get("stale_binds"))),

    Rule("ros.discovery", "ROS 发现层故障（0 个发布者）", "restart_stack",
         "检查 ROS_DOMAIN_ID / 网卡 / DDS 发现；再整栈重启",
         (),
         lambda c: _any_root(c, "ros_discovery")),

    Rule("ros.data_path", "有发布者但收不到数据", "human",
         "查 QoS 兼容、MTU 分片、WiFi 质量",
         (),
         lambda c: _any_root(c, "ros_data_path")),

    Rule("service.crash_loop", "服务在崩溃循环（被 Restart=always 掩盖）", "fix_config",
         "看 journalctl -u <unit>；NRestarts 在涨说明它一直在死",
         (),
         lambda c: bool(ev(c, "service").get("crash_loop_units"))),

    Rule("service.ordering_cycle", "开机时出现 systemd ordering cycle", "fix_config",
         "修 unit 的 After/Wants；arm-grasp inactive (dead) 常见于此",
         (),
         lambda c: (_number(ev(c, "config_drift").get("ordering_cycles"), 0) > 0)),

    Rule("service.enabled_but_dead", "enabled 的服务处于 inactive (dead)", "fix_config",
         "systemctl status 看失败原因；别只重启，先修根因",
         (),
         lambda c: bool(ev(c, "config_drift").get("enabled_but_dead"))),

    Rule("build.stale", "install 比源码旧", "fix_config",
         "colcon build --symlink-install，再重启服务",
         (),
         lambda c: bool(ev(c, "config_drift").get("stale_packages"))),

    Rule("k230.host_unknown", "K230 地址未知", "wait",
         "确认板子上电/连 WiFi；Pi 重启后 /tmp 清空、尚无 peer 属正常",
         ("board",),
         lambda c: ("host" in raw_ev(c, "net:board")
                    and raw_ev(c, "net:board").get("host") is None)),

    Rule("k230.push_stall", "板子连着推流口但关键帧不更新", "human",
         "检查 K230 推流与 WiFi；板子不能主动请求 IDR",
         ("k230_video",),
         lambda c: (ev(c, "file:framed-status").get("connected") is True
                    and _number(ev(c, "file:latest-idr").get("age_s"), 0) > 5.0)),

    Rule("battery.low", "电池电压低", "wait",
         "11.2–12.3V 性能差近一倍，别误判成控制环路问题",
         ("battery",),
         lambda c: (_number(ev(c, "topic:/battery_voltage").get("data"), 99) < 11.2)),

    Rule("arm.estop", "机械臂固件急停位置位", "human",
         "检查急停开关/固件状态；/arm/status bit7",
         ("arm",),
         lambda c: bool(int(ev(c, "topic:/arm/status").get("data") or 0) & 0x80)),

    Rule("chassis.faults", "底盘固件报告故障位", "human",
         "看 /chassis_faults 与驱动日志；欠压/IMU/链路超时是常见位",
         ("chassis",),
         lambda c: (ev(c, "topic:/chassis_faults").get("data") not in (None, 0))),

    Rule("host.load_high", "主机负载高", "wait",
         "查是不是软解视频/编译/多个节点叠加；高负载会放大抖动",
         ("host",),
         lambda c: (_number(ev(c, "host").get("load1"), 0) >= 6.0)),

    Rule("host.cpu_high", "主机 CPU 使用率高", "wait",
         "持续 ≥85% 会放大所有超时判据的抖动；先找是谁在吃 CPU",
         ("host",),
         lambda c: (_number(ev(c, "host").get("cpu_pct"), 0) >= 85.0)),

    Rule("service.cpu_high", "有服务 CPU 占用高", "human",
         "单服务 >200%（约 2 核）通常是软解/死循环",
         (),
         lambda c: any((_number(v, 0) or 0) > 200
                       for v in (ev(c, "service").get("cpu_pct") or {}).values())),

    Rule("host.wifi_weak", "WiFi 信号弱", "human",
         "信号 < -75dBm 或链路质量 <30：换信道/靠近 AP/换 5G",
         ("host",),
         lambda c: ((_number(ev(c, "host").get("wifi_signal_dbm"), 0) < -75)
                    or (_number(ev(c, "host").get("wifi_link"), 99) < 30))),

    Rule("service.memory_high", "有服务内存占用过高", "human",
         "看是否内存泄漏；>1GB 通常会拖垮整机",
         (),
         lambda c: any((_number(v, 0) or 0) > 1024
                       for v in (ev(c, "service").get("memory_mb") or {}).values())),

    Rule("voice.wakeword_lost", "唤醒模块掉线", "human",
         "唤醒模块被拔/重枚举；按路径重连已实现，仍不行就换口",
         ("voice",),
         lambda c: (ev(c, "file:voice-heartbeat").get("wakeword_ready") is False
                    or ev(c, "file:voice-heartbeat").get("wakeword_thread_alive") is False)),

    Rule("voice.api_stuck", "语音助手在忙状态卡住", "restart_unit",
         "看 ASR/LLM 报错；历史上 API 400 会卡在 THINKING",
         ("voice",),
         lambda c: (_number(ev(c, "file:voice-heartbeat").get("busy_for_s"), 0) >= 180)),

    Rule("voice.mic_error", "麦克风读取失败", "human",
         "USB 重枚举/设备被抢占；换口或重启语音助手",
         ("voice",),
         lambda c: bool(ev(c, "file:voice-heartbeat").get("mic_last_error"))),

    Rule("tof.last_error_stale", "ToF 连着但保留着旧错误", "none",
         "仅提示：该字段重连后不清零，不代表当前故障",
         ("tof",),
         lambda c: (ev(c, "topic:/tof/status").get("connected") is True
                    and bool(ev(c, "topic:/tof/status").get("last_error")))),
)

RULES_BY_ID = {rule.id: rule for rule in RULES}
```

`pi/carhealth/carhealth/diagnose.py`：

```python
"""把已知故障库求值成 Diagnosis。规则异常不得影响主流程。"""

from __future__ import annotations

from .faults import FaultContext, RULES
from .types import Diagnosis


def diagnose(ctx: FaultContext) -> dict[str | None, list[Diagnosis]]:
    out: dict[str | None, list[Diagnosis]] = {}
    for rule in RULES:
        try:
            hit = rule.when(ctx)
        except Exception:                                   # noqa: BLE001
            hit = False
        if not hit:
            continue
        diagnosis = Diagnosis(rule.id, rule.title, rule.action_class, rule.hint)
        for target in (rule.components or (None,)):
            out.setdefault(target, []).append(diagnosis)
    return out
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_faults.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/faults.py pi/carhealth/carhealth/diagnose.py pi/carhealth/tests/test_faults.py
git commit -m "feat(health): 已知故障库与诊断"
```

---

## Task 10: 引擎（轮询、判定、状态迁移）

**Files:**
- Create: `pi/carhealth/carhealth/engine.py`
- Test: `pi/carhealth/tests/test_engine.py`

**Interfaces:**
- Consumes: `types`、`classify`、`journal`、`busy`
- Produces: `EngineConfig(state_dir, journal_dir, interval_s=1.0)`、`Engine(specs, probes, config, clock=time.monotonic, wall=time.time)`（`run_once()`、`last_busy`、`close()`、`journal`）、`default_state_dir()`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_engine.py`：

```python
import json

from carhealth.engine import Engine, EngineConfig
from carhealth.types import Check, ComponentSpec, ProbeResult, Verdict


class FakeProbe:
    def __init__(self, name, evidence, interval_s=0.0, ok=True):
        self.name = name
        self.interval_s = interval_s
        self.evidence = evidence
        self.ok = ok
        self.polls = 0

    def poll(self):
        self.polls += 1
        return ProbeResult(ok=self.ok, evidence=dict(self.evidence))


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def make(probe_evidence, specs=None, tmp_path=None):
    clock = Clock()
    probes = {p.name: p for p in probe_evidence}
    specs = specs or (
        ComponentSpec("a", "A", (
            Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
        )),
    )
    cfg = EngineConfig(state_dir=tmp_path / "state", journal_dir=tmp_path / "journal")
    return Engine(specs, probes, cfg, clock=clock, wall=lambda: 1763388000.0 + clock.now), clock


def test_ok_state(tmp_path):
    eng, _ = make([FakeProbe("p", {"age_s": 0.1})], tmp_path=tmp_path)
    states = eng.run_once()
    assert states["a"].verdict is Verdict.OK
    payload = json.loads((tmp_path / "state" / "state.json").read_text())
    assert payload["components"]["a"]["verdict"] == "OK"


def test_stale_state_and_reason(tmp_path):
    eng, _ = make([FakeProbe("p", {"age_s": 9.0})], tmp_path=tmp_path)
    assert eng.run_once()["a"].verdict is Verdict.STALE


def test_probe_exception_becomes_unknown(tmp_path):
    class Boom(FakeProbe):
        def poll(self):
            raise RuntimeError("boom")

    eng, _ = make([Boom("p", {})], tmp_path=tmp_path)
    assert eng.run_once()["a"].verdict is Verdict.UNKNOWN


def test_probe_interval_is_respected(tmp_path):
    probe = FakeProbe("p", {"age_s": 0.1}, interval_s=10.0)
    eng, clock = make([probe], tmp_path=tmp_path)
    eng.run_once()
    clock.now += 1.0
    eng.run_once()
    assert probe.polls == 1
    clock.now += 10.0
    eng.run_once()
    assert probe.polls == 2


def test_hysteresis_needs_two_bad_and_five_good(tmp_path):
    probe = FakeProbe("p", {"age_s": 0.1})
    eng, clock = make([probe], tmp_path=tmp_path)
    assert eng.run_once()["a"].verdict is Verdict.OK

    probe.evidence["age_s"] = 9.0
    clock.now += 1
    assert eng.run_once()["a"].verdict is Verdict.OK        # 1 次坏：不动
    clock.now += 1
    assert eng.run_once()["a"].verdict is Verdict.STALE     # 2 次坏：降级

    probe.evidence["age_s"] = 0.1
    for _ in range(4):
        clock.now += 1
        assert eng.run_once()["a"].verdict is Verdict.STALE
    clock.now += 1
    assert eng.run_once()["a"].verdict is Verdict.OK        # 5 次好：恢复


def test_since_resets_only_when_committed_verdict_changes(tmp_path):
    probe = FakeProbe("p", {"age_s": 0.1})
    eng, clock = make([probe], tmp_path=tmp_path)
    first = eng.run_once()["a"]
    clock.now += 5
    second = eng.run_once()["a"]
    assert first.since == second.since
    probe.evidence["age_s"] = 9.0
    clock.now += 1
    eng.run_once()            # 第一次坏，仍 OK
    clock.now += 1
    third = eng.run_once()["a"]
    assert third.since == clock.now


def test_diagnosis_is_attached_to_component(tmp_path):
    specs = (
        ComponentSpec("power", "供电", (
            Check("power", "eq", False, field="uv_now", level=Verdict.BLOCKED,
                  reason="欠压"),
        )),
    )
    eng, _ = make([FakeProbe("power", {"uv_now": True})], specs=specs,
                  tmp_path=tmp_path)
    st = eng.run_once()["power"]
    assert st.verdict is Verdict.BLOCKED
    assert [d.id for d in st.diagnoses] == ["power.undervoltage"]


def test_default_state_dir_respects_env(monkeypatch, tmp_path):
    import carhealth.engine as engine_mod
    monkeypatch.setenv("CARHEALTH_STATE_DIR", str(tmp_path))
    assert engine_mod.default_state_dir() == tmp_path


def test_busy_is_computed_and_written(tmp_path):
    eng, _ = make(
        [FakeProbe("p", {"age_s": 0.1}),
         FakeProbe("topic:/cmd_vel_mux", {"age_s": 0.1, "linear_x": 0.3, "angular_z": 0.0})],
        tmp_path=tmp_path)
    eng.run_once()
    assert eng.last_busy.chassis is True
    payload = json.loads((tmp_path / "state" / "state.json").read_text())
    assert payload["busy"]["chassis"] is True
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_engine.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.engine'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/engine.py`：

```python
"""轮询探针 -> 判定 -> 状态迁移 -> 写 state.json。

一期不包含任何恢复动作；`journal` 里 action 恒为 null。
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path

from .busy import BusyState, compute
from .classify import attach_graph_root_cause, evaluate
from .diagnose import diagnose
from .faults import FaultContext
from .journal import Journal
from .types import ComponentSpec, ComponentState, ProbeResult, Verdict


def default_state_dir() -> Path:
    env = os.environ.get("CARHEALTH_STATE_DIR")
    if env:
        return Path(env)
    run = Path("/run/carhealth")
    if os.access(run, os.W_OK):
        return run
    return Path.home() / ".local/state/carhealth"


def default_journal_dir() -> Path:
    env = os.environ.get("CARHEALTH_JOURNAL_DIR")
    if env:
        return Path(env)
    return Path.home() / ".local/state/carhealth"


@dataclass
class EngineConfig:
    state_dir: Path
    journal_dir: Path
    interval_s: float = 1.0
    bad_after: int = 2          # 连续几次坏才降级
    good_after: int = 5         # 连续几次好才恢复


class Engine:
    def __init__(self, specs, probes, config: EngineConfig,
                 clock=time.monotonic, wall=time.time):
        self.specs = tuple(specs)
        self.probes = dict(probes)
        self.config = config
        self._clock = clock
        self._wall = wall
        self._results: dict[str, ProbeResult] = {}
        self._last_poll: dict[str, float] = {}
        self._committed: dict[str, ComponentState] = {}
        self._prev: dict[str, ComponentState] = {}
        self._bad: dict[str, int] = {}
        self._good: dict[str, int] = {}
        self._incidents: dict[str, dict] = {}
        self._journal = Journal(config.journal_dir, clock=wall)
        self.last_busy = BusyState(False, False, False, False)
        self.global_diagnoses = ()
        self._last_blind_wall = 0.0
        self.closer = None
        Path(config.state_dir).mkdir(parents=True, exist_ok=True)

    @property
    def journal(self) -> Journal:
        return self._journal

    def close(self) -> None:
        if self.closer is not None:
            try:
                self.closer()
            except Exception:                               # noqa: BLE001
                pass

    def _poll_due(self) -> None:
        now = self._clock()
        for name, probe in self.probes.items():
            last = self._last_poll.get(name)
            if last is not None and now - last < getattr(probe, "interval_s", 1.0):
                continue
            try:
                result = probe.poll()
            except Exception as e:                          # noqa: BLE001
                result = ProbeResult(ok=None, error=repr(e))
            self._results[name] = result
            self._last_poll[name] = now

    def _commit(self, spec, ev, now: float) -> ComponentState:
        """迟滞：≥2 次坏才降级，≥5 次好才恢复。首次观测直接采用，避免开机空窗。"""
        name = spec.name
        prev = self._committed.get(name)
        if prev is None:
            return self._state_from(spec, ev, now)

        if ev.verdict is Verdict.OK:
            self._good[name] = self._good.get(name, 0) + 1
            self._bad[name] = 0
            if prev.verdict is Verdict.OK or self._good[name] < self.config.good_after:
                return replace(prev, updated=now)
            return self._state_from(spec, ev, now)

        if (ev.verdict, ev.root_cause) == (prev.verdict, prev.root_cause):
            self._bad[name] = 0
            self._good[name] = 0
            return replace(prev, updated=now, reason=ev.reason,
                           evidence=ev.evidence, failing=ev.failing_reasons)

        self._bad[name] = self._bad.get(name, 0) + 1
        self._good[name] = 0
        if self._bad[name] < self.config.bad_after:
            return replace(prev, updated=now)
        return self._state_from(spec, ev, now)

    @staticmethod
    def _state_from(spec, ev, now: float) -> ComponentState:
        return ComponentState(
            name=spec.name, display=spec.display, verdict=ev.verdict,
            reason=ev.reason, evidence=ev.evidence, root_cause=ev.root_cause,
            failing=ev.failing_reasons, diagnoses=(), since=now, updated=now,
        )

    def run_once(self) -> dict[str, ComponentState]:
        self._poll_due()

        evals = {}
        for spec in self.specs:
            evals[spec.name] = evaluate(spec, self._results, evals)
        evals = attach_graph_root_cause(evals, self.specs)

        now = self._clock()
        states = {spec.name: self._commit(spec, evals[spec.name], now)
                  for spec in self.specs}

        ctx = FaultContext(self._results, evals, now)
        diag = diagnose(ctx)
        for name, state in states.items():
            state.diagnoses = tuple(diag.get(name, ()))
        self.global_diagnoses = tuple(diag.get(None, ()))

        voice_result = self._results.get("file:voice-heartbeat")
        self.last_busy = compute(
            self._results,
            voice_result.evidence if voice_result and voice_result.ok else {},
        )
        self._record_transitions(states, now)
        self._prev = states
        self._committed = states
        self._write_state(states)
        return states

    def _record_transitions(self, states, now: float) -> None:
        # 详细实现见 Task 11。
        pass

    def _write_state(self, states) -> None:
        # 详细实现见 Task 11。
        pass
```

> `_record_transitions` / `_write_state` 在 Task 11 补全；本任务的测试只要求状态与 `state.json`。为了让本任务测试通过，Step 3 里 `_write_state` 必须已经写入 `state.json`。因此**实际实现时把 Task 11 的 `_write_state` 代码直接写入本任务**，Task 11 只补 `_record_transitions`。测试文件不因任务顺序而改变。

- [ ] **Step 4: 为 Task 10 补上 `_write_state`（本步即刻完成，不等 Task 11）**

```python
    def _write_state(self, states) -> None:
        now = self._clock()
        payload = {
            "wall": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self._wall())),
            "wall_ts": round(self._wall(), 3),
            "pid": os.getpid(),
            "interval_s": self.config.interval_s,
            "busy": asdict(self.last_busy),
            "diagnoses_global": [asdict(d) for d in self.global_diagnoses],
            "components": {
                name: {
                    "display": st.display,
                    "verdict": st.verdict.value,
                    "reason": st.reason,
                    "root_cause": st.root_cause,
                    "failing": list(st.failing),
                    "diagnoses": [asdict(d) for d in st.diagnoses],
                    "since_s": round(now - st.since, 1),
                    "evidence": st.evidence,
                }
                for name, st in states.items()
            },
        }
        path = Path(self.config.state_dir) / "state.json"
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False, default=str),
                       encoding="utf-8")
        os.replace(tmp, path)
```

- [ ] **Step 5: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_engine.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 6: 提交**

```bash
git add pi/carhealth/carhealth/engine.py pi/carhealth/tests/test_engine.py
git commit -m "feat(health): 判定引擎与状态文件"
```

---

## Task 11: 故障日志接入与指纹

**Files:**
- Modify: `pi/carhealth/carhealth/engine.py`（补 `_record_transitions`）
- Test: `pi/carhealth/tests/test_engine.py`（追加）

**Interfaces:**
- Consumes: `classify.fingerprint`、`Journal`
- Produces: 日志事件 `incident_open` / `incident_update` / `incident_close`，字段与设计 §8 一致

- [ ] **Step 1: 追加失败测试**

追加到 `pi/carhealth/tests/test_engine.py`：

```python
def _events(tmp_path):
    eng_journal = EngineConfig(state_dir=tmp_path / "state", journal_dir=tmp_path / "journal")
    from carhealth.journal import Journal
    return Journal(eng_journal.journal_dir).read()


def test_open_and_close_incident(tmp_path):
    probe = FakeProbe("p", {"age_s": 9.0})
    eng, clock = make([probe], tmp_path=tmp_path)
    eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["incident_open"]
    assert evs[0]["component"] == "a"
    assert evs[0]["to"] == "STALE"
    assert evs[0]["action"] is None
    probe.evidence["age_s"] = 0.1
    for _ in range(5):          # 迟滞：5 次好才恢复
        clock.now += 1
        eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["incident_open", "incident_close"]
    assert evs[1]["duration_s"] == 5.0


def test_unknown_does_not_open_component_incident(tmp_path):
    class Boom(FakeProbe):
        def poll(self):
            raise RuntimeError("probe blind")

    eng, clock = make([Boom("p", {})], tmp_path=tmp_path)
    eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["monitor_blind"]
    clock.now += 10
    eng.run_once()
    assert [e["event"] for e in _events(tmp_path)] == ["monitor_blind"]  # 限流


def test_same_incident_not_duplicated(tmp_path):
    eng, clock = make([FakeProbe("p", {"age_s": 9.0})], tmp_path=tmp_path)
    eng.run_once()
    eng.run_once()
    assert len(_events(tmp_path)) == 1


def test_fingerprint_change_emits_update(tmp_path):
    probe = FakeProbe("p", {"age_s": 9.0})
    from carhealth.types import Check, ComponentSpec, Verdict
    specs = (
        ComponentSpec("a", "A", (
            Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
            Check("p", "gt", 10.0, field="age_s", level=Verdict.DEGRADED, reason="帧率低"),
        )),
    )
    eng, _ = make([probe], specs=specs, tmp_path=tmp_path)
    eng.run_once()
    probe.evidence["age_s"] = 20.0
    eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["incident_open", "incident_update"]
    assert evs[0]["fingerprint"] != evs[1]["fingerprint"]


def test_interlock_is_written_into_incident(tmp_path):
    eng, _ = make(
        [FakeProbe("p", {"age_s": 9.0}),
         FakeProbe("topic:/cmd_vel_mux", {"age_s": 0.1, "linear_x": 0.3, "angular_z": 0.0})],
        tmp_path=tmp_path)
    eng.run_once()
    assert _events(tmp_path)[0]["interlock"]["chassis"] is True
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_engine.py -q`
Expected: FAIL — 新增 4 个测试失败（日志为空）

- [ ] **Step 3: 实现 `_record_transitions`**

把 `engine.py` 中的占位替换为：

```python
    def _record_transitions(self, states, now: float) -> None:
        for name, cur in states.items():
            prev = self._prev.get(name)
            prev_v = prev.verdict if prev else None
            fp = fingerprint(name, cur.verdict, cur.root_cause, list(cur.failing))

            # 失明不是组件故障：只记限流的 monitor_blind，不进 incident 时间线。
            if cur.verdict is Verdict.UNKNOWN:
                self._record_blind(name, cur)
                continue

            if cur.verdict is Verdict.OK:
                info = self._incidents.pop(name, None)
                if info is not None:
                    self._journal.record({
                        "event": "incident_close",
                        "incident": info["id"],
                        "component": name,
                        "from": prev_v.value if prev_v else "UNKNOWN",
                        "to": "OK",
                        "duration_s": round(now - info["started"], 1),
                        "fingerprint": info["fingerprint"],
                    })
                continue

            info = self._incidents.get(name)
            if info is None:
                info = {
                    "id": "I-%s-%s" % (
                        time.strftime("%Y%m%d-%H%M%S", time.localtime(self._wall())), name),
                    "started": now,
                    "fingerprint": fp,
                }
                self._incidents[name] = info
                event = "incident_open"
            elif info["fingerprint"] != fp:
                info["fingerprint"] = fp
                event = "incident_update"
            else:
                continue

            self._journal.record({
                "event": event,
                "incident": info["id"],
                "component": name,
                "from": prev_v.value if prev_v else "UNKNOWN",
                "to": cur.verdict.value,
                "reason": cur.reason,
                "root_cause": cur.root_cause,
                "fingerprint": fp,
                "failing": list(cur.failing),
                "diagnoses": [asdict(d) for d in cur.diagnoses],
                "evidence": cur.evidence,
                "interlock": asdict(self.last_busy),
                "action": None,
            })

    def _record_blind(self, name: str, cur: ComponentState) -> None:
        wall = self._wall()
        if wall - self._last_blind_wall < 300:
            return
        self._last_blind_wall = wall
        self._journal.record({
            "event": "monitor_blind",
            "component": name,
            "reason": cur.reason,
            "evidence": cur.evidence,
            "action": None,
        })
```

并在 `engine.py` 顶部导入：

```python
from .classify import attach_graph_root_cause, evaluate, fingerprint
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_engine.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/engine.py pi/carhealth/tests/test_engine.py
git commit -m "feat(health): 故障日志接入与指纹"
```

---

## Task 12: 探针/契约装配（registry）

**Files:**
- Create: `pi/carhealth/carhealth/registry.py`
- Test: `pi/carhealth/tests/test_registry.py`

**Interfaces:**
- Consumes: `contracts.build_specs`、全部探针、`Engine`
- Produces: `ProbeBundle(probes, closer)`、`build_probes(ros=True, board_host=None, fake_throttled=None) -> ProbeBundle`、`build_engine(state_dir=None, journal_dir=None, ros=True, board_host=None, fake_throttled=None, interval_s=1.0) -> Engine`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_registry.py`：

```python
from carhealth.contracts import build_specs
from carhealth.registry import build_engine, build_probes


def test_all_contract_probes_are_built():
    bundle = build_probes(ros=False, board_host="127.0.0.1")
    try:
        needed = {c.probe for s in build_specs() for c in s.checks}
        missing = needed - set(bundle.probes)
        assert not missing, missing
    finally:
        if bundle.closer:
            bundle.closer()


def test_build_engine_wires_specs_and_probes(tmp_path):
    eng = build_engine(state_dir=tmp_path, journal_dir=tmp_path,
                       ros=False, board_host="127.0.0.1")
    try:
        assert {s.name for s in eng.specs} == {s.name for s in build_specs()}
        assert "topic:/tof/status" in eng.probes
        assert "unit:ps2-teleop" in eng.probes
        assert {"service", "usb", "dds", "config_drift", "udev:js0"} <= set(eng.probes)
    finally:
        eng.close()


def test_ros_false_does_not_import_rclpy():
    bundle = build_probes(ros=False, board_host="127.0.0.1")
    try:
        assert bundle.closer is None
    finally:
        if bundle.closer:
            bundle.closer()
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_registry.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.registry'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/registry.py`：

```python
"""把契约、探针、引擎装配起来。daemon 与 doctor 共用。"""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

from .contracts import (
    FRAMED_STATUS, LATEST_IDR, RESULTD_STATUS, ROSBRIDGE_ADDR,
    VOICE_HEARTBEAT_PATHS, build_specs,
)
from .engine import Engine, EngineConfig, default_journal_dir, default_state_dir
from .probes.config_drift import ConfigDriftProbe
from .probes.dds import DdsProbe
from .probes.file_status import FileStatusProbe
from .probes.host import HostProbe
from .probes.net import PingProbe, TcpProbe, resolve_board_host
from .probes.power import PowerProbe
from .probes.ros_topic import RosNodeBackend, RosTopicStore, TopicProbe
from .probes.service_health import ServiceHealthProbe
from .probes.systemd import SystemdCache, UnitProbe
from .probes.udev import UdevProbe
from .probes.usb_events import UsbEventsProbe

UNITS = ("ps2-teleop", "tof-a010", "k230-resultd", "k230-bridge",
         "k230-framed", "rosbridge", "arm-grasp")

TOPICS = ("/tof/status", "/tof/depth", "/wheel_odom", "/imu/data_raw",
          "/arm/feedback", "/arm/online", "/k230/result",
          "/health/arm_grasp", "/cmd_vel_mux", "/arm/command",
          "/battery_voltage", "/chassis_faults", "/arm/status")


@dataclass
class ProbeBundle:
    probes: dict
    closer: object = None


def build_probes(ros: bool = True, board_host: str | None = None,
                 fake_throttled: str | None = None,
                 systemd_runner=subprocess.run, repo: str | None = None) -> ProbeBundle:
    store = RosTopicStore()
    probes: dict = {}

    probes["power"] = PowerProbe(fake_output=fake_throttled)
    probes["host"] = HostProbe()

    cache = SystemdCache(UNITS, runner=systemd_runner)
    unit_probes = {}
    for unit in UNITS:
        probe = UnitProbe("unit:" + unit, unit, cache)
        probes[probe.name] = probe
        unit_probes[unit] = probe
    probes["service"] = ServiceHealthProbe(unit_probes)

    probes["udev:l150pro"] = UdevProbe("udev:l150pro", ["/dev/l150pro"])
    probes["udev:a010"] = UdevProbe(
        "udev:a010", ["/dev/a010", "/dev/serial/by-id/*SIPEED*if00-port0"],
        expect_interface="00")
    probes["udev:js0"] = UdevProbe("udev:js0", ["/dev/input/js0"])

    probes["file:resultd-status"] = FileStatusProbe("file:resultd-status", [RESULTD_STATUS])
    probes["file:framed-status"] = FileStatusProbe("file:framed-status", [FRAMED_STATUS])
    probes["file:latest-idr"] = FileStatusProbe("file:latest-idr", [LATEST_IDR],
                                                parse_json=False)
    probes["file:voice-heartbeat"] = FileStatusProbe(
        "file:voice-heartbeat", VOICE_HEARTBEAT_PATHS)

    if board_host is None:
        host_fn = lambda: resolve_board_host(  # noqa: E731
            status_path=str(RESULTD_STATUS))
    else:
        host_fn = lambda: board_host          # noqa: E731
    probes["net:board"] = PingProbe("net:board", host_fn=host_fn)
    probes["net:rosbridge"] = TcpProbe("net:rosbridge", ROSBRIDGE_ADDR[0],
                                       ROSBRIDGE_ADDR[1])

    probes["usb"] = UsbEventsProbe()
    probes["dds"] = DdsProbe()
    probes["config_drift"] = ConfigDriftProbe(
        repo=repo or os.environ.get(
            "CARHEALTH_REPO", str(Path(__file__).resolve().parents[3])))

    for topic in TOPICS:
        probes["topic:" + topic] = TopicProbe("topic:" + topic, topic, store)

    closer = None
    if ros:
        backend = RosNodeBackend(store)
        if backend.start():
            closer = backend.stop
    return ProbeBundle(probes=probes, closer=closer)


def build_engine(state_dir=None, journal_dir=None, ros: bool = True,
                 board_host: str | None = None, fake_throttled: str | None = None,
                 interval_s: float = 1.0, systemd_runner=subprocess.run,
                 repo: str | None = None) -> Engine:
    state_dir = Path(state_dir) if state_dir else default_state_dir()
    journal_dir = Path(journal_dir) if journal_dir else default_journal_dir()
    bundle = build_probes(ros=ros, board_host=board_host,
                          fake_throttled=fake_throttled,
                          systemd_runner=systemd_runner, repo=repo)
    engine = Engine(build_specs(), bundle.probes,
                    EngineConfig(state_dir=state_dir, journal_dir=journal_dir,
                                 interval_s=interval_s))
    engine.closer = bundle.closer
    return engine
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_registry.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/registry.py pi/carhealth/tests/test_registry.py
git commit -m "feat(health): 探针与契约装配"
```

---

## Task 13: 常驻守护入口

**Files:**
- Create: `pi/carhealth/carhealth/daemon.py`
- Test: `pi/carhealth/tests/test_daemon.py`

**Interfaces:**
- Consumes: `registry.build_engine`、`engine.default_state_dir`
- Produces: `main(argv=None, engine_factory=None) -> int`；参数 `--state-dir`、`--journal-dir`、`--interval`、`--once`、`--no-ros`

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_daemon.py`：

```python
from carhealth import daemon
from carhealth.types import ComponentState, Verdict


class FakeEngine:
    def __init__(self):
        self.runs = 0
        self.closed = False

    def run_once(self):
        self.runs += 1
        return {"a": ComponentState("a", "A", Verdict.OK, "正常", {}, None, (),
                                    (), 1.0, 1.0)}

    def close(self):
        self.closed = True


def test_once_runs_exactly_once_and_closes(tmp_path):
    engine = FakeEngine()
    code = daemon.main(["--once", "--state-dir", str(tmp_path)],
                       engine_factory=lambda args: engine)
    assert code == 0
    assert engine.runs == 1
    assert engine.closed is True


class FakeStop:
    def __init__(self):
        self.calls = []

    def is_set(self):
        return False

    def wait(self, seconds):
        self.calls.append(seconds)
        return True          # 立刻“收到停止信号”


def test_loop_stops_on_event(tmp_path):
    engine = FakeEngine()
    stop = FakeStop()
    daemon._loop(engine, 1.0, stop)
    assert engine.runs == 1
    assert stop.calls == [1.0]
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_daemon.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.daemon'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/daemon.py`：

```python
#!/usr/bin/env python3
"""carhealthd：常驻健康守护。一期只观察、只记录。"""

from __future__ import annotations

import argparse
import signal
import sys
import threading

from .engine import default_journal_dir, default_state_dir
from .registry import build_engine


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="整车健康守护（一期：只观察）")
    p.add_argument("--state-dir", default=str(default_state_dir()))
    p.add_argument("--journal-dir", default=None)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--once", action="store_true", help="跑一轮就退出（验收用）")
    p.add_argument("--no-ros", action="store_true", help="不启动 rclpy 后端")
    return p.parse_args(argv)


def _loop(engine, interval_s: float, stop) -> None:
    while not stop.is_set():
        engine.run_once()
        if stop.wait(interval_s):
            break


def _default_factory(args):
    return build_engine(state_dir=args.state_dir,
                        journal_dir=args.journal_dir or str(default_journal_dir()),
                        ros=not args.no_ros,
                        interval_s=args.interval)


def main(argv=None, engine_factory=None) -> int:
    args = parse_args(argv)
    engine = (engine_factory or _default_factory)(args)

    stop = threading.Event()

    def _stop(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        if args.once:
            engine.run_once()
            return 0
        _loop(engine, args.interval, stop)
        return 0
    finally:
        engine.close()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_daemon.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/daemon.py pi/carhealth/tests/test_daemon.py
git commit -m "feat(health): carhealthd 入口"
```

---

## Task 14: CLI `status / history / why`

**Files:**
- Create: `pi/carhealth/carhealth/cli.py`
- Test: `pi/carhealth/tests/test_cli.py`

**Interfaces:**
- Consumes: `state.json` 结构（见 Task 10）、`Journal`
- Produces: `main(argv=None) -> int`；`cmd_status(args)`、`cmd_history(args)`、`cmd_why(args)`、`parse_since(text) -> float`

`state.json` 结构（与 Task 10 一致）：

```json
{"wall": "...", "wall_ts": 1763388000.0, "pid": 123, "interval_s": 1.0,
 "busy": {"arm": false, "chassis": false, "grasp": false, "voice": false},
 "components": {"tof": {"display": "ToF 深度", "verdict": "STALE", "reason": "...",
   "root_cause": null, "failing": ["..."], "since_s": 12.0, "evidence": {}}}}
```

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_cli.py`：

```python
import json

from carhealth import cli


def write_state(tmp_path, verdict="OK", stale=False, wall_ts=None):
    state_dir = tmp_path / "state"
    state_dir.mkdir()
    payload = {
        "wall": "2026-11-17T14:00:00",
        "wall_ts": (1763388000.0 if wall_ts is None else wall_ts),
        "pid": 123,
        "interval_s": 1.0,
        "busy": {"arm": False, "chassis": False, "grasp": False, "voice": False},
        "diagnoses_global": [],
        "components": {
            "tof": {"display": "ToF 深度", "verdict": verdict, "reason": "没数据",
                    "root_cause": None, "failing": ["没数据"], "since_s": 12.0,
                    "diagnoses": [{"id": "usb.disconnect_storm",
                                   "title": "USB 在反复重枚举",
                                   "action_class": "human",
                                   "hint": "换线/换口/查供电"}],
                    "evidence": {}},
        },
    }
    (state_dir / "state.json").write_text(json.dumps(payload))
    return state_dir


def test_status_prints_table(tmp_path, capsys):
    state_dir = write_state(tmp_path, verdict="STALE")
    code = cli.main(["status", "--state-dir", str(state_dir),
                     "--now", "1763388005"])
    out = capsys.readouterr().out
    assert code == 0
    assert "tof" in out
    assert "STALE" in out
    assert "没数据" in out
    assert "USB 在反复重枚举" in out
    assert "换线/换口/查供电" in out


def test_status_reports_stale_daemon(tmp_path, capsys):
    state_dir = write_state(tmp_path, wall_ts=1763388000.0)
    code = cli.main(["status", "--state-dir", str(state_dir),
                     "--now", "1763388100"])
    out = capsys.readouterr().out
    assert code == 2
    assert "守护未运行" in out


def test_status_json(tmp_path, capsys):
    state_dir = write_state(tmp_path)
    code = cli.main(["status", "--state-dir", str(state_dir),
                     "--now", "1763388005", "--json"])
    assert code == 0
    assert json.loads(capsys.readouterr().out)["components"]["tof"]["verdict"] == "OK"


def test_history_and_why(tmp_path, capsys):
    journal_dir = tmp_path / "journal"
    journal_dir.mkdir()
    line = json.dumps({"ts": 1763388001.0, "event": "incident_open",
                       "incident": "I-1", "component": "tof", "to": "STALE",
                       "reason": "没数据", "evidence": {"x": 1}})
    (journal_dir / "journal-20261117.jsonl").write_text(line + "\n")

    assert cli.main(["history", "--journal-dir", str(journal_dir),
                     "--since", "all"]) == 0
    assert "incident_open" in capsys.readouterr().out

    assert cli.main(["why", "tof", "--journal-dir", str(journal_dir)]) == 0
    out = capsys.readouterr().out
    assert "I-1" in out
    assert "没数据" in out


def test_why_unknown_component(tmp_path, capsys):
    journal_dir = tmp_path / "journal"
    journal_dir.mkdir()
    assert cli.main(["why", "nope", "--journal-dir", str(journal_dir)]) == 1
    assert "没有记录" in capsys.readouterr().out


def test_parse_since():
    assert cli.parse_since("1h") == 3600
    assert cli.parse_since("30m") == 1800
    assert cli.parse_since("2d") == 172800
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_cli.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'carhealth.cli'`

- [ ] **Step 3: 写实现**

`pi/carhealth/carhealth/cli.py`：

```python
#!/usr/bin/env python3
"""carhealthctl：status / doctor / history / why。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .engine import default_state_dir
from .journal import Journal

STATE_STALE_S = 10.0


def parse_since(text: str) -> int:
    unit = text[-1]
    value = int(text[:-1])
    return value * {"m": 60, "h": 3600, "d": 86400}[unit]


def _load_state(state_dir: Path) -> dict | None:
    try:
        return json.loads((Path(state_dir) / "state.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def cmd_status(args) -> int:
    payload = _load_state(args.state_dir)
    now = float(args.now) if args.now else time.time()
    if payload is None:
        print("守护未运行：找不到 %s/state.json" % args.state_dir)
        return 2
    age = now - float(payload.get("wall_ts", 0))
    if age > STATE_STALE_S:
        print("守护未运行：状态文件已过期 %.0fs（%s）" % (age, args.state_dir))
        return 2
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    busy = payload.get("busy", {})
    print("carhealthd pid=%s  wall=%s  状态年龄=%.0fs"
          % (payload.get("pid"), payload.get("wall"), age))
    print("互锁: 臂=%s 底盘=%s 抓取=%s 语音=%s"
          % (busy.get("arm"), busy.get("chassis"), busy.get("grasp"), busy.get("voice")))
    print()
    print("%-12s %-9s %-6s %-14s %s" % ("组件", "状态", "持续", "根因", "原因"))
    for name, c in payload.get("components", {}).items():
        print("%-12s %-9s %-6ss %-14s %s" % (
            name, c.get("verdict"), c.get("since_s", 0),
            c.get("root_cause") or "-", c.get("reason", "")))
        for d in c.get("diagnoses") or []:
            print("    → [%s] %s：%s" % (d.get("action_class"), d.get("title"),
                                        d.get("hint")))
    globals_ = payload.get("diagnoses_global") or []
    if globals_:
        print()
        print("全局结论:")
        for d in globals_:
            print("  → [%s] %s：%s" % (d.get("action_class"), d.get("title"),
                                      d.get("hint")))
    return 0


def cmd_history(args) -> int:
    journal = Journal(args.journal_dir)
    if args.since in (None, "all"):
        events = journal.read()
    else:
        events = journal.read_since(float(parse_since(args.since)))
    if not events:
        print("没有记录")
        return 0
    for e in events:
        print("%s  %-16s %-12s %s->%s  %s" % (
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(e.get("ts", 0))),
            e.get("event", "?"), e.get("component", "-"),
            e.get("from", "-"), e.get("to", "-"),
            e.get("reason", e.get("root_cause", ""))))
    return 0


def cmd_why(args) -> int:
    journal = Journal(args.journal_dir)
    events = [e for e in journal.read()
              if e.get("component") == args.component]
    if not events:
        print("没有记录：%s" % args.component)
        return 1
    for e in events[-5:]:
        print(json.dumps(e, ensure_ascii=False, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="carhealthctl")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("status", help="表格 / JSON 查看当前健康")
    s.add_argument("--state-dir", default=str(default_state_dir()))
    s.add_argument("--json", action="store_true")
    s.add_argument("--now", default=None, help="测试用：覆盖当前时间戳")
    s.set_defaults(func=cmd_status)

    h = sub.add_parser("history", help="查看故障时间线")
    h.add_argument("--journal-dir", default=str(default_state_dir()))
    h.add_argument("--since", default="1h", help="1h / 30m / 2d / all")
    h.set_defaults(func=cmd_history)

    w = sub.add_parser("why", help="某组件最近一次异常的完整证据")
    w.add_argument("component")
    w.add_argument("--journal-dir", default=str(default_state_dir()))
    w.set_defaults(func=cmd_why)

    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
```

> `doctor` 在 Task 15 加进同一个 `build_parser()`；本任务先不注册，避免测试互相依赖。

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_cli.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/cli.py pi/carhealth/tests/test_cli.py
git commit -m "feat(health): status/history/why 命令"
```

---

## Task 15: `doctor` 出车体检

**Files:**
- Modify: `pi/carhealth/carhealth/cli.py`
- Test: `pi/carhealth/tests/test_doctor.py`

**Interfaces:**
- Consumes: `Engine`、`build_specs`、`build_probes`（通过注入的 `engine_factory`）
- Produces: `cmd_doctor(args, engine_factory=None, sleeper=time.sleep) -> int`；退出码 0 PASS / 1 WARN / 2 FAIL

- [ ] **Step 1: 写失败测试**

`pi/carhealth/tests/test_doctor.py`：

```python
import argparse
import json
import time

from carhealth import cli
from carhealth.types import ComponentState, Verdict


class FakeEngine:
    def __init__(self, verdict):
        self.verdict = verdict
        self.closed = False

    def run_once(self):
        return {
            "power": ComponentState("power", "供电", self.verdict, "欠压", {}, "power",
                                    ("欠压",), (), 0.0, 0.0),
        }

    def close(self):
        self.closed = True


def args(tmp_path, **kw):
    base = dict(state_dir=str(tmp_path), journal_dir=str(tmp_path),
                no_ros=True, no_daemon=True, warmup=0.0,
                board_host=None, fake_throttled=None)
    base.update(kw)
    return argparse.Namespace(**base)


def write_state(tmp_path, verdict="OK", wall_ts=None):
    state_dir = tmp_path / "state"
    state_dir.mkdir(exist_ok=True)
    (state_dir / "state.json").write_text(json.dumps({
        "wall_ts": time.time() if wall_ts is None else wall_ts,
        "components": {"power": {"display": "供电", "verdict": verdict,
                                 "reason": "欠压", "root_cause": "power",
                                 "diagnoses": []}},
        "diagnoses_global": [],
    }))
    return state_dir


def test_doctor_pass(tmp_path, capsys):
    eng = FakeEngine(Verdict.OK)
    code = cli.cmd_doctor(args(tmp_path), engine_factory=lambda a: eng,
                          sleeper=lambda s: None)
    assert code == 0
    assert "PASS" in capsys.readouterr().out
    assert eng.closed is True


def test_doctor_fail_on_blocked(tmp_path, capsys):
    eng = FakeEngine(Verdict.BLOCKED)
    code = cli.cmd_doctor(args(tmp_path), engine_factory=lambda a: eng,
                          sleeper=lambda s: None)
    assert code == 2
    assert "FAIL" in capsys.readouterr().out


def test_doctor_warn_on_degraded(tmp_path, capsys):
    eng = FakeEngine(Verdict.DEGRADED)
    code = cli.cmd_doctor(args(tmp_path), engine_factory=lambda a: eng,
                          sleeper=lambda s: None)
    assert code == 1
    assert "WARN" in capsys.readouterr().out


def test_doctor_reuses_fresh_daemon_state(tmp_path, capsys):
    state_dir = write_state(tmp_path, verdict="STALE")
    def factory(a):
        raise AssertionError("守护在跑时不应自己起 ROS 节点")
    code = cli.cmd_doctor(args(state_dir, no_daemon=False), engine_factory=factory,
                          sleeper=lambda s: None)
    assert code == 2
    out = capsys.readouterr().out
    assert "carhealthd" in out
    assert "FAIL" in out


def test_doctor_ignores_stale_daemon_state(tmp_path, capsys):
    state_dir = write_state(tmp_path, verdict="STALE", wall_ts=time.time() - 100)
    eng = FakeEngine(Verdict.OK)
    code = cli.cmd_doctor(args(state_dir, no_daemon=False),
                          engine_factory=lambda a: eng, sleeper=lambda s: None)
    assert code == 0
    assert eng.closed is True
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/carhealth && python3 -m pytest tests/test_doctor.py -q`
Expected: FAIL — `AttributeError: module 'carhealth.cli' has no attribute 'cmd_doctor'`

- [ ] **Step 3: 写实现**

在 `cli.py` 加入：

```python
def _level(verdict) -> tuple[str, int]:
    value = getattr(verdict, "value", verdict)
    if value == "OK":
        return "PASS", 0
    if value in ("DEGRADED", "UNKNOWN"):
        return "WARN", 1
    return "FAIL", 2


def _read_fresh_state(state_dir):
    payload = _load_state(state_dir)
    if payload is None:
        return None
    if time.time() - float(payload.get("wall_ts", 0)) > STATE_STALE_S:
        return None
    return payload


def _print_daemon_state(payload) -> int:
    print("（使用运行中的 carhealthd 状态，未自建 ROS 节点）")
    worst = 0
    print("%-12s %-6s %s" % ("组件", "结果", "说明"))
    for name, c in payload.get("components", {}).items():
        level, code = _level(c.get("verdict"))
        worst = max(worst, code)
        print("%-12s %-6s %s%s" % (
            name, level, c.get("reason", ""),
            "" if not c.get("root_cause") else "（根因 %s）" % c["root_cause"]))
        for d in c.get("diagnoses") or []:
            print("    → [%s] %s：%s" % (d.get("action_class"),
                                        d.get("title"), d.get("hint")))
    for d in payload.get("diagnoses_global") or []:
        print("    → [%s] %s：%s" % (d.get("action_class"),
                                    d.get("title"), d.get("hint")))
    print()
    print({0: "总体: PASS", 1: "总体: WARN", 2: "总体: FAIL"}[worst])
    return worst


def cmd_doctor(args, engine_factory=None, sleeper=time.sleep) -> int:
    import tempfile

    from .registry import build_engine

    # 守护在跑且状态新鲜：直接读它的结果。FastDDS 在这台 Pi 上发现慢，
    # 自己起节点等几秒容易误判 STALE。
    if not getattr(args, "no_daemon", False):
        payload = _read_fresh_state(args.state_dir)
        if payload is not None:
            return _print_daemon_state(payload)

    # 自建探针时用临时目录，**绝不能覆盖常驻守护的 state.json / journal**。
    with tempfile.TemporaryDirectory(prefix="carhealth-doctor-") as td:
        args.state_dir = td
        args.journal_dir = td

        if engine_factory is None:
            def engine_factory(a):
                return build_engine(state_dir=a.state_dir, journal_dir=a.journal_dir,
                                    ros=not a.no_ros, board_host=a.board_host,
                                    fake_throttled=a.fake_throttled)

        engine = engine_factory(args)
        try:
            if not args.no_ros:
                sleeper(float(getattr(args, "warmup", 8.0)))
            states = engine.run_once()
        finally:
            engine.close()

        worst = 0
        print("%-12s %-6s %s" % ("组件", "结果", "说明"))
        for name, st in states.items():
            level, code = _level(st.verdict)
            worst = max(worst, code)
            print("%-12s %-6s %s%s" % (
                name, level, st.reason,
                "" if st.root_cause is None else "（根因 %s）" % st.root_cause))
            for d in st.diagnoses:
                print("    → [%s] %s：%s" % (d.action_class, d.title, d.hint))
        for d in getattr(engine, "global_diagnoses", ()):
            print("    → [%s] %s：%s" % (d.action_class, d.title, d.hint))
        print()
        print({0: "总体: PASS", 1: "总体: WARN", 2: "总体: FAIL"}[worst])
        return worst
```

并在 `build_parser()` 里注册：

```python
    d = sub.add_parser("doctor", help="出车体检")
    d.add_argument("--state-dir", default=str(default_state_dir()))
    d.add_argument("--no-daemon", action="store_true",
                   help="不读运行中守护的状态，强制自建探针")
    d.add_argument("--warmup", type=float, default=8.0,
                   help="自建 ROS 节点时等发现的时间（秒）")
    d.add_argument("--no-ros", action="store_true")
    d.add_argument("--board-host", default=None)
    d.add_argument("--fake-throttled", default=None,
                   help="测试/验收用：伪造 vcgencmd 输出，如 0x50000")
    d.set_defaults(func=cmd_doctor)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/carhealth && python3 -m pytest tests/test_doctor.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/carhealth/cli.py pi/carhealth/tests/test_doctor.py
git commit -m "feat(health): doctor 出车体检"
```

---

## Task 16: 语音助手心跳

**Files:**
- Create: `pi/voice-chatbot/utils/heartbeat.py`
- Test: `pi/voice-chatbot/tests/test_heartbeat.py`
- Modify: `pi/voice-chatbot/core/conversation.py`

**Interfaces:**
- Consumes: 无
- Produces: `Heartbeat(path, clock=time.monotonic, wall=time.time, interval_s=1.0)`、`default_path()`、`update(state, busy)`、`payload()`、`write_once()`、`start()`、`stop()`

- [ ] **Step 1: 写失败测试**

`pi/voice-chatbot/tests/test_heartbeat.py`：

```python
import json

from utils.heartbeat import Heartbeat, default_path


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_payload_tracks_state_and_busy_duration(tmp_path):
    clock = Clock()
    hb = Heartbeat(tmp_path / "hb.json", clock=clock, wall=lambda: 1000.0)
    hb.update("idle", busy=False)
    clock.now = 10.0
    hb.update("thinking", busy=True)
    clock.now = 25.0
    p = hb.payload()
    assert p["state"] == "thinking"
    assert p["busy"] is True
    assert p["busy_for_s"] == 15.0
    assert p["state_age_s"] == 15.0


def test_busy_for_resets_when_idle(tmp_path):
    clock = Clock()
    hb = Heartbeat(tmp_path / "hb.json", clock=clock)
    hb.update("thinking", busy=True)
    clock.now = 10.0
    hb.update("idle", busy=False)
    p = hb.payload()
    assert p["busy"] is False
    assert p["busy_for_s"] == 0.0


def test_status_provider_is_merged(tmp_path):
    hb = Heartbeat(
        tmp_path / "hb.json", clock=lambda: 1.0,
        status_provider=lambda: {"wakeword_ready": False, "mic_open": True})
    p = hb.payload()
    assert p["wakeword_ready"] is False
    assert p["mic_open"] is True


def test_status_provider_exception_is_ignored(tmp_path):
    def boom():
        raise RuntimeError("device driver exploded")

    hb = Heartbeat(tmp_path / "hb.json", clock=lambda: 1.0, status_provider=boom)
    assert hb.payload()["busy"] is False


def test_write_once_is_atomic_json(tmp_path):
    path = tmp_path / "hb.json"
    hb = Heartbeat(path, clock=lambda: 1.0, wall=lambda: 1000.0)
    hb.update("listening", busy=False)
    hb.write_once()
    data = json.loads(path.read_text())
    assert data["state"] == "listening"
    assert data["busy"] is False
    assert not (tmp_path / "hb.json.tmp").exists()


def test_write_failure_is_swallowed(tmp_path):
    blocker = tmp_path / "blocker"
    blocker.write_text("x")          # 拿文件当父目录，mkdir 必失败
    hb = Heartbeat(blocker / "hb.json", clock=lambda: 1.0)
    hb.update("idle", busy=False)
    hb.write_once()          # 不抛异常


def test_default_path_returns_first_writable(tmp_path, monkeypatch):
    monkeypatch.setattr("os.access", lambda p, mode: False)
    monkeypatch.setattr("pathlib.Path.home", lambda: tmp_path)
    path = default_path()
    assert str(path).startswith(str(tmp_path))
    assert path.name == "heartbeat.json"
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/voice-chatbot && python3 -m pytest tests/test_heartbeat.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'utils.heartbeat'`

- [ ] **Step 3: 写实现**

`pi/voice-chatbot/utils/heartbeat.py`：

```python
"""给 carhealth 的 1Hz 心跳。写失败绝不打断语音助手。"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path


def default_path() -> Path:
    shm = Path("/dev/shm/voice-heartbeat.json")
    if os.access("/dev/shm", os.W_OK):
        return shm
    base = Path.home() / ".local/state/voice-chatbot"
    base.mkdir(parents=True, exist_ok=True)
    return base / "heartbeat.json"


class Heartbeat:
    def __init__(self, path, clock=time.monotonic, wall=time.time,
                 interval_s: float = 1.0, status_provider=None):
        self.path = Path(path)
        self._clock = clock
        self._wall = wall
        self._interval = interval_s
        self._status_provider = status_provider
        now = clock()
        self._t0 = now
        self._state = "idle"
        self._state_since = now
        self._busy = False
        self._busy_since = now
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def update(self, state: str, busy: bool | None = None) -> None:
        now = self._clock()
        if state != self._state:
            self._state = state
            self._state_since = now
        if busy is not None and bool(busy) != self._busy:
            self._busy = bool(busy)
            if self._busy:
                self._busy_since = now

    def payload(self) -> dict:
        now = self._clock()
        data = {
            "pid": os.getpid(),
            "state": self._state,
            "busy": self._busy,
            "state_age_s": round(now - self._state_since, 1),
            "busy_for_s": round(now - self._busy_since, 1) if self._busy else 0.0,
            "uptime_s": round(now - self._t0, 1),
            "beat_wall": round(self._wall(), 3),
        }
        if self._status_provider is not None:
            try:
                extra = self._status_provider()
            except Exception:                               # noqa: BLE001
                extra = None
            if isinstance(extra, dict):
                data.update(extra)
        return data

    def write_once(self) -> None:
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp.write_text(json.dumps(self.payload(), ensure_ascii=False),
                           encoding="utf-8")
            os.replace(tmp, self.path)
        except OSError:
            pass

    def _run(self) -> None:
        while not self._stop.wait(self._interval):
            self.write_once()

    def start(self) -> None:
        if self._thread is not None:
            return
        self.write_once()
        self._thread = threading.Thread(target=self._run, name="heartbeat", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/voice-chatbot && python3 -m pytest tests/test_heartbeat.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 暴露唤醒引擎 / 麦克风状态并接入 ConversationManager**

**(a) `pi/voice-chatbot/wakeword/engine.py`**（`time` 已导入）：

1. `__init__` 里 `self._last_detection_time = 0.0` 之后：

```python
        self._last_byte_mono: float | None = None
```

2. `_listen_loop` 里 `data = self._ser.read(...)` 成功之后：

```python
                    self._last_byte_mono = time.monotonic()
```

3. `is_ready` 属性后面：

```python
    @property
    def is_listening(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def last_byte_age_s(self):
        if self._last_byte_mono is None:
            return None
        return round(time.monotonic() - self._last_byte_mono, 1)
```

4. 在 `tests/test_wakeword_engine.py` 末尾加：

```python
def test_status_properties_before_start():
    engine = WakeWordEngine(_FakeConfig(**{
        "wakeword.port": "/dev/fake",
    }))
    assert engine.is_listening is False
    assert engine.last_byte_age_s is None
```

**(b) `pi/voice-chatbot/audio/mic.py`**：

1. `__init__` 里 `self._device_index = ...` 之后：

```python
        self.last_error: str | None = None
```

2. 新增属性：

```python
    @property
    def is_open(self) -> bool:
        return self._stream is not None
```

3. `read_chunk` 改成记录错误再抛：

```python
    def read_chunk(self) -> bytes:
        try:
            data = self._stream.read(self._chunk_size, exception_on_overflow=False)
            self.last_error = None
            return data
        except Exception as e:                              # noqa: BLE001
            self.last_error = repr(e)
            raise
```

**(c) `pi/voice-chatbot/core/conversation.py`**：

1. 顶部 import：

```python
from utils.heartbeat import Heartbeat, default_path
```

2. `self._state = StateMachine()` 之后：

```python
        self._heartbeat = Heartbeat(default_path(),
                                    status_provider=self._voice_status)
        for _s in ConversationState:
            self._state.on_enter(
                _s,
                lambda s=_s: self._heartbeat.update(
                    s.value,
                    busy=s in (ConversationState.THINKING, ConversationState.SPEAKING),
                ),
            )
```

> provider 在写文件时才调用，那时 `_mic`/`_wakeword` 早已构造完成。

3. `self._asr = ASRClient(config)` 之后：

```python
        self._asr_last_ok_wall = None
        self._asr_last_err_wall = None
```

4. `run()` 之前新增两个方法：

```python
    def _recognize(self, wav: bytes) -> str:
        try:
            text = self._asr.recognize(wav)
        except Exception:
            self._asr_last_err_wall = time.time()
            raise
        self._asr_last_ok_wall = time.time()
        return text

    def _voice_status(self) -> dict:
        """心跳里带上设备/引擎状态——只看线程活着会漏掉「停在 IDLE 喊不应」。"""
        return {
            "wakeword_ready": self._wakeword.is_ready,
            "wakeword_thread_alive": self._wakeword.is_listening,
            "wakeword_last_byte_age_s": self._wakeword.last_byte_age_s,
            "mic_open": self._mic.is_open,
            "mic_last_error": self._mic.last_error,
            "asr_last_ok_wall": self._asr_last_ok_wall,
            "asr_last_err_wall": self._asr_last_err_wall,
        }
```

5. 把两处 `self._asr.recognize(...)` 换成 `self._recognize(...)`：

   - `user_text = self._asr.recognize(self._pending_wav)` → `self._recognize(self._pending_wav)`
   - `text = (self._asr.recognize(wav_bytes) or "").strip()` → `(self._recognize(wav_bytes) or "").strip()`

6. `run()` 里 `self._wakeword.start(...)` 之前：

```python
        self._heartbeat.start()
```

7. `shutdown()` 里 `self._stop_event.set()` 之后：

```python
        self._heartbeat.stop()
```

- [ ] **Step 6: 跑全量测试并语法检查**

Run: `cd pi/voice-chatbot && python3 -m pytest -q`
Expected: PASS（原有测试全部通过；新增心跳/唤醒状态测试也通过）

- [ ] **Step 7: 提交**

```bash
git add pi/voice-chatbot/utils/heartbeat.py pi/voice-chatbot/tests/test_heartbeat.py pi/voice-chatbot/core/conversation.py
git commit -m "feat(voice): 给 carhealth 的 1Hz 心跳与忙闲标志"
```

---

## Task 17: 抓取服务心跳

**Files:**
- Create: `pi/raspot_ws/src/arm_grasp/arm_grasp/health.py`
- Modify: `pi/raspot_ws/src/arm_grasp/arm_grasp/grasp_server.py`
- Test: `pi/raspot_ws/src/arm_grasp/test/test_health.py`

**Interfaces:**
- Consumes: 无
- Produces: `health_payload(busy, uptime_s, wall=None) -> dict`

- [ ] **Step 1: 写失败测试**

`pi/raspot_ws/src/arm_grasp/test/test_health.py`：

```python
import json

from arm_grasp.health import health_payload


def test_payload_shape():
    p = health_payload(busy=True, uptime_s=12.34, wall=1000.0,
                       feedback_age_s=0.2, last_goal_wall=999.0)
    assert p["component"] == "arm_grasp"
    assert p["busy"] is True
    assert p["uptime_s"] == 12.3
    assert p["wall"] == 1000.0
    assert p["feedback_age_s"] == 0.2
    assert p["last_goal_wall"] == 999.0
    assert isinstance(p["pid"], int)
    json.dumps(p)          # 必须可序列化


def test_busy_is_bool_and_missing_feedback_is_none():
    p = health_payload(busy=1, uptime_s=0, wall=0.0)
    assert p["busy"] is True
    assert p["feedback_age_s"] is None
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/raspot_ws/src/arm_grasp && python3 -m pytest test/test_health.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'arm_grasp.health'`

- [ ] **Step 3: 写实现与接入**

`pi/raspot_ws/src/arm_grasp/arm_grasp/health.py`：

```python
"""抓取服务的 1Hz 心跳载荷。纯函数，方便单测。"""

from __future__ import annotations

import os
import time


def health_payload(busy, uptime_s: float, wall: float | None = None,
                   feedback_age_s=None, last_goal_wall=None) -> dict:
    return {
        "component": "arm_grasp",
        "busy": bool(busy),
        "uptime_s": round(float(uptime_s), 1),
        "pid": os.getpid(),
        "wall": round(time.time() if wall is None else float(wall), 3),
        "feedback_age_s": (None if feedback_age_s is None
                           else round(float(feedback_age_s), 2)),
        "last_goal_wall": (None if last_goal_wall is None
                           else round(float(last_goal_wall), 3)),
    }
```

在 `grasp_server.py`：

1. import：

```python
from arm_grasp.health import health_payload
```

2. `GraspServer.__init__` 里 `self._busy = False` 附近加：

```python
        self._t_start = time.monotonic()
        self._t_fb = None
        self._last_goal_wall = None
        self.pub_health = self.create_publisher(String, '/health/arm_grasp', 10)
        self.create_timer(1.0, self._publish_health)
```

3. `_on_fb` 里 `self.fb = list(msg.position)` 之后：

```python
        self._t_fb = time.monotonic()
```

4. `_execute` 和 `_execute_motion` 里把 `self._busy, self._abort = True, None` 改成：

```python
        self._busy, self._abort = True, None
        self._last_goal_wall = time.time()
```

5. 新增方法（放在 `_on_estop` 附近）：

```python
    def _publish_health(self):
        feedback_age = None if self._t_fb is None else time.monotonic() - self._t_fb
        self.pub_health.publish(String(data=json.dumps(
            health_payload(self._busy, time.monotonic() - self._t_start,
                           feedback_age_s=feedback_age,
                           last_goal_wall=self._last_goal_wall),
            ensure_ascii=False,
        )))
```

> 定时器只能证明 executor 活着；`feedback_age_s` 才能证明臂驱动还在回数据。

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/raspot_ws/src/arm_grasp && python3 -m pytest test/test_health.py test/test_grasp_server_motion.py -q`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add pi/raspot_ws/src/arm_grasp/arm_grasp/health.py pi/raspot_ws/src/arm_grasp/arm_grasp/grasp_server.py pi/raspot_ws/src/arm_grasp/test/test_health.py
git commit -m "feat(grasp): /health/arm_grasp 心跳与 busy"
```

---

## Task 18: framed 状态文件

**Files:**
- Modify: `pi/k230-vision/pi/k230pi.py`
- Create: `pi/k230-vision/pi/pytest.ini`
- Test: `pi/k230-vision/pi/tests/test_framed_status.py`

**Interfaces:**
- Consumes: 无
- Produces: `FRAMED_STATUS` 常量、`framed_status_payload(connected, peer, n_bytes, n_idr, wall=None) -> dict`、`_write_framed_status(connected, peer, n_bytes, n_idr)`

- [ ] **Step 1: 写失败测试**

`pi/k230-vision/pi/tests/test_framed_status.py`：

```python
import json

import k230pi


def test_payload_shape():
    p = k230pi.framed_status_payload(True, "10.0.0.5:1234", 2048, 7, wall=1000.0)
    assert p["connected"] is True
    assert p["peer"] == "10.0.0.5:1234"
    assert p["bytes"] == 2048
    assert p["idr_frames"] == 7
    assert p["wall"] == 1000.0
    json.dumps(p)


def test_disconnected_payload_has_no_peer():
    p = k230pi.framed_status_payload(False, None, 0, 0, wall=1.0)
    assert p["connected"] is False
    assert p["peer"] is None
```

`pi/k230-vision/pi/pytest.ini`：

```ini
[pytest]
testpaths = tests
python_files = test_*.py
addopts = -q
pythonpath = .
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd pi/k230-vision/pi && python3 -m pytest tests/test_framed_status.py -q`
Expected: FAIL — `AttributeError: module 'k230pi' has no attribute 'framed_status_payload'`

- [ ] **Step 3: 写实现**

在 `k230pi.py` 的 framed 常量区加：

```python
FRAMED_STATUS = os.path.join(OUT_DIR, "framed-status.json")
```

在 `framed()` 之前加：

```python
def framed_status_payload(connected, peer, n_bytes, n_idr, wall=None):
    return {
        "pid": os.getpid(),
        "connected": bool(connected),
        "peer": peer,
        "bytes": int(n_bytes),
        "idr_frames": int(n_idr),
        "wall": round(time.time() if wall is None else wall, 3),
    }


def _write_framed_status(connected, peer, n_bytes, n_idr):
    try:
        ensure_out_dir()
        _atomic_write_fast(
            FRAMED_STATUS,
            json.dumps(framed_status_payload(connected, peer, n_bytes, n_idr),
                       ensure_ascii=False).encode("utf-8"),
        )
    except OSError:
        pass
```

修改 `framed()`：

- `srv.listen(2)` 之后加 `srv.settimeout(1.0)`
- 把 `while True: conn, peer = srv.accept()` 改成：

```python
    while True:
        try:
            conn, peer = srv.accept()
        except socket.timeout:
            _write_framed_status(False, None, 0, 0)
            continue
```

- 连接建立后：

```python
        _write_framed_status(True, "%s:%d" % peer, 0, 0)
```

- 初始化 `t_status = time.time()`（和 `t_conn` 同一行附近）
- 在 recv 循环里，每处理完一段数据后：

```python
            if time.time() - t_status >= 1.0:
                _write_framed_status(True, "%s:%d" % peer, tot, ex.frames)
                t_status = time.time()
```

- 断开时（`conn.close()` 之前）：

```python
        _write_framed_status(False, None, tot, ex.frames)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd pi/k230-vision/pi && python3 -m pytest tests/test_framed_status.py -q`
Expected: PASS（全部通过；实际数量以 pytest 收集为准）

- [ ] **Step 5: 提交**

```bash
git add pi/k230-vision/pi/k230pi.py pi/k230-vision/pi/pytest.ini pi/k230-vision/pi/tests/test_framed_status.py
git commit -m "feat(k230): framed 状态文件供 carhealth 判活"
```

---

## Task 19: systemd 单元、CLI 包装、部署接入

**Files:**
- Create: `pi/systemd/system/carhealthd.service`
- Create: `pi/carhealth/carhealthctl`
- Create: `pi/carhealth/README.md`
- Modify: `pi/deploy.sh`

**Interfaces:**
- Consumes: 前面全部代码
- Produces: 可安装的 `carhealthd.service` 与 `carhealthctl`

- [ ] **Step 1: 创建 systemd 单元**

`pi/systemd/system/carhealthd.service`：

```ini
[Unit]
Description=整车健康守护 carhealthd（一期：只观察，不执行恢复动作）
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=cy
WorkingDirectory=/home/cy/ROS-car/pi/carhealth
RuntimeDirectory=carhealth
RuntimeDirectoryMode=0750
Environment=HOME=/home/cy
Environment=PYTHONUNBUFFERED=1
Environment=CARHEALTH_RECOVERY=off
Environment=CARHEALTH_REPO=/home/cy/ROS-car
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /home/cy/ROS-car/pi/raspot_ws/install/setup.bash && exec /usr/bin/python3 -m carhealth.daemon'
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

> 一期没有 `WatchdogSec`：daemon 还没实现 sd_notify。二期加恢复动作时一起加。

- [ ] **Step 2: 创建 CLI 包装**

`pi/carhealth/carhealthctl`：

```bash
#!/usr/bin/env bash
# carhealthctl：从任意目录调用 carhealth.cli。
# readlink -f 是必须的：安装时 /usr/local/bin/carhealthctl 是软链，
# 直接取 BASH_SOURCE 的 dirname 会得到 /usr/local/bin，PYTHONPATH 就错了。
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec env PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}" \
    /usr/bin/python3 -m carhealth.cli "$@"
```

然后：

```bash
chmod +x pi/carhealth/carhealthctl
```

- [ ] **Step 3: 写 README（安装、命令、验收）**

`pi/carhealth/README.md`：

```markdown
# carhealth — 整车健康守护（一期）

设计见 `docs/plans/2026-11-17-car-health-guard-design.md`。
一期**只观察、只记录，绝不执行任何恢复动作**。

## 安装（Pi 上）

```bash
sudo cp ~/ROS-car/pi/systemd/system/carhealthd.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now carhealthd
sudo ln -sf ~/ROS-car/pi/carhealth/carhealthctl /usr/local/bin/carhealthctl
```

## 命令

```bash
carhealthctl status            # 表格：组件/状态/持续/根因/原因
carhealthctl status --json
carhealthctl doctor            # 出车体检：PASS/WARN/FAIL，退出码 0/1/2
carhealthctl history --since 1h
carhealthctl why tof           # 最近 5 条该组件的完整证据
```

## 一期验收

| 注入的故障 | 期望 |
|---|---|
| `sudo systemctl stop k230-bridge` | `k230_link` 变 `ABSENT`，原因含「k230-bridge 单元不在运行」；**没有任何东西被自动重启** |
| `carhealthctl doctor --no-daemon --fake-throttled 0x1` | `power` FAIL；**底盘/臂/ToF 若本身健康，仍显示 PASS，不被 BLOCKED 掩盖** |
| `carhealthctl doctor --no-daemon --fake-throttled 0x50000` | `power` 只 WARN（历史位，时间未知），**不是 FAIL** |
| `carhealthctl doctor --no-daemon --board-host 203.0.113.1` | `board` FAIL，`k230_link/k230_video` 为 `BLOCKED`，根因 `board` |
| 撤掉 `K230_HOST` 且 `/tmp/k230/resultd-status.json` 无 peer | `board` 为 WARN/UNKNOWN，规则给 `k230.host_unknown`；**不得去 ping 任何写死的旧 IP** |
| `XDG_RUNTIME_DIR=/run/user/$(id -u) systemctl --user stop voice-chatbot` | 6 秒后 `voice` 变 `STALE`，原因含「心跳超过 5s」 |
| 拔掉唤醒模块 USB | `voice` 的 `wakeword_ready=false`，规则给 `voice.wakeword_lost` |
| 拔掉 ToF USB | `tof` 为 `ABSENT`，规则给 `udev`/`usb` 相关结论 |
| `touch pi/raspot_ws/src/arm_grasp/setup.py` 后 `doctor --no-daemon` | 出现 `build.stale` 规则结论 |
| 制造一次 `systemctl restart tof-a010` 并等 60s | `service.crash_loop` 不误报（只涨 1 次不改判据） |
| 臂/底盘动作时看 `status` | `busy` 对应字段为 true；`history` 的 incident 里 `interlock` 记录一致 |
| `ls /run/carhealth/state.json` 且看 SD 卡 IO | 状态文件在 tmpfs；`history` 里的 journal 才在 `~/.local/state/carhealth` |

每条验收后都要确认：`carhealthctl history --since 10m` 里有对应的
`incident_open`，且 `action` 为 `null`；`UNKNOWN` 只产生 `monitor_blind`，
不产生组件 incident。
```

- [ ] **Step 4: 接入 deploy.sh**

把 `pi/deploy.sh` 的重启段改成：

```bash
sudo systemctl restart k230-resultd k230-framed k230-bridge rosbridge ps2-teleop tof-a010 arm-grasp
sudo systemctl restart carhealthd 2>/dev/null || echo "  （carhealthd 尚未安装，跳过）"
```

并把最后的状态行加上 `carhealthd`：

```bash
systemctl --no-pager --lines=0 status k230-resultd k230-framed k230-bridge rosbridge ps2-teleop tof-a010 arm-grasp carhealthd || true
```

- [ ] **Step 5: 语法与命令检查**

Run:

```bash
bash -n pi/carhealth/carhealthctl
bash -n pi/deploy.sh
cd pi/carhealth && python3 -m py_compile carhealth/*.py carhealth/probes/*.py
```

Expected: 全部无输出、退出码 0。

- [ ] **Step 6: 提交**

```bash
git add pi/systemd/system/carhealthd.service pi/carhealth/carhealthctl pi/carhealth/README.md pi/deploy.sh
git commit -m "feat(health): systemd 单元、CLI 包装与部署接入"
```

---

## Task 20: 故障注入脚本与现场验收

**Files:**
- Create: `pi/carhealth/tools/inject_fault.sh`
- Create: `pi/carhealth/docs/2026-11-17-一期验收记录.md`（执行后填写）
- Modify: `pi/carhealth/README.md`（如验收中发现阈值需要调整，先记录、不改代码）

**Interfaces:**
- Consumes: 全部
- Produces: 可重复执行的验收脚本

- [ ] **Step 1: 写注入脚本**

`pi/carhealth/tools/inject_fault.sh`：

```bash
#!/usr/bin/env bash
# carhealth 一期故障注入。默认只读地跑 doctor；需要停服务的 case 会提示 sudo。
set -u

CTL="${CARHEALTHCTL:-carhealthctl}"
case "${1:-}" in
  bridge)
    sudo systemctl stop k230-bridge
    echo "已停 k230-bridge，5s 后看状态（期望 k230_link=ABSENT，且无人重启它）"
    sleep 5
    "$CTL" status
    ;;
  bridge-restore)
    sudo systemctl start k230-bridge
    echo "已恢复 k230-bridge"
    ;;
  power-now)
    echo "伪造当前欠压 0x1（期望 power FAIL；健康的底盘/臂仍 PASS，不被 BLOCKED 掩盖）"
    "$CTL" doctor --no-daemon --fake-throttled 0x1
    ;;
  power-history)
    echo "伪造历史欠压 0x50000（期望 power 只 WARN，不是 FAIL）"
    "$CTL" doctor --no-daemon --fake-throttled 0x50000
    ;;
  board)
    echo "把 K230 指向不可达地址（期望 board FAIL，k230_link/k230_video BLOCKED）"
    "$CTL" doctor --no-daemon --board-host 203.0.113.1
    ;;
  build-stale)
    touch "$(git -C "${CARHEALTH_REPO:-$HOME/ROS-car}" rev-parse --show-toplevel)/pi/raspot_ws/src/arm_grasp/setup.py"
    echo "制造 build 过期（期望出现 build.stale 规则结论）"
    "$CTL" doctor --no-daemon
    ;;
  voice)
    XDG_RUNTIME_DIR="/run/user/$(id -u)" systemctl --user stop voice-chatbot
    echo "已停 voice-chatbot，6s 后看状态（期望 voice=STALE）"
    sleep 6
    "$CTL" status
    ;;
  voice-restore)
    XDG_RUNTIME_DIR="/run/user/$(id -u)" systemctl --user start voice-chatbot
    echo "已恢复 voice-chatbot"
    ;;
  history)
    "$CTL" history --since 30m
    ;;
  *)
    cat <<'EOF'
用法: inject_fault.sh <case>

  bridge          停 k230-bridge，验证 k230_link=ABSENT
  bridge-restore  恢复 k230-bridge
  power-now       伪造当前欠压，验证 BLOCKED 不掩盖健康组件
  power-history   伪造历史欠压，验证 doctor 不会永久 FAIL
  board           伪造板子不可达，验证 board 与 k230_* BLOCKED
  build-stale     制造 build 过期，验证配置漂移规则
  voice           停 voice-chatbot，验证 voice=STALE
  voice-restore   恢复 voice-chatbot
  history         看最近 30 分钟时间线
EOF
    exit 2
    ;;
esac
```

然后：

```bash
chmod +x pi/carhealth/tools/inject_fault.sh
```

- [ ] **Step 2: 在 Pi 上安装并启动守护**

在 Pi 上执行：

```bash
cd ~/ROS-car
git pull --ff-only
sudo cp pi/systemd/system/carhealthd.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now carhealthd
sudo ln -sf ~/ROS-car/pi/carhealth/carhealthctl /usr/local/bin/carhealthctl
carhealthctl status
```

Expected：打出表格，所有组件都有状态；`k230_video` 在 framed 未重编/未重启时可能短暂 STALE，重启 `k230-framed` 后变 `OK`。

- [ ] **Step 3: 逐条执行 README 的验收矩阵**

依次执行：

```bash
bash pi/carhealth/tools/inject_fault.sh bridge
bash pi/carhealth/tools/inject_fault.sh bridge-restore
bash pi/carhealth/tools/inject_fault.sh power-now
bash pi/carhealth/tools/inject_fault.sh power-history
bash pi/carhealth/tools/inject_fault.sh board
bash pi/carhealth/tools/inject_fault.sh build-stale
bash pi/carhealth/tools/inject_fault.sh voice
bash pi/carhealth/tools/inject_fault.sh voice-restore
carhealthctl history --since 30m
```

Expected：每一条都与 README 表格一致；`history` 中所有事件 `action` 均为 `null`；没有单元被意外重启（用 `systemctl show -p NRestarts` 对比注入前后）；`UNKNOWN` 只产生 `monitor_blind`，不产生组件 incident；`power-now` 下健康的底盘/臂仍是 `PASS`。

- [ ] **Step 4: 真实使用一次并记录**

跑一次正常语音交互（含一次抓取），然后：

```bash
carhealthctl history --since 1h
carhealthctl status
```

把观察结果写进 `pi/carhealth/docs/2026-11-17-一期验收记录.md`，至少包含：

- 运行时长与场景
- 出现过的每条 incident（组件/状态/原因/证据是否足够定位）
- 误报（如有）：组件/判据/阈值/建议
- 漏报（如有）：现场发生了什么但日志没有
- 待标定阈值：tof fps、k230 result、voice busy 180s、IDR 5s（GOP 间隔）、
  load1 6.0、WiFi -75dBm/30、USB 10 分钟 3 次、电池 11.5/11.2V
- 规则命中率：哪些规则这次真的用上了，哪些从没命中（可能是死规则）
- DDS / USB / config_drift 三个新探针的权限与误报情况

- [ ] **Step 5: 提交**

```bash
git add pi/carhealth/tools/inject_fault.sh pi/carhealth/docs/2026-11-17-一期验收记录.md
git commit -m "test(health): 一期故障注入脚本与验收记录"
```

---

## 计划自检

**设计覆盖：**

| 设计 §9 一期交付 | 对应任务 |
|---|---|
| carhealth 包骨架 | Task 1 |
| 分类器 / 直接证据根因 / 指纹 | Task 2 |
| 故障日志（boot_id/mono） | Task 3、11 |
| 组件契约（含 battery/host/voice） | Task 4 |
| 主机/供电/load/WiFi 探针 | Task 5 |
| systemd 资源与 udev/by-id | Task 6 |
| 状态文件与网络探针（地址未知→UNKNOWN） | Task 7 |
| ROS 话题（raw/发布者计数/电池/故障位） | Task 8 |
| 服务资源、USB 事件 | Task 8A |
| DDS 旧 IP、配置漂移 | Task 8B |
| 互锁信号（含 odom 线速度） | Task 9 |
| 已知故障库与诊断 | Task 9A |
| 引擎（迟滞、UNKNOWN 语义）与 state.json | Task 10–11 |
| 装配 | Task 12 |
| 常驻入口 | Task 13 |
| status/history/why | Task 14 |
| doctor（复用守护状态 / 临时目录 / 8s warmup） | Task 15 |
| voice 心跳（唤醒/麦克风状态） | Task 16 |
| arm-grasp 心跳（feedback 年龄） | Task 17 |
| framed 状态 | Task 18 |
| systemd/部署（RuntimeDirectory） | Task 19 |
| 单测、变异自检与故障注入 | 每个任务 + Task 20 |
| 验收：自动动作为 0 | Task 20 第 3 步明确检查 `action` 全为 null |

**占位符扫描：** 无 TBD/TODO；Task 10 的 `_record_transitions` 在 Task 11 补全，且 Task 10 的测试不依赖它。

**变异自检（必须实际做）：** 至少对以下三处故意改坏实现，确认对应测试变红，再改回：

1. `classify.evaluate` 里把「字段缺失 → UNKNOWN」改成「直接跳过」→ `test_missing_field_is_unknown_not_ok` 必须红。
2. `classify.evaluate` 里恢复成「依赖 BLOCKED 直接返回」→ `test_healthy_component_is_not_masked_by_blocked_dependency` 必须红。
3. `Engine._commit` 里去掉迟滞（直接用 raw）→ `test_hysteresis_needs_two_bad_and_five_good` 必须红。

**类型一致性：** `Evaluation.evidence` / `ComponentState.evidence` 都是 `dict[probe名, dict]`；`ComponentState.diagnoses` 与 `Diagnosis` 同型；所有组件名、探针名在 Task 4 的 `known_probes` 与 Task 12 的装配中一致；`fingerprint` 在 Task 2 定义、Task 11 使用；`FaultContext` 在 Task 9A 定义、Task 10 使用。

**测试数量核对（执行时以实际收集数为准，不以此处数字为准）：** 计划里写的 passed 数是估算；新增/修改测试后先跑一次 `pytest --collect-only -q` 核对，再更新 Expected。
