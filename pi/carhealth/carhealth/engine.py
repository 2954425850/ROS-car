"""轮询探针 -> 判定 -> 状态迁移 -> 写 state.json。

一期不包含任何恢复动作；`journal` 里 action 恒为 null。
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

from .busy import BusyState, compute
from .classify import attach_graph_root_cause, evaluate, fingerprint
from .diagnose import diagnose
from .faults import FaultContext
from .journal import Journal
from .recovery import Recovery, RecoveryConfig
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
    # 默认关闭，测试/一期行为不变；daemon 按 CARHEALTH_RECOVERY=on 打开。
    recovery: RecoveryConfig = field(default_factory=RecoveryConfig)


class Engine:
    def __init__(self, specs, probes, config: EngineConfig,
                 clock=time.monotonic, wall=time.time,
                 recovery: Recovery | None = None):
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
        self.recovery = recovery or Recovery(config.recovery, clock=clock)
        self._last_ips: list[str] | None = None
        self._network_changed_at: float | None = None
        self._last_deferred_wall = 0.0
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
        self._detect_network_change(now)
        verify = self.recovery.observe(states=states, diag=diag, now=now)
        if verify is not None:
            self._journal.record({"event": "recovery_verify", **verify})

        self._record_transitions(states, now)
        self._prev = states
        self._committed = states
        self._write_state(states)
        self._maybe_recover(states, diag, now)
        return states

    def _detect_network_change(self, now: float) -> None:
        result = self._results.get("dds")
        if result is None or result.ok is not True:
            return
        ips = list(result.evidence.get("current_ips") or ())
        if not ips:
            return
        if self._last_ips is not None and set(ips) != set(self._last_ips):
            self._network_changed_at = now
            self._journal.record({
                "event": "network_changed",
                "from": self._last_ips,
                "to": ips,
                "action": None,
            })
        self._last_ips = ips

    def _maybe_recover(self, states, diag, now: float) -> None:
        network_changed = (self._network_changed_at is not None
                           and now - self._network_changed_at < 180.0)
        request = self.recovery.decide(
            states=states, diag=diag, busy=self.last_busy,
            network_changed=network_changed, now=now,
        )
        if request is None:
            return
        if request.deferred:
            wall = self._wall()
            if wall - self._last_deferred_wall < 60.0:
                return
            self._last_deferred_wall = wall
            self._journal.record({
                "event": "recovery_deferred",
                "action": request.action,
                "reason": request.reason,
                "detail": request.detail,
                "targets": list(request.targets),
                "interlock": asdict(self.last_busy),
            })
            return

        self._journal.record({
            "event": "recovery_attempt",
            "action": request.action,
            "reason": request.reason,
            "targets": list(request.targets),
            "interlock": asdict(self.last_busy),
        })
        result = self.recovery.execute(request, now)
        self._journal.record({"event": "recovery_result", **result})

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

    def _write_state(self, states) -> None:
        now = self._clock()
        payload = {
            "wall": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self._wall())),
            "wall_ts": round(self._wall(), 3),
            "pid": os.getpid(),
            "interval_s": self.config.interval_s,
            "busy": asdict(self.last_busy),
            "recovery": self.recovery.snapshot(),
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
