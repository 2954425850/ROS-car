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
