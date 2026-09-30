"""`K230Target` 的客户端测试 —— 跑子进程的替身，不插 K230、不碰串口。

工具层的测试在 `tests/test_follow_tools.py`（失败 → 人话那一层）。
"""

import subprocess

import pytest

from vision.k230_follow import DEFAULT_CMD, K230Target, TargetError


class _R:
    def __init__(self, code=0, out="ok", err=""):
        self.returncode = code
        self.stdout = out
        self.stderr = err


class _Runner:
    """记录每次调用，按序返回预设结果。"""

    def __init__(self, results):
        self.calls = []
        self._results = list(results)

    def __call__(self, argv, capture_output=True, text=True, timeout=None):
        self.calls.append((argv, timeout))
        r = self._results.pop(0)
        if isinstance(r, BaseException):
            raise r
        return r


def _target(results, **cfg):
    runner = _Runner(results)
    t = K230Target(_Cfg({"follow": cfg}) if cfg else None, runner=runner)
    return t, runner


class _Cfg:
    def __init__(self, data):
        self._data = data

    def get(self, key, default=None):
        node = self._data
        for part in key.split("."):
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return default
        return node


# ---------------------------------------------------------------- 正常路径

def test_follow_runs_k230ctl_follow_with_the_name():
    t, r = _target([_R(out="ok\n")])
    assert t.follow("张三") == "ok"
    argv, _ = r.calls[0]
    assert argv == [DEFAULT_CMD, "follow", "张三"]


def test_follow_tolerates_non_string_name():
    t, r = _target([_R()])
    t.follow(123)
    assert r.calls[0][0][-1] == "123"


def test_point_builds_pt_with_size():
    t, r = _target([_R()])
    t.point(0.5, 0.4, size=0.12)
    argv = r.calls[0][0]
    assert argv[1] == "pt" and argv[2] == "0.5" and "--size" in argv


def test_point_without_size_omits_the_flag():
    t, r = _target([_R()])
    t.point(0.5, 0.4)
    assert "--size" not in r.calls[0][0]


# ---------------------------------------------------------------- stop = stop + auto

def test_stop_also_restores_detector_auto_lock():
    """★ "别跟了"= 停 + 切回自动锁定 —— 只发 stop 的话板子会一直停在
    "外部驱动"状态，之后连自动锁定都没了（det_auto 一旦关掉只能重启）。"""
    t, r = _target([_R(out="ok"), _R(out="ok")])
    assert t.stop() == "ok"
    assert [c[0][1] for c in r.calls] == ["stop", "auto"]


def test_stop_still_succeeds_when_auto_fails():
    """停止本身成功了，就不该因为"切不回来"而报成失败 —— 但要带上提醒。"""
    t, r = _target([_R(out="ok"), _R(code=1, err="boom")])
    out = t.stop()
    assert out.startswith("ok")
    assert "自动锁定" in out


# ---------------------------------------------------------------- 失败

def test_nonzero_exit_becomes_target_error_with_stderr():
    t, _ = _target([_R(code=1, err="TARGET FAILED: 连不上 K230")])
    with pytest.raises(TargetError) as e:
        t.follow("张三")
    assert "连不上 K230" in str(e.value)


def test_timeout_becomes_target_error():
    t, _ = _target([subprocess.TimeoutExpired(cmd="k230ctl", timeout=15.0)])
    with pytest.raises(TargetError) as e:
        t.follow("张三")
    assert "没回话" in str(e.value)


def test_missing_command_becomes_target_error():
    t, _ = _target([OSError("No such file")])
    with pytest.raises(TargetError) as e:
        t.follow("张三")
    assert "k230ctl" in str(e.value)


def test_default_timeout_is_longer_than_the_board_wait():
    """板子的 FOLLOW_TIMEOUT_MS 是 8s —— 客户端超时必须比它长，
    否则会把"还在等人脸"误判成"板子没响应"。"""
    t, r = _target([_R()])
    t.follow("张三")
    assert r.calls[0][1] >= 9.0
