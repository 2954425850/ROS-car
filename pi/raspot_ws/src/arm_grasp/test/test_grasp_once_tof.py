# -*- coding: utf-8 -*-
"""Task: Pi 侧 ToF 深度接进 `GraspLink` 那条真实接线。

★ 这些测试**故意不 import `tof_a010`** —— 用的是假客户端。
  `arm_grasp` 的测试不该因为 ToF 包没装/没 build 就跑不了。
"""
import importlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from arm_grasp import observe


@pytest.fixture
def cli(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools'))
    mod = importlib.import_module('grasp_once')
    mod._LAST_TOF_WHY.clear()
    return mod


def _result(depth):
    obj = {'cls': 'chair', 'score': .9, 'box': [.4, .4, .6, .6], 'src': 'track'}
    if depth is not None:
        obj['depth_mm'] = depth
    return {'ts': 1, 'frame': 11, 'w': 1280, 'h': 720, 'objs': [obj]}


class FakeTof(object):
    """最小替身。`value=None` = 拿不到；`boom=True` = 抛异常。"""

    def __init__(self, value=None, why='框内没有有效回波', boom=False):
        self.value, self.why, self.boom = value, why, boom
        self.calls = []

    def depth_for_box(self, box_norm):
        self.calls.append(list(box_norm))
        if self.boom:
            raise RuntimeError('串口炸了')
        return self.value, {'src': 'tof', 'why': self.why}


# --------------------------------------------------------------------------
# 裁决顺序：ToF → K230 → None
# --------------------------------------------------------------------------

def test_tof_wins_when_it_has_a_value(cli):
    d, src = cli._depth_for_box([.4, .4, .6, .6], {'depth_mm': 999}, FakeTof(420))
    assert (d, src) == (420, 'tof')


def test_falls_back_to_k230_when_tof_has_nothing(cli):
    d, src = cli._depth_for_box([.4, .4, .6, .6], {'depth_mm': 999}, FakeTof(None))
    assert (d, src) == (999, 'k230')


def test_none_when_neither_source_has_it(cli):
    """★ 契约：拿不到就是 None，**绝不回落成常数**（回落 = 闸形同虚设）。"""
    d, src = cli._depth_for_box([.4, .4, .6, .6], {'depth_mm': None}, FakeTof(None))
    assert (d, src) == (None, None)


def test_no_tof_client_at_all_still_works(cli):
    d, src = cli._depth_for_box([.4, .4, .6, .6], {'depth_mm': 420}, None)
    assert (d, src) == (420, 'k230')


def test_a_broken_tof_degrades_instead_of_raising(cli):
    """ToF 抛异常**不许**把抓取主循环掀掉，只降级。"""
    d, src = cli._depth_for_box([.4, .4, .6, .6], {'depth_mm': 420}, FakeTof(boom=True))
    assert (d, src) == (420, 'k230')
    assert any('ToF 异常' in str(w) for w in cli._LAST_TOF_WHY)


def test_broken_tof_with_no_fallback_is_none_not_a_crash(cli):
    d, src = cli._depth_for_box([.4, .4, .6, .6], {}, FakeTof(boom=True))
    assert (d, src) == (None, None)


# --------------------------------------------------------------------------
# 走真实的 GraspLink.obs()
# --------------------------------------------------------------------------

def test_grasplink_uses_tof_depth(cli, monkeypatch):
    monkeypatch.setattr(observe, 'fresh_result_aged', lambda: (_result(999), 0.0))
    link = cli.GraspLink(SimpleNamespace(), '127.0.0.1', [.4, .4, .6, .6],
                         tof=FakeTof(420))
    o = link.obs()
    assert o is not None
    assert o.depth_mm == 420
    assert link.last_depth_src == 'tof'
    assert (link.n_depth, link.n_depth_tof) == (1, 1)


def test_grasplink_records_when_only_k230_has_depth(cli, monkeypatch):
    monkeypatch.setattr(observe, 'fresh_result_aged', lambda: (_result(999), 0.0))
    link = cli.GraspLink(SimpleNamespace(), '127.0.0.1', [.4, .4, .6, .6],
                         tof=FakeTof(None))
    o = link.obs()
    assert o.depth_mm == 999
    assert link.last_depth_src == 'k230'
    assert (link.n_depth, link.n_depth_tof) == (1, 0)


def test_grasplink_without_tof_behaves_exactly_as_before(cli, monkeypatch):
    """★ `--no-tof` 必须和加 ToF 之前**逐字一样**（这是 A/B 对照的前提）。"""
    monkeypatch.setattr(observe, 'fresh_result_aged', lambda: (_result(999), 0.0))
    link = cli.GraspLink(SimpleNamespace(), '127.0.0.1', [.4, .4, .6, .6])
    o = link.obs()
    assert o.depth_mm == 999
    assert link.last_depth_src == 'k230'


def test_one_obs_returns_the_source_alongside_the_observation(cli, monkeypatch):
    monkeypatch.setattr(observe, 'fresh_result', lambda: _result(999))
    got = cli.one_obs([.4, .4, .6, .6], timeout=0.2, tof=FakeTof(420))
    assert got is not None
    obs, src = got
    assert obs.depth_mm == 420 and src == 'tof'


# --------------------------------------------------------------------------
# 打印辅助
# --------------------------------------------------------------------------

def test_fmt_age(cli):
    assert cli._fmt_age(None) == '无'
    assert cli._fmt_age(0.25) == '250ms'
