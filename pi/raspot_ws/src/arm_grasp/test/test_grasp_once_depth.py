# test/test_grasp_once_depth.py
# Task 6：走 GraspLink.obs() 那条真实接线，证明 depth_mm 一路透传到 Obs。
import importlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from arm_grasp import observe


@pytest.fixture
def cli(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools'))
    return importlib.import_module('grasp_once')


def _result(depth):
    obj = {'cls': 'chair', 'score': .9, 'box': [.4, .4, .6, .6], 'src': 'track'}
    if depth is not None:
        obj['depth_mm'] = depth
    return {'ts': 1, 'frame': 11, 'w': 1280, 'h': 720, 'objs': [obj]}


def test_grasplink_carries_depth(cli, monkeypatch):
    monkeypatch.setattr(observe, 'fresh_result_aged', lambda: (_result(420), 0.0))
    link = cli.GraspLink(SimpleNamespace(), '127.0.0.1', [.4, .4, .6, .6])
    o = link.obs()
    assert o is not None and o.depth_mm == 420


def test_grasplink_tolerates_missing_depth(cli, monkeypatch):
    monkeypatch.setattr(observe, 'fresh_result_aged', lambda: (_result(None), 0.0))
    link = cli.GraspLink(SimpleNamespace(), '127.0.0.1', [.4, .4, .6, .6])
    o = link.obs()
    assert o is not None and o.depth_mm is None
