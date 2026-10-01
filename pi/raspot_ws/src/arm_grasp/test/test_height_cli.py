import importlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from arm_grasp import collect, grasp
from arm_grasp.height import HeightRefused
from .test_height_capture import observer
from .test_height_vision import make_session


@pytest.fixture
def cli(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools'))
    return importlib.import_module('grasp_once')


def forbidden(*args, **kwargs):
    raise AssertionError('read-only command attempted live robot I/O')


def test_height_replay_dry_run_never_touches_robot(cli, monkeypatch, tmp_path):
    path = make_session(tmp_path)
    monkeypatch.setattr(collect, 'ArmIO', forbidden)
    monkeypatch.setattr(collect, '_svc', forbidden)
    monkeypatch.setattr(collect, '_start_driver', forbidden)
    assert cli.main(['--height-session', str(path), '--dry-run']) == 0


@pytest.mark.parametrize('arguments', [
    ['--auto-height', '--dry-run'],
    ['--auto-height', '--h', '.016'],
    ['--height-session', 'old.json'],
    ['--measure-only'],
    ['--support-z-mm', '-135.5'],
    ['--auto-height', '--phase', 'lift'],
])
def test_invalid_modes_stop_before_robot_io(cli, monkeypatch, arguments):
    monkeypatch.setattr(collect, 'ArmIO', forbidden)
    assert cli.main(arguments) == 2


def test_failed_live_measurement_restores_driver_and_service(cli, monkeypatch, tmp_path):
    wiring = importlib.import_module('auto_height_grasp')
    calls = []
    fields = observer()[1]
    io = SimpleNamespace(fb=fields, fb_n=10, spin=lambda _: None,
                         wait_feedback=lambda: None, close=lambda: calls.append('close'))
    monkeypatch.setattr(collect, 'ArmIO', lambda: io)
    monkeypatch.setattr(wiring, 'stationary_feedback', lambda *_: fields)
    monkeypatch.setattr(collect, 'k230_host_from_result', lambda: 'fake')
    def service(action):
        calls.append(action)
        return SimpleNamespace(returncode=0, stderr='', stdout='')
    monkeypatch.setattr(collect, '_svc', service)
    monkeypatch.setattr(collect, '_svc_active', lambda: 'inactive')
    monkeypatch.setattr(collect, '_start_driver', lambda _: calls.append('driver-start') or 'driver')
    monkeypatch.setattr(collect, '_stop_driver', lambda _: calls.append('driver-stop'))
    def capture(io, host, box, directory, **_):
        directory.mkdir(parents=True)
        path = directory / 'session.json'
        path.write_text('{}')
        return path
    monkeypatch.setattr(wiring, 'capture_scan', capture)
    def refuse(*_):
        raise HeightRefused('insufficient top texture')
    monkeypatch.setattr(wiring, 'measure_session', refuse)
    monkeypatch.setattr(grasp, 'run', forbidden)
    monkeypatch.setattr(wiring, 'restore_pose',
                        lambda _io, start, **_: calls.append(('restore', list(start))) or start)
    assert cli.main(['--auto-height', '--yes', '--box', '.4,.4,.6,.6',
                     '--height-out', str(tmp_path)]) == 1
    # Both hand-overs move the arm: back to state 1 before scanning and after.
    assert calls == ['stop', 'driver-start', ('restore', list(fields)), 'driver-stop', 'start',
                     ('restore', list(fields)), 'close']
    reports = list(tmp_path.glob('*/height-report.json'))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text(encoding='utf-8'))
    assert report['ok'] is False
    assert 'top_z_m' not in report


@pytest.mark.parametrize('grasp_ok', [True, False])
def test_successful_grasp_ends_lifted_with_last_command(cli, monkeypatch, tmp_path, grasp_ok):
    wiring = importlib.import_module('auto_height_grasp')
    grasp_once = importlib.import_module('grasp_once')
    calls, fields = [], observer()[1]
    io = SimpleNamespace(fb=fields, fb_n=10, spin=lambda _: None,
                         wait_feedback=lambda: None, close=lambda: calls.append('close'))
    monkeypatch.setattr(collect, 'ArmIO', lambda: io)
    monkeypatch.setattr(wiring, 'stationary_feedback', lambda *_: fields)
    monkeypatch.setattr(collect, 'k230_host_from_result', lambda: 'fake')
    monkeypatch.setattr(collect, '_svc', lambda action: calls.append(action)
                        or SimpleNamespace(returncode=0, stderr='', stdout=''))
    monkeypatch.setattr(collect, '_svc_active', lambda: 'inactive')
    monkeypatch.setattr(collect, '_start_driver', lambda _: 'driver')
    monkeypatch.setattr(collect, '_stop_driver', lambda _: None)
    def capture(io, host, box, directory, **_):
        directory.mkdir(parents=True)
        return directory / 'session.json'
    monkeypatch.setattr(wiring, 'capture_scan', capture)
    report = {'schema': 'arm_grasp.height/v1', 'ok': True, 'support_z_m': -.12,
              'top_z_m': -.107, 'object_height_m': .013, 'target_point_m': [.1, .05, -.107],
              'compute_seconds': 1.0}
    monkeypatch.setattr(wiring, 'measure_session', lambda _: report)
    monkeypatch.setattr(wiring, 'save_diagnostic', lambda *_: None)
    monkeypatch.setattr(wiring, 'measured_cfg', lambda *_: SimpleNamespace(hz=10.0))
    monkeypatch.setattr(wiring, 'plan_measured_grasp', lambda *_: None)
    monkeypatch.setattr(wiring, 'load_session', lambda _: ({}, []))
    monkeypatch.setattr(wiring, 'reference_box_at_pose', lambda *_: [.4, .4, .6, .6])
    monkeypatch.setattr(wiring.observe, 'k230_cmd', lambda *_: {'ok': True})
    monkeypatch.setattr(grasp_once, 'one_obs', lambda _: object())
    lifted = [578.0, 497.0, 300.0, 400.0, 500.0, 60.0]   # jaws closed, arm raised
    class Link:
        def __init__(self, *a, **k):
            self.log = [(0.0, list(fields), list(fields)), (1.0, lifted, lifted)]
    monkeypatch.setattr(grasp_once, 'GraspLink', Link)
    monkeypatch.setattr(grasp, 'run', lambda *a, **k: {
        'ok': grasp_ok, 'stopped': 'lifted' if grasp_ok else 'lost', 'ticks': 1, 'obs_n': 1,
        'obs_bad': 0, 'O_last': None, 'err_m': 0.0, 'phases': []})
    monkeypatch.setattr(wiring, 'restore_pose',
                        lambda _io, target, **_: calls.append(('restore', list(target))) or target)
    cli.main(['--auto-height', '--yes', '--phase', 'all', '--box', '.4,.4,.6,.6',
              '--height-out', str(tmp_path)])
    restores = [c for c in calls if isinstance(c, tuple)]
    assert restores[0] == ('restore', list(fields))   # scan starts from state 1
    if grasp_ok:
        assert restores[1:] == [('restore', lifted)]
        assert calls.index('start') < calls.index(restores[1])
    else:
        assert restores[1:] == []
