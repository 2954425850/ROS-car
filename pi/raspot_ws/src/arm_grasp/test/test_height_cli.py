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
    def capture(io, host, box, directory):
        directory.mkdir(parents=True)
        path = directory / 'session.json'
        path.write_text('{}')
        return path
    monkeypatch.setattr(wiring, 'capture_scan', capture)
    def refuse(*_):
        raise HeightRefused('insufficient top texture')
    monkeypatch.setattr(wiring, 'measure_session', refuse)
    monkeypatch.setattr(grasp, 'run', forbidden)
    assert cli.main(['--auto-height', '--yes', '--box', '.4,.4,.6,.6',
                     '--height-out', str(tmp_path)]) == 1
    assert calls == ['stop', 'driver-start', 'driver-stop', 'start', 'close']
    reports = list(tmp_path.glob('*/height-report.json'))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text(encoding='utf-8'))
    assert report['ok'] is False
    assert 'top_z_m' not in report
