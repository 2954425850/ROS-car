"""Offline height-session replay. Never imports ROS or sends arm commands."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main(argv=None):
    ap = argparse.ArgumentParser(description='离线多视角测高（不动机械臂）')
    ap.add_argument('session', help='采集的 session.json')
    ap.add_argument('--out', help='报告 JSON 路径；默认同目录 height-report.json')
    args = ap.parse_args(argv)
    output = Path(args.out) if args.out else Path(args.session).parent / 'height-report.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        from arm_grasp.height_vision import measure_session, save_diagnostic
        report = measure_session(args.session)
        save_diagnostic(args.session, report, output.with_suffix('.jpg'))
    except (ValueError, OSError, ImportError, RuntimeError) as e:
        report = {'schema': 'arm_grasp.height/v1', 'ok': False,
                  'reason': str(e), 'session': str(Path(args.session).resolve()),
                  'diagnostics': getattr(e, 'diagnostics', {})}
    with output.open('w', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
    if not report['ok']:
        print('测高失败：%s；不提供默认高度。报告：%s' % (report['reason'], output))
        return 1
    print('支撑面 z=%.1f mm，顶面 z=%.1f mm，物体法向高度=%.1f mm'
          % tuple(report[k] * 1000 for k in ('support_z_m', 'top_z_m', 'object_height_m')))
    print('计算 %.2fs；几何检查通过，真机精度尚未验证。报告：%s'
          % (report['compute_seconds'], output))
    return 0


if __name__ == '__main__':
    sys.exit(main())
