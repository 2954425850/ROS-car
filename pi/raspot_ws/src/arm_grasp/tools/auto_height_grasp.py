"""Automatic-height wiring used by grasp_once; service ownership is centralized here."""
import json
from pathlib import Path
import time
import uuid

from arm_grasp import geom, grasp, observe
from arm_grasp.arm_kin import from_fields
from arm_grasp.height import HeightRefused, measurement_target
from arm_grasp.height_capture import capture_scan, plan_scan, restore_pose, stationary_feedback
from arm_grasp.height_vision import load_session, measure_session, save_diagnostic


def measured_cfg(args, report):
    from grasp_once import build_cfg
    point = measurement_target(report)
    cfg = build_cfg(point[2], max_step=args.max_step, patience=args.patience,
                    alpha0=args.alpha0, joint_comp=args.joint_comp, hz=args.hz,
                    bias_r=args.bias_r, max_seconds=args.max_seconds)
    # Hold the object clear of the support after closing (operator: 5-6 cm).
    cfg = cfg._replace(lift_m=.055)
    if args.alpha0 is None:
        # An unknown plane may be much lower than the old cap fixture. Search
        # the full configured angle range instead of requiring a new manual seed.
        cfg = cfg._replace(alpha0=(cfg.alpha_lo + cfg.alpha_hi) / 2,
                           alpha_span=cfg.alpha_hi - cfg.alpha_lo)
    return cfg


def print_measurement(report):
    print('自动测高：支撑 z=%.1f mm，顶面 z=%.1f mm，物体法向高度=%.1f mm'
          % tuple(report[k] * 1000 for k in ('support_z_m', 'top_z_m', 'object_height_m')))
    print('已测目标点：(%.4f, %.4f, %.4f) m；几何检查通过，真机精度未验证。'
          % measurement_target(report))
    print('采集 %s s，计算 %.2f s'
          % (report.get('acquisition_seconds'), report['compute_seconds']))


def write_report(directory, report):
    path = directory / 'height-report.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print('测高报告：%s' % path)


def plan_measured_grasp(point, cfg):
    level, target, msg = grasp.plan_verdict(point, cfg)
    print('S0：%s — %s' % (level, msg))
    if level == 'refuse' or target is None:
        raise HeightRefused('measured target is unreachable; grasp refused')
    pre = grasp.pick_target(point, cfg, cfg.s_pre_m, cfg.alpha0, balance=True)
    if pre.slack < cfg.min_slack:
        raise HeightRefused('measured approach has insufficient joint margin')


def reference_box_at_pose(session, report, joints):
    """Project the reference ROI's corners on the fitted top after returning."""
    box = session['reference_box']
    normal = report['top_plane']['normal']
    offset = report['top_plane']['offset_m']
    reference = session['views'][0]['joints']
    corners = []
    for u, v in ((box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])):
        c, d = geom.pixel_ray(reference, u * geom.AI_W, v * geom.AI_H)
        den = sum(normal[i] * d[i] for i in range(3))
        if abs(den) < .1:
            raise HeightRefused('reference box ray too parallel to top')
        distance = -(sum(normal[i] * c[i] for i in range(3)) + offset) / den
        if distance <= 0:
            raise HeightRefused('reference top behind camera')
        p = [c[i] + distance * d[i] for i in range(3)]
        px, py = geom.project(joints, p)
        corners.append((px / geom.AI_W, py / geom.AI_H))
    projected = [max(0., min(p[0] for p in corners)), max(0., min(p[1] for p in corners)),
                 min(1., max(p[0] for p in corners)), min(1., max(p[1] for p in corners))]
    if projected[0] >= projected[2] or projected[1] >= projected[3]:
        raise HeightRefused('measured target left current image')
    return projected


def run_auto_height(args):
    from grasp_once import GraspLink, parse_box, one_obs
    if args.height_session:
        try:
            report = measure_session(args.height_session)
            print_measurement(report)
            cfg = measured_cfg(args, report)
            plan_measured_grasp(measurement_target(report), cfg)
            print('离线预演完成：没有连接 ROS/相机，没有发指令或更改服务。')
            return 0
        except (ValueError, OSError, RuntimeError) as e:
            print('测高/预演失败：%s；不抓取。' % e)
            return 1
    if not args.box:
        print('❌ --auto-height 需要 --box l,t,r,b 指定目标区域。')
        return 2
    try:
        box = parse_box(args.box)
    except ValueError as e:
        print('❌ %s' % e)
        return 2

    from arm_grasp import collect
    io, driver, stopped, locked, host = None, None, False, False, None
    directory = None
    # The arm must end where the flow left it, but the hand-back to the
    # service moves it (live: shoulder 566 -> 624 between our driver stopping
    # and the service's coming up). So once the service is back, re-command:
    # measure-only -> the operator's start pose (state 1, its readings);
    # successful grasp -> the last grasp COMMAND (lifted, jaws closed - a
    # closed-on-object gripper reads wider than commanded, so its reading
    # would loosen the grip). A failed grasp or Ctrl-C leaves the arm alone.
    start_fields, end_fields = None, None
    try:
        io = collect.ArmIO()
        io.wait_feedback()
        fields = stationary_feedback(io)
        start_fields = list(fields)
        poses = plan_scan(from_fields(fields), fields, box=box)
        print('扫描预检：%d 个观察姿态；每次从起始姿态小范围平移，再返回。' % len(poses))
        for i, j in enumerate(poses):
            print('  %d：光心 (%.4f, %.4f, %.4f) m' % ((i + 1,) + geom.camera_center(j)))
        if args.scan_preview:
            print('只读预检结束：没有发指令或更改服务；小范围路径检查不等于碰撞检测。')
            return 0
        host = args.k230_host or collect.k230_host_from_result()
        if not args.yes:
            print('将动臂观察并%s；结束恢复服务时会归位%s。'
                  % (('仅测高', '，随后回到当前起始姿态') if args.measure_only else ('按指定相位抓取', '')))
            input('确认起始观察范围无碰撞、场景保持静止，回车开始：')
        end_fields = start_fields
        rc = collect._svc('stop')
        if rc.returncode:
            raise RuntimeError('cannot stop teleop service: %s' % (rc.stderr or rc.stdout))
        stopped = True
        if collect._svc_active() != 'inactive':
            raise RuntimeError('teleop service not inactive')
        driver = collect._start_driver(args.t_ms)
        io.spin(2.0)
        io.wait_feedback()
        # The hand-over itself lets the loaded shoulder droop (live: 594 ->
        # 629, the target left the operator's box). Scan from state 1 itself.
        back = restore_pose(io, start_fields, min_wait=.5)
        print('驱动接管后回到状态 1：%s（记录 %s）'
              % ([round(x) for x in back], [round(x) for x in start_fields]))
        directory = Path(args.height_out or '~/arm-height').expanduser() / (
            time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8])
        # Snapshot supports raw imaging without locking a potentially stale ROI.
        session_path = capture_scan(
            io, host, box, directory, reference_command=start_fields,
            known_support_z=None if args.support_z_mm is None else args.support_z_mm / 1000.0)
        try:
            report = measure_session(session_path)
            save_diagnostic(session_path, report, directory / 'height-report.jpg')
        except (ValueError, OSError, RuntimeError) as e:
            write_report(directory, {'schema': 'arm_grasp.height/v1', 'ok': False,
                                     'session': str(session_path), 'reason': str(e),
                                     'diagnostics': getattr(e, 'diagnostics', {})})
            raise
        write_report(directory, report)
        print_measurement(report)
        if args.measure_only:
            return 0
        point = measurement_target(report)
        cfg = measured_cfg(args, report)
        plan_measured_grasp(point, cfg)  # --yes cannot override measurement/plan refusal.
        session, _ = load_session(session_path)
        fields = stationary_feedback(io)
        want = reference_box_at_pose(session, report, from_fields(fields))
        # Release any previous tracker before binding the measured object ROI.
        observe.k230_cmd(host, {'cmd': 'stop'})
        locked = True  # cleanup is necessary even if a request times out after being applied.
        response = observe.k230_cmd(host, {'box': want})
        if not response.get('ok'):
            raise HeightRefused('cannot reacquire measured target')
        if one_obs(want) is None:
            raise HeightRefused('no fresh observation of measured target')
        end_fields = None
        link = GraspLink(io, host, want, hz=cfg.hz)
        result = grasp.run(cfg, link, phase=args.phase, initial_point=point, log=print)
        if result['ok'] and link.log:
            end_fields = list(link.log[-1][1])
        summary = {key: result[key] for key in ('ok', 'stopped', 'ticks', 'obs_n', 'obs_bad',
                                               'O_last', 'err_m', 'phases')}
        (directory / 'grasp-report.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        print('抓取结果：%s' % result['stopped'])
        return 0 if result['ok'] else 1
    except (KeyboardInterrupt, EOFError):
        end_fields = None
        print('已取消；不继续抓取。')
        return 1
    except (ValueError, OSError, RuntimeError, ImportError) as e:
        print('自动测高流程失败：%s；不使用手填/默认高度继续抓取。' % e)
        if directory is not None and directory.exists() and not (directory / 'height-report.json').exists():
            write_report(directory, {'schema': 'arm_grasp.height/v1', 'ok': False, 'reason': str(e),
                                     'diagnostics': getattr(e, 'diagnostics', {})})
        return 1
    finally:
        cleanup_errors = []
        if locked:
            try:
                observe.k230_cmd(host, {'cmd': 'stop'})
            except Exception as e:
                cleanup_errors.append('释放跟踪器：%s' % e)
        if driver is not None:
            try:
                collect._stop_driver(driver)
            except Exception as e:
                cleanup_errors.append('停止独立驱动：%s' % e)
        if stopped:
            try:
                rc = collect._svc('start')
                if rc.returncode:
                    cleanup_errors.append('恢复服务失败：%s' % (rc.stderr or rc.stdout))
                print('服务恢复状态：%s；恢复服务会归位。' % collect._svc_active())
            except Exception as e:
                cleanup_errors.append('恢复服务：%s' % e)
            if end_fields is not None and io is not None and not cleanup_errors:
                try:
                    print('服务接管后回到结束姿态……')
                    back = restore_pose(io, end_fields)
                    print('已回到结束姿态：%s（目标 %s）'
                          % ([round(x) for x in back], [round(x) for x in end_fields]))
                except Exception as e:
                    cleanup_errors.append('回到结束姿态：%s' % e)
        if io is not None:
            try:
                io.close()
            except Exception as e:
                cleanup_errors.append('关闭 ROS：%s' % e)
        if cleanup_errors:
            print('清理未完成：%s' % '; '.join(cleanup_errors))
            return 1
