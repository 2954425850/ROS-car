#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最简单的黑线循迹：USB 摄像头 → /cmd_vel_nav。

控制只用两条行：
    e_near  近行（转向 180° 后是车头附近）的横向偏差
    e_far   远行（转向 180° 后是更远处）的横向偏差
    wz = -(kp*e_near + kf*e_far)        # 远行让弯道提前打方向
速度固定（--speed，默认 0.15 m/s），弯道自动略减速。

线宽不均匀不影响：每行只找"离上一行中心最近"的黑块，宽度范围只是粗滤。
赛道不一样也不用改算法；到现场只需要标一次阈值（默认启动时自动 Otsu）。

安全：
  · 默认只识别、不发速度；加 --drive 才动。
  · 发 cmd_vel_nav（cmd_mux 最低优先级）：PS2 一动立刻接管；
    检测到别的控制源接管 / 丢线超过 --hold-s / Ctrl-C → 停止发布，
    cmd_mux 0.5 s 无消息自动归零（驱动端还有 0.5 s 超时兜底）。
  · 真跑期间临时打开 /l150pro_driver 的偏航闭环，退出时恢复原值——
    不改语音/PS2/App 看到的设置。

本车默认参数（2026-11-24 实车调过）：相机倒装 → 默认 --rotate；
近行 280、远行 140；速度 0.15；丢线保持 0.5 s。

用法（Pi 上）：
    python3 line_follow.py --overlay              # 只识别，存 /tmp/linecam/overlay.jpg
    source /opt/ros/jazzy/setup.bash
    python3 line_follow.py --drive                # 真跑

现场标定：
    1) 车摆到直道上、线在车正下方，跑 --overlay，看近行 u 是不是接近 --center-u；
       不接近就按实际 u 改 --center-u（相机装偏了多少就补多少）。
    2) 阈值默认启动时 Otsu 标一次，会打印黑像素占比：正常 1~20%；
       如果 40%+ 说明视野里没线，标定无效。
    3) 对焦用 --focus <值> 锁死（空白画面自动对焦会来回找）。
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

DEFAULT_CAM = '/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0'
TOPIC = 'cmd_vel_nav'


# ---------------------------------------------------------------- 参数

def parse_args():
    p = argparse.ArgumentParser(description='黑线循迹（近行 + 远行 P 控制）')
    p.add_argument('--camera', default=DEFAULT_CAM)
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--contrast', type=int, default=37,
                   help='摄像头对比度（这颗头默认会被设成 2，很灰）')
    p.add_argument('--focus', type=int, default=-1,
                   help='手动对焦 0~1023；-1 = 不动（保持自动对焦）')
    p.add_argument('--rotate', dest='rotate', action='store_true', default=True,
                   help='图像转 180°（本车相机就是倒装的，默认开）')
    p.add_argument('--no-rotate', dest='rotate', action='store_false',
                   help='相机装正了才用')
    p.add_argument('--thresh', type=int, default=0,
                   help='黑线阈值；0 = 启动时用 Otsu 标一次后固定')
    p.add_argument('--calib-frames', type=int, default=25,
                   help='Otsu 标定取多少帧的中位数')
    p.add_argument('--near-row', type=int, default=280, help='近行 y（转向 180° 后是车头附近）')
    p.add_argument('--far-row', type=int, default=140, help='远行 y（转向 180° 后是更远处）')
    p.add_argument('--min-width', type=int, default=8, help='黑块最小像素宽（粗滤）')
    p.add_argument('--max-width', type=int, default=120, help='黑块最大像素宽（粗滤）')
    p.add_argument('--center-u', type=float, default=None,
                   help='直行时线应该在的列；默认画面正中 W/2')
    p.add_argument('--speed', type=float, default=0.15, help='直线速度 m/s')
    p.add_argument('--kp', type=float, default=1.0, help='近行增益 rad/s 每单位误差')
    p.add_argument('--kf', type=float, default=0.6, help='远行增益（弯道预判）')
    p.add_argument('--max-wz', type=float, default=1.0, help='角速度上限 rad/s')
    p.add_argument('--invert-wz', action='store_true',
                   help='转向反号（相机左右与车相反、或底盘 +wz 实际右转时用）')
    p.add_argument('--hold-s', type=float, default=0.5,
                   help='丢线后按上一拍继续走的时间，之后停发')
    p.add_argument('--drive', action='store_true', help='真的发 cmd_vel_nav')
    p.add_argument('--overlay', action='store_true',
                   help='存标记图 /tmp/linecam/overlay.jpg')
    p.add_argument('--overlay-every', type=int, default=3,
                   help='每 N 帧写一次标记图')
    return p.parse_args()


# ---------------------------------------------------------------- 相机

def open_cam(a):
    cap = cv2.VideoCapture(a.camera, cv2.CAP_V4L2)
    if not cap.isOpened() and a.camera != '/dev/video0':
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit('打不开摄像头：%s' % a.camera)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    if a.contrast >= 0:
        cap.set(cv2.CAP_PROP_CONTRAST, a.contrast)
    if a.focus >= 0:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, a.focus)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def grab(cap, a):
    ok, f = cap.read()
    if not ok:
        return None
    if a.rotate:
        f = cv2.rotate(f, cv2.ROTATE_180)
    return f


def calibrate(cap, a):
    """用若干帧 Otsu 的中位数定一个固定阈值。"""
    y0 = max(0, a.far_row - 60)
    vals, fracs = [], []
    for _ in range(max(5, a.calib_frames)):
        f = grab(cap, a)
        if f is None:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        roi = g[y0:]
        t, _ = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        vals.append(int(t))
        fracs.append(float((roi < t).mean()))
    if not vals:
        raise SystemExit('标定失败：一帧都读不到')
    th = int(np.median(vals))
    frac = float(np.median(fracs))
    print('阈值标定：th=%d（%.0f~%.0f）黑像素占比 %.1f%%'
          % (th, min(vals), max(vals), frac * 100))
    if frac > 0.4:
        print('⚠️  视野里黑东西太多（占比 >40%），这次的阈值不可信；'
              '把车摆到线上、线在视野内再跑。')
    return th


# ---------------------------------------------------------------- 找线

def runs_in_row(row, thresh, min_w, max_w):
    """一行里所有黑块的 (起点, 终点)，终点不含；按宽度粗滤。"""
    b = (row < thresh).astype(np.int8)
    d = np.diff(np.concatenate(([0], b, [0])))
    st = np.flatnonzero(d == 1)
    en = np.flatnonzero(d == -1)
    return [(int(s), int(e)) for s, e in zip(st, en) if min_w <= e - s <= max_w]


def line_points(gray, thresh, rows, min_w, max_w, last_u=None):
    """从近到远逐行找线，每行选离上一行中心最近的黑块。

    第一行优先跟上一帧的线心 `last_u`（没有才取最宽的）——旁边放个黑色
    物体时，最宽的不一定是线（实测茄子会把第一行抢走，然后整帧跟错）。
    """
    pts, prev = [], None
    for y in rows:
        rs = runs_in_row(gray[y], thresh, min_w, max_w)
        if not rs:
            continue
        if prev is None and last_u is not None:
            s, e = min(rs, key=lambda t: abs((t[0] + t[1]) / 2.0 - last_u))
        elif prev is None:
            s, e = max(rs, key=lambda t: t[1] - t[0])
        else:
            s, e = min(rs, key=lambda t: abs((t[0] + t[1]) / 2.0 - prev))
        prev = (s + e) / 2.0
        pts.append((int(y), float(prev)))
    return pts


def line_errors(pts, near_y, far_y, center_u, half_w):
    """把点拟合成 u(y)，在近行/远行取值，返回归一化误差（右为正）。"""
    if len(pts) < 2:
        # 一个点（线已经甩到画面边缘或只剩一角）不拟合——单点的"远点"是
        # 外推出来的，实测会让车按一个假误差满舵打，不如按丢线处理。
        return None
    else:
        ys = np.array([p[0] for p in pts], dtype=float)
        us = np.array([p[1] for p in pts], dtype=float)
        k, c = np.polyfit(ys, us, 1)
        u_near = k * near_y + c
        u_far = k * far_y + c
        fit = (float(k), float(c))
        # 弯道急的时候线会先从远行/近行出画，拟合会朝着看不见的地方外推，
        # 实测会出一个假的大误差（eF 到 1.5）让车满舵。没覆盖到的端点就用
        # 最远/最近那个真实测到的点，别外推。
        bot_y, bot_u = pts[0]
        top_y, top_u = pts[-1]
        if bot_y < near_y - 25:
            u_near = bot_u
        if top_y > far_y + 25:
            u_far = top_u
    return ((u_near - center_u) / half_w, (u_far - center_u) / half_w,
            u_near, u_far, fit)


# ---------------------------------------------------------------- 画图

def draw(frame, a, th, frac, pts, fit, err, wz, vx):
    vis = frame.copy()
    cv2.line(vis, (0, max(0, a.far_row - 60)), (vis.shape[1] - 1, max(0, a.far_row - 60)),
             (0, 255, 255), 1)
    for y in rows_for(a):
        cv2.line(vis, (0, y), (vis.shape[1] - 1, y), (80, 80, 80), 1, cv2.LINE_AA)
    for y, u in pts:
        cv2.circle(vis, (int(u), int(y)), 3, (0, 0, 255), -1, cv2.LINE_AA)
    if fit is not None:
        k, c = fit
        for y in (a.near_row, a.far_row):
            u = int(k * y + c)
            cv2.drawMarker(vis, (u, y), (255, 0, 0), cv2.MARKER_CROSS, 12, 2)
    txt = 'th=%d frac=%.1f%%' % (th, frac * 100)
    if err is not None:
        txt += '  eN=%+.3f eF=%+.3f  wz=%+.2f vx=%.2f' % (err[0], err[1], wz, vx)
    else:
        txt += '  LOST'
    cv2.putText(vis, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                cv2.LINE_AA)
    return vis


def rows_for(a):
    return [int(round(y)) for y in np.linspace(a.near_row, a.far_row, 9)]


# ---------------------------------------------------------------- 偏航闭环参数
#
# 偏航闭环是底盘驱动的**全局参数**。本脚本只在真跑期间临时打开，退出时恢复
# 原值——这样语音/PS2/App 那边看到的状态不会被我们改掉。

def yaw_loop_get(node):
    """读 /l150pro_driver 的 yaw_loop_enable；读不到返回 None。"""
    import rclpy
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/l150pro_driver/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        req = GetParameters.Request()
        req.names = ['yaw_loop_enable']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        res = fut.result() if fut.done() else None
        if not res or not res.values:
            return None
        v = res.values[0]
        return bool(v.bool_value) if v.type == ParameterType.PARAMETER_BOOL else None
    finally:
        node.destroy_client(cli)


def yaw_loop_set(node, value):
    """设置 /l150pro_driver 的 yaw_loop_enable；成功返回 True。"""
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    cli = node.create_client(SetParameters, '/l150pro_driver/set_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=2.0):
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='yaw_loop_enable',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=bool(value)))]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        res = fut.result() if fut.done() else None
        return bool(res and res.results and all(r.successful for r in res.results))
    except Exception:
        return False
    finally:
        node.destroy_client(cli)


# ---------------------------------------------------------------- 主循环

def main():
    a = parse_args()
    if a.center_u is None:
        a.center_u = a.width / 2.0
    half_w = a.width / 2.0
    rows = rows_for(a)

    cap = open_cam(a)
    # 丢掉起手几帧（有的头前几帧是灰的）
    for _ in range(10):
        grab(cap, a)
    th = a.thresh if a.thresh > 0 else calibrate(cap, a)

    node = pub = Twist = None
    other_source = {'hit': False, 'name': ''}
    yaw_meas = {'v': None}
    yaw_prev = None
    if a.drive:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.node import Node
        from geometry_msgs.msg import Twist as _Twist
        from std_msgs.msg import Float32MultiArray, String
        Twist = _Twist
        # 自己管 Ctrl-C：默认的 SIGINT 处理会先把 context 关掉，
        # 导致退出时既发不出零速、也恢复不了偏航闭环参数。
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node('line_follow')
        pub = node.create_publisher(Twist, TOPIC, 10)

        def on_active(msg):
            if msg.data and msg.data != 'nav':
                other_source['hit'] = True
                other_source['name'] = msg.data

        def on_yaw(msg):
            if len(msg.data) >= 2 and msg.data[1] == msg.data[1]:   # 跳过 NaN
                yaw_meas['v'] = float(msg.data[1])

        node.create_subscription(String, 'cmd_mux/active', on_active, 10)
        node.create_subscription(Float32MultiArray, 'yaw_loop', on_yaw, 10)

        yaw_prev = yaw_loop_get(node)
        if yaw_prev is None:
            print('⚠️  读不到 /l150pro_driver 的 yaw_loop_enable，按原样跑（可能转不够）')
        elif yaw_loop_set(node, True):
            print('偏航闭环：本次临时打开（原为%s，退出时恢复）'
                  % ('开' if yaw_prev else '关'))
        else:
            print('⚠️  偏航闭环打开失败（驱动没起？），按原样跑')
        print('真跑：发 %s；PS2/语音/App 一旦接管就退出' % TOPIC)
    else:
        print('只识别、不发速度（加 --drive 才动）')

    print('近行 y=%d，远行 y=%d，center_u=%.0f，速度 %.2f m/s'
          % (a.near_row, a.far_row, a.center_u, a.speed))

    os.makedirs('/tmp/linecam', exist_ok=True)
    last_seen = 0.0
    last_wz = 0.0
    last_u = None
    last_print = 0.0
    n = 0
    t_start = time.monotonic()
    try:
        while True:
            f = grab(cap, a)
            if f is None:
                print('读帧失败，重开相机…')
                cap.release()
                time.sleep(1.0)
                cap = open_cam(a)
                continue
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0)
                if other_source['hit']:
                    print('检测到 %r 接管，退出（车子交给它）' % other_source['name'])
                    break

            gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            roi = gray[max(0, a.far_row - 60):]
            frac = float((roi < th).mean())

            pts = []
            err = None
            if frac <= 0.4:
                pts = line_points(gray, th, rows, a.min_width, a.max_width, last_u)
                err = line_errors(pts, a.near_row, a.far_row, a.center_u, half_w)
                if err is not None:
                    last_u = err[2]

            now = time.monotonic()
            wz = 0.0
            vx = 0.0
            if err is not None:
                wz = -(a.kp * err[0] + a.kf * err[1])
                if a.invert_wz:
                    wz = -wz
                wz = max(-a.max_wz, min(a.max_wz, wz))
                vx = a.speed * (1.0 - 0.5 * min(1.0, abs(wz) / a.max_wz))
                last_seen, last_wz = now, wz
                if pub is not None:
                    m = Twist()
                    m.linear.x = float(vx)
                    m.angular.z = float(wz)
                    pub.publish(m)
            elif now - last_seen <= a.hold_s:
                # 短暂丢线：按上一拍的转向、半速冲过去
                vx = a.speed * 0.5
                wz = last_wz
                if pub is not None:
                    m = Twist()
                    m.linear.x = float(vx)
                    m.angular.z = float(wz)
                    pub.publish(m)
            # 超过 hold_s：不发布，mux 0.5 s 后归零

            if now - last_print >= 0.5:
                last_print = now
                meas = ('%+.2f' % yaw_meas['v']) if yaw_meas['v'] is not None else 'n/a'
                if err is not None:
                    print('[%5.1fs] th=%d frac=%.1f%% pts=%d  eN=%+.3f eF=%+.3f '
                          'wz=%+.2f meas=%s vx=%.2f'
                          % (now - t_start, th, frac * 100, len(pts),
                             err[0], err[1], wz, meas, vx))
                else:
                    print('[%5.1fs] th=%d frac=%.1f%%  LOST' % (now - t_start, th, frac * 100))

            if a.overlay and n % max(1, a.overlay_every) == 0:
                vis = draw(f, a, th, frac, pts, err[4] if err else None, err, wz, vx)
                cv2.imwrite('/tmp/linecam/overlay.jpg', vis)
            n += 1

    except KeyboardInterrupt:
        pass
    finally:
        if pub is not None:
            # 先发零速；rclpy 的 SIGINT 默认处理已用 SignalHandlerOptions.NO 关掉，
            # 这里 context 仍然有效。
            try:
                m = Twist()
                for _ in range(3):
                    pub.publish(m)
                    time.sleep(0.02)
            except Exception as e:
                print('退出时发零速失败（mux 会自己归零）：%s' % e)
            if yaw_prev is not None:
                try:
                    yaw_loop_set(node, yaw_prev)
                    print('偏航闭环：已恢复为%s' % ('开' if yaw_prev else '关'))
                except Exception as e:
                    print('⚠️  偏航闭环恢复失败：%s' % e)
            try:
                node.destroy_node()
                rclpy.try_shutdown()
            except Exception:
                pass
        cap.release()
        print('退出')


if __name__ == '__main__':
    main()
