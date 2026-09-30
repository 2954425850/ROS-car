# -*- coding: utf-8 -*-
"""上机采集：停服务 -> 走姿态 -> 读 /arm/feedback + K230 像素 -> 存 JSON -> 恢复服务。

这是整个包里**唯一碰硬件**的文件。前面三个模块（arm_kin / cam_model /
calib_solve）纯 Python，能离线跑测试；出问题时边界就清楚。

    ros2 run arm_grasp collect --point 16,0,-12 --poses 10 --holdout 2

## 安全约束（照计划 §全局约束）
- **程序控制臂之前必须先停 `ps2-teleop.service`**（`Restart=always`，
  只能 `sudo systemctl stop`，**不许 kill**）。做完**无论成败**都在 `finally`
  里 `start` 回来。
- 停完之后**要核实真的停了**（`systemctl is-active`）才动臂 —— 没停就发
  `/arm/command`，会和 `ps2_teleop_node` 的 25 Hz 摇杆流互相踩
  （同 `l150pro-cmdvel-multi-publisher` 那一类问题）。
- 开跑前把**要发的全部关节值**打印出来让人确认。
- 只订阅 `/arm/*`、只发布 `/arm/command`。不改任何现有文件。
"""
import argparse
import json
import math
import os
import random
import signal
import socket
import subprocess
import sys
import time

from .arm_kin import (FIELD_HI, FIELD_LO, Unreachable, fk, ikine, to_fields)
from .calib_solve import Sample, save_samples

SERVICE = 'ps2-teleop.service'
# ⚠️ **不能只停服务就完事**：`l150pro_driver_node`（发 `/arm/feedback`、管串口的那个）
# 和 `ps2_teleop_node` **装在同一个 service 里**（见 raspot_teleop 的
# ps2_bringup.launch.py）。2026-09-28 第一次上机就是这么栽的：
# 停完 service 后收不到 `/arm/feedback`，直接失败。
# 正确做法（和底座标定时同一条路）：**停服务 -> 单独起驱动 -> 采完杀掉 -> 恢复服务**。
# `require_online:=False` 是必须的：本固件从不置 ONLINE 位（已知缺陷）。
def driver_cmd(t_ms=None):
    """单独起驱动的命令行。

    `arm_t_ms` 是**现成的参数**（驱动 `declare_parameter('arm_t_ms', 40)`）：
    每条 `/arm/command` 都发一帧 0xAC，t_ms 就是这一帧的移动时长。
    默认 40ms —— **实测这速度"太冲"（用户 2026-09-28 反馈）**，
    调到 600ms 左右就慢了。**不用改驱动一行代码。**
    """
    cmd = ['ros2', 'run', 'raspot_teleop', 'l150pro_driver_node', '--ros-args',
           '-p', 'serial_path:=/dev/l150pro', '-p', 'require_online:=False']
    if t_ms is not None:
        cmd += ['-p', 'arm_t_ms:=%d' % int(t_ms)]
    return cmd
DRIVER_LOG = '/tmp/arm_grasp_driver.log'
# ⚠️ 不能只用 `__file__` 推 docs：`ros2 run` 跑的是 **install 树**里的副本，
# 那样样本会写到 `install/.../site-packages/docs/`（2026-09-28 踩过），
# 计划和文档都约定在源码树。所以**优先源码树**，没有才退回包旁边。
_SRC_DOCS = os.path.expanduser('~/raspot_ws/src/arm_grasp/docs')
DOCS_DIR = _SRC_DOCS if os.path.isdir(_SRC_DOCS) else os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs')

# ⚠️ 与 raspot_teleop/protocol.py 的 ARM_JOINT_NAMES **必须逐字一致**。
# 这里刻意**不 import**：arm_grasp 要能脱离 raspot_teleop 单独跑测试
# （Task 1 的 --packages-select 就只编这个包）。所以改成**运行期核对**，
# 见 `_check_joint_names()` —— 版本对不上时当场喊出来，而不是默默发错顺序。
JOINT_NAMES = ('gripper', 'wrist_roll', 'wrist_pitch', 'forearm',
               'shoulder', 'base')

# 采集用的关节采样箱（计划指定的范围）
PLAN_BOX = {'base': (-20.0, 20.0), 'shoulder': (70.0, 120.0),
            'elbow': (-80.0, -25.0), 'wrist_pitch': (-85.0, -40.0)}
# 夹爪尖高度窗口（cm）。★ 2026-09-28 随 L1 修正一起平移：
#   模型 z 的参考面从「臂自己的底板」改成了「**车体安装面**」，整体上移 6.81cm
#   （见 arm_kin.py 的 L1 说明），所以这个窗口也要 +6.81。
#   **没跟着改的后果实测过**：基准姿态 tip z=2.08 就已经落在旧框 (-16, 2) 外，
#   jacobian 工具的 4cm 位移三步全被 pose_is_safe 挡掉。
#   ⚠️ 注意这个窗口是**保守**的：它不允许爪尖低于 -9.19，也就是**够不到桌面(-13.6)**。
#      脚本化采集若要拍贴着桌面的姿态，得把下界放到 ≈-14。
TIP_Z_RANGE = (-9.19, 8.81)        # = 旧值 (-16.0, 2.0) + 6.81

# 蓝色块最少多少像素才算"画面里真有瓶盖"（瓶盖直径约 150px -> 面积约 1.7e4）
MIN_BLUE_PX = 500

# 「集中化」窗口：只保留离中位数这么近的蓝像素，把远处零星的蓝色反光剔掉。
# ★ 2026-09-29 从 130 放宽到 200：**抓取那一刻瓶盖的等效半径约 139px**
#   （12.4cm 距离 / 3.1cm 直径），130 的窗口会把瓶盖**横向截掉**
#   ⇒ `blue_blob_box` 的包围盒宽度少 18px ⇒ `servo.cap_center()` 的圆心估计
#   少算 ≈8px（≈1.8mm）。这个数是 `tools/servo_grasp.py --selftest` 抓出来的。
BLUE_WINDOW_PX = 200.0

RESULT_PATH = '/tmp/k230/latest-result.json'
TARGET_PORT = 8557                  # K230 的「外部目标入口」（一次一条 JSON）


# --------------------------------------------------------------------------
# 纯函数（可离线测）
# --------------------------------------------------------------------------

def pose_is_safe(joints):
    """这个关节姿态能不能发出去？

    两条硬条件：
      1. 肩/肘/腕的 field 落在固件真实钳位 125..875 内（**这才是真保护**；
         官方限位比本臂紧，见 design §4.6）
      2. 夹爪尖的高度在 TIP_Z_RANGE 内 —— 标定时标记点放在桌面上，
         夹爪得够得着，不然相机看不到它
    """
    f = to_fields(joints, 240.0, 496.0)
    for v in (f[2], f[3], f[4]):
        if not (FIELD_LO <= v <= FIELD_HI):
            return False
    tip, _ = fk(joints)
    return TIP_Z_RANGE[0] <= tip[2] <= TIP_Z_RANGE[1]


def _sample_joints(rnd):
    return {k: rnd.uniform(v[0], v[1]) for k, v in PLAN_BOX.items()}


def _signature(joints):
    """任务空间签名：相机能感知到的那些自由度。

    (tip 半径, tip 高度, 下扎角 alpha, 底座 yaw)。
    **不直接用关节角** —— 关节散不散开不是重点，重点是这个姿态让相机
    看到了什么；5 个待标定参数(dist)的可辨识性只由后者决定。
    """
    tip, axis = fk(joints)
    a = math.degrees(math.asin(max(-1.0, min(1.0, axis[2]))))
    return (math.hypot(tip[0], tip[1]), tip[2], a, joints['base'])


def plan_poses(n, seed=1, pool_factor=40):
    """返回 n 条互不相同的安全姿态，尽量铺满可达工作空间。确定性的。

    ⚠️ **与计划原文的偏离（2026-09-28 实测后改，理由见报告）**：
    计划写的是"在关节箱里均匀采样 + 过滤掉不安全的，凑够 n 条"。
    实测**那个做法不可靠**：安全集只占关节箱的 **11%**，而且是四维
    **同时偏低**的一个角（shoulder 上不去，105.9° 封顶）。300 个种子里
    有 **68 个**抽出来的 8 条 shoulder 跨度 ≤ 15° —— 也就是计划自己那条
    测试会随机变红；更要紧的是散不开的标定数据会让 5 个参数**不可辨识**，
    而残差**照样能算出来**（又一类"自洽但错"，看报告看不出来）。

    做法：先按计划均匀采样 + 过滤得到候选池，再在 `_signature` 上做
    **贪心最远点**挑选。签名维度先归一化到 [0,1] 再算欧氏距离，
    起点取 alpha 最低的那条（保证从可达区的一角起步）。
    """
    if n < 1:
        raise ValueError('n 必须 >= 1')
    rnd = random.Random(seed)
    want = max(n, pool_factor * n)
    pool = []
    for _ in range(200000):
        if len(pool) >= want:
            break
        j = _sample_joints(rnd)
        if pose_is_safe(j):
            pool.append(j)
    if len(pool) < n:
        raise RuntimeError('采样箱里只找到 %d 个安全姿态（要 %d 个）'
                           % (len(pool), n))

    sig = [_signature(j) for j in pool]
    lo = [min(s[i] for s in sig) for i in range(4)]
    hi = [max(s[i] for s in sig) for i in range(4)]
    span = [max(hi[i] - lo[i], 1e-9) for i in range(4)]

    def dist(a, b):
        return math.sqrt(sum(((a[i] - b[i]) / span[i]) ** 2 for i in range(4)))

    picked = [min(range(len(pool)), key=lambda i: sig[i][2])]   # 最低 alpha
    while len(picked) < n:
        best_d, best_i = -1.0, None
        for i in range(len(pool)):
            if i in picked:
                continue
            d = min(dist(sig[i], sig[k]) for k in picked)
            if d > best_d:
                best_d, best_i = d, i
        picked.append(best_i)
    return [pool[i] for i in picked]


# 实测锚点（2026-09-28）：在 [234,468,172,128,505,21] 这个姿态上（夹爪尖
# (12.33,1.64,-4.14)，α=-79.2°），瓶盖 (17,0,-12.3) **确认在画面里**
# （跟踪框和颜色质心差 4px，我肉眼也看过）。该姿态下
#   夹爪尖->瓶盖的仰角 = -60.8°，而 α = -79.2°  =>  差 18.4°
# 所以"相机能看到瓶盖"的经验条件是 **α ≈ 仰角(tip->cap) - 18.4°**。
# 这是从真机上量出来的，不是模型推的（模型的那 5 个参数现在还没标定）。
AIM_OFFSET_DEG = -18.4


def plan_poses_aimed(n, cap, seed=1, base_span=15.0, r_range=(11.0, 16.5),
                     z_range=(-6.5, -1.0), min_tip_to_cap=6.0,
                     aim_offset=AIM_OFFSET_DEG, path_clear=3.0,
                     alpha_range=(-88.5, -70.0), pool_factor=40):
    """按"相机确实能看到 cap"来排姿态。确定性的。

    为什么需要它（2026-09-28 血案）：原 `plan_poses` 只按**关节角散开 + 夹爪尖
    高度**筛，**从来没管相机朝哪**。实测 α 浅于 ≈-65° 时相机就是平着往前看，
    画面里是抽屉壁、**瓶盖根本不在**；跟踪器于是跟到背景纹理上，
    我还拿那些框当"瓶盖像素"用。散得越开，废数据越多。

    约束：
      1. 底座在**瓶盖方位角** ±base_span 内（横向扫动用来看 u*/roll）
      2. 夹爪尖在 cap 附近的 (r, z) 框里，**且离瓶盖 >= min_tip_to_cap**（别撞）
      3. α 按上面的实测锚点瞄准瓶盖
      4. `pose_is_safe`（field 在 125..875、tip 高度合理）
      5. **相邻姿态之间，夹爪尖走过的直线上离瓶盖 >= path_clear**（别扫到瓶盖）
    最后在任务空间签名上做贪心最远点，保证散得开。
    """
    if n < 1:
        raise ValueError('n 必须 >= 1')
    rnd = random.Random(seed)
    cap_az = math.degrees(math.atan2(cap[1], cap[0]))
    pool = []
    for _ in range(200000):
        if len(pool) >= pool_factor * n:
            break
        phi = math.radians(cap_az + rnd.uniform(-base_span, base_span))
        r = rnd.uniform(*r_range)
        z = rnd.uniform(*z_range)
        x, y = r * math.cos(phi), r * math.sin(phi)
        d = (cap[0] - x, cap[1] - y, cap[2] - z)
        nrm = math.sqrt(sum(c * c for c in d))
        if nrm < min_tip_to_cap:
            continue
        al = math.degrees(math.asin(d[2] / nrm)) + aim_offset
        # ⚠️ α 必须够陡 —— 实测 α 浅于 ≈-65° 时相机是**平着往前看**，
        # 画面里是抽屉壁、瓶盖根本不在（2026-09-28 就是这么采了一批废数据）。
        # 所以这里把上限卡在 -70°，不是 -20°（那只是 IK 的可达范围）。
        if not (alpha_range[0] <= al <= alpha_range[1]):
            continue
        try:
            j = ikine(x, y, z, al)
        except Unreachable:
            continue
        if not pose_is_safe(j):
            continue
        pool.append(j)
    if len(pool) < n:
        raise RuntimeError('按瞄准条件只找到 %d 个安全姿态（要 %d 个）'
                           % (len(pool), n))

    sig = [_signature(j) for j in pool]
    lo = [min(s[i] for s in sig) for i in range(4)]
    hi = [max(s[i] for s in sig) for i in range(4)]
    span = [max(hi[i] - lo[i], 1e-9) for i in range(4)]

    def dist(a, b):
        return math.sqrt(sum(((a[i] - b[i]) / span[i]) ** 2 for i in range(4)))

    picked = [min(range(len(pool)), key=lambda i: sig[i][2])]
    while len(picked) < n:
        best_d, best_i = -1.0, None
        for i in range(len(pool)):
            if i in picked:
                continue
            d = min(dist(sig[i], sig[k]) for k in picked)
            if d > best_d:
                best_d, best_i = d, i
        picked.append(best_i)
    out = [pool[i] for i in picked]

    # 相邻姿态的路径离瓶盖够不够远（在**候选池**上贪心换一个更安全的，
    # 换不到就报错——宁可不动，不要扫到瓶盖）
    for a in range(len(out)):
        b = (a + 1) % len(out)
        if _path_to_cap_ok(out[a], out[b], cap, path_clear):
            continue
        for cand in pool:
            if cand in out:
                continue
            if (_path_to_cap_ok(out[a], cand, cap, path_clear)
                    and _path_to_cap_ok(cand, out[b], cap, path_clear)):
                out[a] = cand
                break
        else:
            raise RuntimeError('第 %d 个姿态的路径离瓶盖太近，池子里换不到替代'
                               % (a + 1))
    return out


def _path_clear_of(ta, tb, cap, clear, steps=24):
    """夹爪尖从 ta 直线走到 tb，全程离 cap 都 >= clear 吗（纯几何）。"""
    for i in range(steps + 1):
        f = i / float(steps)
        p = tuple(ta[k] + f * (tb[k] - ta[k]) for k in range(3))
        if math.dist(p, cap) < clear:
            return False
    return True


def _path_to_cap_ok(ja, jb, cap, clear):
    """两个姿态之间夹爪尖走过的路离瓶盖 >= clear 吗。"""
    return _path_clear_of(fk(ja)[0], fk(jb)[0], cap, clear)


def pick_track(objs):
    """挑 `src == "track"` 那一条（跟踪器锁定、帧间连续）。没有就 None。"""
    if not isinstance(objs, list):
        return None
    for o in objs:
        if isinstance(o, dict) and o.get('src') == 'track':
            return o
    return None


def box_to_px(box, w, h):
    """归一化框 [l,t,r,b] -> 像素。返回 dict(u, v, w, h)。"""
    l, t, r, b = box
    return {'u': (l + r) * 0.5 * w, 'v': (t + b) * 0.5 * h,
            'w': (r - l) * w, 'h': (b - t) * h}


def read_result(path=RESULT_PATH, max_age=1.5, now=None):
    """读最近一条 K230 结果。返回 dict 或抛 RuntimeError（带人话原因）。"""
    now = time.time() if now is None else now
    if not os.path.exists(path):
        raise RuntimeError('没有 %s —— 板子没在推结果（resultd 没起来，'
                           '或者板子没联网）' % path)
    age = now - os.path.getmtime(path)
    if age > max_age:
        raise RuntimeError('结果文件 %.1fs 没更新（上限 %.1fs）—— 结果流断了'
                           % (age, max_age))
    last = None
    for _ in range(3):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (ValueError, OSError) as e:
            last = e
            time.sleep(0.05)
    raise RuntimeError('结果文件读不出来（可能在写）：%s' % last)


def lock_target(host, u, v, size=None, timeout=6.0, port=TARGET_PORT):
    """向 K230 发一次外部目标锁定 `{"pt":[u,v]}`（归一化）。

    协议与 `k230pi.send_target` 一致：**一次连接一条** ——
    连 -> 发一行 JSON -> 收一行 JSON -> 关。这里刻意内联实现（不 import
    k230pi），免得 arm_grasp 依赖另一个工程的目录。
    """
    payload = {'pt': [float(u), float(v)]}
    if size is not None:
        payload['size'] = float(size)
    line = json.dumps(payload).encode('utf-8') + b'\n'
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
    except OSError as e:
        raise RuntimeError('连不上 K230 %s:%d —— %s' % (host, port, e))
    try:
        sock.sendall(line)
        sock.settimeout(timeout)
        buf = b''
        while b'\n' not in buf:
            chunk = sock.recv(256)
            if not chunk:
                break
            buf += chunk
    finally:
        try:
            sock.close()
        except OSError:
            pass
    if not buf.strip():
        return None
    try:
        return json.loads(buf.split(b'\n')[0].decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return None


def k230_host_from_result(path=RESULT_PATH):
    """从结果文件的 `_peer` 里取出板子的地址。

    板子走 DHCP、地址会变（见记忆 k230-net-pc-hotspot-8555），
    但它每次连过来都会把自己的地址告诉 resultd，所以这里不用手填 IP。
    """
    with open(path, 'r', encoding='utf-8') as f:
        d = json.load(f)
    peer = d.get('_peer')
    if not peer:
        raise RuntimeError('结果文件里没有 _peer，拿不到板子地址；'
                           '用 --k230-host 手工指定')
    return peer.rsplit(':', 1)[0]


def _describe_blue(blob):
    if blob is None:
        return '⚠️ 画面里找不到蓝色瓶盖'
    return '瓶盖 px=(%.1f, %.1f) 归一化=(%.3f, %.3f) 面积 %d px' % (
        blob[0], blob[1], blob[0] / 1280.0, blob[1] / 720.0, blob[2])


def capture_frame(path, host, timeout=8.0, upright_suffix='-up.jpg'):
    """从 K230 的 RTSP 抓**最后一帧**存成 jpg。返回 True/False。

    ⚠️ 两个坑（2026-09-28 都踩过）：
    1. `k230ctl snap` 是坏的（报"中间产物 0 字节"），但 RTSP 流本身是好的。
    2. **`multifilesink` 没有 `num-buffers` 属性** —— 我第一版写了它，管道直接
       建不起来，而我又把 gst 的输出吞了，于是**静默返回 False**，查了半天。
       现在：用 `timeout -s INT`（SIGINT 让 gst 干净收尾）+ `-e`，
       并且**失败时把 stderr 打出来**。
    3. **相机是倒置的（用户 2026-09-28 确认）** —— 存一份**转正**的副本给人看，
       顺手也在这里做。⚠️ 但**跟踪器报的、`blue_blob_px` 出的都是原始系坐标**，
       别把两者混起来。转正映射：`(u, v) -> (W-u, H-v)`。
    """
    # ★ 2026-09-29：先建目录。/tmp 会被清空（重启/清理），目录一没了 gst 只会报
    #   "Error while writing to file ..."（看不懂），其实就差个 mkdir。别让它再坑一次。
    _d = os.path.dirname(path)
    if _d:
        try:
            os.makedirs(_d, exist_ok=True)
        except OSError as e:
            print('        ⚠️ 拍照: 建目录 %s 失败 %s' % (_d, e))
            return False
    if os.path.exists(path):
        try:
            os.remove(path)
        except OSError:
            pass
    gst = ('rtspsrc location=rtsp://%s:8554/k230 protocols=tcp latency=0 '
           '! rtph264depay ! h264parse ! avdec_h264 ! videoconvert '
           '! jpegenc quality=85 ! multifilesink location=%s' % (host, path))
    cmd = (['timeout', '-s', 'INT', '%.1f' % timeout,
            'gst-launch-1.0', '-e', '-q'] + gst.split())
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 6)
    except (subprocess.TimeoutExpired, OSError) as e:
        print('        ⚠️ 拍照: gst 跑不起来 %s' % e)
        return False
    if not (os.path.exists(path) and os.path.getsize(path) > 2000):
        print('        ⚠️ 拍照失败 rc=%s: %s'
              % (r.returncode, (r.stderr or '').strip()[:180]))
        return False
    # 存一份转正的给人看（相机倒置）
    try:
        from PIL import Image
        up = path[:-4] + upright_suffix if path.endswith('.jpg') else path + upright_suffix
        Image.open(path).rotate(180).save(up, quality=85)
    except Exception:                                        # noqa: BLE001
        pass
    return True


def _blue_points(path):
    """「瓶盖蓝」的那些像素坐标 (xs, ys)。找不到返回 None。

    ★ 阈值是 2026-09-28 拿**真实正/负样本**量出来的，不是拍脑袋：
        瓶盖本体 R=37  G=133 B=255  （饱和蓝：B-R=218, B-G=122）
        假阳性   R=169 G=224 B=252  （蓝白色的床单/桌面：B-R=83, B-G=28）
      旧阈值 (35,25,90) 两个都放过 —— 负样本能数出 26904 px，
      于是**相机明明扫到别处、它却报"找到瓶盖"**，正是我最怕的那种假绿。
      (60,60,110) 实测：有瓶盖 17704 px，没瓶盖 27 px，干净分开。
    中位数 ±130px 的那一步是**集中化**：把远处零星的蓝色反光剔掉。
    """
    try:
        from PIL import Image
        import numpy as np
    except ImportError:
        return None
    try:
        a = np.asarray(Image.open(path).convert('RGB')).astype(np.int16)
    except Exception:                                        # noqa: BLE001
        return None
    m = (a[:, :, 2] > a[:, :, 0] + 60) & (a[:, :, 2] > a[:, :, 1] + 60) \
        & (a[:, :, 2] > 110)
    ys, xs = np.nonzero(m)
    if len(xs) < MIN_BLUE_PX:
        return None
    mx, my = float(np.median(xs)), float(np.median(ys))
    k = (abs(xs - mx) < BLUE_WINDOW_PX) & (abs(ys - my) < BLUE_WINDOW_PX)
    xs, ys = xs[k], ys[k]
    if len(xs) < MIN_BLUE_PX:
        return None
    return xs, ys


def blue_blob_px(path):
    """画面里的蓝色块（瓶盖）质心和大小。找不到返回 None。

    ★ 这是**独立于跟踪器**的判据，2026-09-28 加：跟踪器（NanoTrack）会漂到
    背景纹理上还照样报一个框，而它报的框长宽比会乱跳。所以要有一把
    "瓶盖到底在不在画面里"的尺子 —— 颜色。返回 (u, v, 蓝像素数)。
    """
    p = _blue_points(path)
    if p is None:
        return None
    xs, ys = p
    return float(xs.mean()), float(ys.mean()), int(len(xs))


def blue_blob_box(path):
    """蓝块的**包围盒 + 质心**（1280x720 系）。找不到返回 None。

    ★ 伺服要的是包围盒，不是质心 —— 见 `servo.cap_center()`：
      瞄准像素在画框最下沿（v=709.8/720），瓶盖对上去时**底边一定出画**，
      而质心只按可见部分算 ⇒ 会稳定偏 ~60px（≈7mm），偏多少还随距离变。
    返回 dict：`u0/u1/v0/v1` 是包围盒，`cu/cv` 是质心，`n` 是蓝像素数。
    """
    p = _blue_points(path)
    if p is None:
        return None
    xs, ys = p
    return {'u0': float(xs.min()), 'u1': float(xs.max()),
            'v0': float(ys.min()), 'v1': float(ys.max()),
            'cu': float(xs.mean()), 'cv': float(ys.mean()),
            'n': int(len(xs))}


def _check_joint_names():
    """运行期核对关节名顺序 —— 顺序错了第一次真跑会表现得很明显（臂乱动）。

    这是 design §9 风险 #1：`p3/p4/p5` 的顺序是从 protocol.py 读出来的、
    **未独立验证**。所以这里主动去比对一次，把"默默发错"变成"当场报错"。
    """
    try:
        from raspot_teleop.protocol import ARM_JOINT_NAMES
    except Exception as e:                                  # noqa: BLE001
        print('⚠️  没能核对 ARM_JOINT_NAMES（%s）—— 假定本地这份是对的' % e)
        return
    if tuple(ARM_JOINT_NAMES) != JOINT_NAMES:
        raise RuntimeError('关节名顺序对不上！\n  raspot_teleop: %s\n  arm_grasp:     %s'
                           % (tuple(ARM_JOINT_NAMES), JOINT_NAMES))


# --------------------------------------------------------------------------
# 硬件：服务控制 + ROS IO
# --------------------------------------------------------------------------

def _sudo(args, password=None):
    """跑一条 sudo 命令。

    ⚠️ 本机 cy **没有免密 sudo**（2026-09-28 实测 `sudo -n true` 报"需要密码"），
    所以不能只写 `subprocess.run(['sudo', ...])` —— 非交互下它必然失败。
    顺序：先试 `sudo -n`（免密时直接过）→ 不行就交互问密码（`-S` 从 stdin 喂）
    → 连 tty 都没有就把失败原样返回，让调用方报清楚。

    密码也可以从环境变量 `GRASP_SUDO_PASS` 拿（自动化跑的时候用）。
    """
    password = password or os.environ.get('GRASP_SUDO_PASS')
    if password is None:
        r = subprocess.run(['sudo', '-n'] + args, capture_output=True, text=True)
        if r.returncode == 0 or not sys.stdin.isatty():
            return r
        import getpass
        password = getpass.getpass('sudo 密码: ')
    return subprocess.run(['sudo', '-S', '-p', ''] + args,
                          input=password + '\n', capture_output=True, text=True)


def _svc(cmd):
    return _sudo(['systemctl', cmd, SERVICE])


def _svc_active():
    r = subprocess.run(['systemctl', 'is-active', SERVICE],
                       capture_output=True, text=True)
    return r.stdout.strip()


# ⚠️ 判据必须用**节点可执行文件的路径**，不能用 'l150pro_driver_node' 这个名字：
# ① `ros2 run raspot_teleop l150pro_driver_node` 这条**启动器**命令行里也含这个名字，
#    会被一起匹配到（实测起一个驱动会数出 2 个进程）；
# ② 更要命的是 `pgrep -f <串>` 会匹配**任何**命令行里含这个串的进程 ——
#    包括我自己那条 shell（2026-09-28 这样杀过自己的 shell，命令静默消失）。
# 用安装路径当判据，两个问题一起解决。
DRIVER_PATTERN = 'lib/raspot_teleop/l150pro_driver_node'


def _self_tree():
    """我自己 + 我所有祖先进程的 PID。

    `pgrep -f <串>` 会匹配**命令行里含这个串的任何进程**，包括"跑着这条命令的
    shell" —— 2026-09-28 因此**把自己的 shell 杀了两次**（命令静默消失、什么
    输出都没有）。所以必须把自己这一条进程链整个排除掉。
    """
    out, pid = set(), os.getpid()
    for _ in range(24):
        out.add(pid)
        try:
            with open('/proc/%d/stat' % pid, 'r') as f:
                pid = int(f.read().split(') ', 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
        if pid <= 1:
            break
    return out


def _driver_pids():
    """当前在跑的驱动**节点**进程（不含 ros2 run 启动器、不含自己这一条链）。"""
    r = subprocess.run(['pgrep', '-f', DRIVER_PATTERN],
                       capture_output=True, text=True)
    mine = _self_tree()
    return sorted({int(t) for t in (r.stdout or '').split() if t.isdigit()}
                  - mine)


def _kill_all_drivers(why):
    """把所有 l150pro_driver_node 杀干净。

    ★ 2026-09-28 血案：`ros2 run` 是个启动器，它 spawn 真正的节点进程；
    `Popen.terminate()` 只杀掉 `ros2 run` 本身，**节点被 init 收养后继续跑**
    （实测 PPID=1）。于是它一直占着 /dev/l150pro，和随后服务里的驱动
    **抢串口** -> 驱动日志刷 `multiple access on port?` -> 固件反复走 INIT_HOME
    -> **INIT_HOME 期间 /arm/feedback 是固件总线调试计数器、不是关节角**
    -> 回读被污染，我误判"没到位"。**标定数据会整批是垃圾，而且看不出来。**
    """
    pids = _driver_pids()
    if not pids:
        return
    print('⚠️  %s：发现 %d 个残留的驱动节点（%s）—— 杀掉，免得抢串口'
          % (why, len(pids), pids))
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(1.5)
    for pid in _driver_pids():
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(0.5)
    left = _driver_pids()
    if left:
        raise RuntimeError('杀不干净：还剩 %s —— **不动臂**，先手工处理' % left)


def _start_driver(t_ms=None):
    """单独起驱动节点（服务的其余部分保持停着）。返回 Popen。

    `start_new_session=True` 让 `ros2 run` 和它 spawn 出的节点**同在一个新会话/
    进程组**里，这样 `_stop_driver` 能 `killpg` 一次带走两个 —— 这正是上面那个
    孤儿进程 bug 的正解。
    """
    _kill_all_drivers('起驱动之前')
    log = open(DRIVER_LOG, 'w', encoding='utf-8')
    cmd = driver_cmd(t_ms)
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True)
    print('→ 单独起了 l150pro_driver_node（pid %d，arm_t_ms=%s），日志 %s'
          % (p.pid, t_ms, DRIVER_LOG))
    time.sleep(1.0)
    pids = _driver_pids()
    if len(pids) != 1:
        raise RuntimeError('期望恰好 1 个驱动进程，实际 %s —— **不动臂**' % pids)
    print('   已确认串口独占：只有 1 个驱动进程')
    return p


def _stop_driver(p, timeout=6.0):
    """停掉单独起的驱动 —— **必须杀整个进程组**，否则会留孤儿抢串口。

    （`Popen.terminate()` 只杀 `ros2 run` 启动器，节点会被 init 收养继续跑。
    见 `_kill_all_drivers` 的说明。）
    """
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except OSError:
            try:
                p.terminate()
            except OSError:
                pass
        t0 = time.time()
        while p.poll() is None and time.time() - t0 < timeout:
            time.sleep(0.1)
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except OSError:
                pass
    _kill_all_drivers('收尾时')
    print('→ 已停掉单独起的驱动节点（并确认没有残留）')


class ArmIO:
    """最小 ROS 桥：订阅 /arm/feedback、/arm/online，发布 /arm/command。"""

    def __init__(self):
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        self.rclpy = rclpy
        self.rclpy.init()
        self.node = self.rclpy.create_node('arm_grasp_collect')
        self.fb = None
        self.fb_n = 0
        self.online = None
        self.sub = self.node.create_subscription(
            JointState, '/arm/feedback', self._on_fb, 10)
        self.sub_on = self.node.create_subscription(
            Bool, '/arm/online', self._on_online, 10)
        self.pub = self.node.create_publisher(JointState, '/arm/command', 10)
        self._js = JointState

    def _on_fb(self, msg):
        self.fb = [float(v) for v in msg.position]
        self.fb_n += 1

    def _on_online(self, msg):
        self.online = bool(msg.data)

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_feedback(self, timeout=5.0):
        t0 = time.time()
        while self.fb_n == 0 and time.time() - t0 < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.fb_n == 0:
            raise RuntimeError(
                '收不到 /arm/feedback —— 单独起的驱动没起来？看 %s '
                '（串口被占？/dev/l150pro 在不在？）' % DRIVER_LOG)

    def wait_fresh_feedback(self, since_n, timeout=2.0):
        """等一条**比 since_n 更新**的回读（等到位再取数，别拿旧帧）。"""
        t0 = time.time()
        while self.fb_n <= since_n and time.time() - t0 < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.fb

    @staticmethod
    def _arrived(fb, cmd, tol):
        """这一拍的回读能不能用：p3/p4/p5 非 0（0=该拍没读到）且与指令一致。"""
        if fb is None or len(fb) < 6:
            return False
        for i in (2, 3, 4):
            if fb[i] == 0:
                return False
            if abs(fb[i] - cmd[i]) > tol:
                return False
        return True

    def wait_arrived(self, cmd_fields, settle=2.5, tol=20.0, timeout=10.0,
                     hz=10.0, hold=0.8, stab=4.0):
        """发指令，**等到真的到位**再返回回读。返回 (fields 或 None, 用时s)。

        2026-09-28 上机实测出来的必要性：固件是**轮询**读那 5 个舵机的
        （`protocol.py: 'joints': 0 = 该拍没读到`，读不到就是 0），而且大动作
        （#5/#6 肩部一次跨 34°=143 count）时 2.5s 的回读**还停在移动前的值**：
        实测 #5 回读 p3/p4 = 221/190，而那正是**上一个姿态**的值（指令是 154/167）。
        直接采就会把"移动前的姿态"当成这一拍的姿态 -> 解出来的关节角全错，
        而残差只会"看起来有点大" —— 又是一类**"自洽但错"**。

        所以：先按 hz 流式发够 settle 秒，然后继续发、继续看回读，
        直到出现「非 0 且与指令一致」的值、**并且这个值稳住 hold 秒**
        （稳够一个轮询周期，免得把重复发布的旧帧当成到位）。超时给 None。

        ⚠️ "稳住"的判据是**抖动在 ±stab count 内**，不是逐位相等 ——
        2026-09-28 这里踩过坑：原来要求 6 个字段逐位完全相同，
        而底座电位器本底噪就是 ±1 count、p2/p3 也抖 ±1，
        于是那个计时器**每来一帧就被重置**、永远攒不够，必然超时。
        判据比被测量还严，就是必然误判。
        """
        msg = self._js()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in cmd_fields]
        dt = 1.0 / hz
        t0 = time.time()
        good_since, good_vals = None, None
        while True:
            msg.position = [float(v) for v in cmd_fields]
            self.pub.publish(msg)
            self.rclpy.spin_once(self.node, timeout_sec=dt)
            el = time.time() - t0
            if el < settle:
                continue
            fb = self.fb
            if self._arrived(fb, cmd_fields, tol):
                if good_vals is None or any(abs(fb[i] - good_vals[i]) > stab
                                            for i in range(6)):
                    good_vals, good_since = list(fb), time.time()
                elif time.time() - good_since >= hold:
                    return good_vals, time.time() - t0
            else:
                good_vals, good_since = None, None
            if el > settle + timeout:
                return None, time.time() - t0

    def stream(self, fields, seconds, hz=10.0):
        """按 hz 持续发同一条指令并稳定一会儿。

        为什么不是发一条就等：本固件**每条消息即发一帧 0xAC**，
        单帧丢了就没人补。流式发是驱动节点自己的用法（ps2_teleop 25 Hz），
        跟它保持一致最稳。
        """
        msg = self._js()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in fields]
        dt = 1.0 / hz
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.pub.publish(msg)
            self.rclpy.spin_once(self.node, timeout_sec=dt)

    def close(self):
        try:
            self.node.destroy_node()
        finally:
            self.rclpy.shutdown()


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------

def _parse_point(s):
    parts = [float(v) for v in s.replace(' ', '').split(',')]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('--point 要写成 x,y,z（cm），例如 16,0,-12')
    return tuple(parts)


def _fmt_fields(f):
    return '[%s]' % ', '.join('%4.0f' % v for v in f)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='机械臂视觉抓取 · 标定采集（会动臂！）')
    ap.add_argument('--poses', type=int, default=10, help='走几个姿态')
    ap.add_argument('--point', type=_parse_point, required=True,
                    help='标记点实测位置 x,y,z（cm；x/y 从底座转轴量起，'
                         'z 从底座安装面量起）')
    ap.add_argument('--holdout', type=int, default=2,
                    help='末尾留出几条不参与拟合（只写进元数据，'
                         '求解时由 calib_report 切）')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--planner', choices=('aimed', 'spread'), default='aimed',
                    help='aimed=按"相机确实能看到标记点"排姿态（默认，'
                         '2026-09-28 实测后改成默认）；'
                         'spread=旧的按关节角散开（实测会让相机看别处）')
    ap.add_argument('--settle', type=float, default=2.5,
                    help='每个姿态走完后先稳几秒，再开始等回读跟指令对上')
    ap.add_argument('--tol', type=float, default=20.0,
                    help='到位判据：回读与指令的差 <= 多少个 count（默认 20）')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='等到位的最长秒数（超了就跳过这条）')
    ap.add_argument('--lock', default=None, metavar='U,V',
                    help='开跑前向 K230 发一次外部目标锁定（归一化 u,v）')
    ap.add_argument('--k230-host', default=None,
                    help='K230 地址；默认从结果文件的 _peer 自动取')
    ap.add_argument('--max-age', type=float, default=1.5,
                    help='K230 结果允许的最大陈旧度（s）')
    ap.add_argument('--t-ms', type=int, default=600,
                    help='0xAC 每帧的移动时长（驱动参数 arm_t_ms）。'
                         '默认 40 太快，实测"太冲"，这里默认放缓到 600；'
                         '设 0 = 用驱动默认')
    ap.add_argument('--shot-dir', default='/tmp/k230/shots',
                    help='每个姿态前后各拍一张，存这里（留空 = 不拍）')
    ap.add_argument('--shot-timeout', type=float, default=12.0)
    ap.add_argument('--out-dir', default=DOCS_DIR,
                    help='样本/元数据写到哪（默认 %s）' % DOCS_DIR)
    ap.add_argument('--yes', action='store_true', help='跳过回车确认')
    args = ap.parse_args(argv)

    if args.planner == 'aimed':
        poses = plan_poses_aimed(args.poses, args.point, seed=args.seed)
    else:
        poses = plan_poses(args.poses, seed=args.seed)

    print('=' * 72)
    print('机械臂视觉抓取 · 标定采集')
    print('=' * 72)
    print('标记点实测位置 (x, y, z) = (%.2f, %.2f, %.2f) cm' % args.point)
    print('要走 %d 个姿态（seed=%d），每个停 %.1fs：' % (len(poses), args.seed,
                                                        args.settle))
    print()
    print('  #  base  shldr  elbow  wrist |                        fields |'
          ' tip r/z / alpha')
    for i, p in enumerate(poses):
        tip, axis = fk(p)
        al = math.degrees(math.asin(max(-1.0, min(1.0, axis[2]))))
        print('  %d %6.1f %6.1f %6.1f %6.1f | %s | r=%5.2f z=%6.2f a=%6.1f'
              % (i + 1, p['base'], p['shoulder'], p['elbow'], p['wrist_pitch'],
                 _fmt_fields(to_fields(p, 240.0, 496.0)),
                 math.hypot(tip[0], tip[1]), tip[2], al))
    print()
    print('⚠️  马上要做的三件事：')
    print('   1) 停 %s（Restart=always，必须 systemctl stop）' % SERVICE)
    print('   2) **单独**起 l150pro_driver_node —— 驱动和摇杆在同一个 service 里，')
    print('      只停服务会把回读/串口一起停掉（2026-09-28 踩过）')
    print('   3) 直接流式发 /arm/command —— 期间摇杆**无效**')
    print('   结束（无论成败）会自动杀掉驱动、start 回服务。')
    if not args.yes:
        try:
            input('\n确认上面这些姿态在你那儿不会撞到东西，回车开跑（Ctrl-C 取消）：')
        except EOFError:
            print('\n没有交互输入，用 --yes 才能跑。退出。')
            return 2

    _check_joint_names()

    rc = _svc('stop')
    if rc.returncode != 0:
        print('❌ sudo systemctl stop %s 失败：%s'
              % (SERVICE, (rc.stderr or rc.stdout).strip()))
        return 1
    time.sleep(1.0)
    st = _svc_active()
    if st != 'inactive':
        print('❌ %s 还是 %s —— **没有动臂**。请手工 `sudo systemctl stop %s`'
              % (SERVICE, st, SERVICE))
        return 1
    print('✅ %s 已停（inactive）' % SERVICE)
    left = _driver_pids()
    if left:
        print('⚠️  停服务后还发现 %d 个驱动进程（%s）—— 这是上次留下的孤儿，'
              '它们会抢串口、污染回读，先清掉' % (len(left), left))
        _kill_all_drivers('停服务之后')

    io = None
    drv = None
    samples, meta = [], []
    t_start = time.strftime('%Y%m%d-%H%M%S')
    try:
        drv = _start_driver(args.t_ms or None)
        time.sleep(2.0)                      # 等它把串口打开、节点上线

        host = args.k230_host or k230_host_from_result()
        if args.shot_dir:
            os.makedirs(args.shot_dir, exist_ok=True)

        if args.lock:
            u, v = [float(x) for x in args.lock.split(',')]
            print('→ 锁定 K230 外部目标 host=%s pt=(%.3f, %.3f)' % (host, u, v))
            print('   应答: %r' % (lock_target(host, u, v),))
            time.sleep(0.5)

        io = ArmIO()
        io.wait_feedback()
        print('✅ /arm/feedback 有数据（online=%s；本固件不置 ONLINE 位是已知的，'
              '不拦）' % io.online)

        first_box = None
        for i, p in enumerate(poses):
            fields = to_fields(p, 240.0, 496.0)
            print('\n[%d/%d] 发 %s' % (i + 1, len(poses), _fmt_fields(fields)))

            # ① 移动前拍一张：记录"出发时"瓶盖在不在画面里
            shot_before = None
            if args.shot_dir:
                bp = os.path.join(args.shot_dir, 'pose-%02d-before.jpg' % (i + 1))
                if capture_frame(bp, host, args.shot_timeout):
                    shot_before = blue_blob_px(bp)
                print('        移动前: %s' % _describe_blue(shot_before))

            fb, waited = io.wait_arrived(fields, settle=args.settle,
                                         tol=args.tol, timeout=args.timeout)
            if fb is None:
                print('   ❌ 等了 %.1fs 回读还没跟指令对上，**跳过这条**'
                      '（宁可少一条，不要一条假姿态）' % waited)
                continue
            print('        %.1fs 到位，回读 %s' % (waited, _fmt_fields(fb)))

            # ② 到位后再拍一张：这才是"这一拍的画面"，而且用**颜色**独立判断
            #    瓶盖在不在 —— 不信跟踪器（它会漂到背景上还照样给框）
            shot_after = None
            if args.shot_dir:
                ap_ = os.path.join(args.shot_dir, 'pose-%02d-after.jpg' % (i + 1))
                if capture_frame(ap_, host, args.shot_timeout):
                    shot_after = blue_blob_px(ap_)
                print('        移动后: %s' % _describe_blue(shot_after))
                if shot_after is None:
                    print('        ⚠️ **这一拍画面里没有瓶盖**，这条是废数据')

            d = read_result(max_age=args.max_age)
            o = pick_track(d.get('objs'))
            if o is None:
                print('   ❌ 这一帧没有 src=="track" 的目标（objs=%d 条），跳过'
                      % len(d.get('objs') or []))
                continue
            box = box_to_px(o['box'], d['w'], d['h'])
            if first_box is None:
                first_box = box['w']
            grow = box['w'] / first_box if first_box else float('nan')
            flag = ' ⚠️ 框膨胀 x%.2f（>2 就是废数据）' % grow if grow > 2.0 else ''
            print('        像素 u=%7.2f v=%7.2f  框 %.1fx%.1f px  涨 x%.2f%s'
                  % (box['u'], box['v'], box['w'], box['h'], grow, flag))
            # 跟踪器 vs 照片：两者差太远 = 跟踪器已经不在瓶盖上
            if shot_after is not None:
                du, dv = box['u'] - shot_after[0], box['v'] - shot_after[1]
                if math.hypot(du, dv) > 60.0:
                    print('        ⚠️ 跟踪框离照片里的瓶盖 %.0f px —— 跟踪器漂了'
                          % math.hypot(du, dv))
                    flag += ' 跟踪漂了'

            # 回读的 field -> 关节角（**用回读，不用指令值**：
            # 指令和实际到位之间有回差，标定要的是"实际是哪个姿态"）
            from .arm_kin import from_fields
            j = from_fields(fb)
            samples.append(Sample(joints=j, point=args.point,
                                  uv=(box['u'], box['v'])))
            drift = None
            if shot_after is not None:
                drift = math.hypot(box['u'] - shot_after[0],
                                   box['v'] - shot_after[1])
            meta.append({'i': i + 1, 'cmd_fields': fields, 'fb_fields': fb,
                         'frame': d.get('frame'), 'ts': d.get('ts'),
                         'u': box['u'], 'v': box['v'],
                         'box_w': box['w'], 'box_h': box['h'],
                         'box_growth': grow,
                         'blue_before': shot_before,
                         'blue_after': shot_after,
                         'track_vs_blue_px': drift,
                         'suspect': bool(grow > 2.0
                                         or shot_after is None
                                         or (drift is not None and drift > 60))})
    except KeyboardInterrupt:
        print('\n⚠️  被 Ctrl-C 打断，已采到的样本照常存盘。')
    except Exception as e:                                   # noqa: BLE001
        print('\n❌ 出错了：%s' % e)
        print('   已采到的样本照常存盘。')
    finally:
        if io is not None:
            try:
                io.close()
            except Exception:                                # noqa: BLE001
                pass
        _stop_driver(drv)
        rc = _svc('start')
        st = _svc_active()
        print('\n%s %s -> %s' % ('✅' if st == 'active' else '❌',
                                 'systemctl start', st))

    if not samples:
        print('❌ 一条样本都没采到，不写文件。')
        return 1

    path = os.path.join(args.out_dir, 'samples-%s.json' % t_start)
    os.makedirs(args.out_dir, exist_ok=True)
    save_samples(samples, path)
    mpath = path[:-5] + '.meta.json'
    with open(mpath, 'w', encoding='utf-8') as f:
        json.dump({'point': list(args.point), 'seed': args.seed,
                   'poses_requested': args.poses, 'settle': args.settle,
                   'holdout': args.holdout, 'service': SERVICE,
                   'samples': meta}, f, ensure_ascii=False, indent=1)

    print('\n' + '=' * 72)
    print('采到 %d 条样本' % len(samples))
    for m in meta:
        print('  #%-2d u=%7.2f v=%7.2f 框宽=%6.1f 涨 x%.2f%s'
              % (m['i'], m['u'], m['v'], m['box_w'], m['box_growth'],
                 '  ⚠️可疑' if m['suspect'] else ''))
    print('可疑（框膨胀 >2 倍）: %d 条'
          % sum(1 for m in meta if m['suspect']))
    print('样本: %s' % path)
    print('元数据: %s' % mpath)
    print('\n下一步：')
    print('  ros2 run arm_grasp calib_report --samples %s --holdout %d'
          % (path, args.holdout))
    return 0


if __name__ == '__main__':
    sys.exit(main())
