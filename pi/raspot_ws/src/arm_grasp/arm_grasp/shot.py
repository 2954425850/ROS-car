# -*- coding: utf-8 -*-
"""预合爪快照（close shot）：进 `close` 的那一刻存一份现场，事后离线算真实落点偏差。

## 为什么要有它

现在的末段是**视觉闭眼**：进 `close` 之前冻结目标点 O，之后只用关节回读往下走。
于是每轮实际的横向偏差 = **冻结时 O 不准** + **冻结后臂执行不到位**，两者混在一起，
光看"抓没抓到"分不开。2026-11 的交接结论是：先量出来是哪一半，再决定要不要上末段 IBVS。

本模块只做两件事：

1. `build()` / `save()` —— 把采集侧的**原始输入**（框、像素、实测深度、关节、时间戳）落盘。
   **采集侧一行几何都不算**：派生量在 `deviation()` 里重算，换方法不用再跑一遍真机。
2. `deviation()` —— 离线把误差拆成三块，单位**米**，基座系：

   | 量 | 含义 |
   |---|---|
   | `d_O`  | 冻结之后若继续更新，O 还会被拉走多少 ⇒ **冻结的代价** |
   | `d_tip`| 爪尖（回读 + FK）到**物体真值**的距离 ⇒ **本轮真实的落点偏差** |
   | 分量   | `axial`（沿夹爪轴）/ `lat`（垂直于轴）/ `x,y,z`（基座系）/ `radial`（径向）|

   `d_tip` 才是"会不会抓到"的那个数；`d_O` 大 = 冻结太早/太脏，`d_O` 小而 `d_tip` 大 =
   冻结之后臂没走到位（回差、下垂、底座滞后）。

## 它**不能**证明什么

`d_tip` 用的是**同一套几何模型**（FK + 手眼 + 内参）。模型本身偏了，这个数会一起偏，
所以照片（`*.jpg`）必须一起存 —— 人要能用眼睛复核"模型说的落点到不到那儿"。

## 数据在看哪儿

默认 `~/.local/share/arm_shots/`，一次抓取一对文件：`<时间>.json` + `<时间>.jpg`。
看汇总：`python3 -m arm_grasp.shot <目录>`，或 `tools/grasp_shots.py`。
"""
import glob
import json
import math
import os
import time

from . import geom
from .arm_kin import fk, from_fields

SCHEMA = 1


# ---------------------------------------------------------------- 采集

def build(*, phase_from, reason, O, O_u, est_n, est_rejects, alpha,
          f_fb, f_cmd, f_tgt, s_ach, s_cmd, err_m, comps,
          obs, obs_age, j_obs, obs_n, obs_bad, depth_refused,
          cfg=None, t_mono=None, latency_s=None):
    """把进 `close` 那一刻的现场拼成一条可 JSON 化的记录。

    参数全部是**标量/列表**，不接收任何对象（`Obs`/`GraspConfig` 由这里摊平），
    这样它能在真机上存、也能在测试里手搓。
    """
    o = None
    if obs is not None:
        o = {'t_arrive': _f(obs.t_arrive), 'u_ai': _f(obs.u_ai), 'v_ai': _f(obs.v_ai),
             'frame': obs.frame, 'src': obs.src,
             'box': _fl(obs.box_norm), 'depth_mm': _f(getattr(obs, 'depth_mm', None))}
    rec = {
        'schema': SCHEMA,
        't_wall': time.strftime('%Y-%m-%dT%H:%M:%S'),
        't_mono': _f(t_mono),
        'phase_from': phase_from,
        'reason': reason,
        # 冻结前后的目标点（基座系，米）。O = 估计器的，O_u = 真正喂给 pick_target 的
        # （含 bias_m / bias_r_m / grip_depth_m）。
        'O': _fl(O), 'O_u': _fl(O_u),
        'est_n': est_n, 'est_rejects': est_rejects,
        'alpha_deg': _f(None if alpha is None else math.degrees(alpha)),
        # 关节：6 个 field 原样（含 p1）—— 解码交给消费方，别在这儿丢掉信息
        'f_fb': _fl(f_fb), 'f_cmd': _fl(f_cmd), 'f_tgt': _fl(f_tgt),
        'tip_fb': None,               # 下面由 deviation() 算，这里留位方便人眼看 json
        's_ach_m': _f(s_ach), 's_cmd_m': _f(s_cmd), 'err_m': _f(err_m),
        'comps': _fl(comps),
        'obs': o,
        'obs_age_s': _f(obs_age),
        'j_obs': _fl(j_obs),          # 拍那张图时的关节（6 field）
        'obs_n': obs_n, 'obs_bad': obs_bad, 'depth_refused': depth_refused,
        'latency_s': _f(latency_s),
        'cfg': _cfg_summary(cfg),
        'image': None,                # ServerLink 抓完照片回填文件名
        'result': None,               # grasp_server 跑完回填 code/ok/reason/grip_p1
    }
    rec['tip_fb'] = _fl(_tip(rec['f_fb']))
    return rec


def save(dirpath, rec):
    """写 `<dir>/<时间>-<毫秒>.json`，返回路径。目录按需建。"""
    os.makedirs(os.path.expanduser(dirpath), exist_ok=True)
    stem = '%s-%03d' % (time.strftime('%Y%m%d-%H%M%S'), int(time.time() * 1000) % 1000)
    path = os.path.join(os.path.expanduser(dirpath), stem + '.json')
    _write(path, rec)
    return path


def update(path, **kw):
    """把几个键补写回 json（照片名、跑完的结果）。文件没了就静默忽略。"""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            rec = json.load(f)
    except (OSError, ValueError):
        return False
    rec.update(kw)
    _write(path, rec)
    return True


def _write(path, rec):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(rec, f, ensure_ascii=False, indent=1, sort_keys=False)
    os.replace(tmp, path)            # 断电/并发读都不留半个文件


# ---------------------------------------------------------------- 离线分析

def obs_point(rec):
    """**按观测自己的时刻**用实测深度还原物体在基座系的位置（米）。算不出返回 None。

    用 `j_obs`（拍那张图时的关节）而不是进 close 时的关节 —— 相机跟着臂动，
    拿错时刻的关节去做几何，就是设计文档里反复出现的"配错姿态"那个坑。
    """
    o, j = rec.get('obs'), rec.get('j_obs')
    if not o or not j or o.get('depth_mm') is None:
        return None
    try:
        P, _t = geom.target_from_pixel_depth(from_fields(j), o['u_ai'], o['v_ai'],
                                             float(o['depth_mm']) / 1000.0)
    except (ValueError, KeyError):
        return None
    return list(P)


def deviation(rec):
    """把一条快照的误差拆开。返回 dict（米 / 度），算不出的项是 None。

    键：
      `d_O`    : 冻结的 O → 用最后一条观测重算的物体位置（≈ "继续更新会走多远"）
      `d_tip`  : 爪尖（进 close 时的回读）→ 物体位置（≈ 本轮真实落点偏差）
      `axial`  : d_tip 沿夹爪轴的分量（+ = 目标比爪尖更"深/更远"）
      `lat`    : d_tip 垂直于夹爪轴的分量（长度）  ← **横向偏差就是它**
      `x/y/z`  : d_tip 在基座系的分量
      `radial` : d_tip 的径向分量（+ = 目标比爪尖更远离底座；`bias_r_m` 调的就是这个）
      `px`     : 物体此刻应落在的 AI 像素 − 对准时应落的像素（张开态的 aim_pixel）
    """
    out = dict(d_O=None, d_tip=None, axial=None, lat=None,
               x=None, y=None, z=None, radial=None, px=None,
               tip=None, obj=None)
    P = obs_point(rec)
    out['obj'] = P
    O = rec.get('O')
    if P is not None and O is not None:
        out['d_O'] = math.dist(P, O)
    j = _joints(rec.get('f_fb'))
    tip = _tip(rec.get('f_fb'))
    out['tip'] = tip
    if P is None or tip is None:
        return out
    d = [P[i] - tip[i] for i in range(3)]
    out['x'], out['y'], out['z'] = d
    out['d_tip'] = math.sqrt(sum(v * v for v in d))
    try:
        _tip_cm, axis = fk(j)
    except Exception:                                       # noqa: BLE001
        return out
    ax = sum(d[i] * axis[i] for i in range(3))
    lat = [d[i] - ax * axis[i] for i in range(3)]
    out['axial'] = ax
    out['lat'] = math.sqrt(sum(v * v for v in lat))
    r = math.hypot(tip[0], tip[1])
    if r > 1e-6:
        out['radial'] = (d[0] * tip[0] + d[1] * tip[1]) / r
    try:
        u, v = geom.project(j, P)
        ua, va = geom.aim_pixel(closed=False)
        out['px'] = (u - ua, v - va)
    except ValueError:
        pass
    return out


def summarize(rec, dev=None):
    """一行字，给 CLI/日志用。"""
    dev = deviation(rec) if dev is None else dev
    res = rec.get('result') or {}
    tag = res.get('code') or '?'
    parts = ['%s' % (rec.get('t_wall') or '?'), '%-14s' % tag]
    if dev['lat'] is not None:
        parts.append('lat=%5.1f' % (dev['lat'] * 1000.0))
    if dev['d_tip'] is not None:
        parts.append('tot=%5.1f' % (dev['d_tip'] * 1000.0))
    if dev['axial'] is not None:
        parts.append('ax=%+5.1f' % (dev['axial'] * 1000.0))
    if dev['radial'] is not None:
        parts.append('rad=%+5.1f' % (dev['radial'] * 1000.0))
    if dev['d_O'] is not None:
        parts.append('dO=%5.1f' % (dev['d_O'] * 1000.0))
    if dev['px'] is not None:
        parts.append('px=%+5.1f,%+5.1f' % dev['px'])
    for k in ('s_ach_m', 'err_m'):
        if rec.get(k) is not None:
            parts.append('%s=%4.1f' % (k[:5], rec[k] * 1000.0))
    if rec.get('image'):
        parts.append('img')
    return '  '.join(parts)


def aggregate(recs):
    """一批快照的统计。返回 dict：各量在成功/全部里的 中位数 p50 与 95 分位 p95。"""
    groups = {'all': recs,
              'ok': [r for r in recs if (r.get('result') or {}).get('ok')]}
    out = {}
    for gname, rs in groups.items():
        devs = [deviation(r) for r in rs]
        out[gname] = {'n': len(rs)}
        for key in ('lat', 'd_tip', 'd_O', 'axial', 'radial'):
            vals = sorted(d[key] for d in devs if d[key] is not None)
            if not vals:
                continue
            out[gname][key] = {'p50': _pct(vals, 0.5), 'p95': _pct(vals, 0.95),
                               'mean': sum(vals) / len(vals)}
    return out


def load_all(dirpath):
    recs = []
    for p in sorted(glob.glob(os.path.join(os.path.expanduser(dirpath), '*.json'))):
        try:
            with open(p, 'r', encoding='utf-8') as f:
                rec = json.load(f)
        except (OSError, ValueError):
            continue
        rec['_path'] = p
        recs.append(rec)
    return recs


# ---------------------------------------------------------------- 小工具

def _pct(vals, q):
    """最近秩分位（vals 已排序）。样本少时比插值更不会骗人。"""
    if not vals:
        return None
    k = max(0, min(len(vals) - 1, int(round(q * len(vals) + 0.5)) - 1))
    return vals[k]


def _joints(f):
    return None if not f else from_fields(f)


def _tip(f):
    j = _joints(f)
    if j is None:
        return None
    try:
        return list(geom.gripper_tip(j, closed=False))
    except Exception:                                       # noqa: BLE001
        return None


def _f(x):
    if x is None:
        return None
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _fl(xs):
    if xs is None:
        return None
    return [_f(v) for v in xs]


def _cfg_summary(cfg):
    if cfg is None:
        return None
    keys = ('alpha0', 's_pre_m', 's_stop_m', 'grip_depth_m', 'bias_r_m', 'bias_m',
            'joint_comp', 'close_field', 'hz', 'latency_s', 'aim_hold_m', 'require_depth')
    out = {}
    for k in keys:
        v = getattr(cfg, k, None)
        out[k] = _fl(v) if isinstance(v, (tuple, list)) else _f(v)
    return out


def main(argv=None):
    """`python3 -m arm_grasp.shot [目录]` —— 打印每一条 + 统计。"""
    import sys
    args = list(sys.argv[1:] if argv is None else argv)
    d = os.path.expanduser(args[0] if args else '~/.local/share/arm_shots')
    recs = load_all(d)
    if not recs:
        print('没有快照：%s' % d)
        return 1
    print('%-19s %-14s %s' % ('时间', '结果', '（毫米；px = AI 像素）'))
    for r in recs:
        print(summarize(r))
    agg = aggregate(recs)
    for g in ('all', 'ok'):
        a = agg.get(g) or {}
        if not a.get('n'):
            continue
        line = ['%s n=%d' % (g, a['n'])]
        for key, name in (('lat', '横向'), ('d_tip', '落点'), ('d_O', '冻结后O漂')):
            if key in a:
                line.append('%s p50=%.1f p95=%.1f' % (name, a[key]['p50'] * 1000.0,
                                                      a[key]['p95'] * 1000.0))
        print('  ' + ' | '.join(line))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
