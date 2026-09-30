# -*- coding: utf-8 -*-
"""读采集样本 -> 解退化手眼参数 -> 打印并写出残差报告。

    ros2 run arm_grasp calib_report --samples docs/samples-<ts>.json --holdout 2

## 判据（Task 6 Step 5，**写死在代码里，不靠人眼**）
    px_rms(留出集) < 5 px  且  cm_max(留出集) < 1.5 cm   -> 通过

`cm_*` 是"像素误差折算到地面上的厘米"，回答"到底能不能抓"；
`px_*` 回答"模型对不对"。两个都要报。
"""
import argparse
import json
import math
import os
import sys

from .calib_solve import Sample, load_samples, solve

# 里程碑 1 的判据阈值（事先约定，不达标就停下来查，不许糊过去）
PX_RMS_PASS = 5.0
CM_MAX_PASS = 1.5

# 框宽涨到首帧的多少倍就判"跟踪漂了、这条废"
BOX_GROW_SUSPECT = 2.0


def _fmt_stat(s):
    return ('px_rms=%.3f px_max=%.3f  cm_rms=%.3f cm_max=%.3f'
            % (s['px_rms'], s['px_max'], s['cm_rms'], s['cm_max']))


def _load_meta(samples_path):
    """样本旁边可能有 collect 写的 .meta.json（框宽、姿态、可疑标记）。"""
    p = samples_path[:-5] + '.meta.json'
    if not os.path.exists(p):
        return None
    try:
        with open(p, 'r', encoding='utf-8') as f:
            m = json.load(f)
    except (ValueError, OSError):
        return None
    # 脚本化采集（collect.py）写的是 {'samples': [...]}；
    # 手动采集（rec_sample.py, §7）写的是**裸 list**，字段也少。
    # 统一成 {'samples': [...]}，缺字段交给 _meta_note 兜。
    if isinstance(m, list):
        return {'samples': m}
    return m


def _meta_note(m):
    """meta 的一条 -> 屏幕上那行注解。手动采集缺 box_growth/suspect，缺就不报。"""
    if not isinstance(m, dict) or 'box_w' not in m or 'box_h' not in m:
        return ''
    note = '  框 %.1fx%.1f px' % (m['box_w'], m['box_h'])
    if 'box_growth' in m:
        note += '  涨 x%.2f' % m['box_growth']
    if m.get('suspect'):
        note += '  ⚠️可疑'
    return note


def judge(rep):
    """里程碑 1 的判定：**两条都要满足**。

    抽成函数是为了能直接钉住语义 —— 变异自检发现改成 `or` 时
    没有任何用例会红（真实数据上两条残差总是一起超标，分不出来）。
    """
    return rep['px_rms'] < PX_RMS_PASS and rep['cm_max'] < CM_MAX_PASS


def parameter_sensitivity(samples, dz=2.0, fit_principal=False):
    """标记点高度每量错 1 cm，参数会跟着跑多少 —— 这次标定的参数能信到几位。

    ★ 2026-09-28 实测后加的，理由是个**真问题**：
      残差判据**抓不住小的标记点坐标误差**。用合成数据实测（f=1100 真值）：
        标记点 z 偏 -3 cm -> px_rms 2.70、cm_max 0.63  **照样"通过"**
                              但拟合出来的 f 已经偏 **12.1%**
        z 偏 -5 cm        -> px_rms 4.71、cm_max 1.19  **仍然通过**，f 偏 22.4%
        z 偏 -10 cm       -> px_rms 9.46、cm_max 3.23  这才不通过
      也就是"标记点量错"这条（计划 Task 6 Step 5 的排查第 1 条）
      **不会**像计划说的那样"只会让残差整体偏大" —— 小的量错会被参数悄悄吃掉，
      又是一类"自洽但错"。所以报告里必须额外给出参数的**敏感度**，
      让人知道 f/t 到底能信几位，而不是只看残差绿了就放心。

    返回 {'df_per_cm','dt_per_cm','dus_per_cm','dvs_per_cm'}。
    """
    def _fit(shift):
        ss = [Sample(joints=x.joints,
                     point=(x.point[0], x.point[1], x.point[2] + shift),
                     uv=x.uv) for x in samples]
        p, _ = solve(ss, fit_principal=fit_principal, holdout=0)
        return p

    up, dn = _fit(dz), _fit(-dz)
    k = 2.0 * dz                      # 中心差分，除以 2*dz
    return {'dz_cm': dz,
            'df_per_cm': (up.f - dn.f) / k,
            'dt_per_cm': (up.t - dn.t) / k,
            'dus_per_cm': (up.u_star - dn.u_star) / k,
            'dvs_per_cm': (up.v_star - dn.v_star) / k}


def build_report(samples_path, holdout=2, fit_principal=False):
    samples = load_samples(samples_path)
    params, rep = solve(samples, fit_principal=fit_principal, holdout=holdout)
    meta = _load_meta(samples_path)
    rep['sensitivity'] = parameter_sensitivity(samples,
                                                fit_principal=fit_principal)
    return params, rep, meta, samples


def _mc(meta, i, k):
    """write_report 的三列：框宽 / 涨倍数 / 可疑标记。

    手动采集（rec_sample.py）的 meta **没有 box_growth / suspect**，
    直接取键会 KeyError（2026-09-28 实测：--fit-principal 落盘时崩在这里）。
    缺就填 '-'。
    """
    if not meta or i >= len(meta['samples']) or not isinstance(meta['samples'][i], dict):
        return '-'
    m = meta['samples'][i]
    if k == 0:
        return '%.1f' % m['box_w'] if 'box_w' in m else '-'
    if k == 1:
        return 'x%.2f' % m['box_growth'] if 'box_growth' in m else '-'
    return '⚠️' if m.get('suspect') else ''


def print_report(samples_path, params, rep, meta):
    print('=' * 72)
    print('退化手眼标定 · 残差报告')
    print('=' * 72)
    print('样本: %s' % samples_path)
    print('参数: %r' % params)
    print('       （主点 %s：u0=%.1f v0=%.1f）'
          % ('也参与了拟合' if rep['fit_principal'] else '固定画面中心',
             params.u0, params.v0))
    print()
    print('拟合集 n=%-3d: %s' % (rep['n_fit'], _fmt_stat(rep['fit'])))
    print('留出集 n=%-3d: %s   <-- 这个才是真判据'
          % (rep['n_test'], _fmt_stat(rep['test'])))
    print()

    print('逐条（留出集）：')
    idx = list(range(rep['n_fit'], rep['n_fit'] + rep['n_test']))
    for k, (i, e) in enumerate(zip(idx, rep['per_sample'])):
        note = (_meta_note(meta['samples'][i])
                if meta and i < len(meta['samples']) else '')
        print('  #%-3d px=%7.3f  cm=%7.4f%s' % (i + 1, e['px'], e['cm'], note))

    print()
    print('拟合集逐条：')
    fit_px = rep['fit'].get('px', [])
    fit_cm = rep['fit'].get('cm', [])
    for i, (a, b) in enumerate(zip(fit_px, fit_cm)):
        note = (_meta_note(meta['samples'][i])
                if meta and i < len(meta['samples']) else '')
        print('  #%-3d px=%7.3f  cm=%7.4f%s' % (i + 1, a, b, note))

    if meta:
        n_sus = sum(1 for m in meta['samples']
                   if isinstance(m, dict) and m.get('suspect'))
        print()
        print('可疑样本（框膨胀 >%.1f 倍，跟踪漂了）: %d 条'
              % (BOX_GROW_SUSPECT, n_sus))
        if n_sus:
            print('  -> 先剔掉它们重解一遍，再谈参数精度。')

    sens = rep.get('sensitivity')
    if sens:
        print()
        print('参数敏感度（标记点高度每量错 1 cm，参数会跑多少）：')
        print('  df/f  = %+7.2f px/cm   (%.2f%%)'
              % (sens['df_per_cm'], 100.0 * sens['df_per_cm'] / params.f))
        print('  dt    = %+7.2f cm/cm' % sens['dt_per_cm'])
        print('  du*   = %+7.2f px/cm   dv* = %+7.2f px/cm'
              % (sens['dus_per_cm'], sens['dvs_per_cm']))
        print('  -> 参数只在这个量级上可信。残差绿色**不代表**标记点量对了：')
        print('     实测 z 偏 3 cm 时残差照样通过，但 f 已经偏 12%。')

    passed = judge(rep)
    print()
    print('-' * 72)
    print('判据: px_rms(留出) < %.1f px  且  cm_max(留出) < %.1f cm'
          % (PX_RMS_PASS, CM_MAX_PASS))
    print('实际: px_rms(留出) = %.3f px      cm_max(留出) = %.3f cm'
          % (rep['px_rms'], rep['cm_max']))
    print('结论: %s' % ('✅ 通过，里程碑 1 达成' if passed else '❌ 不通过'))
    if not passed:
        print()
        print('没通过 -> 按这个顺序查（design §9 / 计划 Task 6 Step 5）：')
        print('  1. 标记点的 (x,y,z) 量错了？ —— 最常见，且只会让残差整体偏大')
        print('  2. 采集时框漂了？ —— 看上面的"框宽/涨"，膨胀 >2 倍的是废数据，'
              '剔掉重解')
        print('  3. from_fields 的 p3/p4/p5 顺序错了？（全项目唯一没独立验证的'
              '假设）判据：换个姿态残差**系统性变大**，而不是随机散布')
        print('  4. 平行假设真的成立吗？ —— 1~3 都排除后，回头找用户')
        print('  先试 --fit-principal（放开主点）看看是不是主点偏了')
    print('-' * 72)
    return passed


def write_report(samples_path, params, rep, meta, passed):
    base = samples_path[:-5] if samples_path.endswith('.json') else samples_path
    jpath = base + '.report.json'
    mpath = base + '.report.md'

    def _clean(d):
        return {k: v for k, v in d.items() if k not in ('px', 'cm')}

    payload = {'samples': samples_path, 'params': repr(params),
               'params_vec': params.as_vector(), 'fit_principal':
               rep['fit_principal'], 'iters': rep['iters'], 'cost': rep['cost'],
               'n_fit': rep['n_fit'], 'n_test': rep['n_test'],
               'fit': _clean(rep['fit']), 'test': _clean(rep['test']),
               'px_rms': rep['px_rms'], 'px_max': rep['px_max'],
               'cm_rms': rep['cm_rms'], 'cm_max': rep['cm_max'],
               'sensitivity': rep.get('sensitivity'),
               'thresholds': {'px_rms': PX_RMS_PASS, 'cm_max': CM_MAX_PASS},
               'passed': passed}
    with open(jpath, 'w', encoding='utf-8') as f:
        json.dump(payload, f, ensure_ascii=False, indent=1)

    L = []
    L.append('# 退化手眼标定 · 残差报告\n')
    L.append('- 样本：`%s`' % samples_path)
    L.append('- 参数：`%r`' % params)
    L.append('- 主点：%s（u0=%.1f, v0=%.1f）'
             % ('参与拟合' if rep['fit_principal'] else '固定画面中心',
                params.u0, params.v0))
    L.append('')
    L.append('| 集合 | n | px_rms | px_max | cm_rms | cm_max |')
    L.append('|---|---|---|---|---|---|')
    for name, s in (('拟合集', rep['fit']), ('留出集（真判据）', rep['test'])):
        L.append('| %s | %d | %.3f | %.3f | %.4f | %.4f |'
                 % (name, rep['n_fit'] if name == '拟合集' else rep['n_test'],
                    s['px_rms'], s['px_max'], s['cm_rms'], s['cm_max']))
    L.append('')
    L.append('## 逐条残差\n')
    L.append('| # | 集 | px | cm | 框宽 px | 涨 | 可疑 |')
    L.append('|---|---|---|---|---|---|---|')
    for i, (a, b) in enumerate(zip(rep['fit'].get('px', []),
                                   rep['fit'].get('cm', []))):
        L.append('| %d | 拟合 | %.3f | %.4f | %s | %s | %s |'
                 % (i + 1, a, b,
                    _mc(meta, i, 0), _mc(meta, i, 1), _mc(meta, i, 2)))
    off = rep['n_fit']
    for k, e in enumerate(rep['per_sample']):
        i = off + k
        L.append('| %d | 留出 | %.3f | %.4f | %s | %s | %s |'
                 % (i + 1, e['px'], e['cm'],
                    _mc(meta, i, 0), _mc(meta, i, 1), _mc(meta, i, 2)))
    L.append('')
    sens = rep.get('sensitivity')
    if sens:
        L.append('## 参数敏感度\n')
        L.append('标记点高度每量错 1 cm，参数会跑多少 —— 参数只在这个量级上可信：\n')
        L.append('- `df/f = %+.2f px/cm (%.2f%%)`' % (sens['df_per_cm'],
                  100.0 * sens['df_per_cm'] / params.f))
        L.append('- `dt = %+.2f cm/cm`' % sens['dt_per_cm'])
        L.append('- `du* = %+.2f px/cm`, `dv* = %+.2f px/cm`'
                 % (sens['dus_per_cm'], sens['dvs_per_cm']))
        L.append('')
        L.append('> ⚠️ 残差绿色**不代表**标记点量对了：实测 z 偏 3 cm 时残差照样'
                 '通过，但 f 已经偏 12%。')
        L.append('')
    L.append('## 判定\n')
    L.append('- 判据：`px_rms(留出) < %.1f px` 且 `cm_max(留出) < %.1f cm`'
             % (PX_RMS_PASS, CM_MAX_PASS))
    L.append('- 实际：`px_rms = %.3f px`，`cm_max = %.3f cm`'
             % (rep['px_rms'], rep['cm_max']))
    L.append('- 结论：**%s**' % ('✅ 通过' if passed else '❌ 不通过'))
    L.append('')
    with open(mpath, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L))
    return jpath, mpath


def main(argv=None):
    ap = argparse.ArgumentParser(description='退化手眼标定残差报告')
    ap.add_argument('--samples', required=True, help='samples-*.json 路径')
    ap.add_argument('--holdout', type=int, default=2,
                    help='末尾留出几条只用于报残差（默认 2）')
    ap.add_argument('--fit-principal', action='store_true',
                    help='连主点 u0/v0 一起拟合（5 -> 7 个参数）')
    ap.add_argument('--no-write', action='store_true', help='只打印不落盘')
    args = ap.parse_args(argv)

    params, rep, meta, _ = build_report(args.samples, holdout=args.holdout,
                                        fit_principal=args.fit_principal)
    passed = print_report(args.samples, params, rep, meta)
    if not args.no_write:
        j, m = write_report(args.samples, params, rep, meta, passed)
        print('已写出: %s' % j)
        print('已写出: %s' % m)
    return 0 if passed else 3


if __name__ == '__main__':
    sys.exit(main())
