# -*- coding: utf-8 -*-
"""P3 结果反解：由「爪尖在瓶盖上时的瓶盖像素」直接反算**手眼横向偏移 (a,b)**。

原理（推导文档 §5.1）：爪尖落在目标上时，Δ = 爪尖 − 光心 = (−a, −b, |c|)，
而它在工具系里的方向就是那条像素射线。所以

    a = c · d_Tx ,   b = c · d_Ty        （c = 负的沿轴距离，d_T 是像素射线在工具系里的方向）

**这条完全不依赖 FK** —— P3 本来就不依赖 FK（瞄准像素是常数）。
"""
import json
import math
import sys

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
from arm_grasp import geom
from arm_grasp.arm_kin import fk, from_fields
from arm_grasp.cam_model import tool_axes

P3 = '/home/cy/raspot_ws/src/arm_grasp/docs/p3.json'
K = geom.K_AI320x180_CHN2
entries = json.load(open(P3, encoding='utf-8'))
print('共 %d 条\n' % len(entries))
print(' #  α      tip(cm)              r_tip   瓶盖AI像素      反算 a,b(cm)     与实测比')
print('                                                    (实测 a=-0.75 b=+3.40)')
for i, e in enumerate(entries):
    j = from_fields(e['fb'])
    tip_cm, axis = fk(j)
    xh, yh, zh = tool_axes(tip_cm, axis)
    r_tip = math.hypot(tip_cm[0], tip_cm[1])
    cu, cv = e['cap_px_stream']
    ua, va = geom.to_ai(cu, cv)
    uu, vv = geom.undistort(ua, va)
    dTx = (uu - K.cx) / K.fx
    dTy = (vv - K.cy) / K.fy
    c = geom.CAM_OPEN[2]                       # −0.1190（张开态）
    a, b = c * dTx, c * dTy                    # 米
    # 判定「爪尖是否真的在瓶盖上」：光心到爪尖的方向 vs 那条射线（工具系里比）
    d = (tip_cm[0] - (geom.camera_center(j)[0] * 100),
         tip_cm[1] - (geom.camera_center(j)[1] * 100),
         tip_cm[2] - (geom.camera_center(j)[2] * 100))
    DT = (sum(d[k] * xh[k] for k in range(3)),
          sum(d[k] * yh[k] for k in range(3)),
          sum(d[k] * zh[k] for k in range(3)))
    # 归一化到 d_Tz = 1 再比
    ang = math.degrees(math.acos(max(-1, min(1, (
        DT[0] * dTx + DT[1] * dTy + DT[2] * 1.0) /
        (math.sqrt(sum(x * x for x in DT)) * math.sqrt(dTx**2 + dTy**2 + 1))))))
    print(' %d %6.1f  (%7.2f,%7.2f,%6.2f) %6.2f  (%7.1f,%7.1f)  (%+6.2f,%+6.2f)   %+6.2f / %+6.2f'
          % (i + 1, e['alpha'], tip_cm[0], tip_cm[1], tip_cm[2], r_tip, ua, va,
             a * 100, b * 100, a * 100 - (-0.75), b * 100 - 3.40))
    print('      「爪尖方向 vs 瓶盖视线」夹角 = %.1f°   (≈0 才说明爪尖真在瓶盖上)' % ang)
