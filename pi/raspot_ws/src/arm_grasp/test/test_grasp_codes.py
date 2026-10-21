# -*- coding: utf-8 -*-
"""grasp_codes：grasp.py 里每一种停止原因都要归到一个码（不能落到 ERROR）。"""
import re
from pathlib import Path

from arm_grasp.grasp_codes import code_for


def test_known_reasons():
    assert code_for('抬升完成：s=55mm', True) == 'OK'
    assert code_for('进 descend 前从未拿到有效深度（盲区）', False) == 'NO_DEPTH'
    assert code_for('refuse：目标 field 出界 [..]', False) == 'OUT_OF_REACH'
    assert code_for('refuse：IK 无解', False) == 'REFUSED'
    assert code_for('合爪超时兜底：…', False) == 'CLOSE_TIMEOUT'
    assert code_for('超时（150s）', False) == 'TIMEOUT'
    assert code_for('卡住：连续 25 拍臂没动', False) == 'STUCK'
    assert code_for('观测丢失 5.2s（接近阶段丢了就停下）', False) == 'OBS_LOST'
    assert code_for(None, False) == 'TIMEOUT'


def test_every_stop_reason_in_grasp_py_is_classified():
    src = (Path(__file__).resolve().parents[1] / 'arm_grasp' / 'grasp.py').read_text(encoding='utf-8')
    reasons = re.findall(r"(?:\['stopped'\]\s*=\s*\(?|advance_or_stop\()\s*['\"]([^'\"%]{2,})", src)
    assert reasons, '没从 grasp.py 里抓到任何 stopped = ... —— 正则要跟着改'
    for r in reasons:
        ok = r.startswith(('对准完成', '扎到位', '合爪完成', '抬升完成'))
        if ok or r.startswith('下扎到机械极限'):     # 后者只是转合爪，不是终止原因
            continue
        assert code_for(r, False) != 'ERROR', f'没归类的停止原因：{r!r}'
