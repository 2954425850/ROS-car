# -*- coding: utf-8 -*-
"""grasp.run 的中文停止原因 → 结构化错误码（纯函数，给 grasp_server / 语音 / 规划器用）。

原因字符串在 grasp.py 里各处 `stopped = '...'`；这里按前缀/关键词归类。
新增停止原因时要在这里补一条，test_grasp_codes 会拦"没归类"的情况。
"""

OK_PREFIXES = ('对准完成', '扎到位', '合爪完成', '抬升完成')

# (关键词, 码)，按顺序匹配第一个命中的
_RULES = (
    ('从未拿到有效深度', 'NO_DEPTH'),
    ('已测目标核验失败', 'NO_TARGET'),
    ('目标 field 出界', 'OUT_OF_REACH'),
    ('refuse', 'REFUSED'),
    ('合爪超时兜底', 'CLOSE_TIMEOUT'),
    ('超时', 'TIMEOUT'),
    ('卡住', 'STUCK'),
    ('观测丢失', 'OBS_LOST'),
)


def code_for(stopped, ok):
    if ok:
        return 'OK'
    if not stopped:
        return 'TIMEOUT'           # max_ticks 跑完、stopped 还是 None
    for key, code in _RULES:
        if key in stopped:
            return code
    return 'ERROR'
