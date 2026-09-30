# -*- coding: utf-8 -*-
"""observe 的纯逻辑测试（不需要 ROS / 不需要板子）。

夹具照 `5.K230/sdcard/k230vision/` 的**真实** schema 写
（检测条目没有 `src`；跟踪条目有 `src:"track"` + `track_id:1`）。

⚠️ 与计划 Task 7 Step 1 的一处**有意改动**：计划里 bottle 的框 [0.10,0.10,0.20,0.30]
   面积比 chair 的小 ⇒ "优先 track" 那条用例**改不改实现都是绿的**（够不着变异）。
   计划自己也写了这条要注意（"bottle 的面积不一定最大"），所以这里把 bottle 框放大成
   全场最大的一个 —— 这样"优先 track"才是真的被测到（`pool = tracks or objs` 一改就红）。
"""
import pytest

from arm_grasp import observe

# 检测条目：**没有 `src`**     跟踪条目：`src == "track"` + `track_id`
R = {"w": 1280, "h": 720, "frame": 7,
     "objs": [{"cls": "bottle", "score": 0.41, "box": [0.10, 0.10, 0.60, 0.50]},
              {"cls": "chair", "score": 0.9, "box": [0.80, 0.44, 0.86, 0.84],
               "track_id": 1, "src": "track"}]}


def test_pick_box_prefers_track():
    """没给 want 时优先跟踪框 —— 即使检测框更大（bottle 面积 0.20 > chair 0.024）。"""
    b = observe.pick_box(R)
    assert b['src'] == 'track'


def test_pick_box_matches_want_by_iou():
    b = observe.pick_box(R, want_norm=[0.10, 0.10, 0.60, 0.50], iou_min=0.5)
    assert b['cls'] == 'bottle'
    b2 = observe.pick_box(R, want_norm=[0.0, 0.0, 0.01, 0.01], iou_min=0.5)
    assert b2 is None                      # 都不重叠 ⇒ 明确返回 None，不是随便给一个


def test_pick_box_prefers_track_when_both_boxes_overlap_want():
    """want 同时和两个框重叠且**都达标**时，仍取跟踪框（它连续、抖动小）。

    这里检测框的 IoU(0.325) 比跟踪框的 IoU(0.281) **更大** —— 所以"取 IoU 最大"
    这一条单独解释不了结果，必须真的实现了"优先 track"才拿得到 chair。
    """
    r = {"objs": [{"cls": "bottle", "score": 0.5, "box": [0.00, 0.00, 0.50, 0.50]},
                  {"cls": "chair", "score": 0.9, "box": [0.35, 0.35, 0.75, 0.75],
                   "track_id": 1, "src": "track"}]}
    b = observe.pick_box(r, want_norm=[0.15, 0.15, 0.65, 0.65], iou_min=0.2)
    assert b['src'] == 'track'


def test_box_center_ai_is_uniform_div4():
    u, v = observe.box_center_ai([0.4, 0.5, 0.6, 0.7])
    assert (u, v) == pytest.approx(((0.4 + 0.6) / 2 * 1280 / 4,
                                    (0.5 + 0.7) / 2 * 720 / 4))


def test_iou_basics():
    assert observe.iou([0, 0, 1, 1], [0, 0, 1, 1]) == pytest.approx(1.0)
    assert observe.iou([0, 0, 1, 1], [2, 2, 3, 3]) == pytest.approx(0.0)


def test_k230_cmd_returns_error_dict_instead_of_raising():
    """8557 的异常路径：**必须返回 dict**（T9 的 finally 要靠它收尾，不能抛）。

    本地没有板子，就朝一个必定拒绝连接的端口发一条 —— 走的就是 OSError 那条分支。
    """
    r = observe.k230_cmd('127.0.0.1', {'cmd': 'stop'}, port=1, timeout=0.5)
    assert isinstance(r, dict)
    assert r['ok'] is False and 'err' in r
