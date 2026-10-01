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


# --------------------------------------------------------------------------
# BoxFollower（2026-10-01 复查补的）：参考框要**跟着目标走**，不能一直是人画的那个
# --------------------------------------------------------------------------

WANT = [0.40, 0.40, 0.50, 0.55]


def _track(box):
    return {"cls": "obj", "score": 0.9, "box": box, "track_id": 1, "src": "track"}


def _det(box, cls="obj"):
    return {"cls": cls, "score": 0.5, "box": box}


def test_follower_keeps_the_track_after_the_camera_moved_it_away_from_want():
    """★ 臂（相机）一动，目标在画面里就挪走了 ⇒ 跟人画的框 IoU 掉到 0。

    旧写法每拍拿**人画的框**去比 IoU ⇒ 板子明明一直在发 `src=track` 的框，Pi 这边整条扔掉
    （"相机一动就丢"有一部分是自己丢的）。板子锁的就是这个目标（跟踪器一次只跟一个），
    ⇒ 有跟踪框就认它。
    变异（已实测）：把"有跟踪框就认它"那支关掉（`if tracks:` → `if False:`）⇒ 这条红。
    """
    moved = [0.70, 0.20, 0.80, 0.35]                    # 与 WANT 完全不重叠
    assert observe.iou(moved, WANT) == 0.0
    assert observe.pick_box({"objs": [_track(moved)]}, WANT) is None   # 旧行为：丢
    f = observe.BoxFollower(WANT)
    b = f.pick({"objs": [_track(moved)]})
    assert b is not None and b['box'] == moved
    assert f.ref == moved                                # 参考框跟过去了


def test_follower_falls_back_to_iou_against_the_last_box_for_detections():
    """跟踪丢了（只有检测框）⇒ 按**上一次接受的框**比 IoU，不是按人画的框；
    不重叠的检测框（别的东西）不许认。
    变异（已实测）：去掉 `self.ref = ...` 那行 ⇒ 第二步拿 WANT 比 ⇒ 红。
    """
    f = observe.BoxFollower(WANT)
    step1 = [0.60, 0.40, 0.70, 0.55]
    assert f.pick({"objs": [_track(step1)]})['box'] == step1
    near1 = [0.61, 0.41, 0.71, 0.56]                     # 跟踪丢了，检测器在原地附近认出它
    assert observe.iou(near1, WANT) < 0.2
    b = f.pick({"objs": [_det(near1), _det([0.0, 0.0, 0.1, 0.1], "chair")]})
    assert b is not None and b['box'] == near1
    assert f.pick({"objs": [_det([0.0, 0.0, 0.1, 0.1], "chair")]}) is None
    assert f.ref == near1                                # 没找到时参考框不动


def test_fresh_result_aged_reports_file_age(tmp_path):
    """观测时刻要用"现在 − 文件年龄"往回推（GraspLink）——年龄必须是 mtime 算出来的真值。"""
    import json as _j
    import os as _os
    import time as _t
    p = tmp_path / 'r.json'
    p.write_text(_j.dumps({"frame": 3, "objs": []}))
    old = _t.time() - 0.8
    _os.utime(str(p), (old, old))
    r, age = observe.fresh_result_aged(str(p), max_age_s=1.5)
    assert r['frame'] == 3
    assert 0.75 <= age <= 1.2, age
    assert observe.fresh_result_aged(str(p), max_age_s=0.5) is None     # 太旧
    assert observe.fresh_result(str(p), max_age_s=1.5)['frame'] == 3     # 老接口不变
