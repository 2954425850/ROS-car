"""vision/locate + tools/track（离线：假拍照、假画框、假板子）。"""

from tools.registry import ToolRegistry
from tools.track import register_track_tools
from vision.locate import Locator, parse_box


def test_parse_box_scales_and_rejects():
    assert parse_box('{"found": true, "box": [337, 476, 391, 615]}') == (0.337, 0.476, 0.391, 0.615)
    assert parse_box('{"found": false, "box": [0,0,0,0]}') is None
    assert parse_box("找不到") is None
    assert parse_box('{"box": [500, 500, 501, 501]}') is None          # 退化框
    assert parse_box('```json\n{"box":[0,0,1200,1000]}\n```') == (0.0, 0.0, 1.0, 1.0)


class _Snap:
    def snap(self):
        return "/tmp/x.jpg"


class _Boxer:
    def __init__(self, box):
        self.b = box

    def box(self, path, q):
        return self.b, "raw"


class _Target:
    def __init__(self):
        self.boxes = []

    def box(self, l, t, r, b):
        self.boxes.append((l, t, r, b))
        return "ok"


def test_track_object_sends_vlm_box_to_board():
    tg = _Target()
    reg = ToolRegistry()
    register_track_tools(reg, Locator(_Snap(), _Boxer((0.3, 0.4, 0.4, 0.6))), tg, None)
    out = reg.dispatch("track_object", {"target_desc": "黄色的辣椒"}).result
    assert tg.boxes == [(0.3, 0.4, 0.4, 0.6)] and "盯住" in out


def test_track_object_not_found_does_not_touch_board():
    tg = _Target()
    reg = ToolRegistry()
    register_track_tools(reg, Locator(_Snap(), _Boxer(None)), tg, None)
    out = reg.dispatch("track_object", {"target_desc": "独角兽"}).result
    assert tg.boxes == [] and "没能找到" in out


def test_grasp_tool_routes_box_to_client():
    from tools.grasp import register_grasp_tools
    class _G:
        def __init__(self):
            self.boxes = []
        def grasp(self, box):
            self.boxes.append(box)
            return "抓好了"
    class _Snap:
        def snap(self):
            return "/tmp/x.jpg"
    class _Boxer:
        def box(self, p, q):
            return (0.1, 0.2, 0.3, 0.4), "raw"
    g = _G()
    reg = ToolRegistry()
    register_grasp_tools(reg, Locator(_Snap(), _Boxer()), g)
    assert reg.dispatch("grasp_object", {"target_desc": "辣椒"}).result == "抓好了"
    assert g.boxes == [(0.1, 0.2, 0.3, 0.4)]
