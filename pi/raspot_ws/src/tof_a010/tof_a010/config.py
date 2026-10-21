# -*- coding: utf-8 -*-
"""配置**单一真源**：`config/tof.yaml`。

节点（`tof_node.py`）和抓取链路（`tof_client.py`，被 `arm_grasp` 用）读**同一个文件**。
—— 分成两处一定会飘：标定完改了 `map` 却只改了节点那份，抓取用的还是旧映射，
症状是"节点里看着对、抓取落点却偏"，而且极难查。

查找顺序（先命中先用）：
1. 环境变量 `TOF_A010_CONFIG`
2. **源码树** `/home/cy/ROS-car/pi/raspot_ws/src/tof_a010/config/tof.yaml`
3. `ament_index` 的 share 目录（`colcon build` 装出来的那份）

## ★ 为什么**源码树排在 share 前面**

`colcon build` 会把 yaml **拷**一份到 `install/.../share/tof_a010/config/`。
节点是走 `ament_index` 找配置的，所以默认会读**那份副本** —— 于是出现这个坑：
你改了源码的 `map`，`tof_client`（跑在源码树里）立刻生效，
`tof_node`（跑在 install 里）还在用旧的 ⇒ **"节点里对、抓取偏"**，
而这正是整个包设计要避免的"两处真源"问题。

标定时 `map` 要来回改几十次，每次都 `colcon build` 既不现实也迟早会忘。
所以：**两边都以源码那份为准**，share 只当没有源码树时的兜底。
"""
from __future__ import annotations

import os

try:
    import yaml
except ImportError:                    # pragma: no cover
    yaml = None

# 本机工作区的源码配置（这台 Pi 上就是这个路径）
REPO_YAML = "/home/cy/ROS-car/pi/raspot_ws/src/tof_a010/config/tof.yaml"

DEFAULTS = {
    "device": "/dev/a010",
    "baud": 115200,
    "unit_mm_per_count": 4.0,          # mm/计数（AT+UNIT）。见 a010 模块头 ★
    "disp_mode": 3,                    # ★ USB 路径要的是 3 = LCD(1)|USB(2)，不是 5
    "binn": 1,                         # 1 = 100x100 满分辨率
    "frame_id": "tof",
    "min_mm": 150.0,                   # 量程下限（近场假回波）
    "val_max": 254,                    # > 254 当无效（255 = 饱和）
    # ToF 传感器平面 → 相机光心的基准差（mm）。**缺省 0**：yaml 没读到时不偷偷加偏置。
    "offset_mm": 0.0,
    "pad": 0.06,                       # 框四周余量（归一化）
    "map": {"ax": 1.0, "bx": 0.0, "ay": 1.0, "by": 0.0},
    "exclude": [],                     # ToF 归一化排除区列表 [[l,t,r,b], ...]
    "fx": 71.4, "fy": 86.6,            # 点云用；由 FOV H70°xV60° 在 100x100 上推得
    "cx": 49.5, "cy": 49.5,
    "publish_cloud": False,            # rviz 调试时才开
    "max_age_s": 0.25,                 # 深度的**新鲜度上限**；超龄当没拿到
}

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_YAML = os.path.join(os.path.dirname(_HERE), "config", "tof.yaml")


def _share_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("tof_a010"), "config", "tof.yaml")
    except Exception:                  # noqa: BLE001
        return None


def config_path(explicit=None):
    for p in (explicit, os.environ.get("TOF_A010_CONFIG"), REPO_YAML,
              _SRC_YAML, _share_yaml()):
        if p and os.path.isfile(p):
            return p
    return None


def _warn_if_shadowed(used):
    """两处都有、且内容不一致时喊一声 —— 这正是"改了源码却跑的是副本"那个坑。"""
    import sys
    share = _share_yaml()
    if not used or not share or os.path.abspath(used) == os.path.abspath(share):
        return
    if not os.path.isfile(share):
        return
    try:
        if open(used, "rb").read() != open(share, "rb").read():
            print("⚠️ tof 配置有两份且内容不同：\n"
                  "     在用 %s\n"
                  "     副本 %s（**没生效**；要同步就 colcon build）"
                  % (used, share), file=sys.stderr)
    except OSError:
        pass


def resolve_device(cfg):
    """把配置里的 `device` 变成一个**真的存在**的路径。返回 `(path, 说明)`。

    ★ 为什么要兜底：`/dev/ttyUSB*` 的序号会随插拔漂移，所以配置里想写 `/dev/a010`
      （自定义 udev 规则）。但那条规则要 root 装，**不该成为跑起来的前提**。

    `/dev/serial/by-id/...-if00-port0` 是 udev **自带**的稳定名（不用写任何规则），
    而 `if00` 正是 FT2232 的**通道 A** = 数据口（通道 B 是 `if01`）。
    所以顺序是：配置名 → by-id 的通道 A → 裸 /dev/ttyUSB0。
    """
    want = cfg.get("device") or "/dev/a010"
    if os.path.exists(want):
        return want, "配置指定"
    by_id = "/dev/serial/by-id"
    if os.path.isdir(by_id):
        import glob as _glob
        hits = sorted(_glob.glob(os.path.join(by_id, "*Meta_Sense_*if00*")))
        if hits:
            return hits[0], "by-id 通道 A（udev 自带，已自动兜底）"
    if os.path.exists("/dev/ttyUSB0"):
        return "/dev/ttyUSB0", "裸 /dev/ttyUSB0（**没找到 by-id，建议查一下**）"
    return want, "不存在（模块没插？）"


def load(path=None):
    """读配置。**缺项一律用 DEFAULTS 兜底**，不抛异常。

    ⚠️ 故意**不做严格校验**：标定途中你会反复手改这个文件，
    一个小笔误让抓取脚本直接崩掉，比拿一个偏了的默认值更烦人。
    """
    cfg = dict(DEFAULTS)
    cfg["map"] = dict(DEFAULTS["map"])
    cfg["exclude"] = list(DEFAULTS["exclude"])
    p = config_path(path)
    if p and yaml is not None:
        with open(p, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        for k, v in raw.items():
            if k == "map" and isinstance(v, dict):
                cfg["map"].update(v)
            else:
                cfg[k] = v
        cfg["_path"] = p
        _warn_if_shadowed(p)
    else:
        cfg["_path"] = None
    return cfg
