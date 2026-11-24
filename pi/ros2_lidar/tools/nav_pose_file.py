"""AMCL 位姿存盘的路径规则：**一张地图一个文件**。

2026-11-24 之前是 `maps/last_pose.json`，整个目录共用。后果：换一张地图
导航时，auto_localize 会把上一张地图的位姿拿来校验；位姿恰好能过阈值就
直接恢复，机器人于是"以为"自己在另一张图里的位置（当天实测：错位 4 m、
147°，打分 0.12 仍低于 0.25 被接受）。

现在的默认：`<map 目录>/<map 文件名>_last_pose.json`。
`pose_file` 显式给出时优先；连地图都没有（单独跑 pose_persist）时保留旧路径。
"""
import os

LEGACY = '/home/cy/ros2_lidar/maps/last_pose.json'


def pose_path(map_yaml, pose_file=''):
    if pose_file:
        return pose_file
    if map_yaml:
        stem = os.path.splitext(os.path.basename(map_yaml))[0]
        return os.path.join(os.path.dirname(os.path.abspath(map_yaml)),
                            stem + '_last_pose.json')
    return LEGACY
