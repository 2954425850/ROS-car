# 2D 导航（nav2 + AMCL）— 2026-11-17 首次真车

在 `mapping.launch.py` 建完图之后使用。SLAM 与导航**不能同时跑**：两者都发
`map->odom`。

```
ros2 launch /home/cy/ros2_lidar/launch/nav.launch.py
```

## 这次启动了什么

| 节点 | 作用 |
|---|---|
| `lidar_reset.py`（前置） | STOP + RESET 雷达。上个驱动退出后雷达常停在扫描状态，新驱动会卡在 `getDeviceInfo`（没有 S/N、没有 `/scan`） |
| `rplidar_composition` | 雷达，7 Hz；参数与 mapping.launch 相同（`auto_standby=false`、by-id 串口） |
| `scan_self_filter.py` | `/scan` → `/scan_filtered`，滤掉车体 footprint 内的点 |
| `robot_state_publisher` | `base_footprint -> base_link -> laser`（urdf/robot.urdf） |
| `nav2_bringup/localization_launch.py` | `map_server` + `amcl` + lifecycle |
| `nav2_navigation_launch.py`（本地改过的副本） | controller / planner / bt_navigator / behaviors / smoother / collision_monitor |

速度链路（`cmd_mux` 里导航优先级最低，10）：

```
controller_server -> cmd_vel_raw -> velocity_smoother -> cmd_vel_smoothed
  -> collision_monitor -> cmd_vel_nav -> cmd_mux -> cmd_vel_mux -> 驱动
```

> `nav2_navigation_launch.py` 是把官方文件里的 `cmd_vel -> cmd_vel_nav` 改成了
> `cmd_vel -> cmd_vel_raw`，这样最终输出能由 collision_monitor 收口到
> `/cmd_vel_nav`，不会落到 `/cmd_vel`（那是 legacy，优先级 80，会压过语音）。

## 必须按次修改的参数

`config/nav2_params.yaml` → `amcl.initial_pose`：

```yaml
set_initial_pose: true
initial_pose: {x: -1.157, y: 0.556, z: 0.0, yaw: 0.0}   # 2026-11-17 那次
```

换位置起车必须改成当次位姿，否则 AMCL 从错的位置开始。

## 地图

`maps/nav_20261117.pgm/.yaml`：553×139 @0.05 m（27.6 × 7.0 m），
unknown 82.8% / free 15.4% / occupied 1.8%（正确保存，204→205 未知区）。
同一目录下的 `.posegraph/.data` 是 slam_toolbox 序列化图（只在 Pi 上，未入库，8 MB）。

## 今天踩到的五个坑（都已修）

1. **`bt_navigator` 的 action 超时默认只有 20 ms**
   官方 `default_server_timeout`。DDS 发现稍慢就 abort 整个目标，日志是
   `Timed out while waiting for action server ... compute_path_to_pose`。
   → 改成 `2000`。
2. **机械臂被雷达当成障碍**
   雷达在车尾（-0.256 m），机械臂在车头 → `/scan` 的 0° 有固定回波 **0.44 m**，
   落在 collision_monitor 的停车区 → 速度永远为 0。
   → `scan_self_filter.py` 把车体 footprint（0.60×0.40 + 2 cm）内的点置 `inf`，
   AMCL / costmap / collision_monitor 全部改用 `/scan_filtered`。
3. **雷达重启后卡在初始化**
   → 前置 `lidar_reset.py`（STOP + RESET，DTR 拉低）。
4. **Fast DDS SHM 端口残留**
   反复重启 nav2 后出现 `RTPS_TRANSPORT_SHM ... portNNNN open_and_lock_file failed`，
   服务调用要 30~40 s。→ launch 里 export `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
5. **CPU 过载**
   Pi 4 核跑到 load 17.8：controller 实际只有 5~15 Hz（目标 15），
   collision_monitor 报 scan 时间戳落后 0.7 s，最后 **behavior_server 4 s 没心跳，
   lifecycle manager 把整组节点关掉** → 返回目标 ABORTED。
   建议：控制频率降到 10 Hz、少起 route/docking/smoother、跑完一轮重启 Pi 清 DDS。

## 2026-11-17 实测结果

- 地图：走廊 27.6 × 7.0 m，unknown 82.8%，建图 30 分钟零 warning/error
- 去程：从 `(-1.157, 0.556)` 到 `(4.0, 0.45)`，**SUCCEEDED**，停车误差 ~5 cm
- 返程：目标接受后走到 `(0.34, -0.03)`，因 `behavior_server` 心跳丢失 ABORTED
- 电压：开始时 12.36 V，结束时 11.67 V（11.2 V 以下性能减半，跑之前充满）
