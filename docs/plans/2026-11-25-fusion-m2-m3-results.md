# 融合导航 + 自主探索 — M0~M3 实车结果（2026-11-25）

> **状态**：M0/M1/M2/M3 已实车验证；代码 `3ae92dd` 已 push。
> **场地**：室内房间（非长走廊）；电池从 12.39 V 跑到 11.55 V，结束时主动停栈充电。
> **结论**：融合模式、点未知区分步推进、自主探索、边界线均可用；
> 关键坑不在 nav2 StaticLayer，而在**选点的 clearance / 前沿 approach**。

---

## 1. 形态与代码

| 项 | 结果 |
|---|---|
| 模式 | `fusion`：slam_toolbox 提供 `map→odom`，nav2 只跑规划/控制 |
| 节点 | `/slam_toolbox`、`controller_server`、`planner_server`、`bt_navigator`、`collision_monitor`、`fusion_nav` |
| 雷达 | `/scan_filtered` 约 6.2~6.9 Hz |
| 速度链 | `no_pivot_guard`（融合模式 `allow_slow_pivot=true`, 0.35 rad/s）→ collision_monitor → cmd_mux |
| 终点朝向 | `yaw_goal_tolerance=3.15`（π）：融合模式只到位置、不管朝向 |
| 代码 | `feat/arm-voice-motion`，最新 `3ae92dd`，已 push；Pi 用 bundle fast-forward 同步 |

---

## 2. M0 / M1（已知点、未知点、存图）

| 场景 | 结果 |
|---|---|
| 第一个 direct 点 (-0.35,-0.74) | 成功；地图 9.83→20.06 m²；1.48 m |
| 硬禁原地转下的 U 型调头 | **死锁**：`guard clamped=231`、`pivot_zeroed=40` 仍不前进 |
| 放开融合原地转后 | `allow_slow_pivot=true`；`pivot_allowed` 持续增加、`pivot_zeroed=0`；调头成功 |
| direct 腿到达判定 | `legs=1`、`reason=arrived`；不再在目标附近反复重磨 |
| 未知区 (-5.54,1.49) | 自动选中间腿 (-3.24,0.35) → (-5.54,0.80)；地图 39.75→41.41 m²；最终点被墙挡住后 `no_reachable_frontier` 安全停 |
| 存图 + 位姿 | `fusion_m0/m1/m1b/m1c/m2a/m2b/m2_20261125` 全部写入 yaml/pgm/png + `_last_pose.json` |

**U 型调头的结论**：导航模式保持硬禁；融合/探索模式允许慢速原地转。这是设计 D3 的实车修正。

---

## 3. M2 自主探索

在开阔区（PS2/人工 direct 先把车挪出 9 m² 角落）开启探索：

| 指标 | 值 |
|---|---|
| 探索腿数 | 4 |
| 地图 free 面积 | **28.27 → 44.55 m²** |
| 用时 | 75.2 s（到 `no_frontier`） |
| 里程 | 5.49 m |
| 规划失败 / stop_zone / 卡死 | **0 / 0 / 0** |
| 结束原因 | `no_frontier`（剩余 7 个前沿在墙侧，approach/兜底都选不出可达点） |
| 结束位姿 | (-4.58, 1.46)，朝向西侧 |
| 电量 | 11.556 V（结束时） |

车从 (0.08, 0.34) 自主走到 (-4.58, 1.46)，沿途地图持续增长；
`pivot_allowed` 17、`pivot_zeroed=0`。**在开阔区，探索器是可用的。**

---

## 4. M3 边界线（L1）

| 步骤 | 结果 |
|---|---|
| 设界线 | anchor=(-4.58,1.46)，segment `[[-2.0,-5.0],[-2.0,10.0]]` |
| 线外点击 (0,1.5) | HTTP 接收，但 `fusion_nav` 拒绝：`state=blocked, reason=outside_boundary`；车**完全没动** |
| 探索前沿过滤 | 界线打开后 `frontiers` **7 → 5**（线东的前沿被丢弃） |
| 清除 | `anchor:null, segments:[]` → `GET /api/boundary` 返回 `null` |

L1 只覆盖探索目标和人工点击；L2 KeepoutFilter 仍未做（见 §7）。

---

## 5. 五个真实坑（按发现顺序）

### 5.1 StaticLayer 是“假警报”

- 现象：`/map` 已 295×159，但 `costmap` / `static_layer` **话题**读到 284×144、robot/leg2 为 -1。
- 真相：日志持续 `StaticLayer: Resizing costmap to 295 X 159`；
  `/global_costmap/get_static_layer` 服务返回 **295×159、原点与 `/map` 一致、目标格 0（free）**。
- 话题读到的是 **transient_local 缓存旧帧**；`map_subscribe_transient_local` 运行时修改被节点明确拒绝（非动态参数）。
- 教训：**不要用带 transient_local 缓存的 costmap 话题断言内部状态**；用 `get_static_layer` / `get_costmap` 服务。

### 5.2 分步选点 clearance 0.35 → 0.6

- 现象：手动点未知区，中间腿选出来后 Smac `error_code=208 / no valid path found`。
- 证据：`get_cost_global_costmap` 在 leg2 返回 **253（inscribed）**，robot/leg1 为 0；
  mid3/mid5 也是 253，mid4 202。
- 原因：`choose_intermediate` 用了 0.35，而设计 §5.2 要求 0.6；选到了障碍物内切区。
- 修复：`clearance=0.6`；新增 `test_choose_intermediate_respects_inflation_clearance`。

### 5.3 前沿边缘在 unknown 膨胀区

- 现象：探索第一条腿 `no valid path`，前沿点 (-0.30,1.70) 的 master cost = 253。
- 代价曲线：前沿 253 → 退 0.9 m 245 → 退 1.2 m 141 → 车下 0。
- 原因：不能直接把前沿边缘当目标；unknown/障碍的膨胀把边缘压到 253。
- 修复：`choose_frontier_goal` 从簇质心朝机器人退 0.6~1.2 m，
  要求 `unknown_distance>=0.6`、`obstacle_distance>=0.6`、`line_free`。

### 5.4 簇质心与机器人重合

- 现象：紧凑房间/走廊的环形前沿，质心正好在机器人位置；`dist<1e-6` 把整个簇
  `continue` 掉，兜底没跑，探索直接 `no_frontier`。
- 修复：质心重合时只跳过 approach，仍走“簇内自由格”兜底（0.6 → 0.36 两档）；
  新增 `test_choose_frontier_goal_falls_back_in_tight_map`。

---

### 5.5 雷达射程末端的假墙

- 现象：开阔区雷达射程末端（约 12 m）出现红点，多次后在地图上变成一堵弧墙。
- 原因：RPLIDAR A1M8 对“无回波”返回**有限** ~12 m，slam_toolbox 把它当成真障碍；
  `max_laser_range: 12.0` 正好等于这个值，所以没有拒绝。
- 修复：`scan_self_filter.py` 把 >= 11.5 m 的有限值改成 `inf`，输出 `range_max=11.5`；
  `slam_params.yaml` / `slam_params_fusion.yaml` 的 `max_laser_range` 12.0 → 11.5。
- 影响：仓库副本（融合、仓库版建图/导航）；`/home/cy/ros2_lidar` 旧副本未改。
- 旧地图里的假墙不会消失，需要重建。

## 6. 诊断手法（可复用）

| 目的 | 命令/服务 |
|---|---|
| 真实占地/代价图 | `/global_costmap/get_static_layer`、`/global_costmap/get_cost_global_costmap` |
| 全局规划拒绝原因 | `ComputePathToPose` action；看 `error_code` 与 planner log |
| 扫描延迟 | `ros2 topic delay /scan_filtered`（实测 0.01~0.02 s；0.5 s 警告是负载尖峰） |
| 地图发布 | `ros2 topic info /map -v`：单发布者 slam_toolbox，RELIABLE+TRANSIENT_LOCAL |
| 模式探测 | `pgrep -f /lib/slam_toolbox/` + `/lib/nav2_controller/`，避免裸包名误命中 shell |

---

## 7. 未做 / 后续（M4）

1. **L2 KeepoutFilter**：把边界线做成 nav2 costmap filter 掩码，
   让**规划层**也不能越线（L1 只拒绝点击/探索目标）。
2. **posegraph 续建**：存图时序列化 slam_toolbox posegraph，下次在已有图上继续建，
   不用每次从零开始。
3. **低电自动停车/返航**：本轮电量 11.55 V 前靠人工盯；M4 接 `/battery_voltage`，
   <11.3 V 停探索、<11.0 V 原地停（返航另议）。
4. **局部图/膨胀调参**：4×4 局部图、窄口 inflation 0.5 的实车验证。
5. 小尾巴：`fusion_nav` 停止时 `rcl_shutdown already called` 的 exit 1；
   页面把 `outside_boundary`/`no_reachable_frontier` 等原因码翻成中文；
   `area_stalled` 在 `no_frontier` 静止 60 s 后自动关探索，需更明确的状态。
6. 旧 nav 20 次往返回归按用户决定**跳过**（验证缺口，记在这里）。

---

## 8. 最终状态

- 车已 idle，融合栈已停（`mode=idle`），`upper` 服务仍在；
- 电量 **11.55 V**，先充电；
- 7 张地图在 `~/ros2_lidar/maps/fusion_*_20261125.*`；
- 代码 `origin/feat/arm-voice-motion` = `3ae92dd`。
