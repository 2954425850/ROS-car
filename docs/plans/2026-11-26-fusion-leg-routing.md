# 融合模式中间腿选点重做（2026-11-26）

> **状态**：代码 + 单测完成，待上 Pi 实车复核。
> **范围**：`pi/ros2_lidar/tools/fusion_plan.py`（只改分步追目标，探索前沿逻辑
> `choose_frontier_goal` 不动）、`config/nav2_params_fusion.yaml` 一项参数。

---

## 1. 现象

“边建边航”点一个未知区/地图外的点时，`fusion_nav` 会先在地图内选一个中间腿。
现场反馈：

1. 中间点常贴在已知墙边，Smac 报 `no valid path`（208），腿失败、黑名单重试，看起来卡住；
2. 中间点整体偏，走弯路；
3. 正前方明明能直线推进，却给出很大的转角。

## 2. 根因（对应旧 `choose_intermediate`）

| # | 根因 | 后果 |
|---|---|---|
| R1 | 候选打分只有“到最终目标的欧氏距离” | 必然选自由区里最贴近目标的那一格，也就是贴墙/贴 unknown 边界 |
| R2 | 只允许 60° 锥 + 直线可达（`line_free`） | 绕墙、绕噪点直接无解或被迫偏到侧面 |
| R3 | 中间点只查了离 occupied 0.6 m，没查离 unknown 的余量 | `allow_unknown=false` + `track_unknown_space` 下 unknown 是 lethal，footprint 压到 unknown 就 no valid path（M2/M3 §5.3 在前沿选点修过，中间腿漏了） |
| R4 | `clearance_ok` 只有 8 条射线 | 0.6 m 处相邻射线间隔 0.47 m，墙端/噪点会从两条射线之间漏过去 |
| R5 | `send_leg` 把“当前位置→中间点”的直线夹角写成终点朝向，而 Smac `use_final_approach_orientation=false` 会按这个朝向规划末段 | 绕弯/走廊腿末要多做一段掉头或倒车，表现成“很大一个转角” |

## 3. 方案

### 3.1 路线优先，不再锥内贪心

新增 `plan_route(grid, start, goal)`：free+unknown 上 8 邻域 A*。

- 不可通行：occupied、界外、黑名单格、**free 格**离 occupied < 0.22 m（unknown 格不做硬性贴墙限制：用户可能点在紧贴墙外侧的 unknown 里）；
- unknown 可通行但代价 +4/m，路线能指向未知区，同时有已知走廊时不抄近路压墙；
- 软代价：离墙 1 m 内按平方加价（`clear_weight=3`），路线走走廊中间；
- 自车 0.6 m 内豁免硬性贴墙限制（self filter 常把自车贴墙格留成 unknown）。

`choose_intermediate` 改为：

1. `plan_route` 出路线；
2. 沿路线取 `max_leg` 以内**最后一个** free 格，要求：
   - 离 occupied ≥ 0.6 m（24 射线）；
   - 离 unknown ≥ 0.5 m，够不着就退 0.35 → 0（紧凑场地/窄走廊不因此不出腿）；
   - 不越过第一个 unknown（不能先穿 unknown 再去取后面的点）；
   - 不穿用户画的边界、不进黑名单。
3. 返回的中间点就在实际路线上，朝向是路线切线，不再给 60° 锥外的点。

### 3.2 进度判据改成“沿路线剩余里程”

`Decision.remaining` = 从本腿终点沿路线到最终目标的剩余里程；`/fusion/status`
新增 `remaining` 字段。

- 绕墙时欧氏距离会**先变大**，旧 `last_dist` 判据会把正常绕行误判成
  `no_progress`（合成房间测试里 3 腿就被叫停）；
- 新判据：剩余里程减少 ≥0.2 m 才算推进；剩余里程**变长 >0.5 m** 说明新地图把
  原路线作废了（刚看到墙/门），这是信息增加，计数清零；只有“几乎不变”才累计
  stall，累计 3 次仍停 `no_progress`。

### 3.3 其他两项

- `clearance_ok` 射线 8 → 24（0.6 m 处间隔降到 0.157 m）。
- `nav2_params_fusion.yaml`：`use_final_approach_orientation: true`。融合模式
  `yaw_goal_tolerance=3.15`，本来就不管终点朝向；按实际接近方向收尾，腿末
  不再为目标 yaw 加机动。导航模式 `nav2_params.yaml` 不动（`yaw=0.2`）。

## 4. 参数（默认值）

| 参数 | 值 | 说明 |
|---|---|---|
| `min_leg` / `max_leg` | 0.9 / 3.0 m | 不变 |
| `clearance` | 0.6 m | 中间点离 occupied |
| `unknown_clearance` | 0.5 m | 中间点离 unknown；兜底 0.35 → 0 |
| `unknown_penalty` | 4.0 /m | A* 里走 unknown 的加价 |
| `clear_weight` / `clear_soft` | 3.0 / 1.0 m | 贴墙软代价 |
| `hard_clearance` | 0.22 m | free 格硬性贴墙下限（≈ 内切半径） |
| `escape_radius` | 0.6 m | 自车附近豁免 |
| `blacklist_radius` | 0.5 m | 不变；A* 现在会绕开黑名单圆 |

旧的 `cone_deg` / `step_cells` 参数已移除（只被 `choose_intermediate` 自己用）。

## 5. 验证

单测（`tests/test_fusion_plan.py`、`test_fusion_config.py`，共 50 项；全仓 84 项）：

- 新增：unknown 余量、直行不跑偏、绕出旧 60° 锥外的墙、0.8 m 窄走廊兜底、
  目标在 unknown 且紧贴墙格、路线变长不误判 `no_progress`、原地绕圈仍会
  `no_progress`、融合参数 `use_final_approach_orientation=true`。

真实地图 `maps/nav_20261124`，随机 66 个“直线无墙遮挡”的未知目标对比：

| 指标 | 旧 | 新 |
|---|---|---|
| 中间点离 occupied < 0.6 m | 23/36 | 3/38 |
| 平均离墙余量 | 0.52 m | 0.75 m |
| 平均方向偏差 | 15.3° | 15.6° |
| 能出腿的目标数 | 36/66 | 38/66 |

合成房间（中间隔墙 + 门洞，4 个目标）：旧算法 2 个 `no_reachable_frontier`；
新算法 4 个全部到达（其中绕门洞那条 9 腿 / 17.6 m，模拟里不再被误判卡住）。

性能：295×159（融合模式 @M3 的 `/map` 尺寸）单次选点约 0.04 s（PC）；
Pi 预计 0.2 s 级，且只在每条腿结束/换目标时调用一次（`replan` 有腿时直接返回）。

## 6. 上线与现场复核

1. Pi：`bash ~/ROS-car/pi/ros2_lidar/deploy_nav.sh`；
2. **必须重启融合栈**（nav2 参数改了，reload 不生效）；
3. 现场看：
   - 中间点落在走廊中间、方向顺；
   - `/fusion/status` 的 `remaining` 每次成功换腿后下降（允许因新地图变长）；
   - 不再出现中间点 `no valid path` / `leg_failed`；
   - 绕墙时不再提前 `no_progress`。
4. 回滚：`git revert` 本次提交；仅想恢复终点朝向行为可单独把
   `use_final_approach_orientation` 改回 `false`。
