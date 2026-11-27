#!/usr/bin/env bash
# 构建 ROS-car 的 nav2 overlay（只编两个包，不重编整套 nav2）。
#
# 补丁做了两件事（见同目录 nav2_overlay.patch）：
#   1) nav2_smac_planner：节点有效性只看「足迹是否真的压到 254 致命格」，
#      不再把 253（inscribed 环，墙外 0.20 m）当作碰撞。
#      起点贴墙/贴未知边时不再 "Start occupied"。
#   2) RPP：不再对「当前位姿」做碰撞否决（位姿是物理事实；代价图说压墙
#      说明地图/定位错）。路径前方的投射检查保留。
#
# 用法：
#   bash ~/ROS-car/pi/nav2_patch/build_nav2_overlay.sh
#
# 单次约 3~8 分钟（Pi 4 核）。之后 nav.service / slam.service / upper run.sh
# 会在 source /opt/ros/jazzy 之后 source ~/nav2_ws/install/setup.bash。
set -euo pipefail

WS="${NAV2_WS:-$HOME/nav2_ws}"
SRC="$WS/src/navigation2"
PATCH="$(cd "$(dirname "$0")" && pwd)/nav2_overlay.patch"
# 补丁是按这个 jazzy 提交生成的；源码更新后如果 apply 失败，先看这里。
PIN="${NAV2_COMMIT:-94d1c2a3}"

if [ ! -f "$PATCH" ]; then
  echo "找不到补丁: $PATCH" >&2
  exit 1
fi

if [ ! -d "$SRC/.git" ]; then
  echo "==> $SRC 不存在，拉取 jazzy 分支"
  mkdir -p "$WS/src"
  git clone -b jazzy --single-branch \
    https://github.com/ros-navigation/navigation2.git "$SRC"
  git -C "$SRC" checkout -B jazzy "$PIN"
fi

branch="$(git -C "$SRC" branch --show-current)"
if [ "$branch" != "jazzy" ]; then
  echo "==> $SRC 当前分支是 '$branch'，必须切到 jazzy" >&2
  exit 1
fi

head="$(git -C "$SRC" rev-parse --short HEAD)"
echo "==> 源码 HEAD: $head $(git -C "$SRC" log --oneline -1)"
if [ "$head" != "$PIN" ]; then
  echo "!! 注意：HEAD 与补丁基准 $PIN 不同；如果 apply 失败就切回去" >&2
fi

if git -C "$SRC" apply --reverse --check "$PATCH" 2>/dev/null; then
  echo "==> 补丁已应用"
else
  echo "==> 应用补丁"
  git -C "$SRC" apply --check "$PATCH"
  git -C "$SRC" apply "$PATCH"
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "$WS"

echo "==> colcon build（nav2_smac_planner + RPP，Release）"
colcon build \
  --packages-select nav2_smac_planner nav2_regulated_pure_pursuit_controller \
  --parallel-workers "$(nproc)" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

test -f "$WS/install/setup.bash"
echo
echo "完成。使用前先 source（顺序不能反）："
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source $WS/install/setup.bash"
echo "  ros2 pkg prefix nav2_smac_planner   # 应指向 $WS/install/nav2_smac_planner"
