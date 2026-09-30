# /sdcard/k230vision/obstacles.py
"""避障数据编码 —— 把 KPU 的检测结果编成 Pi 侧路径规划能直接吃的「障碍物语义」。

## 这一路在整条链路里的位置

    雷达扫不清的东西（细杆、玻璃、低矮台阶）视觉能看见，但视觉**给不出真实距离**。
    所以这里刻意**不发距离，只发三样雷达给不了的东西**：

        是什么   -> type     （chair / person / …）
        挡在哪   -> box      （归一化，和 results 同约定）
        有多挡路 -> depth_zone / priority / free_zones（都是**粗档**，不是物理量）

    真距离由 Pi 侧的雷达提供，两边在 Pi 上融合。**K230 不假装知道自己有多远。**

## ⚠️ 三条必须记住的边界

1. **单目没有距离。** 摄像头还是**眼在手上**（装在机械臂末端），外参随机械臂动 ——
   换算成米在本架构下**没有意义**。所以 `depth_zone` 只是"框占画面多大"的粗分档，
   机械臂一伸镜头一近，同一个东西就从 far 变 near。**别把它当测距用。**
2. **`free_zones` 是个粗糙代理量，不是"可通行区域"的真值。**
   判据是"框底压到画面下部"，所以远处的地面障碍（在画面里偏上）会被当成不挡路。
   它只配当便宜的初筛，Pi 侧**不要拿它当结论**。见 `free_zones()` 的注释。
3. **只发检测器的结果，不发跟踪器的。** 跟踪器跟的是**人指定的那个目标**
   （桃、辣椒、被跟随的人）—— 那是"要抓/要跟的东西"，不是要避开的障碍物。
   把跟踪目标混进障碍物列表会把规划器带偏。

## 为什么是纯函数 + 单独模块

`targets.py` 已经定过这个分法：**能脱离 socket 测的放纯函数里**。
socket 那层直接用 `reporter.Reporter`（已验证的 NDJSON 客户端，非阻塞、只丢不排队），
所以本模块**一行 socket 代码都没有**。
"""
import ujson


# ---------------- 纯函数 ----------------

def depth_zone(box, near_hi, mid_hi):
    """归一化框 [l,t,r,b] -> "near" / "mid" / "far"。

    **判据是框高比**（高 ÷ 1.0，因为坐标已归一化），不是面积、也不是框宽：
    宽高比随物体形状变（椅子窄高、沙发宽扁），拿宽度判会把沙发判近、椅子判远。
    """
    h = float(box[3]) - float(box[1])
    if h >= near_hi:
        return "near"
    if h >= mid_hi:
        return "mid"
    return "far"


def priority_of(cls, high, low):
    """类别 -> "high" / "mid" / "low"。**按类别而不是按框大小排。**

    人无论远近都是 high：它会动，而且撞上代价最大。
    软目标（椅子/箱子）是 low：撞上了还能推开，不是"必须停下"的理由。
    """
    if cls in high:
        return "high"
    if cls in low:
        return "low"
    return "mid"


def free_zones(objs, zones, bottom_min, h_min):
    """把画面横向等分，返回**未被近处障碍物占住**的连续区间 [[u0,u1], ...]。

    ⚠️ **这是粗糙的代理量，不是"可通行区域"的测量值：**

    - 只有**框底压到画面下部**（`b >= bottom_min`）的障碍物才算"挡在近处地面上"。
      远处的东西在画面里偏上，框底压不到下部 ⇒ 被当成不挡路。**这一条是有意的**
      —— 但它的代价是：真正远处的障碍（比如走廊尽头的椅子）不会被算进 free_zones，
      规划器只看这个字段就会往那边开。**所以它只配当初筛。**
    - 框高比 < `h_min` 的当"远处的、不挡路"，也不占格（否则满屏小目标会把
      所有格子占满，free_zones 恒为空 —— 那比粗糙更糟：看起来像"无路可走"）。

    **右边界取闭区间**：障碍物右沿只"碰到"某格左沿，那一格也算被占 —— 这是
    **有意偏保守**（宁可误报"不通"，也不要把障碍边缘报成"可通行"）。
    所以两个障碍物之间的空隙会比实际**窄最多 1/n**。理由见函数内注释。

    返回的区间可能为空列表（**每一格都被占了**）。调用方要能处理空。
    """
    n = int(zones)
    if n <= 0:
        return []
    blocked = [False] * n
    for o in objs:
        box = o.get("box")
        if not box or len(box) != 4:
            continue
        b = float(box[3])
        if b < bottom_min:
            continue
        if (float(box[3]) - float(box[1])) < h_min:
            continue
        l = float(box[0])
        r = float(box[2])
        if r <= l:
            continue
        i0 = int(l * n)
        i1 = int(r * n)
        if i1 >= n:                 # 右边界压到 1.0 时 int() 会正好落在 n 上
            i1 = n - 1
        if i0 < 0:
            i0 = 0
        # ⚠️ 右端 `i1` 是**闭区间**：右边界只"碰到"某格左沿，也算占住那一格。
        #    这是**有意偏保守的方向** —— 宁可把一整格误报成"不通"，
        #    也不要把障碍物边缘那一条报成"可通行"、让规划器照着往里开。
        #    代价：每个障碍物最多多占 1/n（n=10 时 = 画面宽的 10%）。
        #    在"只配当初筛、别当结论"的前提下，这个方向的错更便宜。
        #
        #    不改成"按重叠面积算"，是因为 `r*n` 在**单精度下会飘**
        #    （0.3*10 = 3.0000000000000004），判"是不是正好落在格线上"反而引入新坑。
        for i in range(i0, i1 + 1):
            blocked[i] = True

    out = []
    i = 0
    while i < n:
        if blocked[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and not blocked[j + 1]:
            j += 1
        out.append([round(i / float(n), 4), round((j + 1) / float(n), 4)])
        i = j + 1
    return out


def build(objs, near_hi, mid_hi, prio_high, prio_low,
          zones, bottom_min, h_min):
    """检测结果 -> 障碍物列表（按"多挡路"排前面）。

    排序：near 优先，其次 high 优先级，再次框大的。
    **排序是给人看的**（日志/调试），Pi 侧不应该依赖顺序 —— 它该自己按需排。
    """
    rank = {"near": 0, "mid": 1, "far": 2}
    out = []
    for o in objs:
        box = o.get("box")
        if not box or len(box) != 4:
            continue
        cls = o.get("cls")
        h = float(box[3]) - float(box[1])
        out.append({
            "type": cls,
            "score": o.get("score"),
            "box": [round(float(v), 4) for v in box],
            "h_ratio": round(h, 4),
            "depth_zone": depth_zone(box, near_hi, mid_hi),
            "priority": priority_of(cls, prio_high, prio_low),
        })
    out.sort(key=lambda d: (rank[d["depth_zone"]],
                            0 if d["priority"] == "high" else 1,
                            -d["h_ratio"]))
    return out


def encode(obstacles, free, frame_no, ts_ms):
    """打包成一行 NDJSON 的 bytes（不含换行，由 Reporter 补）。

        {"ts": <epoch_ms>, "frame": <n>, "obstacles": [...], "free_zones": [[u0,u1],...]}

    坐标一律**归一化到 AI 帧**（与 results 同约定）—— 调用方乘自己的显示尺寸即可。
    """
    return ujson.dumps({
        "ts": ts_ms,
        "frame": frame_no,
        "obstacles": obstacles,
        "free_zones": free,
    }).encode()
