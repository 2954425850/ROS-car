# -*- coding: utf-8 -*-
"""云端底盘模型的逐行移植（云平台前端 `index-CozHwP5J.js` 的 `Gv` + 轮盘）。

**这里只放纯函数、不 import rclpy** —— 这样能脱离 ROS 直接对拍
（`test/test_chassis.py` 把 JS 那两段照抄一遍逐点比）。

云端（混淆后）真正生效的只有两段：
  1. 轮盘指针 → (x, y)：单位圆裁剪 + **径向**死区（`we.joystickDeadzone = .12`）；
  2. `Gv(s, o, n)` 弧线模型。
配置里的 `joystickSmoothing:.35` 在整份 bundle 里**只出现 1 次、没有任何消费者**，
所以云端没有低通 —— 这里也没有（PS2 旧版自己加的那层 LPF 按"完全照抄"删掉了）。

出处：`cloud-upper/frontend/assets/index-CozHwP5J.js`，上游原文抄在两个函数的
docstring 里，跟上面两段一一对得上。
"""
import math


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def gv(fwd, turn, gear, c):
    """云平台 `Gv(s, o, n)`：弧线模型。返回 `(vx, wz)`。

    上游原文（`s`=fwd 前后、`o`=turn 转向、`n`=gear 速度档 m/s）::

        function Gv(e,t,n){const s=Et(e,-1,1),o=Et(t,-1,1);
          if(s===0&&o===0)return{vx:0,wz:0};
          let r=n*Math.abs(s);
          Math.abs(o)>we.latEpsilon&&(r=Math.max(r,we.vMin));
          const i=s<0?-r:r,
                a=Et(i*o/we.rMin,-1.5,we.maxWz);
          return{vx:i,wz:a===0?0:a}}

    照抄时**刻意保留**、容易被"顺手优化"掉的三处：
      · `s==0` 且 `o!=0` 时 `i = +r = +vMin` ⇒ **只推左右会以 0.12 m/s 前蠕画弧**
        （旧版 PS2 有个"无油门不蠕行"的守卫把它挡掉了，按"完全照抄"已删）。
      · `|o| <= latEpsilon` 时 r 不抬到 vMin；此时若 `s==0` 则 vx=wz=0。
      · 低端限幅在 JS 里写死 `-1.5`，恰好等于它自己的 `maxWz`；这里写成
        `-c['WZ_MAX']`，免得以后调 `WZ_MAX` 时成了"自洽但错"。
      · 顺带一提：档位上限 0.5 时 `|a|` 最大只到 `0.5/0.6 = 0.833`，那个 ±1.5
        的限幅其实**够不着**（云端同理）—— 留着是为了逐行一致，不是它在起作用。
    """
    s = clamp(fwd, -1.0, 1.0)
    o = clamp(turn, -1.0, 1.0)
    if s == 0.0 and o == 0.0:
        return 0.0, 0.0
    r = gear * abs(s)
    if abs(o) > c['LAT_EPS']:
        r = max(r, c['V_MIN'])
    i = -r if s < 0.0 else r
    a = clamp(i * o / c['R_MIN'], -c['WZ_MAX'], c['WZ_MAX'])
    return i, (0.0 if a == 0.0 else a)


def ps2_drive(x, y, gear, c):
    """PS2 专用底盘律（2026-10-22 用户要求，**不再照抄云端**）。

    云端前端跑的是它自己的 JS，这里只服务 PS2 手柄；上面的 `gv` / `stick_to_drive`
    仍是云端移植版，留着对拍，本函数不调用它们。

    模型（用户原话：前进向量 v1 + 向左推幅度 a·v1，合成方向决定转向）：
      · 有转向输入、不推前后时，**默认以当前档位向前**（不是 0.12 蠕行）
      · 摇杆完全回中 → **停车**（否则一开节点车就自己一直往前跑）
      · 前推不改变速度（档位已是上限）；只有后拉才减速/倒车：
        行程中点 = 0，拉满 = -gear（线性）
      · 左推量 `a = |x|` 直接决定半径 `R = R_MIN / a`：
          a=1   → R = 1.0 m（满左推，最小半径）
          a=0.5 → R = 2.0 m
          a→0   → 直行
      · `wz = vx / R = vx·a / R_MIN`，**不乘 √(1+a²)**（转弯不加速）
      · x 右为正 ⇒ 前进时 wz<0（右转）、左推 ⇒ wz>0（左转）。
      · **倒车时转向随 vx 翻向**（`wz ∝ vx`，不是 `|vx|`）：后退中往左推，
        车尾往左走。这就是原厂 `balance.c:454` 的 `if(Move_X<0) Move_Z=-Move_Z`；
        云端轮盘缺这一行，所以云端「左后」会走成右后（见记忆
        `l150pro-turn-decisions` ②）。PS2 这边我们自己有映射，按原厂补上。

    `c` 需要 `STICK_DEADZONE`、`R_MIN`。
    """
    dz = c['STICK_DEADZONE']
    a = clamp(abs(x), 0.0, 1.0)
    if a < dz:
        a = 0.0
    back = clamp((y - dz) / (1.0 - dz), 0.0, 1.0)    # y 下为正：后拉
    fwd = clamp((-y - dz) / (1.0 - dz), 0.0, 1.0)    # 前推：只用于"有没有输入"
    if a == 0.0 and back == 0.0 and fwd == 0.0:
        return 0.0, 0.0                              # 摇杆完全回中 → 停
    vx = gear * (1.0 - 2.0 * back)
    if a == 0.0 or vx == 0.0:
        return vx, 0.0
    # 注意是 vx（带符号），不是 abs(vx)：倒车时转向翻向，车尾才跟着摇杆走。
    wz = (-1.0 if x > 0.0 else 1.0) * vx * a / c['R_MIN']
    return vx, wz


def stick_to_drive(x, y, gear, c):
    """轮盘坐标 → `(vx, wz)`。x **右**为正、y **下**为正（前端 `clientY` 的方向）。

    上游原文（`h(v)` = 指针事件；`O` 是单位圆裁剪、`L/$` 是裁完的坐标）::

        const x=(v.clientX-(E.left+E.width/2))/(E.width/2),
              T=(v.clientY-(E.top+E.height/2))/(E.height/2),
              M=Math.hypot(x,T), O=M>1?1/M:1;
        let L=x*O,$=T*O;
        Math.hypot(L,$)<we.joystickDeadzone&&(L=0,$=0);
        a.push({vx:Et(-$,-1,1),wz:Et(L,-1,1)});      // 节流 16ms 往外发

    再由 store 走 `Gv(analog.vx, -analog.wz, gear)` —— 第二个参数**取了负号**：
    `analog.wz = x`（右推为正），而 `Gv` 的 `turn` 是"左为正"，
    所以右推 ⇒ `turn<0` ⇒ `wz<0`。ROS 里 wz>0=逆时针=左转，这就是"右推右转"。

    顺序照抄：**先**单位圆裁剪、**再**判死区；死区是**径向**的（判矢量模长），
    不是逐轴。阈值用 `c['STICK_DEADZONE']`，别跟臂那套逐轴 `DEADZONE` 混。

    物理摇杆怎么变成 (x, y)：`ps2_teleop_node` 里左摇杆 X→x（轴正方向=右推，
    同号）、Y→y（轴负方向=上推，而这里 y 下为正，也是同号），DIR 反了就翻。
    """
    m = math.hypot(x, y)
    if m > 1.0:
        x, y = x / m, y / m
    if math.hypot(x, y) < c['STICK_DEADZONE']:
        x = y = 0.0
    return gv(clamp(-y, -1.0, 1.0), -clamp(x, -1.0, 1.0), gear, c)
