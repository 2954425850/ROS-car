# WS2811 车灯灯效 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 L150PRO 固件上实现 40 像素 WS2811 环带灯效 —— 行驶时全圈同色随轮速由青渐变到红，静止时呼吸/状态色，并叠加转向、刹车、急停、低压等事件灯效。

**Architecture:** 把灯效拆成"纯逻辑层"与"硬件胶水层"两段，用 `WS_LOGIC_BEGIN/END` 注释标记切分。纯逻辑层不含任何 STM32 头文件、只接受标量入参，可被 Python 脚本整段抽出来在宿主机 gcc 上跑断言测试（沿用项目 `_ros_tooling/` 既有模式）。硬件层负责 SPI1(PA7)+DMA 的 800kHz 编码输出。

**Tech Stack:** C99 / STM32F407 / FreeRTOS / SPI1 + DMA2 / Keil MDK (uvprojx) / 宿主机 gcc 断言测试

## Global Constraints

- **既有源文件的编码**：**已有的** `HARDWARE/*.c|h`、`BALANCE/*.h`、`USER/main.c` 均为 **GBK + CRLF**。禁止用文本编辑器或 Edit 工具改这些文件，必须写 Python 脚本按字节替换（`open(p,'rb')` / `open(p,'wb')`），且锚点串须含 `\r\n` 行尾。`USER/WHEELTEC.uvprojx` 是 **UTF-8 XML，但行尾是 bare LF（CRLF 计数为 0）** —— 实测确认，锚点必须用 `\n`。
- **本计划新建的两个文件的编码**：`HARDWARE/ws2811.c` / `ws2811.h` 实测为 **纯 ASCII + 纯 LF**（CRLF 计数为 0），**不是** CRLF。往这两个文件里定位锚点要用 `\n`，**别照搬上一条的 `\r\n`**（Task 4 执行时踩到过）。若将来往这两个文件里加中文注释，需重新确认编码，因为它们最终要跟 GBK 工程一起编译。
- **只关入口不删代码**：所有灯效函数保留，仅通过 `WS2811_ENABLE` 宏与调用点开关功能。禁止为"简化"删除已实现的灯效分支。
- **验证方式**：纯逻辑走宿主机 gcc 断言测试；SPI 时序、时序余量、接线、电平 —— **一律不写宿主机测试，标记为上板验证**。宿主机测试无法覆盖电气问题。
- **宿主机测试文件**：一律放 `_ros_tooling/`，脚本命名 `_*.py`，产物 `_*.c` / `_*.exe`，与既有文件同级。
- **编译命令**：`gcc -std=c99 -O2 -Wall -Wextra`（与 `_ros_tooling/_gen_diff.py:189` 一致），必须零警告通过。**禁止加 `-w` 掩盖警告。**
- **像素数固定 40**；灯带 2m × 60 灯/m，三灯一控。
- **数据脚固定 PA7 = SPI1_MOSI (AF5)**。不得改用 TIM 方案（TIM1/TIM3/TIM8 均已被占用）。
- **任务调用周期固定 40ms**（25fps）。所有动画函数以 `ms` 为入参，禁止函数内部读全局时钟 —— 否则无法宿主机测试。
- **帧缓冲字节序是灯带序（GRB），不是 RGB**。任何读 `ws_frame` 的代码（含测试）必须经 `frame_get()` 之类的转换，不得直接当 RGB 用。

---

## File Structure

| 文件 | 动作 | 职责 |
|---|---|---|
| `HARDWARE/ws2811.h` | 新建 | 公开 API、配置宏、任务参数 |
| `HARDWARE/ws2811.c` | 新建 | 全部灯效逻辑 + SPI/DMA 硬件层（两段用标记切分） |
| `BALANCE/system.h` | 修改 | 加 `#include "ws2811.h"`（字节级） |
| `USER/main.c` | 修改 | 加一行 `xTaskCreate`（字节级） |
| `USER/WHEELTEC.uvprojx` | 修改 | 加 `ws2811.c` 文件条目（字节级，UTF-8） |
| `_ros_tooling/_gen_ws_test.py` | 新建 | 抽取逻辑段 → 生成宿主机测试 → gcc 编译 → 运行 |
| `_ros_tooling/_ws_tests.c` | 新建 | 断言用例（由上面脚本拼接进宿主文件） |
| `_ros_tooling/_apply_ws2811.py` | 新建 | 对上述 3 个既有工程文件做字节级接入 |

**为什么测试生成器写成脚本而不是手写测试文件**：项目既有模式是"从固件源码**逐字抽取**再套桩"，这样测的是真实固件代码而非副本。手抄一份到测试文件会失去这个性质（项目记忆教训："用例看不见接线"——副本与被测物会漂移）。

**为什么测试断言全部用"关系式"而非"具体数值"**：灯效颜色经 HSV→RGB 整数运算 + 亮度缩放，不同亮度下数值差异巨大。断言 `b > 100` 这类绝对值会在改亮度时集体失效。统一断言语义关系（如青色恒有 `g > b > r`），与亮度解耦。

---

### Task 1: 骨架 + 抽取脚本 + 色相→RGB

**Files:**
- Create: `HARDWARE/ws2811.h`
- Create: `HARDWARE/ws2811.c`
- Create: `_ros_tooling/_gen_ws_test.py`
- Create: `_ros_tooling/_ws_tests.c`

**Interfaces:**
- Consumes: 无
- Produces:
  - `void ws_hsv2rgb(u16 h, u8 s, u8 v, u8 *r, u8 *g, u8 *b)` — h∈[0,360), s/v∈[0,255]
  - 标记 `/* ==== WS_LOGIC_BEGIN ==== */` … `/* ==== WS_LOGIC_END ==== */`
  - `_gen_ws_test.py` 可执行：抽取逻辑段、拼接 `_ws_tests.c`、编译、运行；断言失败则退出码非 0

- [ ] **Step 1: 写 `HARDWARE/ws2811.h`**

用 Write 工具新建（全英文注释，避免 GBK 转换问题）。

```c
#ifndef __WS2811_H
#define __WS2811_H
#include "sys.h"
#include "system.h"

/* ---------------- master switch ---------------- */
/* 0 = whole effect engine compiled out at the call site, code kept intact */
#define WS2811_ENABLE       1

/* ---------------- geometry ---------------- */
#define WS_PIXELS           40      /* 2m * 60 LED/m / 3 LEDs-per-chip   */

/* ---------------- wiring / calibration ---------------- */
/* +1 or -1: increasing pixel index runs clockwise or counter-clockwise   */
#define WS_DIR              1
/* 0..39: physical index of the pixel facing straight ahead (nose)       */
#define WS_PHASE            0
/* 1 = boot into calibration mode: one white pixel walks the ring        */
#define WS_CALIB_MODE       0
/* 1 = strip is GRB (most WS2811), 0 = RGB                               */
#define WS_ORDER_GRB        1

/* ---------------- brightness / power ---------------- */
#define WS_BRIGHT_MAX       80      /* 0..255, global ceiling            */

/* ---------------- speed scale ---------------- */
#define WS_VMAX_MPS         0.75f   /* full-scale wheel speed, m/s       */
#define WS_VMIN_MPS         0.02f   /* below this the ring is "idle"     */

/* ---------------- low battery ---------------- */
#define WS_LOWBAT_V         10.0f   /* matches ROS_FAULT_UNDERVOLT doc   */

/* ---------------- FreeRTOS task ---------------- */
#define WS2811_TASK_PRIO    2
#define WS2811_STK_SIZE     256

/* ---------------- logic-layer API (host-testable) ---------------- */
typedef struct { u16 hue; u8 bright; } WS_SpeedOut;

void ws_hsv2rgb(u16 h, u8 s, u8 v, u8 *r, u8 *g, u8 *b);

/* ---------------- hardware-layer API ---------------- */
void ws2811_task(void *pvParameters);

#endif
```

- [ ] **Step 2: 写 `HARDWARE/ws2811.c` 骨架 + 色相转换**

```c
#include "ws2811.h"

/* ==== WS_LOGIC_BEGIN ==== */
/* Nothing below this marker may include an STM32 header or touch a global
 * owned by another module. The host test harness slices this region out
 * verbatim and compiles it against plain stubs, so keep it parametric. */

/**************************************************************************
Function: HSV -> RGB, integer only, no libm dependency.
Input   : h 0..359, s 0..255, v 0..255
Output  : *r *g *b 0..255
**************************************************************************/
void ws_hsv2rgb(u16 h, u8 s, u8 v, u8 *r, u8 *g, u8 *b)
{
	u8  region, rem;
	u16 p, q, t;

	if (h >= 360) h %= 360;
	region = (u8)(h / 60);          /* 0..5 */
	rem    = (u8)(h % 60);          /* 0..59 */

	p = (u16)((u32)v * (255 - s) / 255);
	q = (u16)((u32)v * (255 - (u32)s * rem / 60) / 255);
	t = (u16)((u32)v * (255 - (u32)s * (60 - rem) / 60) / 255);

	switch (region) {
	case 0:  *r = (u8)v; *g = (u8)t; *b = (u8)p; break;
	case 1:  *r = (u8)q; *g = (u8)v; *b = (u8)p; break;
	case 2:  *r = (u8)p; *g = (u8)v; *b = (u8)t; break;
	case 3:  *r = (u8)p; *g = (u8)q; *b = (u8)v; break;
	case 4:  *r = (u8)t; *g = (u8)p; *b = (u8)v; break;
	default: *r = (u8)v; *g = (u8)p; *b = (u8)q; break;
	}
}
/* ==== WS_LOGIC_END ==== */

/* ==== WS_HW_BEGIN ==== */
void ws2811_task(void *pvParameters)
{
	(void)pvParameters;
	for (;;) {
		vTaskDelay(40);
	}
}
/* ==== WS_HW_END ==== */
```

- [ ] **Step 3: 写抽取脚本 `_ros_tooling/_gen_ws_test.py`**

```python
# -*- coding: utf-8 -*-
"""Slice the host-testable region out of HARDWARE/ws2811.c, append the cases
   from _ws_tests.c, build with gcc and run. Exit code propagates."""
import os, subprocess, sys

HERE   = os.path.dirname(os.path.abspath(__file__))
ROOT   = os.path.dirname(HERE)
SRC    = os.path.join(ROOT, 'HARDWARE', 'ws2811.c')
TESTS  = os.path.join(HERE, '_ws_tests.c')
OUT_C  = os.path.join(HERE, '_ws_host_test.c')
OUT_EXE= os.path.join(HERE, '_ws_host_test.exe')

PRELUDE = r'''
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef unsigned char  u8;
typedef signed   char  s8;
typedef unsigned short u16;
typedef signed   short s16;
typedef unsigned int   u32;

/* ---- config macros, copied verbatim from ws2811.h ---- */
#define WS_PIXELS           40
#define WS_DIR              1
#define WS_PHASE            0
#define WS_ORDER_GRB        1
#define WS_BRIGHT_MAX       80
#define WS_VMAX_MPS         0.75f
#define WS_VMIN_MPS         0.02f
#define WS_LOWBAT_V         10.0f

'''

def main():
    src = open(SRC, 'rb').read().decode('gbk')
    m0 = src.index('/* ==== WS_LOGIC_BEGIN ==== */')
    m1 = src.index('/* ==== WS_LOGIC_END ==== */')
    m0 = src.index('\n', src.index('*/', m0)) + 1
    body = src[m0:m1].replace('\r\n', '\n')
    if 'stm32f4' in body.lower() or 'vTaskDelay' in body or '#include' in body:
        sys.exit('ws2811.c logic region must stay free of headers and RTOS calls')

    tests = open(TESTS, 'rb').read().decode('utf-8')
    open(OUT_C, 'w', encoding='ascii').write(
        PRELUDE
        + '\n/* ==== extracted verbatim from HARDWARE/ws2811.c ==== */\n'
        + body
        + '\n/* ==== test helpers ==== */\n'
        + HELPERS
        + '\n/* ==== cases ==== */\n'
        + tests)

    r = subprocess.run(['gcc', '-std=c99', '-O2', '-Wall', '-Wextra',
                        '-o', OUT_EXE, OUT_C])
    if r.returncode != 0:
        sys.exit('gcc failed')
    print('built %s' % os.path.basename(OUT_EXE))
    sys.exit(subprocess.run([OUT_EXE]).returncode)

if __name__ == '__main__':
    main()
```

其中 `HELPERS` 是紧随其后的第二个字面量（同一文件内定义在 `PRELUDE` 之后）：

```python
HELPERS = r'''
/* The frame buffer is stored in strip order. Read it back into RGB so the
 * cases can assert on colour semantics instead of byte positions. */
static void frame_get(u8 phys, u8 *r, u8 *g, u8 *b)
{
	u8 *p = &ws_frame[(u16)phys * 3];
#if WS_ORDER_GRB
	*g = p[0]; *r = p[1]; *b = p[2];
#else
	*r = p[0]; *g = p[1]; *b = p[2];
#endif
}

static int g_fail = 0;

static void ok(const char *what) { printf("ok   %s\n", what); }

/* Colour semantics that hold at ANY brightness, so the cases stay valid
 * when WS_BRIGHT_MAX or the idle breathing range is retuned. */
static int is_cyanish(u8 r, u8 g, u8 b)   { return g > b && b > r && r < 20; }
static int is_purpleish(u8 r, u8 g, u8 b) { return b > r && r > g && g < 20; }
static int is_orangeish(u8 r, u8 g, u8 b) { return r > g && g > b && b < 20 && (u16)g * 2 <= (u16)r + 2; }
static int is_yellowish(u8 r, u8 g, u8 b) { return r > g && g > b && b < 20 && (u16)g * 2 >  (u16)r + 2; }
static int is_reddish(u8 r, u8 g, u8 b)   { return r > 150 && g < 40 && b < 40; }
'''
```

**必须再加两个 shim**（Task 1 执行时实测确认，缺了编译不过）：

1. `HELPERS` 里的 `frame_get()` 引用 `ws_frame`，而 `ws_frame` 到 **Task 4** 才在逻辑区定义 —— 缺少声明时 `frame_get` 函数体内引用未声明标识符，C99 直接报错。
2. 即便补上声明，Task 1 只用到 `is_cyanish`，另外 4 个 helper 是"已定义未使用的 static 函数" → `-Wunused-function` 报警。全局约束禁止 `-w`，所以必须从根因消除而不是屏蔽。

解法是在脚本里加两个生成片段（**不要改 `PRELUDE` 和 `HELPERS` 正文**）：

```python
SHIM_DECL = r'''
extern u8 ws_frame[WS_PIXELS * 3];
'''

SHIM_ANCHOR = r'''
void ws_harness_anchor(void)
{
	(void)frame_get;
	(void)is_purpleish;
	(void)is_orangeish;
	(void)is_yellowish;
	(void)is_reddish;
}
'''
```

拼装顺序必须是 `PRELUDE → body → SHIM_DECL → HELPERS → SHIM_ANCHOR → tests`：

- `SHIM_DECL` 必须在 `HELPERS` **之前**（helper 体内要引用它）
- `SHIM_ANCHOR` 必须在 `HELPERS` **之后**（要引用 helper）
- anchor 故意写成**非 static**：`-Wunused-function` 只诊断 static 函数，非 static 的未使用函数不报警 —— 这是消除成因，不是屏蔽警告

这两段 shim 在 Task 4 之后依然无害：`extern` 声明跟在同 TU 的定义之后合法，anchor 里的 `(void)fn;` 也仍是合法表达式。**Task 4 起不要删它们。**

- [ ] **Step 4: 写 `_ros_tooling/_ws_tests.c`（第一版，只测 ws_hsv2rgb）**

```c
static void case_hsv(void)
{
	struct { u16 h; u8 s; u8 v; int r, g, b; const char *n; } c[] = {
		{  0, 255, 255, 255,   0,   0, "red"        },
		{ 60, 255, 255, 255, 255,   0, "yellow"     },
		{120, 255, 255,   0, 255,   0, "green"      },
		{180, 255, 255,   0, 255, 255, "cyan"       },
		{240, 255, 255,   0,   0, 255, "blue"       },
		{300, 255, 255, 255,   0, 255, "magenta"    },
		{200,   0, 200, 200, 200, 200, "grey s=0"   },
		{  0, 255, 128, 128,   0,   0, "red v=128"  },
		{160, 255, 255,   0, 255, 170, "idle cyan"  },
		{360, 255, 255, 255,   0,   0, "h=360=red"  },
	};
	unsigned k;
	for (k = 0; k < sizeof(c) / sizeof(c[0]); k++) {
		u8 r = 0, g = 0, b = 0;
		ws_hsv2rgb(c[k].h, c[k].s, c[k].v, &r, &g, &b);
		/* +/-1 absorbs integer truncation in p/q/t */
		if (abs((int)r - c[k].r) > 1 || abs((int)g - c[k].g) > 1 ||
		    abs((int)b - c[k].b) > 1) {
			printf("FAIL hsv %-10s h=%3u s=%3u v=%3u -> (%3u,%3u,%3u) want (%3d,%3d,%3d)\n",
			       c[k].n, c[k].h, c[k].s, c[k].v, r, g, b,
			       c[k].r, c[k].g, c[k].b);
			g_fail++;
		} else {
			printf("ok   hsv %-10s -> (%3u,%3u,%3u)\n", c[k].n, r, g, b);
		}
	}
	/* the cyan used by the idle layer must satisfy the relational helper */
	{
		u8 r, g, b;
		ws_hsv2rgb(160, 255, 255, &r, &g, &b);
		if (!is_cyanish(r, g, b)) { printf("FAIL idle cyan relation\n"); g_fail++; }
		else ok("idle cyan satisfies is_cyanish");
	}
}

int main(void)
{
	case_hsv();
	printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
	return g_fail ? 1 : 0;
}
```

- [ ] **Step 5: 运行，确认通过**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 11 行 `ok`，末行 `ALL PASSED (0 failures)`，退出码 0，零 gcc 警告。

- [ ] **Step 6: 反证测试真的会红**

把 `case_hsv()` 里 `"red"` 那行的期望值临时改成 `255, 255, 0`，重跑，确认输出 `FAIL` 且退出码非 0，然后改回。

> 这不是形式主义：项目记忆里记着"用例看不见接线"的教训，而这里更隐蔽的是**用例可能根本没跑起来**（比如 gcc 静默失败、脚本提前 return）。必须先看见红。

- [ ] **Step 7: 提交**

本目录不是 git 仓库（`git rev-parse` 已确认失败），**跳过 commit**。后续所有任务的"提交"步骤同理，一律跳过。若之后决定初始化仓库再统一补。

---

### Task 2: 速度映射（双维度编码）

**Files:**
- Modify: `HARDWARE/ws2811.c`（`WS_LOGIC` 区内追加）
- Modify: `HARDWARE/ws2811.h`
- Modify: `_ros_tooling/_ws_tests.c`

**Interfaces:**
- Consumes: `ws_hsv2rgb` (Task 1)
- Produces: `WS_SpeedOut ws_speed_map(float mps)` — `mps` 为**非负**轮速绝对值，单位 m/s；返回 `{.hue 0..359, .bright 0..255}`

- [ ] **Step 1: 写失败测试**

在 `_ws_tests.c` 追加用例函数，并在 `main()` 里调用：

```c
static void case_speed(void)
{
	WS_SpeedOut o;
	struct { float v; int h, b; } c[] = {
		{0.00f, 160,  89},   /* idle floor: cyan, 0.35*255             */
		{0.02f, 160,  89},   /* first breakpoint, inclusive upper      */
		{0.06f, 160, 140},   /* HUE LOCKED, brightness climbing        */
		{0.10f, 160, 191},   /* 0.75*255                               */
		{0.20f, 140, 210},   /* midpoint of 160 -> 120                 */
		{0.30f, 120, 230},
		{0.50f,  60, 255},
		{0.65f,  30, 255},
		{0.75f,   0, 255},   /* full scale: red                        */
		{1.50f,   0, 255},   /* clamps above full scale                */
		{0.04f, 160, 115},   /* between 0.02 and 0.10                  */
	};
	unsigned k;
	for (k = 0; k < sizeof(c) / sizeof(c[0]); k++) {
		o = ws_speed_map(c[k].v);
		if (abs((int)o.hue - c[k].h) > 1 || abs((int)o.bright - c[k].b) > 2) {
			printf("FAIL speed v=%.3f -> hue=%u bright=%u want %d/%d\n",
			       c[k].v, o.hue, o.bright, c[k].h, c[k].b);
			g_fail++;
		} else {
			printf("ok   speed v=%.3f -> hue=%3u bright=%3u\n",
			       c[k].v, o.hue, o.bright);
		}
	}
	/* negative input must be treated as magnitude */
	o = ws_speed_map(-0.75f);
	if (o.hue != 0 || o.bright != 255) { printf("FAIL speed negative\n"); g_fail++; }
	else ok("speed negative clamps to red");

	/* the design's whole reason for existing: hue must be FLAT while
	 * brightness moves, across the whole creep band */
	{
		int k, flat = 1, rising = 1;
		u16 h0 = ws_speed_map(0.02f).hue;
		u8  b_prev = ws_speed_map(0.02f).bright;
		for (k = 0; k <= 16; k++) {
			/* Integer k, not float accumulation. Accumulating 0.005f
			 * sixteen times can land a hair ABOVE 0.10f, crossing into
			 * the green segment, where the hue interpolates to
			 * 159.99999 and (u16) truncates it to 159. That is a float
			 * boundary artefact, not an implementation defect, so the
			 * comparison below also carries +/-1 for integer truncation. */
			WS_SpeedOut s = ws_speed_map(0.02f + (float)k * 0.005f);
			if (s.hue > h0 + 1 || s.hue < h0 - 1) flat = 0;
			if (s.bright < b_prev) rising = 0;
			b_prev = s.bright;
		}
		if (!flat || !rising) {
			printf("FAIL creep band: flat=%d rising=%d\n", flat, rising);
			g_fail++;
		} else ok("creep band 0.02..0.10 holds hue flat, brightness rising");
	}
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: gcc 失败，`implicit declaration of function 'ws_speed_map'` / `unknown type name 'WS_SpeedOut'`。

- [ ] **Step 3: 实现**

在 `ws2811.c` 的 `WS_LOGIC` 区内（`ws_hsv2rgb` 之后）追加：

```c
/* Speed -> (hue, brightness) breakpoints. The first segment is deliberate:
 * below 0.10 m/s the hue is LOCKED at 160 deg and only brightness moves,
 * because hue alone is invisible in the cyan->green corner while the
 * operator is creeping off the line. See design doc section 4.1. */
static const float ws_v_tab[6] = {0.02f, 0.10f, 0.30f, 0.50f, 0.65f, 0.75f};
static const u16   ws_h_tab[6] = {160,   160,   120,   60,    30,    0    };
static const u8    ws_b_tab[6] = {89,    191,   230,   255,   255,   255  };
/* 89 = 0.35*255, 191 = 0.75*255, 230 = 0.90*255 */

WS_SpeedOut ws_speed_map(float mps)
{
	WS_SpeedOut o;
	u8 i;
	float t;

	if (mps < 0.0f) mps = -mps;

	if (mps <= ws_v_tab[0]) {
		o.hue = ws_h_tab[0]; o.bright = ws_b_tab[0]; return o;
	}
	if (mps >= ws_v_tab[5]) {
		o.hue = ws_h_tab[5]; o.bright = ws_b_tab[5]; return o;
	}
	for (i = 0; i < 5; i++)
		if (mps < ws_v_tab[i + 1]) break;

	t = (mps - ws_v_tab[i]) / (ws_v_tab[i + 1] - ws_v_tab[i]);
	o.hue    = (u16)((float)ws_h_tab[i] +
	                 t * ((float)ws_h_tab[i + 1] - (float)ws_h_tab[i]));
	o.bright = (u8)((float)ws_b_tab[i] +
	                t * ((float)ws_b_tab[i + 1] - (float)ws_b_tab[i]));
	return o;
}
```

在 `ws2811.h` 的 `WS_SpeedOut` 定义下加 `WS_SpeedOut ws_speed_map(float mps);`。

**同时必须在 `_gen_ws_test.py` 的 `PRELUDE` 里补上 typedef**（Task 2 执行时实测确认，漏了编译不过）：

```python
/* ---- logic-layer types, copied verbatim from ws2811.h ---- */
typedef struct { u16 hue; u8 bright; } WS_SpeedOut;
```

注意**不是**加函数声明就行 —— 抽取出来的实现体第一行就是 `WS_SpeedOut ws_speed_map(float mps)`，缺 typedef 时**实现体自身**就报 `unknown type name 'WS_SpeedOut'`，加多少函数声明都救不了。

> 也不能把这个 typedef 塞进 `ws2811.c` 的逻辑区绕过去：那样 Keil 下会与 `ws2811.h` 里的同名 typedef 重复定义，C99 报错。宿主与固件共用同一份源码，类型只能由 PRELUDE 提供。
>
> **一般规律**：凡是被抽取的逻辑区里**用到**的类型，都必须在 PRELUDE 有定义；函数则不必（实现体先于用例，已被定义）。

- [ ] **Step 4: 运行确认通过**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 全部 `ok`，含 `creep band 0.02..0.10 holds hue flat, brightness rising`。

> `v=0.06` 一行必须显示 `hue=160`（色相锁死）而 brightness 介于 89 和 191 之间 —— 这正是设计里"用亮度补低速可见性"的核心断言。

---

### Task 3: 状态机（滞回 + 交叉淡入）

**Files:**
- Modify: `HARDWARE/ws2811.c`
- Modify: `HARDWARE/ws2811.h`
- Modify: `_ros_tooling/_ws_tests.c`

**Interfaces:**
- Consumes: 无
- Produces:
  - `typedef struct { float filt; u8 moving; u8 blend; u16 start_ms; u16 stop_ms; } WS_State;`
  - `void ws_state_init(WS_State *st)`
  - `void ws_state_step(WS_State *st, float v_abs)` — **必须每 40ms 调用一次**；`blend` 0=待机层、255=运动层

- [ ] **Step 1: 写失败测试**

```c
static void case_state(void)
{
	WS_State st;
	int k, at;

	/* --- enter hysteresis: 0.05 m/s must dwell before latching --- */
	ws_state_init(&st);
	at = -1;
	for (k = 0; k < 50 && at < 0; k++) {
		ws_state_step(&st, 0.05f);
		if (st.moving) at = k;
	}
	/* 100ms low-pass + 100ms dwell => the crossing lands a few steps in */
	if (at < 2 || at > 6) {
		printf("FAIL enter hysteresis: latched at step %d, want 2..6\n", at);
		g_fail++;
	} else printf("ok   enter hysteresis: latched at step %d (~%dms)\n",
	              at, at * 40);

	/* --- exit hysteresis: 500ms dwell below 0.015 --- */
	at = -1;
	for (k = 0; k < 60 && at < 0; k++) {
		ws_state_step(&st, 0.0f);
		if (!st.moving) at = k;
	}
	if (at < 8 || at > 25) {
		printf("FAIL exit hysteresis: released at step %d, want 8..25\n", at);
		g_fail++;
	} else printf("ok   exit hysteresis: released at step %d (~%dms)\n",
	              at, at * 40);

	/* NOTE: the release step still runs the moving branch, so blend is
	 * still 255 here. It only starts decaying on the NEXT step. */
	for (k = 0; k < 12; k++) ws_state_step(&st, 0.0f);
	if (st.blend > 5) {
		printf("FAIL blend did not decay: %u\n", st.blend); g_fail++;
	} else printf("ok   blend decayed to %u after release\n", st.blend);

	/* --- re-entry after a genuine stop --- */
	ws_state_init(&st);
	for (k = 0; k < 3; k++) ws_state_step(&st, 0.05f);
	for (k = 0; k < 20; k++) ws_state_step(&st, 0.0f);   /* 800ms idle */
	if (st.moving) { printf("FAIL 3-step blip latched motion\n"); g_fail++; }
	else ok("3-step blip does not latch");
	for (k = 0; k < 300; k++) ws_state_step(&st, 0.04f); /* sustained */
	if (!st.moving || st.blend < 250) {
		printf("FAIL re-entry: moving=%u blend=%u\n", st.moving, st.blend);
		g_fail++;
	} else printf("ok   re-entry after idle: blend=%u\n", st.blend);

	/* --- a single encoder spike MAY briefly latch, but must release on
	 * its own. A stuck state here would leave the ring red forever after
	 * one noisy sample; that is the failure mode this guards. --- */
	ws_state_init(&st);
	ws_state_step(&st, 0.50f);
	for (k = 0; k < 30; k++) ws_state_step(&st, 0.0f);   /* 1200ms */
	if (st.moving || st.blend != 0) {
		printf("FAIL spike latched in: moving=%u blend=%u\n",
		       st.moving, st.blend);
		g_fail++;
	} else ok("single spike releases within 1200ms");
}
```

并在 `main()` 里加 `case_state();`。

- [ ] **Step 2: 运行确认失败**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: gcc 失败，`unknown type name 'WS_State'`。

- [ ] **Step 3: 实现**

`ws2811.h` 加：

```c
typedef struct {
	float filt;      /* 100ms low-passed |v|                    */
	u8    moving;    /* 0/1 latched motion state                */
	u8    blend;     /* 0 = idle layer, 255 = motion layer      */
	u16   start_ms;  /* dwell above the enter threshold         */
	u16   stop_ms;   /* dwell below the exit threshold          */
} WS_State;

void ws_state_init(WS_State *st);
void ws_state_step(WS_State *st, float v_abs);
```

`ws2811.c` 的 `WS_LOGIC` 区追加：

```c
#define WS_STEP_MS        40
#define WS_ENTER_MPS      0.03f
#define WS_EXIT_MPS       0.015f
#define WS_ENTER_HOLD_MS  100
#define WS_EXIT_HOLD_MS   500
#define WS_FADE_MS        300
/* 255 * 40 / 300 = 34 -> eight steps to cross, 320ms nominal */
#define WS_FADE_STEP      ((u8)(255 * WS_STEP_MS / WS_FADE_MS))
/* first-order LPF: alpha = dt / (tau + dt), tau = 100ms */
#define WS_LPF_ALPHA      (40.0f / 140.0f)

void ws_state_init(WS_State *st)
{
	st->filt     = 0.0f;
	st->moving   = 0;
	st->blend    = 0;
	st->start_ms = 0;
	st->stop_ms  = 0;
}

void ws_state_step(WS_State *st, float v_abs)
{
	if (v_abs < 0.0f) v_abs = -v_abs;
	st->filt += (v_abs - st->filt) * WS_LPF_ALPHA;

	if (!st->moving) {
		if (st->filt > WS_ENTER_MPS) {
			st->start_ms += WS_STEP_MS;
			if (st->start_ms >= WS_ENTER_HOLD_MS) {
				st->moving  = 1;
				st->stop_ms = 0;
			}
		} else {
			st->start_ms = 0;
		}
		st->blend = (st->blend > WS_FADE_STEP)
		          ? (u8)(st->blend - WS_FADE_STEP) : 0;
	} else {
		if (st->filt < WS_EXIT_MPS) {
			st->stop_ms += WS_STEP_MS;
			if (st->stop_ms >= WS_EXIT_HOLD_MS) {
				st->moving   = 0;
				st->start_ms = 0;
			}
		} else {
			st->stop_ms = 0;
		}
		if (st->blend < 255) {
			u16 b = (u16)st->blend + WS_FADE_STEP;
			st->blend = (b > 255) ? 255 : (u8)b;
		}
	}
}
```

- [ ] **Step 4: 运行确认通过**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 五条 `ok`，其中 `enter hysteresis: latched at step 4 (~160ms)`、`exit hysteresis: released at step 16 (~640ms)`。

> **实测确认值（Task 3 执行时得到，以此为准）**：
> - `enter hysteresis` = **step 4**（~160ms）
> - `exit hysteresis` = **step 14**（~560ms）
>
> 退出侧**不是** 16。原因是进入循环一旦 `moving` 置 1 就立刻退出，此时 `filt ≈ 0.0370` 而**不是**收敛值 0.05；从 0.0370 衰减到 0.015 以下需 3 拍，再累计 12 拍（480ms）+1 拍才越过 500ms。用收敛值 0.05 去估会算出 16 —— 那是个**估算陷阱**，别照着推。
>
> 若实测步数落在断言区间外，**不要放宽区间** —— 先核对 `WS_LPF_ALPHA` 与阈值的一致性。

---

### Task 4: 帧缓冲 + 索引映射 + 标定模式

**Files:**
- Modify: `HARDWARE/ws2811.c`
- Modify: `HARDWARE/ws2811.h`
- Modify: `_ros_tooling/_ws_tests.c`

**Interfaces:**
- Consumes: 无
- Produces:
  - `extern u8 ws_frame[WS_PIXELS * 3];` — 按 `WS_ORDER_GRB` 排好字节序
  - `void ws_clear(void)`
  - `void ws_put(u8 phys, u8 r, u8 g, u8 b)` — **物理**索引，不经映射
  - `u8 ws_map(u8 logical)` — 逻辑位置 → 物理索引
  - `void ws_fill_solid(u8 r, u8 g, u8 b)`
  - `void ws_render_calib(u8 step)`

- [ ] **Step 1: 写失败测试**

```c
static void case_frame(void)
{
	u8 s, r, g, b;
	u16 k, nz;

	/* identity mapping under the shipped calibration */
	for (s = 0; s < WS_PIXELS; s++)
		if (ws_map(s) != s) { printf("FAIL ws_map at %u\n", s); g_fail++; return; }
	ok("ws_map identity (WS_DIR=+1, WS_PHASE=0)");

	/* ws_put takes RGB and the buffer holds strip order */
	ws_clear();
	ws_put(3, 0x11, 0x22, 0x33);
	frame_get(3, &r, &g, &b);
	if (r != 0x11 || g != 0x22 || b != 0x33) {
		printf("FAIL roundtrip: got %02X %02X %02X\n", r, g, b); g_fail++;
	} else printf("ok   ws_put/frame_get roundtrip %02X %02X %02X\n", r, g, b);
#if WS_ORDER_GRB
	if (ws_frame[9] != 0x22 || ws_frame[10] != 0x11 || ws_frame[11] != 0x33) {
		printf("FAIL raw GRB order: %02X %02X %02X\n",
		       ws_frame[9], ws_frame[10], ws_frame[11]); g_fail++;
	} else ok("raw buffer really is GRB");
#endif

	/* ws_put must reject out-of-range instead of corrupting memory */
	ws_clear();
	ws_put(WS_PIXELS, 255, 255, 255);
	ws_put(200, 255, 255, 255);
	nz = 0;
	for (k = 0; k < WS_PIXELS * 3; k++) if (ws_frame[k]) nz++;
	if (nz) { printf("FAIL out-of-range ws_put wrote %u bytes\n", nz); g_fail++; }
	else ok("out-of-range ws_put is a no-op");

	/* ws_clear zeroes everything */
	ws_put(0, 1, 2, 3);
	ws_clear();
	nz = 0;
	for (k = 0; k < WS_PIXELS * 3; k++) if (ws_frame[k]) nz++;
	if (nz) { printf("FAIL ws_clear left %u bytes\n", nz); g_fail++; }
	else printf("ok   ws_clear zeroed %u bytes\n", WS_PIXELS * 3);

	/* calibration lights exactly one pixel, the requested one */
	for (s = 0; s < WS_PIXELS; s += 13) {
		u8 lit = 0, lit_at = 0xFF, i;
		ws_render_calib(s);
		for (i = 0; i < WS_PIXELS; i++) {
			frame_get(i, &r, &g, &b);
			if (r || g || b) { lit++; lit_at = i; }
		}
		if (lit != 1 || lit_at != s) {
			printf("FAIL calib(%u): %u lit at %u\n", s, lit, lit_at); g_fail++;
		} else printf("ok   calib(%u) lights only that pixel\n", s);
	}
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: gcc 失败，`'ws_frame' undeclared`。

- [ ] **Step 3: 实现**

`ws2811.c` 的 `WS_LOGIC` 区追加：

```c
u8 ws_frame[WS_PIXELS * 3];

void ws_clear(void)
{
	u16 i;
	for (i = 0; i < WS_PIXELS * 3; i++) ws_frame[i] = 0;
}

void ws_put(u8 phys, u8 r, u8 g, u8 b)
{
	u8 *p;
	if (phys >= WS_PIXELS) return;          /* bounds check, not an assert */
	p = &ws_frame[(u16)phys * 3];
#if WS_ORDER_GRB
	p[0] = g; p[1] = r; p[2] = b;
#else
	p[0] = r; p[1] = g; p[2] = b;
#endif
}

/* Logical position -> physical pixel. Only the position-aware effects
 * (calibration, nav progress, orbit direction) go through this; solid
 * fills ignore it and are therefore immune to how the strip was glued on. */
u8 ws_map(u8 logical)
{
	s16 v = (s16)(WS_DIR * (s16)logical) + WS_PHASE;
	while (v < 0) v += WS_PIXELS;
	return (u8)(v % WS_PIXELS);
}

void ws_fill_solid(u8 r, u8 g, u8 b)
{
	u8 i;
	for (i = 0; i < WS_PIXELS; i++) ws_put(i, r, g, b);
}

void ws_render_calib(u8 step)
{
	ws_clear();
	ws_put(ws_map(step), 255, 255, 255);
}
```

`ws2811.h` 导出 `extern u8 ws_frame[WS_PIXELS * 3];` 与上述五个函数声明。

- [ ] **Step 4: 运行确认通过**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 全部 `ok`，含 `raw buffer really is GRB` 与 `out-of-range ws_put is a no-op`。

---

### Task 5: L1 待机层（呼吸 / 状态色 / 环绕流光）

**Files:**
- Modify: `HARDWARE/ws2811.c`
- Modify: `HARDWARE/ws2811.h`
- Modify: `_ros_tooling/_ws_tests.c`

**Interfaces:**
- Consumes: `ws_put`, `ws_clear`, `ws_map`, `ws_fill_solid`, `ws_hsv2rgb`
- Produces:
  - `typedef struct { u8 estop, lowbat, brake, reverse; s8 turn; u16 nav_permille; u8 ros_link; } WS_Overlay;`
  - `u8 ws_breath(u32 ms, u16 period_ms, u8 lo, u8 hi)`
  - `void ws_render_idle(const WS_Overlay *ov, u32 ms)`

- [ ] **Step 1: 写失败测试**

```c
/* Idle states that route through the orbit renderer light only a few
 * pixels, and which pixels depends on ms. Sampling pixel 0 would read a
 * dark pixel and defeat every colour assertion below, so sample the
 * brightest lit one. Returns 0 if the whole ring is dark. */
static int frame_brightest(u8 *r, u8 *g, u8 *b)
{
	u8  i, br = 0, bg = 0, bb = 0;
	int found = 0, best = -1;

	for (i = 0; i < WS_PIXELS; i++) {
		u8 pr, pg, pb;
		int sum;
		frame_get(i, &pr, &pg, &pb);
		sum = (int)pr + (int)pg + (int)pb;
		if (sum > 0 && sum > best) {
			best = sum; br = pr; bg = pg; bb = pb; found = 1;
		}
	}
	*r = br; *g = bg; *b = bb;
	return found;
}

static void case_idle(void)
{
	WS_Overlay ov;
	u8 r, g, b, k, same;

	/* breathing waveform: trough at 0 and period, peak at half period */
	{
		u8 a = ws_breath(0, 2000, 13, 115);
		u8 p = ws_breath(1000, 2000, 13, 115);
		u8 c = ws_breath(2000, 2000, 13, 115);
		if (!(a < 30 && p > 100 && a == c)) {
			printf("FAIL breath: %u %u %u\n", a, p, c); g_fail++;
		} else printf("ok   breath trough=%u peak=%u\n", a, p);
	}

	/* ros_link = 1 -> steady cyan ring (no orbit, so it must be uniform) */
	memset(&ov, 0, sizeof(ov)); ov.nav_permille = 0xFFFF; ov.ros_link = 1;
	ws_render_idle(&ov, 1000);
	frame_get(0, &r, &g, &b);
	if (!is_cyanish(r, g, b)) {
		printf("FAIL connected colour %u %u %u\n", r, g, b); g_fail++;
	} else printf("ok   connected -> cyan %u %u %u\n", r, g, b);
	same = 1;
	for (k = 1; k < WS_PIXELS; k++) {
		u8 r2, g2, b2;
		frame_get(k, &r2, &g2, &b2);
		if (r2 != r || g2 != g || b2 != b) same = 0;
	}
	if (!same) { printf("FAIL connected ring not uniform\n"); g_fail++; }
	else ok("connected ring is uniform");

	/* ros_link = 0 -> yellow: waiting for the host. This routes through
	 * the ORBIT renderer, so sampling pixel 0 would read a dark pixel. */
	ov.ros_link = 0;
	ws_render_idle(&ov, 1000);
	if (!frame_brightest(&r, &g, &b)) {
		printf("FAIL waiting: ring is dark\n"); g_fail++;
	} else if (!is_yellowish(r, g, b)) {
		printf("FAIL waiting colour %u %u %u\n", r, g, b); g_fail++;
	} else printf("ok   waiting -> yellow %u %u %u\n", r, g, b);

	/* ros_link = 2 -> purple slow blink. ms=300 lands in the BRIGHT half
	 * (800ms period, first half bright; ms=400 is the boundary and falls
	 * into the dim half). The relational assertion holds either way, but
	 * sampling the bright half is the fair test. */
	ov.ros_link = 2;
	ws_render_idle(&ov, 300);
	if (!frame_brightest(&r, &g, &b)) {
		printf("FAIL lost: ring is dark\n"); g_fail++;
	} else if (!is_purpleish(r, g, b)) {
		printf("FAIL lost colour %u %u %u\n", r, g, b); g_fail++;
	} else printf("ok   lost -> purple %u %u %u\n", r, g, b);

	/* lost uses the orbit renderer, so the ring must NOT be uniform */
	{
		u8 r2, g2, b2, uniform = 1;
		for (k = 1; k < WS_PIXELS; k++) {
			frame_get(k, &r2, &g2, &b2);
			if (r2 != r || g2 != g || b2 != b) uniform = 0;
		}
		if (uniform) { printf("FAIL lost ring should show an orbit\n"); g_fail++; }
		else ok("lost ring shows a moving comet");
	}

	/* low battery wins over the link colour and caps brightness. It also
	 * routes through the orbit renderer, so check EVERY lit pixel is
	 * orange -- the comet tail is attenuated but must keep its hue. */
	ov.ros_link = 1; ov.lowbat = 1;
	ws_render_idle(&ov, 1000);
	{
		int lit = 0, all_orange = 1;
		for (k = 0; k < WS_PIXELS; k++) {
			frame_get(k, &r, &g, &b);
			if (r || g || b) {
				lit++;
				if (!is_orangeish(r, g, b)) all_orange = 0;
			}
		}
		if (lit < 3 || !all_orange) {
			printf("FAIL lowbat: lit=%d all_orange=%d\n", lit, all_orange);
			g_fail++;
		} else ok("lowbat -> orange comet, every lit pixel orange");
	}
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: gcc 失败，`unknown type name 'WS_Overlay'`。

- [ ] **Step 3: 实现**

`ws2811.h` 加：

```c
typedef struct {
	u8  estop;         /* 1 = full-band red strobe              */
	u8  lowbat;        /* 1 = full-band orange breathe, capped  */
	u8  brake;         /* 1 = force red, quick flashes          */
	s8  turn;          /* +1 left / 0 none / -1 right           */
	u8  reverse;       /* 1 = white flow                        */
	u16 nav_permille;  /* 0..1000 nav progress, 0xFFFF = off    */
	u8  ros_link;      /* 0 waiting / 1 connected / 2 lost      */
} WS_Overlay;

u8   ws_breath(u32 ms, u16 period_ms, u8 lo, u8 hi);
void ws_render_idle(const WS_Overlay *ov, u32 ms);
```

**同时在 `_gen_ws_test.py` 的 `PRELUDE` 里补上 `WS_Overlay` 的 typedef**（与 Task 2 补 `WS_SpeedOut` 同理 —— 抽取出的逻辑区用到该类型，缺了就是 `unknown type name 'WS_Overlay'`）。把 `ws2811.h` 里那个结构体原样抄进 PRELUDE 即可。

`ws2811.c` 的 `WS_LOGIC` 区追加：

```c
#define WS_IDLE_PERIOD_MS   2000    /* breathe period, slow on purpose  */
#define WS_IDLE_LO          13      /* ~5%  of 255                      */
#define WS_IDLE_HI          115     /* ~45% of 255                      */
#define WS_LOWBAT_PERIOD_MS 4000    /* low battery breathes slower      */
#define WS_LOWBAT_CAP       51      /* ~20% of 255, power saving        */
#define WS_LOST_PERIOD_MS   800     /* purple blink period              */
#define WS_ORBIT_PERIOD_MS  3000
#define WS_ORBIT_TAIL       3       /* pixels in the comet tail         */

u8 ws_breath(u32 ms, u16 period_ms, u8 lo, u8 hi)
{
	u32 ph   = ms % period_ms;
	u32 half = period_ms / 2;
	u32 k    = (ph < half) ? ph : (period_ms - ph);     /* 0..half */
	if (half == 0) return hi;
	return (u8)((u32)lo + ((u32)(hi - lo) * k) / half);
}

/* Idle base colour: status drives the hue, breathing drives brightness. */
static void ws_idle_color(const WS_Overlay *ov, u32 ms, u8 *r, u8 *g, u8 *b)
{
	u16 hue;
	u8  bright;

	if (ov->lowbat) {
		hue    = 30;                                  /* orange       */
		bright = ws_breath(ms, WS_LOWBAT_PERIOD_MS, WS_IDLE_LO, WS_LOWBAT_CAP);
	} else if (ov->ros_link == 2) {
		hue    = 285;                                 /* purple, blink */
		bright = ((ms % WS_LOST_PERIOD_MS) < WS_LOST_PERIOD_MS / 2)
		       ? WS_IDLE_HI : WS_IDLE_LO;
	} else if (ov->ros_link == 0) {
		hue    = 50;                                  /* yellow: wait  */
		bright = ws_breath(ms, WS_IDLE_PERIOD_MS, WS_IDLE_LO, WS_IDLE_HI);
	} else {
		hue    = 160;                                 /* cyan: normal  */
		bright = ws_breath(ms, WS_IDLE_PERIOD_MS, WS_IDLE_LO, WS_IDLE_HI);
	}
	ws_hsv2rgb(hue, 255, bright, r, g, b);
}

/* Comet running around the ring. Position goes through ws_map so a strip
 * glued on backwards only reverses the chase, not its shape. */
static void ws_idle_orbit(u32 ms, u8 r, u8 g, u8 b)
{
	u32 head = (ms % WS_ORBIT_PERIOD_MS) * WS_PIXELS / WS_ORBIT_PERIOD_MS;
	u8  i;

	ws_clear();
	for (i = 0; i < WS_ORBIT_TAIL; i++) {
		u8 atten = (u8)(255 / (i + 1));             /* 255, 127, 85 */
		u8 phys  = ws_map((u8)((head + WS_PIXELS - i) % WS_PIXELS));
		ws_put(phys,
		       (u8)((u32)r * atten / 255),
		       (u8)((u32)g * atten / 255),
		       (u8)((u32)b * atten / 255));
	}
}

void ws_render_idle(const WS_Overlay *ov, u32 ms)
{
	u8 r, g, b;

	ws_idle_color(ov, ms, &r, &g, &b);
	if (ov->ros_link == 1 && !ov->lowbat) {
		ws_fill_solid(r, g, b);        /* calm steady ring: the default */
	} else {
		ws_idle_orbit(ms, r, g, b);    /* otherwise something wants attention */
	}
}
```

- [ ] **Step 4: 运行确认通过**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 全部 `ok`。

> **本任务最容易踩的坑**：`ros_link ∈ {0, 2}` 和 `lowbat` 三种状态都走**环绕流光**分支，只点亮 3 个像素，且**点哪 3 个取决于 `ms`**。所以这些用例必须用 `frame_brightest()` 取最亮的那个已点亮像素，**不能** `frame_get(0, ...)` —— 0 号像素在 `ms=1000` 时（流光头部在 13）是全黑的 `(0,0,0)`，任何颜色断言都会挂。
>
> 只有 `ros_link == 1 且 !lowbat` 那条走 `ws_fill_solid`，整圈同色，才可以直接读 0 号像素。
>
> `frame_brightest()` 定义在 `_ws_tests.c` 里（在 `case_idle` 之前），**不要放进 `_gen_ws_test.py` 的 `HELPERS`**（那段是冻结的）。

---

### Task 6: L2 运动层 + L3 叠加层 + 合成渲染

**Files:**
- Modify: `HARDWARE/ws2811.c`
- Modify: `HARDWARE/ws2811.h`
- Modify: `_ros_tooling/_ws_tests.c`

**Interfaces:**
- Consumes: 全部前述
- Produces: `void ws_render(const WS_State *st, WS_SpeedOut sp, const WS_Overlay *ov, u32 ms)` — 唯一对外渲染入口，返回后 `ws_frame` 即就绪

- [ ] **Step 1: 写失败测试**

```c
static void case_render(void)
{
	WS_State st;
	WS_SpeedOut sp;
	WS_Overlay ov;
	u8 r, g, b, k, same;

	ws_state_init(&st);
	memset(&ov, 0, sizeof(ov)); ov.nav_permille = 0xFFFF; ov.ros_link = 1;

	/* (a) blend = 0 -> pure idle layer */
	sp = ws_speed_map(0.0f);
	ws_render(&st, sp, &ov, 1000);
	frame_get(0, &r, &g, &b);
	if (!is_cyanish(r, g, b)) {
		printf("FAIL (a) blend=0 not idle %u %u %u\n", r, g, b); g_fail++;
	} else printf("ok   (a) blend=0 -> idle cyan %u %u %u\n", r, g, b);

	/* (b) blend = 255 at full speed -> pure motion layer, red */
	st.blend = 255; st.moving = 1;
	sp = ws_speed_map(0.80f);
	ws_render(&st, sp, &ov, 1000);
	frame_get(0, &r, &g, &b);
	if (!is_reddish(r, g, b)) {
		printf("FAIL (b) full speed not red %u %u %u\n", r, g, b); g_fail++;
	} else printf("ok   (b) blend=255 v=0.8 -> red %u %u %u\n", r, g, b);

	/* (c) motion ring must be uniform */
	same = 1;
	for (k = 1; k < WS_PIXELS; k++) {
		u8 r2, g2, b2;
		frame_get(k, &r2, &g2, &b2);
		if (r2 != r || g2 != g || b2 != b) same = 0;
	}
	if (!same) { printf("FAIL (c) motion ring not uniform\n"); g_fail++; }
	else ok("(c) motion ring is uniform");

	/* (d) estop beats everything: pure red, no green/blue anywhere */
	ov.estop = 1;
	ws_render(&st, sp, &ov, 1000);      /* 1000 % 125 = 0 -> lit phase */
	{
		u8 bad = 0;
		for (k = 0; k < WS_PIXELS; k++) {
			frame_get(k, &r, &g, &b);
			if (g > 10 || b > 10) bad = 1;
		}
		if (bad) { printf("FAIL (d) estop leaked colour\n"); g_fail++; }
		else ok("(d) estop forces pure red");
	}
	ov.estop = 0;

	/* (e) low battery overrides the motion layer and caps brightness.
	 * It routes through the idle renderer, so sample the brightest lit
	 * pixel -- pixel 0 is dark whenever the comet is elsewhere. */
	ov.lowbat = 1;
	ws_render(&st, sp, &ov, 1000);
	if (!frame_brightest(&r, &g, &b) || !is_orangeish(r, g, b) || r > 53) {
		printf("FAIL (e) lowbat %u %u %u\n", r, g, b); g_fail++;
	} else printf("ok   (e) lowbat orange, capped r=%u\n", r);
	ov.lowbat = 0;

	/* (f) brake forces red even at zero speed. 900ms lands in the lit
	 * phase of the 150ms flash (900 % 150 == 0). */
	sp = ws_speed_map(0.0f);
	ov.brake = 1;
	ws_render(&st, sp, &ov, 900);
	frame_get(0, &r, &g, &b);
	if (!is_reddish(r, g, b)) {
		printf("FAIL (f) brake %u %u %u\n", r, g, b); g_fail++;
	} else printf("ok   (f) brake -> red %u %u %u\n", r, g, b);
	ov.brake = 0;

	/* (g) turn indicator must break ring uniformity */
	ov.turn = 1;
	ws_render(&st, sp, &ov, 1000);
	{
		u8 uniform = 1;
		frame_get(0, &r, &g, &b);
		for (k = 1; k < WS_PIXELS; k++) {
			u8 r2, g2, b2;
			frame_get(k, &r2, &g2, &b2);
			if (r2 != r || g2 != g || b2 != b) uniform = 0;
		}
		if (uniform) { printf("FAIL (g) turn indicator invisible\n"); g_fail++; }
		else ok("(g) turn indicator varies around the ring");
	}

	/* (g2) the two directions must produce different frames */
	{
		u8 left[WS_PIXELS * 3], right[WS_PIXELS * 3], diff = 0;
		ov.turn = 1;  ws_render(&st, sp, &ov, 1000);
		memcpy(left, ws_frame, sizeof(left));
		ov.turn = -1; ws_render(&st, sp, &ov, 1000);
		memcpy(right, ws_frame, sizeof(right));
		for (k = 0; k < WS_PIXELS * 3; k++) if (left[k] != right[k]) diff = 1;
		if (!diff) { printf("FAIL (g2) left frame == right frame\n"); g_fail++; }
		else ok("(g2) left and right differ");
	}
	ov.turn = 0;

	/* (i) REGRESSION GUARD: the idle comet must reach the strip through
	 * ws_render, not only through ws_render_idle. An earlier revision of
	 * ws_render filled a solid idle colour here, which left the orbit
	 * effect unreachable from the task and therefore dead on the car. */
	{
		WS_State   s2;
		WS_Overlay o2;
		u8 uniform = 1;

		ws_state_init(&s2);                          /* blend = 0 */
		memset(&o2, 0, sizeof(o2));
		o2.nav_permille = 0xFFFF;
		o2.ros_link     = 2;                         /* lost -> comet */
		ws_render(&s2, ws_speed_map(0.0f), &o2, 300);

		frame_get(0, &r, &g, &b);
		for (k = 1; k < WS_PIXELS; k++) {
			u8 r2, g2, b2;
			frame_get(k, &r2, &g2, &b2);
			if (r2 != r || g2 != g || b2 != b) uniform = 0;
		}
		if (uniform) {
			printf("FAIL (i) idle comet does not reach ws_render\n");
			g_fail++;
		} else ok("(i) idle comet survives into ws_render");
	}

	/* (h) brute force: no crash, no out-of-range write, over long sweeps */
	{
		u32 ms;
		int pass;
		for (pass = 0; pass < 2; pass++) {
			ws_state_init(&st);
			st.blend = pass ? 255 : 0;
			for (ms = 0; ms < 8000; ms += 37) {
				memset(&ov, 0, sizeof(ov));
				ov.nav_permille = (pass ? 0xFFFF : (u16)(ms % 1001));
				ov.ros_link     = (u8)(ms % 3);
				ov.turn         = (s8)((ms / 500) % 3) - 1;
				ov.brake        = (u8)((ms / 300) % 2);
				ov.estop        = (u8)((ms / 700) % 2);
				ov.lowbat       = (u8)((ms / 900) % 2);
				ov.reverse      = (u8)((ms / 1100) % 2);
				sp = ws_speed_map((float)(ms % 100) / 100.0f);
				ws_render(&st, sp, &ov, ms);
			}
		}
		ok("(h) brute force survived 2 x 217 frames");
	}
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: gcc 失败，`implicit declaration of function 'ws_render'`。

- [ ] **Step 3: 实现**

`ws2811.h` 加 `void ws_render(const WS_State *st, WS_SpeedOut sp, const WS_Overlay *ov, u32 ms);`

`ws2811.c` 的 `WS_LOGIC` 区追加：

```c
#define WS_TURN_PERIOD_MS   250     /* 4Hz sweep                        */
#define WS_TURN_SIDE        10      /* pixels lit on the active arc     */
#define WS_TURN_HUE         40      /* amber                            */
#define WS_BRAKE_PERIOD_MS  150
#define WS_BRAKE_DIM        90      /* the dark phase of the flash      */
#define WS_ESTOP_PERIOD_MS  125     /* 8Hz strobe                       */
#define WS_NAV_DONE_HUE     210     /* deep blue: already covered       */
#define WS_NAV_HEAD_HUE     195     /* bright cyan: the moving comet    */

/* t = 0 -> a, t = 255 -> b */
static u8 ws_mix(u8 a, u8 b, u8 t)
{
	return (u8)(((u32)a * (255u - t) + (u32)b * (u32)t) / 255u);
}

/* Directional flow drawn ON TOP of the motion base colour, so the speed
 * reading survives while the turn direction is added. */
static void ws_ov_turn(s8 dir, u32 ms)
{
	u32 head = (ms % WS_TURN_PERIOD_MS) * WS_PIXELS / WS_TURN_PERIOD_MS;
	u8  i;

	for (i = 0; i < WS_TURN_SIDE; i++) {
		u32 logical = (dir > 0) ? (head + i) : (head + WS_PIXELS - i);
		u8  ar, ag, ab;
		ws_hsv2rgb(WS_TURN_HUE, 255, 255, &ar, &ag, &ab);
		ws_put(ws_map((u8)(logical % WS_PIXELS)), ar, ag, ab);
	}
}

static void ws_ov_reverse(u32 ms)
{
	u32 head = (ms % WS_ORBIT_PERIOD_MS) * WS_PIXELS / WS_ORBIT_PERIOD_MS;
	u8  i;

	ws_clear();
	for (i = 0; i < WS_ORBIT_TAIL; i++) {
		u8 atten = (u8)(255 / (i + 1));
		u8 phys  = ws_map((u8)((head + i) % WS_PIXELS));
		ws_put(phys, atten, atten, atten);
	}
}

static void ws_ov_nav(u16 permille, u32 ms)
{
	u8 lit, i, head;
	u8 dr, dg, db, hr, hg, hb;

	ws_clear();
	if (permille > 1000) permille = 1000;
	lit  = (u8)((u32)permille * WS_PIXELS / 1000);
	head = (u8)((ms / 100) % WS_PIXELS);
	ws_hsv2rgb(WS_NAV_DONE_HUE, 255, 255, &dr, &dg, &db);
	ws_hsv2rgb(WS_NAV_HEAD_HUE, 255, 255, &hr, &hg, &hb);

	for (i = 0; i < WS_PIXELS; i++) {
		u8 phys = ws_map(i);
		if (i < lit)        ws_put(phys, dr, dg, db);
		else if (i == head) ws_put(phys, hr, hg, hb);
	}
}

void ws_render(const WS_State *st, WS_SpeedOut sp, const WS_Overlay *ov, u32 ms)
{
	u8 mr, mg, mb, i, bl = st->blend;

	/* ---- L3 full-band overrides, highest priority first ---- */
	if (ov->estop) {
		u8 on = ((ms % WS_ESTOP_PERIOD_MS) < WS_ESTOP_PERIOD_MS / 2) ? 255 : 0;
		ws_fill_solid(on, 0, 0);
		return;
	}
	if (ov->lowbat) { ws_render_idle(ov, ms); return; }
	if (ov->brake) {
		u8 on = ((ms % WS_BRAKE_PERIOD_MS) < WS_BRAKE_PERIOD_MS * 2 / 3)
		      ? 255 : WS_BRAKE_DIM;
		ws_fill_solid(on, 0, 0);
		return;
	}
	if (ov->reverse) { ws_ov_reverse(ms); return; }
	if (ov->nav_permille != 0xFFFF) { ws_ov_nav(ov->nav_permille, ms); return; }

	/* ---- L2 motion colour, fully opaque: fast path ---- */
	ws_hsv2rgb(sp.hue, 255, sp.bright, &mr, &mg, &mb);
	if (bl >= 255) {
		ws_fill_solid(mr, mg, mb);
	} else {
		/* L1 idle layer paints FIRST, comet included, then the motion
		 * colour is crossfaded in per pixel.
		 *
		 * Delegating to ws_render_idle -- rather than filling a solid
		 * idle colour -- is what actually lets the comet reach the
		 * strip. An earlier revision called ws_idle_color() directly
		 * here, which made ws_render_idle() reachable only from the
		 * test suite: the orbit effect was dead code on the real car. */
		ws_render_idle(ov, ms);
		if (bl > 0) {
			for (i = 0; i < WS_PIXELS; i++) {
				u8 *p = &ws_frame[(u16)i * 3];
#if WS_ORDER_GRB
				p[0] = ws_mix(p[0], mg, bl);
				p[1] = ws_mix(p[1], mr, bl);
				p[2] = ws_mix(p[2], mb, bl);
#else
				p[0] = ws_mix(p[0], mr, bl);
				p[1] = ws_mix(p[1], mg, bl);
				p[2] = ws_mix(p[2], mb, bl);
#endif
			}
		}
	}

	/* ---- L3 directional overlay ---- */
	if (ov->turn != 0) ws_ov_turn(ov->turn, ms);
}
```

- [ ] **Step 4: 运行确认通过**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 全部 `ok`（a..h）。

> `lowbat` 分支走 `ws_idle_color`，它在 `lowbat=1` 时返回橙色 —— 即使车在动也不显示速度色。这是**有意为之**：低压是安全状态，优先级高于速度显示。若实测觉得太霸道，可降级成"叠加"而非"覆盖"，但那样必须同步改测试 (e)。

- [ ] **Step 5: 可选 —— 地址消毒器再跑一遍**

若本机 gcc 带 ASan，手工跑一次以确认帧缓冲无越界：

```bash
cd "_ros_tooling" && gcc -std=c99 -g -fsanitize=address,undefined \
  -Wall -Wextra -o _ws_host_test_asan.exe _ws_host_test.c && ./_ws_host_test_asan.exe
```
Expected: 与 Step 4 相同输出，且无 `runtime error` / `heap-buffer-overflow`。
若本机无 ASan，跳过即可 —— 非阻塞。

---

### Task 7: SPI1 + DMA 硬件层（上板验证，无宿主机测试）

**Files:**
- Modify: `HARDWARE/ws2811.c`（`WS_HW` 区）
- Modify: `HARDWARE/ws2811.h`

**Interfaces:**
- Consumes: `ws_frame`, `ws_render`, `ws_state_step`, `ws_speed_map`, `ws_render_calib`，以及固件全局 `MOTOR_A/B/C/D.Encoder`、`Voltage`、`Flag_Stop`、`Avoid_ON_Flag`、`Move_Z`、`RosFault`、`RosDown`
- Produces: `void ws2811_task(void *pvParameters)`

> **本任务大部分无法宿主机测试，但有一个重要例外。**
>
> WS2811 的 3-bit 编码被拆成纯函数 `ws_pack_byte()` 放进**逻辑区**，可以用断言验证（Step 1–2）。这是全任务最容易出错、又最难在板子上排查的部分：编码错了，示波器上照样是一串脉冲，但灯带显示乱码或毫无反应，你会先去怀疑电平、接线、灯带好坏。
>
> 剩下的部分（SPI 时序、DMA 触发、PA7 复用、电平余量）**只能上板用示波器/肉眼确认**，那是检查清单，不是断言（Step 5–6）。

- [ ] **Step 1: 编码函数拆两半 —— 纯逻辑进 `WS_LOGIC`，DMA 打包留 `WS_HW`**

**先在 `ws2811.c` 的 `WS_LOGIC` 区（`WS_LOGIC_END` 之前）追加编码核心：**

```c
/**************************************************************************
Function: Expand one source byte into the three SPI bytes that carry it.
          One WS2811 bit -> three SPI bits at 2.625MHz (0.381us each):
             0 -> 100  = 0.381us high, 0.762us low  (spec 0.40/0.85)
             1 -> 110  = 0.762us high, 0.381us low  (spec 0.80/0.45)
          MSB first, so source bit7 becomes the most significant SPI group:
          8 source bits -> 24 SPI bits -> exactly 3 bytes.

          Kept out of ws_pack() so the host suite can assert the bit
          pattern. A wrong pattern still emits *pulses*, it just makes the
          strip show garbage -- which sends you hunting for a level-shifter
          or a bad joint instead of a bad shift.
**************************************************************************/
void ws_pack_byte(u8 src, u8 out[3])
{
	u32 acc = 0;
	u8  bit;

	for (bit = 0; bit < 8; bit++) {
		u32 code = ((src >> (7 - bit)) & 1u) ? 0x6u : 0x4u;   /* 110 : 100 */
		acc = (acc << 3) | code;
	}
	out[0] = (u8)(acc >> 16);
	out[1] = (u8)(acc >> 8);
	out[2] = (u8)(acc);
}
```

`ws2811.h` 加 `void ws_pack_byte(u8 src, u8 out[3]);`。

**然后在 `ws2811.c` 的 `WS_HW` 区（`ws2811_task` 之前）写 DMA 打包：**

```c
#include "delay.h"
#include "adc.h"

/* Each pixel is 3 channels x 8 bits = 24 WS2811 bits; every WS2811 bit
 * expands to 3 SPI bits, so one pixel costs 24*3/8 = 9 SPI BYTES.
 *
 * DO NOT write (WS_PIXELS * 24 * 3) here. That is the SPI BIT count
 * (2880), not the byte count. Sizing the buffer and the DMA transfer to
 * it makes every frame 8x too long -- 8.78ms instead of 1.10ms -- and
 * appends 2520 bytes of junk to the end of the bitstream. */
#define WS_SPI_BYTES    (WS_PIXELS * 9)          /* 360 bytes */

/* Trailing zero SPI bytes. One 0x00 byte is 8 low bit-times = 3.05us of
 * CONTINUOUS low on MOSI, so N bytes give 3.05*N us of low level -- and
 * that low period is what actually latches the strip (WS2811 wants
 * >50us). 20 bytes = 61us leaves only 11us of margin, so use 100 bytes
 * = 305us; it costs 100 bytes and 305us out of a 40ms frame.
 *
 * IMPORTANT: these bytes are NOT "20 WS2811 zero bits". They are never
 * grouped into WS bit triples, they are raw low SPI bits. Analysing them
 * as WS bit patterns (100 100 100 ...) and concluding "that is a pulse
 * train, not a low level" is the wrong frame -- see the arithmetic
 * above. Do not "fix" this loop by deleting it. */
#define WS_RESET_BYTES  100

/* Compile-time guard so the 8x mistake can never come back unnoticed.
 * The byte count is exactly 1/8 of the naive bit count; if someone
 * "simplifies" WS_SPI_BYTES back to the bit expression, this fires. */
#if WS_SPI_BYTES == (WS_PIXELS * 24 * 3)
#error "WS_SPI_BYTES is the SPI BIT count, not the BYTE count (should be WS_PIXELS * 9)"
#endif

static u8  s_tx[WS_SPI_BYTES + WS_RESET_BYTES];
static volatile u8 s_busy = 0;

/**************************************************************************
Function: Pack the whole frame into s_tx.
          The 256-entry LUT is built by CALLING ws_pack_byte, so the bit
          pattern has exactly one definition and the host test covers it.
**************************************************************************/
static void ws_pack(void)
{
	static u8 lut[256][3];
	static u8 lut_ready = 0;
	u16 i, o = 0;

	if (!lut_ready) {
		u16 v;
		for (v = 0; v < 256; v++) ws_pack_byte((u8)v, lut[v]);
		lut_ready = 1;
	}

	/* Global brightness ceiling, applied here at the last mile so every
	 * effect obeys it without each renderer having to know about the cap.
	 * WS_BRIGHT_MAX is an 8-bit ceiling: 255 = full, 0 = dark.
	 * THIS IS THE ONLY PLACE THE MACRO TAKES EFFECT -- if you drop these
	 * lines the macro is dead and every power-saving claim in the design
	 * doc becomes false. */
	for (i = 0; i < WS_PIXELS * 3; i++) {
		u8 v = (u8)(((u32)ws_frame[i] * WS_BRIGHT_MAX) / 255u);
		s_tx[o++] = lut[v][0];
		s_tx[o++] = lut[v][1];
		s_tx[o++] = lut[v][2];
	}
	for (i = 0; i < WS_RESET_BYTES; i++) s_tx[o++] = 0x00;
}
```

- [ ] **Step 2: 宿主机断言编码 —— 期望值按位手算，不要靠跑一遍反推**

在 `_ws_tests.c` 加 `case_pack()`，并在 `main()` 里 `case_render();` 之后调用：

```c
static void case_pack(void)
{
	/* Hand-expanded bit patterns. Each source bit becomes a 3-bit group
	 * (110 for 1, 100 for 0), MSB first, 8 groups = 24 bits = 3 bytes. */
	struct { u8 src, b0, b1, b2; const char *n; } c[] = {
		{0xFF, 0xDB, 0x6D, 0xB6, "all ones"   },
		{0x00, 0x92, 0x49, 0x24, "all zeros"  },
		{0x80, 0xD2, 0x49, 0x24, "msb only"   },
		{0x01, 0x92, 0x49, 0x26, "lsb only"   },
		{0xAA, 0xD3, 0x4D, 0x34, "alternating"},
	};
	unsigned k;

	for (k = 0; k < sizeof(c) / sizeof(c[0]); k++) {
		u8 out[3] = {0, 0, 0};
		ws_pack_byte(c[k].src, out);
		if (out[0] != c[k].b0 || out[1] != c[k].b1 || out[2] != c[k].b2) {
			printf("FAIL pack %-11s src=%02X -> %02X %02X %02X want %02X %02X %02X\n",
			       c[k].n, c[k].src, out[0], out[1], out[2],
			       c[k].b0, c[k].b1, c[k].b2);
			g_fail++;
		} else {
			printf("ok   pack %-11s src=%02X -> %02X %02X %02X\n",
			       c[k].n, c[k].src, out[0], out[1], out[2]);
		}
	}

	/* Every 3-bit group must be 110 or 100 -- never 000, 111, 010 or 101.
	 * That is the property the WS2811 decoder actually cares about, and it
	 * catches an off-by-one in the shift that a 5-vector table might miss. */
	{
		u16 v;
		int bad = 0;
		for (v = 0; v < 256; v++) {
			u8 out[3], g;
			u32 acc;
			ws_pack_byte((u8)v, out);
			acc = ((u32)out[0] << 16) | ((u32)out[1] << 8) | (u32)out[2];
			for (g = 0; g < 8; g++) {
				u32 grp = (acc >> (21 - 3 * g)) & 0x7u;
				/* group must match its source bit: 1 -> 110, 0 -> 100 */
				u32 want = ((v >> (7 - g)) & 1u) ? 0x6u : 0x4u;
				if (grp != want) bad++;
			}
		}
		if (bad) { printf("FAIL pack: %d wrong 3-bit groups over 256 bytes\n", bad); g_fail++; }
		else ok("pack: all 2048 groups correct over all 256 source bytes");
	}
}
```

- [ ] **Step 3: 运行宿主机测试，确认全绿**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 新增 6 条 `ok`（5 个向量 + 1 条全字节扫描），总计 62 条，`ALL PASSED`，退出码 0，gcc stderr 0 字节。

> 若 `all ones` 或 `all zeros` 这两条就错了，说明位序反了（即 SPI 配成了 LSB-first，或 `ws_pack_byte` 里取位的方向反了）。**先查这两条**，它们是最纯粹的信号。

- [ ] **Step 4: SPI1 + DMA2 初始化**

```c
static void ws_spi_init(void)
{
	GPIO_InitTypeDef g;
	SPI_InitTypeDef  s;
	DMA_InitTypeDef  d;
	NVIC_InitTypeDef n;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* PA7 = SPI1_MOSI, AF5 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	g.GPIO_Pin   = GPIO_Pin_7;
	g.GPIO_Mode  = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Speed = GPIO_Speed_100MHz;
	g.GPIO_PuPd  = GPIO_PuPd_UP;      /* keeps DIN defined while idle */
	GPIO_Init(GPIOA, &g);

	/* 84MHz APB2 / 32 = 2.625MHz, MSB first, mode 0 */
	SPI_I2S_DeInit(SPI1);
	s.SPI_Direction         = SPI_Direction_1Line_Tx;
	s.SPI_Mode              = SPI_Mode_Master;
	s.SPI_DataSize          = SPI_DataSize_8b;
	s.SPI_CPOL              = SPI_CPOL_Low;
	s.SPI_CPHA              = SPI_CPHA_1Edge;
	s.SPI_NSS               = SPI_NSS_Soft;
	s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	s.SPI_FirstBit          = SPI_FirstBit_MSB;
	s.SPI_CRCPolynomial     = 7;
	SPI_Init(SPI1, &s);
	SPI_Cmd(SPI1, ENABLE);

	/* SPI1_TX lives on DMA2 Stream3 or Stream5, both Channel 3. Stream5 is
	 * free: lidar.c holds DMA1_Stream0, and adc.c runs with
	 * ADC_DMAAccessMode_Disabled so it holds nothing. */
	DMA_DeInit(DMA2_Stream5);
	d.DMA_Channel            = DMA_Channel_3;
	d.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;
	d.DMA_Memory0BaseAddr    = (u32)s_tx;
	d.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
	d.DMA_BufferSize         = sizeof(s_tx);
	d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	d.DMA_Mode               = DMA_Mode_Normal;
	d.DMA_Priority           = DMA_Priority_High;
	d.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	d.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	d.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	d.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &d);
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);

	n.NVIC_IRQChannel                   = DMA2_Stream5_IRQn;
	n.NVIC_IRQChannelPreemptionPriority = 6;
	n.NVIC_IRQChannelSubPriority        = 0;
	n.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&n);
}
```

- [ ] **Step 5: 中断、`ws_show()` 与任务主体**

```c
void DMA2_Stream5_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5)) {
		DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
		DMA_Cmd(DMA2_Stream5, DISABLE);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) { }
		SPI_Cmd(SPI1, DISABLE);
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
		s_busy = 0;
	}
}

static void ws_show(void)
{
	if (s_busy) return;              /* drop the frame, never block */
	ws_pack();
	s_busy = 1;
	DMA_Cmd(DMA2_Stream5, DISABLE);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
	DMA_SetCurrDataCounter(DMA2_Stream5, sizeof(s_tx));
	/* Order matters: enable the SPI first so TXE is already 1 when the
	 * DMA stream starts, which makes the SPI issue its first TX request
	 * right away.
	 *
	 * Do NOT add a manual SPI_I2S_SendData() to "prime" anything here.
	 * In DMA mode a CPU write to DR is either dropped (when TXE is low)
	 * or injects an extra byte -- and one extra byte shifts the entire
	 * 960-bit frame by 8 bits, turning every pixel into garbage. */
	SPI_Cmd(SPI1, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
}

static float ws_wheel_speed(void)
{
	float a = MOTOR_A.Encoder < 0 ? -MOTOR_A.Encoder : MOTOR_A.Encoder;
	float b = MOTOR_B.Encoder < 0 ? -MOTOR_B.Encoder : MOTOR_B.Encoder;
	float c = MOTOR_C.Encoder < 0 ? -MOTOR_C.Encoder : MOTOR_C.Encoder;
	float d = MOTOR_D.Encoder < 0 ? -MOTOR_D.Encoder : MOTOR_D.Encoder;
	return (a + b + c + d) * 0.25f;
}

static void ws_collect_overlay(WS_Overlay *ov, u32 ms)
{
	static u32   brake_until = 0;
	static float prev = 0.0f;
	float v     = ws_wheel_speed();
	float decel = prev - v;
	prev = v;

	ov->estop  = (Flag_Stop || Avoid_ON_Flag) ? 1 : 0;
	ov->lowbat = (Voltage < WS_LOWBAT_V) ? 1 : 0;
	ov->ros_link = (RosFault & ROS_FAULT_TIMEOUT) ? 2
	             : (RosDown.rx_ready ? 1 : 0);
	ov->reverse  = 0;              /* no reverse channel on this chassis */
	ov->nav_permille = 0xFFFF;     /* no odometry feed yet, see plan notes */
	ov->turn = 0;
	if (Move_Z > 0.20f)       ov->turn =  1;
	else if (Move_Z < -0.20f) ov->turn = -1;

	if (decel > 0.15f) brake_until = ms + 450;   /* three 150ms flashes */
	ov->brake = (ms < brake_until) ? 1 : 0;
}

void ws2811_task(void *pvParameters)
{
	WS_State    st;
	WS_Overlay  ov;
	WS_SpeedOut sp;
	u32 ms = 0;
#if WS_CALIB_MODE
	u8 calib = 0;
#endif

	(void)pvParameters;
	ws_spi_init();
	ws_state_init(&st);

	for (;;) {
		float v = ws_wheel_speed();

		ws_state_step(&st, v);
		memset(&ov, 0, sizeof(ov));
		ov.nav_permille = 0xFFFF;
		ws_collect_overlay(&ov, ms);
		sp = ws_speed_map(v);

#if WS_CALIB_MODE
		ws_render_calib(calib);
		calib = (u8)((calib + 1) % WS_PIXELS);
#else
		ws_render(&st, sp, &ov, ms);
#endif
		ws_show();

		/* FIXED 40ms tick -- do not make this adaptive.
		 *
		 * An earlier revision dropped to 100ms when parked to "save
		 * power". But ws_state_step() is hard-calibrated for a 40ms
		 * step: WS_FADE_STEP, the 100ms enter dwell and the 500ms exit
		 * dwell all assume it. At a 100ms tick the 500ms exit
		 * hysteresis silently became ~1250ms, so the ring stayed in
		 * motion mode for well over a second after stopping.
		 *
		 * The saving was imaginary anyway: the MCU sits in vTaskDelay
		 * either way, and the strip's current is set by WS_BRIGHT_MAX,
		 * not by the refresh rate. */
		vTaskDelay(WS_STEP_MS);
		ms += WS_STEP_MS;
	}
}
```

- [ ] **Step 6: 上板验证清单（替代单元测试）**

按顺序做，每步肉眼/示波器确认，**任一步不过就停下修，不要往下走**：

1. **不接灯带**，PA7 接示波器。烧录后应看到 2880 字节的脉冲串，bit 周期 0.381µs，高电平 0.381µs（逻辑 0）或 0.762µs（逻辑 1）。让车静止在全青状态便于测量。
2. 确认帧尾有 **>50µs 低电平**（`WS_RESET_BYTES × 8 × 0.381µs = 61µs`）。
3. 接上灯带，**只接数据线，灯带单独供电，务必共地**。上电应看到待机青色呼吸。
4. 若颜色错乱（该红变绿）→ 把 `WS_ORDER_GRB` 取反重烧。
5. 若完全无反应 → 大概率是 3.3V 电平不够，按设计文档 §1.3 加 74AHCT1G125。
6. 手动转轮子，确认颜色由青→绿→黄→红连续变化，且**起步阶段靠亮度变化可见**。
7. 松手停止，确认约 500ms 后回到呼吸，且**过渡是渐变的不是跳变**。
8. 打开 `WS_CALIB_MODE` 重烧，确认只有一个白点绕圈。记下"车头正前方"那格的编号 → 写入 `WS_PHASE`；确认旋转方向 → 写入 `WS_DIR`。改回 `0`。
9. **量帧长**。一整帧应当是 **1.10ms 左右**（360 字节 @ 2.625MHz），帧尾另有 305µs 低电平。若量到约 8.8ms，说明 `WS_SPI_BYTES` 又被写成了 SPI **位数**而不是字节数 —— 这是最容易复发的一处。
10. **量帧间空闲电平**。`ws_show()` 结束时会 `SPI_Cmd(SPI1, DISABLE)`，之后 PA7 的电平由复用功能决定。**如果空闲时是高电平，灯带会锁存不上**（WS2811 靠 >50µs 低电平复位）。本设计是靠帧尾那 305µs 的 0x00 字节完成复位的，理论上够；但仍要上示波器确认空闲电平确实是低，否则要改成在 ISR 里把 PA7 切回 GPIO 并主动拉低。
11. **确认静止时的行为**。停下车后约 500ms 应回到待机呼吸（**不是** 1.2 秒 —— 循环已固定为 40ms 步长）。如果明显超过 1 秒，说明 `vTaskDelay` 那行又被改成了变周期。

- [ ] **Step 7: 功耗确认**

灯带单独供电、串万用表测电流。全红（极速）应约 0.8A，全白应约 2.4A。若 DC-DC 撑不住全白，把 `WS_BRIGHT_MAX` 从 80 降到 50 —— 注意这是**全局**上限，会同时压低所有灯效。

---

### Task 8: 工程接入（字节级修改 + Keil 工程文件）

**Files:**
- Create: `_ros_tooling/_apply_ws2811.py`
- Modify: `BALANCE/system.h`（GBK，字节级）
- Modify: `USER/main.c`（GBK，字节级）
- Modify: `USER/WHEELTEC.uvprojx`（UTF-8，字节级）

**Interfaces:**
- Consumes: `ws2811.h` 的 `WS2811_TASK_PRIO` / `WS2811_STK_SIZE` / `ws2811_task`
- Produces: 工程可编译、任务被调度

> **为什么必须写脚本而不是直接改文件**：这些文件是 GBK 编码，用文本工具改动会破坏中文注释（项目记忆 `l150pro-gbk-sources` 明确记录过这个坑）。脚本按字节替换，且**幂等**（重复运行不重复插入）。

- [ ] **Step 1: 写 `_ros_tooling/_apply_ws2811.py`**

> **先别被循环包含吓到**：补丁 1 之后会形成 `ws2811.h → system.h → ws2811.h` 的环。这在本工程里是**既有惯例**——`HARDWARE/LED.H` 同样 `#include "system.h"`，而 `system.h` 又 `#include "led.h"`。include guard 能正确解掉，且预处理器的展开顺序保证 `ws2811.h` 的宏定义体**最终会被执行到**（从 `main.c` 走：`system.h` → `led.h` → `ws2811.h` 首次进入并完整展开 → 回到 `system.h`）。唯一的前提是 **`system.h` 自己不能引用任何 `WS2811_*` 宏** —— 它现在不引用，补丁只加一行 `#include`，别顺手加别的。`ws2811.h` 顶部保留 `#include "system.h"`（它需要 `u8`/`u16`/`s8` 这些 typedef），**不要**为了"消除循环"把它删掉。

```python
# -*- coding: utf-8 -*-
"""Byte-level integration of the WS2811 effect engine.

   Every existing file touched here is GBK; a text editor would corrupt the
   Chinese comments. Idempotent: safe to re-run."""
import os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def patch(rel, old, new, tag):
    p = os.path.join(ROOT, rel)
    b = open(p, 'rb').read()
    # Every patch below has the shape new = old + <extra>, so `old` SURVIVES
    # inside `new`. Detecting "already applied" by the absence of `old`
    # (b.count(old) == 0) is therefore always false after the first run, and
    # a re-run would insert the block a SECOND time -- duplicate #include,
    # duplicate xTaskCreate. Detect by looking for `new` instead.
    if b.count(new) == 1:
        print('%-34s already applied' % tag)
        return
    if b.count(old) != 1:
        sys.exit('%s: anchor found %d times (want 1)' % (tag, b.count(old)))
    open(p, 'wb').write(b.replace(old, new, 1))
    print('%-34s patched' % tag)

# ---- 1. system.h: make the task macros visible to main.c ----------------
patch('BALANCE/system.h',
      b'#include "led.h"\r\n',
      b'#include "led.h"\r\n#include "ws2811.h"\r\n',
      'BALANCE/system.h include')

# ---- 2. main.c: start the task ------------------------------------------
old = (b'    xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,'
       b'      NULL, LED_TASK_PRIO,      NULL);'
       b'\t//LED light flashing task //LED\xb5\xc6\xc9\xc1\xcb\xb8\xc8\xce\xce\xf1\r\n')
new = (old.rstrip(b'\r\n') + b'\r\n'
       + b'#if WS2811_ENABLE\r\n'
       + b'    xTaskCreate(ws2811_task,   "ws2811_task",   WS2811_STK_SIZE,'
       + b'   NULL, WS2811_TASK_PRIO,   NULL);'
       + b'\t//WS2811 ring effect task\r\n'
       + b'#endif\r\n')
patch('USER/main.c', old, new, 'USER/main.c xTaskCreate')

# ---- 3. uvprojx: Keil will not compile ws2811.c otherwise ---------------
# NOTE: the .uvprojx is UTF-8 with bare LF (CRLF count is 0), unlike the
# GBK sources above which are CRLF. Verified: this anchor hits exactly 1.
old = (b'            <File>\n'
       b'              <FileName>LED.C</FileName>\n'
       b'              <FileType>1</FileType>\n'
       b'              <FilePath>..\\HARDWARE\\LED.C</FilePath>\n'
       b'            </File>\n')
new = old + (
       b'            <File>\n'
       b'              <FileName>ws2811.c</FileName>\n'
       b'              <FileType>1</FileType>\n'
       b'              <FilePath>..\\HARDWARE\\ws2811.c</FilePath>\n'
       b'            </File>\n')
patch('USER/WHEELTEC.uvprojx', old, new, 'WHEELTEC.uvprojx file entry')

print('\nall patches applied')
```

- [ ] **Step 2: 运行**

Run: `cd "_ros_tooling" && python _apply_ws2811.py`
Expected: 三行 `patched`，末行 `all patches applied`。

- [ ] **Step 3: 确认幂等（必须数出来，不能只看"没报错"）**

再跑一次同样的命令。Expected: 三行 `already applied`，无报错。

然后**逐个统计插入块出现的次数**，必须都是 1：

```bash
python -c "
for f, pat in [('BALANCE/system.h', b'#include \"ws2811.h\"'),
               ('USER/main.c',       b'ws2811_task'),
               ('USER/WHEELTEC.uvprojx', b'ws2811.c')]:
    b = open(f, 'rb').read()
    print(f, b.count(pat))
"
```
Expected: `BALANCE/system.h 1` / `USER/main.c 1` / `USER/WHEELTEC.uvprojx 1`。

> **为什么必须真的数**：三处补丁的形状都是 `new = old + 新增块`，因此 `old` 在应用之后**仍然存在于 `new` 内部**。任何"检查 `old` 是否已消失"的幂等守卫都会永远失效，重跑时把同一段插第二遍 —— 变成两行重复的 `#include`、或**两个同名 FreeRTOS 任务**。编译期完全看不出来（同名任务首次创建会成功），运行时表现为灯闪得飞快、CPU 占用翻倍、栈被吃掉两份。

- [ ] **Step 4: 确认 GBK 没被破坏**

```bash
python -c "print(open(r'USER/main.c','rb').read().decode('gbk')[-900:])"
```
Expected: 中文注释正常显示（如 `任务`、`LED闪烁任务`），不是 `锟斤拷`。

- [ ] **Step 5: 确认 uvprojx 仍是合法 XML**

```bash
python -c "import xml.etree.ElementTree as E; E.parse(r'USER/WHEELTEC.uvprojx'); print('XML OK')"
```
Expected: `XML OK`。

- [ ] **Step 6: Keil 编译 + 上板**

在 Keil 里打开 `USER/WHEELTEC.uvprojx`，确认左侧 HARDWARE 组下出现 `ws2811.c`，全量重编译。

Expected: **0 error，0 warning**。然后按 Task 7 Step 6 的上板清单跑一遍。

- [ ] **Step 7: 关闭功能验证（"只关入口不删代码"的回归）**

把 `HARDWARE/ws2811.h` 里 `WS2811_ENABLE` 改成 `0`，重新编译。Expected: 编译通过、`ws2811_task` 不再被创建、其余功能（电机、编码器、ROS 通信、OLED、舵机）全部不受影响。改回 `1`。

> 这是本项目一以贯之的验收标准：任何新功能都必须能"一键关掉且不留痕"。

---

### Task 9: PE0 按键灯效预览模式（可整体删除）

**Files:**
- Modify: `HARDWARE/ws2811.h`（`WS_DEMO_ENABLE` 宏 + demo API 声明）
- Modify: `HARDWARE/ws2811.c`（`WS_DEMO_LOGIC_BEGIN/END` + `WS_DEMO_HW_BEGIN/END` 两段 + 任务里 2 处 hook）
- Modify: `BALANCE/show.c`（**GBK，字节级**：oled_show() 顶部 3 行早退分支）
- Create: `_ros_tooling/_apply_ws_demo.py`（改 show.c 的字节级脚本）
- Modify: `_ros_tooling/_ws_tests.c`（`case_demo()`）

**Interfaces:**
- Consumes: `ws_render` / `ws_render_calib` / `ws_fill_solid` / `ws_state_step` / `ws_speed_map` / `click()`（`key.h` 已声明）
- Produces:
  - `u8  ws_demo_frame(float *v, WS_Overlay *ov, u32 ms)` — 返回 1 表示它自己画好了 `ws_frame`（标定/全白两档），返回 0 表示调用方照常 `ws_render()`
  - `u8  ws_demo_active(void)`
  - `void ws_demo_oled(void)` — 演示页整屏绘制，供 `show.c` 调用

**前置事实（已核实，不要重新假设）**
- `PE0` = `KEY_PIN`，已在 `system.c:176` 经 `KEY_Init()` 配好上拉输入。`KEY==0` 为按下。
- `key.c` 的 `click()` 是**按下沿锁存**（`flag_key`），40ms 轮询下可用；`click_N_Double()` 的调用点已被刻意移除，PE0 完全空闲。
- `DATA_IN_ExtSRAM` 是注释掉的，FSMC 未启用，PE0 无外设冲突。
- **OLED 六行 y=0/10/20/30/40/50 全部被 `oled_show()` 占用**，没有空行可蹭 → 演示模式必须**整屏接管**。
- `led_task` 在占用 PE8（1Hz 心跳），**不要用板载 LED 做提示**。

- [ ] **Step 1: 在 `ws2811.h` 加开关与声明**

```c
/* ---------------- PE0 preview / demo mode ---------------- */
/* 0 = demo code compiled out at its hooks, block kept intact.
 * Set to 0 to kill the feature without touching anything else. */
#define WS_DEMO_ENABLE      1
#define WS_DEMO_STAGES      15      /* 1..15, plus 0 = off */

#if WS_DEMO_ENABLE
u8   ws_demo_frame(float *v, WS_Overlay *ov, u32 ms);
u8   ws_demo_active(void);
u8   ws_demo_stage_no(void);
void ws_demo_advance(void);
void ws_demo_oled(void);
#endif
```

- [ ] **Step 2: 在 `ws2811.c` 加 demo 代码（分两段）**

**为什么要分两段**：宿主测试只抽取 `WS_LOGIC` 区。档位表是 demo 里最可能出错的东西 —— OLED 写着 `TURN LEFT` 而灯带往右转，你按 9 下都发现不了。所以**输入合成必须进 `WS_LOGIC`** 才能被断言覆盖。而 `ws_demo_oled()` 要调 `OLED_ShowString`（宿主机编不过），只能留在 `WS_HW`。

同时这也意味着 **`ws_demo_frame()` 里不能调 `click()`** —— 按键轮询挪到任务体里（Step 3），`ws_demo_frame()` 保持纯函数。

**第一段：`WS_LOGIC` 区（`WS_LOGIC_END` 之前），全部纯逻辑**

```c
/* ==== WS_DEMO_LOGIC_BEGIN ==== */
/* PE0 preview mode, split in two because the host suite only extracts
 * WS_LOGIC. This half is the pure part -- the stage table and the input
 * synthesis -- so the "the OLED name matches what you actually see"
 * contract is covered by assertions.
 *
 * TO REMOVE THE WHOLE FEATURE
 *   1. set WS_DEMO_ENABLE to 0 in ws2811.h  -> feature gone, code kept
 *   2. to delete outright, drop: this block, WS_DEMO_HW_BEGIN..END, the
 *      two marked hook blocks in ws2811_task, the marked lines in
 *      BALANCE/show.c, and the WS_DEMO section of ws2811.h
 * Nothing else in the project references any of it. */
#if WS_DEMO_ENABLE

static u8  s_demo_stage = 0;        /* 0 = off, running the real inputs */
static u32 s_demo_ms    = 0;        /* ms since entering the stage       */

/* Snapshots of what the current stage feeds the engine, so the OLED page
 * can report them without recomputing anything. */
static float s_demo_v     = 0.0f;
static u8    s_demo_estop = 0, s_demo_brake = 0;
static s8    s_demo_turn  = 0;
static u8    s_demo_ros   = 1;

static const char *const ws_demo_name[WS_DEMO_STAGES + 1] = {
	"DEMO OFF",
	"IDLE CYAN",     /* 1  */
	"WAIT ROS",      /* 2  */
	"ROS LOST",      /* 3  */
	"LOW BAT",       /* 4  */
	"SPEED RAMP",    /* 5  */
	"CRUISE GRN",    /* 6  */
	"MAX RED",       /* 7  */
	"TURN LEFT",     /* 8  */
	"TURN RIGHT",    /* 9  */
	"BRAKE",         /* 10 */
	"E-STOP",        /* 11 */
	"REVERSE",       /* 12 */
	"NAV PROG",      /* 13 */
	"CALIBRATE",     /* 14 */
	"ALL WHITE",     /* 15 */
};

u8 ws_demo_active(void)   { return s_demo_stage != 0; }
u8 ws_demo_stage_no(void) { return s_demo_stage; }

/* The ONE place that changes stage. ws_demo_poll(), the test hooks and
 * anything else must all go through here: three copies of this increment
 * would mean a mutation test on one of them proves nothing about the
 * other two. */
void ws_demo_advance(void)
{
	s_demo_stage = (u8)(s_demo_stage + 1);
	if (s_demo_stage > WS_DEMO_STAGES) s_demo_stage = 0;
	s_demo_ms = 0;
}

/* Test hooks, so the host suite can drive the stage machine without a
 * button. Both go through ws_demo_advance() on purpose. */
void ws_demo_reset_for_test(void) { s_demo_stage = 0; s_demo_ms = 0; }
void ws_demo_press_for_test(void) { ws_demo_advance(); }

/* Pure: synthesises the engine inputs for the current stage. The button
 * poll happens in the task, NOT here -- click() is hardware and would
 * break the host build. */
u8 ws_demo_frame(float *v, WS_Overlay *ov, u32 ms)
{
	if (!s_demo_stage) return 0;            /* off: leave v/ov untouched */

	s_demo_ms += WS_STEP_MS;

	/* Stage 14/15 paint the frame themselves and bypass ws_render(). */
	if (s_demo_stage == 14) {
		ws_render_calib((u8)((ms / 250) % WS_PIXELS));
		return 1;
	}
	if (s_demo_stage == 15) {
		ws_fill_solid(255, 255, 255);
		return 1;
	}

	/* Every other stage only fabricates the INPUTS. The effect code is
	 * untouched, so what you see is exactly what the car would show. */
	*v = 0.0f;
	memset(ov, 0, sizeof(*ov));
	ov->nav_permille = 0xFFFF;
	ov->ros_link = 1;

	switch (s_demo_stage) {
	case 1: break;                                    /* idle cyan        */
	case 2:  ov->ros_link = 0; break;                 /* waiting yellow   */
	case 3:  ov->ros_link = 2; break;                 /* lost purple      */
	case 4:  ov->lowbat = 1;   break;                 /* low battery      */
	case 5: {                                         /* 8s speed ramp    */
		u32 ph = s_demo_ms % 8000;
		float t = (ph < 4000u ? ph : 8000u - ph) / 4000.0f;
		*v = t * WS_VMAX_MPS;
		break;
	}
	case 6:  *v = 0.25f; break;
	case 7:  *v = WS_VMAX_MPS; break;
	case 8:  *v = 0.45f; ov->turn =  1; break;
	case 9:  *v = 0.45f; ov->turn = -1; break;
	case 10: *v = 0.30f; ov->brake = 1; break;
	case 11: ov->estop = 1; break;
	case 12: ov->reverse = 1; break;
	case 13: /* nav sweeps 0 -> 100% over 3s, then repeats */
		ov->nav_permille = (u16)((s_demo_ms % 3000u) * 1000u / 3000u);
		break;
	default: *v = 0.0f; break;
	}

	/* Snapshot for the OLED page. Stages 14/15 returned early above, so
	 * they keep the previous values -- they paint the frame directly and
	 * have no meaningful v/ov to report. */
	s_demo_v     = *v;
	s_demo_estop = ov->estop;
	s_demo_brake = ov->brake;
	s_demo_turn  = ov->turn;
	s_demo_ros   = ov->ros_link;
	return 0;
}
#endif  /* WS_DEMO_ENABLE -- end of the host-testable half */
/* ==== WS_DEMO_LOGIC_END ==== */

/* ==== WS_DEMO_HW_BEGIN ==== */
/* Second half: the OLED page. Lives in WS_HW because OLED_ShowString is
 * hardware; the host suite cannot compile it. */
#if WS_DEMO_ENABLE

/* oled_show() hands the whole screen over while a stage is active,
 * because all six of its lines are already in use by the normal page. */
void ws_demo_oled(void)
{
	/* line 0: DEMO nn/15 */
	OLED_ShowString(0, 0, "DEMO");
	OLED_ShowNumber(30, 0, s_demo_stage, 2, 12);
	OLED_ShowString(48, 0, "/15   ");

	/* line 1: stage name, padded so a shorter name does not leave tails */
	OLED_ShowString(0, 10, "                ");
	OLED_ShowString(0, 10, (const u8 *)ws_demo_name[s_demo_stage]);

	/* line 2: the synthetic wheel speed this stage feeds the engine */
	OLED_ShowString(0, 20, "v=");
	OLED_ShowNumber(18, 20, (u32)(s_demo_v * 1000.0f), 4, 12);

	/* line 3: what the overlay bits are doing */
	OLED_ShowString(0, 30, "E");
	OLED_ShowNumber(8,  30, (u32)s_demo_estop, 1, 12);
	OLED_ShowString(20, 30, "B");
	OLED_ShowNumber(28, 30, (u32)s_demo_brake, 1, 12);
	OLED_ShowString(40, 30, "T");
	OLED_ShowNumber(48, 30, (u32)(s_demo_turn + 1), 1, 12);
	OLED_ShowString(60, 30, "R");
	OLED_ShowNumber(68, 30, (u32)s_demo_ros, 1, 12);

	/* line 4: real battery voltage, still worth watching while parked */
	OLED_ShowString(0, 40, "BAT");
	OLED_ShowNumber(30, 40, (u32)(Voltage * 10), 4, 12);

	/* line 5: the only control */
	OLED_ShowString(0, 50, "PE0 = NEXT");
}
#endif  /* WS_DEMO_ENABLE */
/* ==== WS_DEMO_HW_END ==== */
```

- [ ] **Step 3: 在 `ws2811_task` 里加两个 `WS_DEMO` hook**

只需两处，都用醒目注释包起来：

```c
	for (;;) {
		WS_Overlay  ov;
		WS_SpeedOut sp;
		float v = ws_wheel_speed();
#if WS_DEMO_ENABLE
		u8 painted;
#endif

		memset(&ov, 0, sizeof(ov));
		ov.nav_permille = 0xFFFF;
		ws_collect_overlay(&ov, ms);

		/* ==== WS_DEMO_HOOK_1_BEGIN ==== */
#if WS_DEMO_ENABLE
		/* click() is falling-edge latched and the tick is 40ms, longer
		 * than these buttons bounce, so no extra debounce is needed. */
		if (click()) ws_demo_advance();
		painted = ws_demo_frame(&v, &ov, ms);
#endif
		/* ==== WS_DEMO_HOOK_1_END ==== */

		ws_state_step(&st, v);
		sp = ws_speed_map(v);

		/* ==== WS_DEMO_HOOK_2_BEGIN ==== */
#if WS_DEMO_ENABLE
		if (!painted)
#endif
		/* ==== WS_DEMO_HOOK_2_END ==== */
			ws_render(&st, sp, &ov, ms);

		ws_show();
		vTaskDelay(WS_STEP_MS);
		ms += WS_STEP_MS;
	}
```

> 注意 `ws_state_step(&st, v)` 必须在 hook 1 **之后** —— 这样演示时状态机跟着合成速度走，你才能看到待机↔运动的 300ms 淡入淡出，而不是只有两个静帧。

- [ ] **Step 4: 在 `show.c` 的 `oled_show()` 顶部加早退分支（GBK，字节级）**

`oled_show()` 的六个 y 值（0/10/20/30/40/50）**全被占用**，没有空行，所以演示态整屏接管。

脚本 `_ros_tooling/_apply_ws_demo.py` 按字节插入，锚点用 `void oled_show(void)\r\n{  \r\n`（**注意 `{` 后面是两个空格**，已核实）：

```python
old = b'void oled_show(void)\r\n{  \r\n'
new = (old
       + b'/* ==== WS_DEMO_BEGIN ==== */\r\n'
       + b'#if WS_DEMO_ENABLE\r\n'
       + b'\tif (ws_demo_active()) { ws_demo_oled(); return; }\r\n'
       + b'#endif\r\n'
       + b'/* ==== WS_DEMO_END ==== */\r\n')
```

`show.c` 通过 `show.h → system.h → ws2811.h` 看得到 `ws_demo_active` / `ws_demo_oled`，无需额外 include。

- [ ] **Step 5: 宿主机断言（`case_demo()`）**

demo 的**合成输入生成逻辑是纯函数**，可以测：

```c
static void case_demo(void)
{
	/* The stage table is the contract with the operator: stage N must
	 * produce the inputs that exercise the effect the OLED names. A
	 * mismatch here means the printed name lies about what you see. */
	struct { u8 st; float v; u8 estop, brake, lowbat, reverse, ros; s8 turn; } c[] = {
		{ 1,  0.00f, 0,0,0,0, 1,  0 },
		{ 2,  0.00f, 0,0,0,0, 0,  0 },
		{ 3,  0.00f, 0,0,0,0, 2,  0 },
		{ 4,  0.00f, 0,0,1,0, 1,  0 },
		{ 6,  0.25f, 0,0,0,0, 1,  0 },
		{ 8,  0.45f, 0,0,0,0, 1,  1 },
		{ 9,  0.45f, 0,0,0,0, 1, -1 },
		{ 10, 0.30f, 0,1,0,0, 1,  0 },
		{ 11, 0.00f, 1,0,0,0, 1,  0 },
		{ 12, 0.00f, 0,0,0,1, 1,  0 },
	};
	unsigned k;
	for (k = 0; k < sizeof(c)/sizeof(c[0]); k++) {
		float v = 99.0f; WS_Overlay ov; u8 painted;
		/* drive ws_demo_frame to stage c[k].st by pressing the button */
		ws_demo_reset_for_test();
		{
			u8 n;
			for (n = 0; n < c[k].st; n++) ws_demo_press_for_test();
			painted = ws_demo_frame(&v, &ov, 1000);
		}
		if (painted || v != c[k].v || ov.estop != c[k].estop ||
		    ov.brake != c[k].brake || ov.lowbat != c[k].lowbat ||
		    ov.reverse != c[k].reverse || ov.ros_link != c[k].ros ||
		    ov.turn != c[k].turn) {
			printf("FAIL demo stage %u: v=%.3f e=%u b=%u l=%u r=%u ros=%u t=%d\n",
			       c[k].st, v, ov.estop, ov.brake, ov.lowbat,
			       ov.reverse, ov.ros_link, ov.turn);
			g_fail++;
		} else {
			printf("ok   demo stage %2u -> v=%.3f e=%u b=%u l=%u r=%u ros=%u t=%d\n",
			       c[k].st, v, ov.estop, ov.brake, ov.lowbat,
			       ov.reverse, ov.ros_link, ov.turn);
		}
	}
	/* stage 14 and 15 paint the frame themselves */
	{
		float v = 99.0f; WS_Overlay ov; u8 i, lit = 0;
		ws_demo_reset_for_test();
		for (i = 0; i < 14; i++) ws_demo_press_for_test();
		if (!ws_demo_frame(&v, &ov, 1000)) {
			printf("FAIL demo stage 14 should paint the frame\n"); g_fail++;
		} else {
			for (i = 0; i < WS_PIXELS; i++)
				if (ws_frame[i*3] || ws_frame[i*3+1] || ws_frame[i*3+2]) lit++;
			if (lit != 1) { printf("FAIL calib stage lit %u px\n", lit); g_fail++; }
			else ok("demo stage 14 lights exactly one pixel");
		}
	}
}
```

> 这需要 demo 里额外暴露两个小钩子（放 `#if WS_DEMO_ENABLE` 里即可，宿主测试照样编得到）：
> ```c
> void ws_demo_reset_for_test(void) { s_demo_stage = 0; s_demo_ms = 0; }
> void ws_demo_press_for_test(void) { ws_demo_advance(); }
> ```
> 两者都走 `ws_demo_advance()` —— 改档逻辑只有一份，突变它才能同时覆盖按键路径和测试路径。

- [ ] **Step 6: 跑宿主机测试**

Run: `cd "_ros_tooling" && python _gen_ws_test.py`
Expected: 62 条旧的 + 新增（10 档 + 标定 1 条）= **73 条**，`ALL PASSED`，退出码 0，gcc stderr 0。

> **`_gen_ws_test.py` 的 `PRELUDE` 必须补两个宏**，否则抽取段编不过：
> ```c
> #define WS_DEMO_ENABLE      1
> #define WS_DEMO_STAGES      15
> ```
> `WS_STEP_MS` **不用**补 —— 它由 Task 3 定义在逻辑区内，会随抽取一起进宿主文件。
>
> 另外 demo 逻辑段用到了 `memset`，`PRELUDE` 已有 `<string.h>`，无需改。

- [ ] **Step 7: 应变验证（证明 demo 断言真的在测）**

把 `case 8` 的 `ov->turn = 1` 改成 `-1`（即左右互换），重跑，确认 **`demo stage 8` FAIL 且退出码非 0**，然后还原并核对 sha256。

先把改档逻辑抽出来再改，是为了让这个验证有意义 —— 如果三个地方各写一份改档代码，改一处测不出另两处。

- [ ] **Step 8: 接入 show.c 并验证字节完整性**

Run: `cd "_ros_tooling" && python _apply_ws_demo.py`
Expected: 一行 `show.c oled_show hook  patched`。

然后：
1. 再跑一次，确认 `already applied`（幂等）
2. 统计 `ws_demo_active` 在 `show.c` 中出现次数，必须为 **1**
3. 确认 `show.c` 的 CRLF 总数增加了 5（新增 5 行），裸 LF 仍为 0
4. **先切片再解码**验证 GBK 完好（`show.c` 可能也有杂散字节，直接整文件 `decode('gbk')` 未必成功）
5. 报告 `show.c` 改动前后 sha256 + 回滚命令

- [ ] **Step 9: Keil 编译（用户手动）+ 逐档走查**

Keil 编译后，**每按一次 PE0 记录**：OLED 显示的档号/名称是否与灯带表现一致。用下面的走查表逐条打勾。

---

## Task 9 走查表（每按一下 PE0）

> **v2 修订（2026-09-18，首次上板走查后）** —— 下表相对最初版本有 4 处变化，以本说明为准：
>
> | 变化 | 原 | 现 | 依据 |
> |---|---|---|---|
> | **通道顺序** | `WS_ORDER_GRB`（两选一） | **`WS_ORDER_RBG`（三选一）** | 实测：发 GRB 时所有彩色档呈 `(R,G,B)→(G,B,R)` 循环位移，红显示成蓝、绿显示成橙、深蓝显示成绿。两选一的开关爱莫能助，故扩成三种 |
> | **档 2 / 4** | 3 像素流光，其余灭 | **整圈呼吸** | 代码与设计文档 §6 不一致（文档写"全带覆盖"）。3 像素对"低压"这种安全指示太弱，且台架上看着像灯坏了。流光现在只保留给档 3（ROS 断连） |
> | **档 8 / 9** | 250ms 扫一圈 | **1500ms 扫一圈** | 250ms = 4 圈/秒，左右方向在台架上完全分辨不出来，这是真设计错误 |
> | **档 16** | 无 | **新增 RGB PROBE** | 前 1/3 红、中 1/3 绿、后 1/3 蓝。**判定通道顺序的权威手段**，比按颜色猜可靠 |

**"真车何时出现"一列是从 `ws_collect_overlay()` 的实际代码读出来的，不是推测。**
触发值：`WS_LOWBAT_V` = 10.0V；转向阈值 `|Move_Z| > 0.20 rad/s`；刹车阈值 `decel > 0.15 m/s`（每个 40ms 周期）。

| 按 | OLED 行0 / 行1 | 灯带应当表现 | **真车何时出现** | 触发源 | 看什么 |
|:--:|---|---|---|---|---|
| 0 | `DEMO/00` `DEMO OFF` | 回到真实状态 | — | — | 应立刻恢复待机呼吸 |
| 1 | `01` `IDLE CYAN` | 青色均匀，2s 慢呼吸 | **上电后、树莓派已连上、车静止** | `ros_link==1 && !lowbat` | 色相偏青、呼吸平滑 |
| 2 | `02` `WAIT ROS` | 黄色呼吸 **+ 环绕流光** | **已上电，但树莓派还没发过 cmd_vel** | `RosDown.rx_ready == 0` | 和 1 颜色明显不同 |
| 3 | `03` `ROS LOST` | 紫色慢闪 **+ 环绕流光** | **树莓派挂了/网线掉了**，20Hz 下行帧断超 200ms | `RosFault & ROS_FAULT_TIMEOUT` | **光点是否在绕圈跑** |
| 4 | `04` `LOW BAT` | 橙色呼吸，**整体明显偏暗** | **电池掉到 10V 以下** | `Voltage < 10.0V` | 亮度是否被压到 20% |
| 5 | `05` `SPEED RAMP` | 8 秒一轮：暗青→亮青→绿→黄→橙→红→退回 | **起步→加速到满速→减速→停** 的全过程 | `ws_wheel_speed()` 实测 | **起步阶段是否靠亮度可见** |
| 6 | `06` `CRUISE GRN` | 稳定绿色 | 中低速匀速跑（约 0.25 m/s） | 同上 | 停在绿色便于判断纯度 |
| 7 | `07` `MAX RED` | 稳定红色 | 全速冲刺（满量程 0.75 m/s） | 同上 | **此时功率最低**（纯红=满白 1/3） |
| 8 | `08` `TURN LEFT` | 红色上叠**逆时针**流动光斑 | **上位机发左转指令**（`wz > +0.20`） | `Move_Z > 0.20` | 流向 |
| 9 | `09` `TURN RIGHT` | 同上但**顺时针** | **上位机发右转指令**（`wz < -0.20`） | `Move_Z < -0.20` | **必须和 8 相反** |
| 10 | `10` `BRAKE` | 红色快闪（亮 100ms / 暗 50ms） | **急减速**：一个 40ms 周期内轮速掉 >0.15 m/s | `decel > 0.15` | 是否在闪 |
| 11 | `11` `E-STOP` | **8Hz 红色爆闪** | **使能开关 PD3 打到 OFF**，或避障触发 | `Flag_Stop \|\| Avoid_ON_Flag` | 比 10 更急更狠 |
| 12 | `12` `REVERSE` | 白色流水 | ⚠️ **本车永远不会出现** | `ov->reverse` 恒为 0（无倒车通道） | 是白色不是彩色 |
| 13 | `13` `NAV PROG` | 深蓝填充变长 + 亮青光点绕圈 | ⚠️ **本车永远不会出现** | `nav_permille` 恒为 `0xFFFF`（里程未接） | **光点起点是否在车头** |
| 14 | `14` `CALIBRATE` | **只有一个白点**，每 250ms 走一格 | 只在标定时按出来 | — | 记"车头正前方"格号 → `WS_PHASE` |
| 15 | `15` `ALL WHITE` | 全带满白 | 只在检查时按出来 | — | **量电流**；颜色错就取反 `WS_ORDER_GRB` |
| 16 | 回到 `00` | — | — | — | 循环 |

### 优先级：真实行驶中叠加规则

**急停(11) 与 低压(4) 会盖掉其他一切。** 所以实际路上的观感是：

| 你在做什么 | 但发生了 | 看到的是 |
|---|---|---|
| 正常转弯（8/9） | 电池掉到 10V 以下 | **橙色暗呼吸(4)**，转向灯消失 |
| 正常巡航（6） | 使能开关被打到 OFF | **红色爆闪(11)**，速度色消失 |
| 正常奔跑（5） | 上位机停发指令 200ms | 先变**紫闪(3)**，同时车也会停 |
| 一切正常 | — | 按 1→2→5→6→7→8/9 依次出现 |

其余优先级：刹车(10) > 转向(8/9) > 运动色(5/6/7) > 待机(1/2/3/4)。

### 结论

- **真车会自发出现**：1、2、3、4、5、6、7、8、9、10、11 —— **11 档**
- **只有按键能看到**：12、13（数据源未接，属预留）、14、15（工具档）—— **4 档**

> 12 和 13 是**故意保留**的：渲染代码和断言都在，等倒车通道/里程数据源接通后无需改灯效代码即可生效。它们现在按得出来，正是为了让你确认"将来接上了会是什么样"。

**这一轮走完，除 SPI 时序/帧长/空闲电平三项外，其余全部效果都肉眼确认过了。**

---

## Self-Review

**Spec coverage** — 设计文档 §1–§9 逐条对账：

| 设计文档章节 | 覆盖任务 |
|---|---|
| §1.1 像素粒度 / 不做点阵 | Task 4（帧缓冲按像素，无图案 API） |
| §1.2 引脚复用 → SPI1 | Task 7 Step 4 |
| §1.3 电平风险 | Task 7 Step 6 第 5 项 |
| §1.4 SPI 时序 2.625MHz | Task 7 Step 1–3（编码，宿主机断言）+ Step 6 第 1–2 项（上板波形） |
| §2 索引映射 + 标定 | Task 4（`ws_map`）+ Task 7 Step 6 第 8 项 |
| §3.1 交叉淡入 | Task 3（`blend`）+ Task 6 用例 (a)(b) |
| §3.2 静止/运动滞回 | Task 3 全部用例 |
| §3.3 优先级 | Task 6 用例 (d)(e)(f) + `ws_render` 分支顺序 |
| §4.1/4.2 双维度编码 + 色标 | Task 2（含 creep band 专项断言） |
| §4.3 HSV 插值 | Task 1 |
| §4.4 数据源与滤波 | Task 3（滤波）+ Task 7（`ws_wheel_speed`） |
| §5 待机层 | Task 5 |
| §6 叠加层 | Task 6（`ws_ov_turn` / `ws_ov_reverse` / `ws_ov_nav` + 三个全带覆盖） |
| §7.3 关键宏 | Task 1（`ws2811.h`） |
| §7.4 任务帧率 | Task 7 Step 5（`vTaskDelay` 分支） |
| §8 验证策略 | Task 1–6 + Task 7 Step 2–3 宿主机；Task 7 Step 6 上板清单 |
| §9 待确认项 | `WS_VMAX_MPS` / `WS_LOWBAT_V` 已给默认值；电平芯片留到 Task 7 Step 6 判定 |

**已知缺口（有意接受，非遗漏）**：

- **导航到点**：`ws_ov_nav` 的渲染与边界测试已实现，但**没有任务把它接到真实里程** —— 该数据源属于 ROS 节点侧，不在本计划范围。`WS_Overlay.nav_permille` 在任务里恒为 `0xFFFF`，渲染分支待数据源就绪后启用。
- **倒车**：`ov->reverse` 在任务里恒为 0，该底盘无倒车指令通道。渲染与测试已实现，接线待需要时补。
- **`WS_BRIGHT_MAX` 的生效点是 `ws_pack()`（Task 7 Step 1），不在渲染层**。这是**有意的分阶段**：设计文档 §7.3 把它列为"总控"，实现点最自然的位置是硬件层对 `ws_frame` 统一缩放，这样每个灯效渲染器都不必知道亮度上限的存在。该缩放**已经写进 Task 7 Step 1 的代码**（主循环里先算 `v` 再查表），照抄即可，不要漏 —— 漏了就退化成死宏，设计文档里所有关于限流的论述都会变成假话。

**类型一致性核对**：

- `WS_SpeedOut` — `ws2811.h` 与 `_gen_ws_test.py` 的 PRELUDE 一致（`u16 hue; u8 bright;`）
- `WS_State` 五字段 `filt/moving/blend/start_ms/stop_ms` — Task 3 定义，Task 3 测试、Task 6 测试、Task 7 任务体一致
- `WS_Overlay` 七字段 `estop/lowbat/brake/turn/reverse/nav_permille/ros_link` — Task 5 定义，Task 6 使用，Task 7 `ws_collect_overlay` 写入，顺序与类型一致
- `ws_map` / `ws_put` / `ws_clear` / `ws_fill_solid` — Task 4 定义，Task 5/6/7 调用签名一致
- 宏 `WS_ORBIT_PERIOD_MS` / `WS_ORBIT_TAIL` — Task 5 定义，Task 6 的 `ws_ov_reverse` 复用，**未重复定义**
- 宏 `WS_LOWBAT_CAP`（51）— Task 5 定义，Task 6 用例 (e) 以字面量 53 作上界断言（51 + 容差 2），未引用未定义符号
- 前置声明类型：PRELUDE 提供 `u8/s8/u16/s16/u32` 五个 typedef，与 `system.h` 中 STM32 侧定义一致
