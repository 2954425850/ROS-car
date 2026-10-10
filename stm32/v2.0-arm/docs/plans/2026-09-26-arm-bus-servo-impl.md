# 机械臂（总线舵机）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 STM32F407 通过 USART2 + BusLinker V3.0 直接驱动 6 轴 LeArm 机械臂 —— ID1..ID5 位置模式按 ROS 下发的位置动作，ID6 底座走电机模式并在 ±225° 范围做角度闭环（电位器有效区内用绝对读数矫正，盲区内开环积分）。

**Architecture:** 分三层。①`HARDWARE/lx_servo.c` 是幻尔 LX/HTS `0x55` 协议驱动（组帧/校验/收发），不碰 USART2 寄存器之外的任何东西；②`BALANCE/arm.c` 是应用层，含上电状态机、关节表、回读调度、故障，以及底座角度跟踪核心 —— 跟踪核心被写成**纯函数**（无 HAL、无全局时钟、全部入参），夹在 `/* ==== ARM_LOGIC_BEGIN ==== */` 标记之间，可被 Python 脚本整段抽到宿主机 gcc 上跑断言；③`arm_task`（50Hz / prio 3）是唯一的调度者，串行占用半双工总线。

**Tech Stack:** C99 / STM32F407 / 标准外设库（**不是 HAL**）/ FreeRTOS / Keil MDK (uvprojx) / 宿主机 gcc 断言测试 / Python 3.11

---

## Global Constraints

- **既有源文件的编码**：`HARDWARE/*.c|h`、`BALANCE/*.c|h` 实测为 **GBK + CRLF**（例：`usartx.c` CRLF=825/LF=825）。**禁止用 Edit / Write 工具改这些文件**，必须写 Python 脚本按字节替换（`open(p,'rb')` / `open(p,'wb')`），且锚点串须含 `\r\n`。**只处理字节、绝不 `decode()`**（见下条）。
- **`USER/main.c` 连 GBK 都解不开**（实测：byte 439 处 `0xba` 是非法 GBK 序列，`gbk`/`gb18030`/`cp936` 全部 FAIL）。**任何情况下不要对 `main.c` 做 `decode()`**。只做字节级定位与替换。
- **`USER/WHEELTEC.uvprojx`** 是 **纯 ASCII + bare LF**（实测 CRLF 计数 = 0，LF = 662）—— 锚点必须用 `\n`，**别照搬 GBK 文件的 `\r\n`**。
- **本计划新建的文件**（`HARDWARE/lx_servo.c|h`、`BALANCE/arm.c|h`、`BALANCE/arm_cal.h`）一律 **纯 ASCII + 纯 LF**，注释全英文，避免 GBK 工程里的乱码。
- **宿主机测试文件**一律放 `_ros_tooling/`，脚本命名 `_*.py`，产物 `_*.c` / `_*.exe`，与既有文件同级。模板见 `_ros_tooling/_gen_ws_test.py`（**已实测可跑通**）。
  - ⚠️ `_ros_tooling/_gen_servo_e2e.py` **已失效**（它找的 `/* ==== servo smoothing block` 标记在 revert 后不存在，实测报 `ValueError: substring not found`）。**不要照抄它**。
- **编译命令**：`gcc -std=c99 -O2 -Wall -Wextra`，必须零警告通过。**禁止加 `-w` 掩盖警告。**
- **测试必须逐字抽取真实固件源码**（用标记切片读 `.c` 文件），不得手抄副本 —— 副本会与被测物漂移。
- **纯逻辑段的约束**：`ARM_LOGIC_BEGIN/END` 段内**不许出现任何 STM32 头文件、不许读全局时钟/全局变量、不许调用 `delay_*`/FreeRTOS**。所有时间以 `dt` 入参传入。
- **未接硬件的行为一律标记为上板验证，不写宿主测试。** USART2 电气、帧间时序、舵机真实响应、电位器盲区行为，宿主测试**无法**覆盖。
- **USART2 NVIC 抢占优先级必须从 `1` 改成 `5`**。`FreeRTOSConfig.h:176` 的 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`，数值小于 5 的中断 `taskENTER_CRITICAL()` 掩不住 → 与 arm_task 共享的邮箱会被撕裂。优先级 5 与 USART1 一致（`usartx.h:66-71` 有同样说明）。
- **ISR 内禁止调用任何 FreeRTOS API**（USART2 优先级 5 属于"受 FreeRTOS 管理"的范围，不能在 ISR 里用 `FromISR` 之外的调用；本项目惯例是**一个都不用**，见 `usartx.c:117-126`）。
- **Tick = 1000Hz**（`FreeRTOSConfig.h:97`）→ `vTaskDelay` 最小粒度 1ms。**亚毫秒间隔只能用 `delay_us()` 忙等**（`SYSTEM/delay/delay.h:7`）。
- **只关入口不删代码**：`AT_Command_Capture()` 等旧 USART2 解析函数保留，只是不再挂到 ISR 上。
- **`0x01`/`0x07` 的参数顺序是【位置低, 位置高, 时间低, 时间高】** —— 位置在前、时间在后。写反了舵机会转到错误角度（`协议规范` §5.3、§11.2 修正项 1）。
- **位置指令的 position 必须双向裁剪 0..1000**。原厂固件只裁了上限、没裁负值（`协议规范` §11.4 第 2 条）。
- **夹爪（ID1）目标位置限幅 ≤700**，防堵转（原厂做法，`协议规范` §7.6）。
- **5 个位置关节的机械可用行程是 pos 125..875**，不是 0..1000（`协议规范` §5.8）。

---

## File Structure

| 文件 | 动作 | 职责 |
|---|---|---|
| `HARDWARE/lx_servo.h` | 新建 | LX/HTS `0x55` 协议常量、API 声明 |
| `HARDWARE/lx_servo.c` | 新建 | 组帧/校验/解析（纯，带标记）+ USART2 收发胶水 |
| `BALANCE/arm.h` | 新建 | `ARM_BASE` 结构、状态枚举、`0xAC`/`0x56` 帧格式、公开 API |
| `BALANCE/arm.c` | 新建 | 跟踪核心（纯，带标记）+ 上电状态机 + 调度 + 故障 |
| `BALANCE/arm_cal.h` | 新建 | 标定参数（M1–M8 实测后回填） |
| `HARDWARE/usartx.h` | 修改 | 加 `0xAC`/`0xAD` 下行、`0x56`/`0x57` 上行的帧常量与 `ROS_ARM` 结构声明（字节级） |
| `HARDWARE/usartx.c` | 修改 | 加 `0xAC`/`0xAD` 解码状态机；**重写 `USART2_IRQHandler`** 为舵机协议专用；`uart2_init` 优先级 1→5（字节级） |
| `BALANCE/system.c` | 修改 | `#include "arm.h"`（字节级，经 system.h） |
| `BALANCE/system.h` | 修改 | 加 `#include "arm.h"`（字节级） |
| `USER/main.c` | 修改 | 加一行 `xTaskCreate(arm_task, ...)`（字节级，**不 decode**） |
| `USER/WHEELTEC.uvprojx` | 修改 | 加 `lx_servo.c` 到 HARDWARE 组、`arm.c` 到 Balance 组（字节级，UTF-8/ASCII + LF） |
| `_ros_tooling/_arm_tests.c` | 新建 | 宿主机断言用例（由生成器拼接进宿主文件） |
| `_ros_tooling/_gen_arm_test.py` | 新建 | 抽取真实源码 → 生成宿主测试 → gcc 编译 → 运行 |
| `_ros_tooling/_arm_probe.py` | 新建 | bring-up 脚本：经 `0xAD` 透传做 M1–M5 实测 |
| `_ros_tooling/_apply_arm.py` | 新建 | 对上述 5 个既有工程文件做字节级接入 |

**为什么跟踪核心要写成纯函数**：底座算法里唯一真正难的东西是"重同步 + 盲区积分 + 双向有界寻零"，而它的正确性完全靠边界条件（标志位被掰错、刚好卡在 `pot_lo`、盲区读数乱跳）。这些边界在真机上要花几十分钟才能复现一次，在宿主机上可以一秒钟跑十万次。把它做成纯函数是让 T5/T6 能存在的前提。

**为什么 T6（寻零安全性）必须是回归测试**：`协议规范` 里没有这条保证 —— 它是本设计自己推出来的安全论证（最坏情况位移不超 ±360°）。安全论证一旦只写在文档里，后续任何一次改 `SEARCH_BOUND` 或改方向逻辑都可能悄悄破坏它。用测试钉住。

---

### Task 1: `lx_servo` 组帧与解析（纯逻辑 + 宿主测试）

**Files:**
- Create: `HARDWARE/lx_servo.h`
- Create: `HARDWARE/lx_servo.c`
- Create: `_ros_tooling/_arm_tests.c`
- Create: `_ros_tooling/_gen_arm_test.py`

**Interfaces:**
- Consumes: 无
- Produces:
  - `u8 lx_build(u8 *buf, u8 id, u8 cmd, const u8 *prm, u8 n)` — 返回帧总长
  - `u8 lx_check(const u8 *buf, u8 len, u8 expect_id)` — 0=OK，1=帧头错，2=ID 错，3=Length 错，4=校验和错
  - `s16 lx_pos_from_resp(const u8 *buf)` — 从 `0x1C` 应答取位置（有符号 short）
  - `u16 lx_u16_from_resp(const u8 *buf)` — 从 `0x1B` 应答取电压
  - 标记 `/* ==== LX_LOGIC_BEGIN ==== */` … `/* ==== LX_LOGIC_END ==== */`
  - `_gen_arm_test.py` 可执行：抽取逻辑段 → 拼接 `_arm_tests.c` → `gcc` 编译 → 运行；断言失败则退出码非 0

- [ ] **Step 1: 写 `HARDWARE/lx_servo.h`**

用 Write 工具新建（**纯 ASCII + LF**）。

```c
#ifndef __LX_SERVO_H
#define __LX_SERVO_H

#include "sys.h"

/* ------------------------------------------------------------------
 * Hiwonder LX / HTS bus servo protocol (0x55 frame, 115200 8N1).
 *
 * Wire format:
 *   55 55 | ID | Length | Cmd | Prm x N | Checksum
 *   Length   = N + 3
 *   frame    = Length + 3 bytes total
 *   Checksum = ~(ID + Length + Cmd + Prm1 + ... + PrmN) & 0xFF
 *
 * Write commands get NO reply. Read commands reply immediately, so the
 * caller must switch to receive as soon as the frame is on the wire.
 * ------------------------------------------------------------------ */

#define LX_HEAD        0x55
#define LX_BROADCAST   0xFE

/* Frame length field values (params + 3) */
#define LX_LEN_NOPRM   3
#define LX_LEN_1PRM    4
#define LX_LEN_4PRM    7

/* ---- write commands (no reply) ---- */
#define LX_MOVE_TIME_WRITE        0x01  /* prm: posL,posH,tL,tH   */
#define LX_MOVE_TIME_WAIT_WRITE   0x07  /* prm: posL,posH,tL,tH   */
#define LX_MOVE_START             0x0B  /* broadcast to trigger    */
#define LX_MOVE_STOP              0x0C
#define LX_ID_WRITE               0x0D
#define LX_ANGLE_OFFSET_ADJUST    0x11
#define LX_ANGLE_OFFSET_WRITE     0x12
#define LX_ANGLE_LIMIT_WRITE      0x14
#define LX_VIN_LIMIT_WRITE        0x16
#define LX_TEMP_MAX_LIMIT_WRITE   0x18
#define LX_OR_MOTOR_MODE_WRITE    0x1D  /* prm: mode,null,spdL,spdH */
#define LX_LOAD_OR_UNLOAD_WRITE   0x1F  /* prm: 0=unload 1=load   */

/* ---- read commands (reply immediately) ---- */
#define LX_MOVE_TIME_READ         0x02
#define LX_MOVE_TIME_WAIT_READ    0x08
#define LX_ID_READ                0x0E
#define LX_ANGLE_OFFSET_READ      0x13
#define LX_ANGLE_LIMIT_READ       0x15
#define LX_TEMP_MAX_LIMIT_READ    0x19
#define LX_TEMP_READ              0x1A
#define LX_VIN_READ               0x1B
#define LX_POS_READ               0x1C
#define LX_OR_MOTOR_MODE_READ     0x1E
#define LX_LOAD_OR_UNLOAD_READ    0x20

/* Reply total lengths (frame header .. checksum), from protocol Table 4 */
#define LX_RESP_POS               8     /* 55 55 ID 05 1C posL posH CS      */
#define LX_RESP_VIN               8     /* 55 55 ID 05 1B vinL vinH CS      */
#define LX_RESP_TEMP              7     /* 55 55 ID 04 1A temp CS           */
#define LX_RESP_ID                7     /* 55 55 ID 04 0E id CS             */
#define LX_RESP_MODE              9     /* 55 55 ID 07 1E mode 00 spdL spdH CS */

/* Servo mode values for LX_OR_MOTOR_MODE_WRITE */
#define LX_MODE_POSITION          0
#define LX_MODE_MOTOR             1

/* ---- pure helpers (host testable) ---- */
u8  lx_build(u8 *buf, u8 id, u8 cmd, const u8 *prm, u8 n);
u8  lx_check(const u8 *buf, u8 len, u8 expect_id);
s16 lx_pos_from_resp(const u8 *buf);
u16 lx_u16_from_resp(const u8 *buf);
u8  lx_u8_from_resp(const u8 *buf);

/* ---- bus glue (hardware only) ---- */
void lx_init(void *unused);            /* reserved, keeps the call site simple */
void lx_send_frame(const u8 *buf, u8 len);
void lx_move_time(u8 id, u16 pos, u16 t_ms);
void lx_move_wait(u8 id, u16 pos, u16 t_ms);
void lx_move_start_all(void);
void lx_move_stop(u8 id);
void lx_torque(u8 id, u8 on);
void lx_set_mode(u8 id, u8 mode, s16 speed);
void lx_set_id(u8 old_id, u8 new_id);
void lx_set_limit(u8 id, u16 min_pos, u16 max_pos);

/* Read helpers return 0 on success, non-zero on timeout / bad checksum.
 * They block for at most LX_READ_TIMEOUT_MS. */
u8 lx_read_pos(u8 id, s16 *pos);
u8 lx_read_vin(u8 id, u16 *mv);
u8 lx_read_temp(u8 id, u8 *degc);
u8 lx_read_id(u8 id, u8 *out);

/* Raw passthrough used by the bring-up debug frame (0xAD). */
u8 lx_raw_exchange(const u8 *tx, u8 tx_len, u8 *rx, u8 rx_max, u16 timeout_ms);

#endif
```

- [ ] **Step 2: 写 `HARDWARE/lx_servo.c` 的纯逻辑段**

用 Write 工具新建（**纯 ASCII + LF**）。这一步只写纯逻辑段与占位尾部；胶水在 Task 2 补。

```c
#include "lx_servo.h"

/* ==== LX_LOGIC_BEGIN ==== */
/*
 * Pure frame assembly / validation. No STM32 registers, no globals, no
 * clock. Everything the host test needs lives between these markers.
 */

/* Build one frame into buf (must hold at least 16 bytes).
 * Returns the total frame length (Length + 3).
 * Checksum = ~(ID + Length + Cmd + Prm1..PrmN) & 0xFF. */
u8 lx_build(u8 *buf, u8 id, u8 cmd, const u8 *prm, u8 n)
{
    u8  i;
    u8  len;
    u16 sum;

    len = (u8)(n + 3);

    buf[0] = LX_HEAD;
    buf[1] = LX_HEAD;
    buf[2] = id;
    buf[3] = len;
    buf[4] = cmd;
    for(i = 0; i < n; i++) buf[5 + i] = prm[i];

    sum = 0;
    for(i = 2; i < (u8)(5 + n); i++) sum = (u16)(sum + buf[i]);
    buf[5 + n] = (u8)(~sum);

    return (u8)(len + 3);
}

/* Validate one complete reply sitting in buf.
 * Returns 0 = OK, 1 = bad header, 2 = wrong ID, 3 = wrong Length, 4 = bad checksum.
 * The Length byte is checked as (len - 3), i.e. "params + 3". */
u8 lx_check(const u8 *buf, u8 len, u8 expect_id)
{
    u8  i;
    u16 sum;

    if(len < 6) return 3;
    if(buf[0] != LX_HEAD || buf[1] != LX_HEAD) return 1;
    if(expect_id != LX_BROADCAST && buf[2] != expect_id) return 2;
    if(buf[3] != (u8)(len - 3)) return 3;

    sum = 0;
    for(i = 2; i < (u8)(len - 1); i++) sum = (u16)(sum + buf[i]);
    if(buf[len - 1] != (u8)(~sum)) return 4;

    return 0;
}

/* POS_READ reply: params are a SIGNED short (protocol Table 4). */
s16 lx_pos_from_resp(const u8 *buf)
{
    return (s16)((u16)buf[5] | ((u16)buf[6] << 8));
}

/* VIN_READ reply: little endian, millivolts. */
u16 lx_u16_from_resp(const u8 *buf)
{
    return (u16)((u16)buf[5] | ((u16)buf[6] << 8));
}

/* Single-byte parameter replies (TEMP_READ, ID_READ, ...). */
u8 lx_u8_from_resp(const u8 *buf)
{
    return buf[5];
}

/* ==== LX_LOGIC_END ==== */
```

- [ ] **Step 3: 写测试用例 `_ros_tooling/_arm_tests.c`**

用 Write 工具新建。这是**被生成器 `#include` 进宿主文件**的片段，只写用例函数体，不写 `main()`。

```c
/* ------------------------------------------------------------------
 * T1: frame assembly, checked byte for byte against the worked examples
 *     in the protocol spec section 5.5. These are the only tests in the
 *     whole project that verify the servo protocol without hardware.
 * T2: checksum edge cases.
 * ------------------------------------------------------------------ */

static int fails = 0;

static void expect_frame(const char *what, const u8 *got, u8 got_len,
                         const u8 *want, u8 want_len)
{
    u8 i;
    if(got_len != want_len)
    {
        printf("FAIL %-34s length %u != %u\n", what, got_len, want_len);
        fails++;
        return;
    }
    for(i = 0; i < want_len; i++)
    {
        if(got[i] != want[i])
        {
            printf("FAIL %-34s byte[%u] %02X != %02X\n",
                   what, i, got[i], want[i]);
            fails++;
            return;
        }
    }
    printf("ok   %-34s ", what);
    for(i = 0; i < got_len; i++) printf("%02X ", got[i]);
    printf("\n");
}

static void test_build_examples(void)
{
    u8 f[16];
    u8 n;

    /* ID1 -> position 500, 1000 ms  (the spec's headline example) */
    {
        u8 p[4] = { 0xF4, 0x01, 0xE8, 0x03 };          /* posL,posH,tL,tH */
        u8 w[10] = { 0x55,0x55,0x01,0x07,0x01,0xF4,0x01,0xE8,0x03,0x16 };
        n = lx_build(f, 0x01, LX_MOVE_TIME_WRITE, p, 4);
        expect_frame("MOVE_TIME_WRITE id1 500/1000ms", f, n, w, 10);
    }
    /* ID1 preset (WAIT) 500 / 1000 ms */
    {
        u8 p[4] = { 0xF4, 0x01, 0xE8, 0x03 };
        u8 w[10] = { 0x55,0x55,0x01,0x07,0x07,0xF4,0x01,0xE8,0x03,0x10 };
        n = lx_build(f, 0x01, LX_MOVE_TIME_WAIT_WRITE, p, 4);
        expect_frame("MOVE_TIME_WAIT_WRITE id1", f, n, w, 10);
    }
    /* broadcast MOVE_START */
    {
        u8 w[6] = { 0x55,0x55,0xFE,0x03,0x0B,0xF3 };
        n = lx_build(f, LX_BROADCAST, LX_MOVE_START, 0, 0);
        expect_frame("broadcast MOVE_START", f, n, w, 6);
    }
    /* read position / temp / vin / id */
    {
        u8 w[6] = { 0x55,0x55,0x01,0x03,0x1C,0xDF };
        n = lx_build(f, 0x01, LX_POS_READ, 0, 0);
        expect_frame("POS_READ id1", f, n, w, 6);
    }
    {
        u8 w[6] = { 0x55,0x55,0x01,0x03,0x1A,0xE1 };
        n = lx_build(f, 0x01, LX_TEMP_READ, 0, 0);
        expect_frame("TEMP_READ id1", f, n, w, 6);
    }
    {
        u8 w[6] = { 0x55,0x55,0x01,0x03,0x1B,0xE0 };
        n = lx_build(f, 0x01, LX_VIN_READ, 0, 0);
        expect_frame("VIN_READ id1", f, n, w, 6);
    }
    {
        u8 w[6] = { 0x55,0x55,0x01,0x03,0x0E,0xED };
        n = lx_build(f, 0x01, LX_ID_READ, 0, 0);
        expect_frame("ID_READ id1", f, n, w, 6);
    }
    {
        u8 w[6] = { 0x55,0x55,0x01,0x03,0x0C,0xEF };
        n = lx_build(f, 0x01, LX_MOVE_STOP, 0, 0);
        expect_frame("MOVE_STOP id1", f, n, w, 6);
    }
    /* torque on / off */
    {
        u8 p[1] = { 0x01 };
        u8 w[7] = { 0x55,0x55,0x01,0x04,0x1F,0x01,0xDA };
        n = lx_build(f, 0x01, LX_LOAD_OR_UNLOAD_WRITE, p, 1);
        expect_frame("LOAD id1", f, n, w, 7);
    }
    {
        u8 p[1] = { 0x00 };
        u8 w[7] = { 0x55,0x55,0x01,0x04,0x1F,0x00,0xDB };
        n = lx_build(f, 0x01, LX_LOAD_OR_UNLOAD_WRITE, p, 1);
        expect_frame("UNLOAD id1", f, n, w, 7);
    }
    /* ID6 motor mode at speed 0 / +300 / -300, and back to position mode */
    {
        u8 p[4] = { 0x01, 0x00, 0x00, 0x00 };
        u8 w[10] = { 0x55,0x55,0x06,0x07,0x1D,0x01,0x00,0x00,0x00,0xD4 };
        n = lx_build(f, 0x06, LX_OR_MOTOR_MODE_WRITE, p, 4);
        expect_frame("motor mode id6 speed 0", f, n, w, 10);
    }
    {
        u8 p[4] = { 0x01, 0x00, 0x2C, 0x01 };
        u8 w[10] = { 0x55,0x55,0x06,0x07,0x1D,0x01,0x00,0x2C,0x01,0xA7 };
        n = lx_build(f, 0x06, LX_OR_MOTOR_MODE_WRITE, p, 4);
        expect_frame("motor mode id6 speed +300", f, n, w, 10);
    }
    {
        u8 p[4] = { 0x01, 0x00, 0xD4, 0xFE };
        u8 w[10] = { 0x55,0x55,0x06,0x07,0x1D,0x01,0x00,0xD4,0xFE,0x02 };
        n = lx_build(f, 0x06, LX_OR_MOTOR_MODE_WRITE, p, 4);
        expect_frame("motor mode id6 speed -300", f, n, w, 10);
    }
    {
        u8 p[4] = { 0x00, 0x00, 0x00, 0x00 };
        u8 w[10] = { 0x55,0x55,0x06,0x07,0x1D,0x00,0x00,0x00,0x00,0xD5 };
        n = lx_build(f, 0x06, LX_OR_MOTOR_MODE_WRITE, p, 4);
        expect_frame("position mode id6", f, n, w, 10);
    }
}

static void test_check_and_extract(void)
{
    /* a good POS_READ reply: 55 55 06 05 1C posL posH CS */
    {
        u8 r[8];
        u16 sum;
        s16 pos;
        r[0]=0x55; r[1]=0x55; r[2]=0x06; r[3]=0x05; r[4]=0x1C;
        r[5]=0x0C; r[6]=0x02;                      /* 0x020C = 524 */
        sum = 0; { u8 i; for(i=2;i<7;i++) sum=(u16)(sum+r[i]); }
        r[7] = (u8)(~sum);

        if(lx_check(r, 8, 0x06) != 0) { printf("FAIL pos reply rejected\n"); fails++; }
        else printf("ok   pos reply accepted\n");

        pos = lx_pos_from_resp(r);
        if(pos != 524) { printf("FAIL pos %d != 524\n", pos); fails++; }
        else printf("ok   pos decode 524\n");

        /* negative reading must be interpreted as a signed short */
        r[5]=0x38; r[6]=0xFF;                      /* 0xFF38 = -200 */
        sum = 0; { u8 i; for(i=2;i<7;i++) sum=(u16)(sum+r[i]); }
        r[7] = (u8)(~sum);
        if(lx_check(r, 8, 0x06) != 0) { printf("FAIL neg reply rejected\n"); fails++; }
        pos = lx_pos_from_resp(r);
        if(pos != -200) { printf("FAIL pos %d != -200\n", pos); fails++; }
        else printf("ok   negative pos decode -200\n");
    }
    /* every rejection path */
    {
        u8 r[8] = { 0x55,0x55,0x06,0x05,0x1C,0x0C,0x02,0x00 };
        u16 sum; u8 i;
        sum = 0; for(i=2;i<7;i++) sum=(u16)(sum+r[i]);
        r[7] = (u8)(~sum);

        if(lx_check(r, 8, 0x06) != 0) { printf("FAIL baseline\n"); fails++; }

        r[0] = 0xAA;
        if(lx_check(r, 8, 0x06) != 1) { printf("FAIL header reject\n"); fails++; }
        else printf("ok   bad header -> 1\n");
        r[0] = 0x55;

        if(lx_check(r, 8, 0x07) != 2) { printf("FAIL id reject\n"); fails++; }
        else printf("ok   wrong id -> 2\n");

        r[3] = 0x04;
        if(lx_check(r, 8, 0x06) != 3) { printf("FAIL len reject\n"); fails++; }
        else printf("ok   wrong length -> 3\n");
        r[3] = 0x05;

        r[7] ^= 0xFF;
        if(lx_check(r, 8, 0x06) != 4) { printf("FAIL cksum reject\n"); fails++; }
        else printf("ok   bad checksum -> 4\n");
    }
    /* broadcast id must be accepted by lx_check for any frame id */
    {
        u8 r[6] = { 0x55,0x55,0x02,0x03,0x0B,0x00 };
        u16 sum = 0; u8 i;
        for(i=2;i<5;i++) sum=(u16)(sum+r[i]);
        r[5] = (u8)(~sum);
        if(lx_check(r, 6, LX_BROADCAST) != 0) { printf("FAIL bcast\n"); fails++; }
        else printf("ok   LX_BROADCAST accepts any id\n");
    }
}

static void test_checksum_edges(void)
{
    /* zero-param frame: Length field is 3 */
    {
        u8 f[8];
        u8 n = lx_build(f, 0xFE, 0x0B, 0, 0);
        if(n != 6 || f[3] != LX_LEN_NOPRM) { printf("FAIL noprm len\n"); fails++; }
        else printf("ok   no-param Length = 3, frame = 6\n");
    }
    /* every id must round-trip through build + check */
    {
        u8 f[16]; u8 id; u8 bad = 0;
        for(id = 0; id < 254; id++)
        {
            u8 n = lx_build(f, id, LX_POS_READ, 0, 0);
            if(lx_check(f, n, id) != 0) bad++;
        }
        if(bad) { printf("FAIL %u ids failed round trip\n", bad); fails++; }
        else printf("ok   all 254 ids round trip\n");
    }
    /* the sum must wrap mod 256: force a carry with a large parameter set */
    {
        u8 p[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
        u8 f[16];
        u16 sum = 0; u8 i; u8 n;
        n = lx_build(f, 0xFF, 0xFF, p, 4);
        for(i = 2; i < (u8)(5 + 4); i++) sum = (u16)(sum + f[i]);
        if(f[9] != (u8)(~sum)) { printf("FAIL wrap\n"); fails++; }
        else printf("ok   checksum wraps mod 256\n");
    }
}
```

- [ ] **Step 4: 写生成器 `_ros_tooling/_gen_arm_test.py`**

用 Write 工具新建。**以 `_ros_tooling/_gen_ws_test.py` 为模板**（实测可跑通）。

```python
# -*- coding: utf-8 -*-
"""Host tests for the arm bus-servo path.

Reads the REAL firmware sources verbatim (sliced between markers) and
compiles them on the host together with the assertion cases in
_arm_tests.c. This is the only place the servo protocol is verified
without hardware.

Follows _gen_ws_test.py. Do NOT copy _gen_servo_e2e.py - it is dead
(it still looks for a '/* ==== servo smoothing block' marker that was
reverted out of motor.c).
"""
import io, os, re, subprocess, sys

ROOT = os.path.dirname(os.path.abspath(__file__))

def read(p):
    """Read a firmware source file as latin-1 text.

    The GBK sources contain byte sequences that are not valid in any
    Unicode codec (USER/main.c does), so we never decode them as text
    characters - latin-1 is a byte-for-byte round trip and keeps every
    anchor match working.
    """
    with io.open(os.path.join(ROOT, '..', p), 'rb') as f:
        return f.read().decode('latin-1')

def slice_between(text, begin, end, name):
    i = text.index(begin)
    j = text.index(end, i)
    assert i >= 0 and j > i, 'markers not found for %s' % name
    return text[i:j]

def must(hay, needle, what):
    assert needle in hay, 'missing %s: %r' % (what, needle)

# ---- slice the real sources -------------------------------------------
lx_c = read('HARDWARE/lx_servo.c')
lx_h = read('HARDWARE/lx_servo.h')

logic = slice_between(lx_c, '/* ==== LX_LOGIC_BEGIN ==== */',
                            '/* ==== LX_LOGIC_END ==== */', 'lx logic')

# pull the #define block out of the header so the host build sees the
# same constants the firmware does
defs = []
for line in lx_h.split('\n'):
    s = line.strip()
    if s.startswith('#define LX_'):
        # keep the macro name and the first token of the value
        m = re.match(r'#define\s+(LX_\w+)\s+(.+?)\s*(?:/\*.*)?$', s)
        if m:
            defs.append('#define %-26s %s' % (m.group(1), m.group(2).strip()))
defs = '\n'.join(defs)
must(defs, 'LX_MOVE_TIME_WRITE', 'cmd defines')
must(defs, 'LX_MODE_MOTOR', 'mode defines')

tests = read('_ros_tooling/_arm_tests.c')

HOST = r'''
#include <stdio.h>
#include <string.h>
typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
typedef          short s16;
typedef          int   s32;

@@DEFS@@

@@LOGIC@@

@@TESTS@@

int main(void)
{
    test_build_examples();
    test_check_and_extract();
    test_checksum_edges();
    if(fails) { printf("\n%u FAILURES\n", fails); return 1; }
    printf("\nALL PASSED (0 failures)\n");
    return 0;
}
'''

src = (HOST.replace('@@DEFS@@', defs)
           .replace('@@LOGIC@@', logic)
           .replace('@@TESTS@@', tests))

out_c = os.path.join(ROOT, '_arm_host_test.c')
out_e = os.path.join(ROOT, '_arm_host_test.exe')
with io.open(out_c, 'w', encoding='latin-1', newline='\n') as f:
    f.write(src)

cmd = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-o', out_e, out_c]
print(' '.join(cmd))
r = subprocess.run(cmd, capture_output=True, text=True)
if r.stdout.strip(): sys.stdout.write(r.stdout)
if r.stderr.strip(): sys.stderr.write(r.stderr)
if r.returncode != 0:
    print('BUILD FAILED'); sys.exit(1)

r = subprocess.run([out_e], capture_output=True, text=True)
sys.stdout.write(r.stdout)
sys.stderr.write(r.stderr)
print('exit=%d' % r.returncode)
sys.exit(r.returncode)
```

- [ ] **Step 5: 运行生成器，确认测试全过**

Run: `python _ros_tooling/_gen_arm_test.py`
Expected: 零 gcc 警告；逐条 `ok  ...`；最后 `ALL PASSED (0 failures)`；`exit=0`。
若出现 `FAIL`，说明 `lx_build()` 与协议不符 —— 对照 `协议规范` §5.5 的报文逐字节排查。

- [ ] **Step 6: 记录结果**

把 `_gen_arm_test.py` 的输出（含帧十六进制）贴进 `docs/plans/2026-09-26-arm-bus-servo-impl.md` 末尾的「执行记录」小节。
**本工程不是 git 仓库**（实测 `git rev-parse --show-toplevel` 报 `not a git repository`），所以没有 commit 步骤；替代动作是记录运行输出。

---

### Task 2: USART2 收发链路（ISR + 优先级 + 邮箱）

**Files:**
- Modify: `HARDWARE/usartx.c:546-...`（`USART2_IRQHandler` 整体替换）
- Modify: `HARDWARE/usartx.c:64-106`（`uart2_init` 的 NVIC 优先级）
- Modify: `HARDWARE/lx_servo.c`（追加胶水段）
- Create: `_ros_tooling/_apply_arm_task2.py`

**Interfaces:**
- Consumes: Task 1 的 `lx_build()` / `lx_check()` / `lx_pos_from_resp()`
- Produces:
  - `LX_RX` 邮箱结构：`volatile u8 buf[12]; volatile u8 len; volatile u8 ready; volatile u32 frames; volatile u32 errs;`
  - `extern LX_RX LxRx;` —— USART2 ISR 写，`lx_read_*()` 读
  - `void lx_rx_byte(u8 c)` —— 字节状态机入口
  - `lx_send_frame()` / `lx_read_*()` / `lx_raw_exchange()` 实现

- [ ] **Step 1: 把 `USART2_IRQHandler` 换成舵机协议专用**

`USART2_IRQHandler` 现在从 `usartx.c:546` 开始，一直到函数结束。用 Python 字节级替换整段。

**先用 grep 定位这个函数的结束行**，再写脚本：

```bash
grep -n "^int USART2_IRQHandler" HARDWARE/usartx.c
awk 'NR>=546 && /^}/{print NR": "$0; exit}' HARDWARE/usartx.c
```

脚本（`_ros_tooling/_apply_arm_task2.py`）—— 注意锚点用 `\r\n`，只处理字节：

```python
# -*- coding: utf-8 -*-
"""Task 2: give USART2 to the bus servos.

1. uart2_init(): NVIC preempt priority 1 -> 5 so taskENTER_CRITICAL()
   actually masks this ISR (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
   is 5). At priority 1 the ISR can tear the mailbox mid-copy.
2. USART2_IRQHandler(): replaced wholesale. The old body fed every byte
   into AT_Command_Capture() and the APP joystick parser, which would
   eat servo replies and scribble on Turn_Flag / Flag_Direction. Those
   functions stay in the file, they are simply no longer wired in.
"""
import io, re, sys

CRLF = b'\r\n'
def enc(s):
    return s.replace('\n', '\r\n').encode('ascii')

p = 'HARDWARE/usartx.c'
b = open(p, 'rb').read()

# ---- 1. NVIC priority ------------------------------------------------
old_prio = (b'	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;' + CRLF +
            b'	//Preempt priority //' )
assert b.count(old_prio) == 1, 'uart2 NVIC anchor = %d' % b.count(old_prio)
print('ok  located uart2 NVIC block')
```

> 执行者注意：`uart2_init()` 里那两行注释是 GBK 中文，锚点**不要包含它们**。改用纯 ASCII 的相邻行做锚点：
> `b'NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;'` 在本文件里**出现两次**（`uart1_init` 是 `USART1_IRQn`，所以实际只出现一次）—— 先 `assert b.count(...) == 1` 确认。
> 替换时只把紧随其后的 `= 1 ;` 改成 `= 5 ;`，用正则限定在 `USART2_IRQn` 之后 200 字节内。

```python
# (continues _apply_arm_task2.py)
i = b.index(b'NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;')
j = b.index(b'USART2_IRQn;', i) + len(b'USART2_IRQn;')
window = b[j:j+200]
assert b'1 ;' in window or b'1;' in window, 'priority literal not found'
new_window = re.sub(rb'([^0-9])1(\s*;)', rb'\g<1>5\g<2>', window, count=1)
assert new_window != window, 'priority was not changed'
b = b[:j] + new_window + b[j+200:]
print('ok  uart2 preempt priority 1 -> 5')

# ---- 2. USART2_IRQHandler -------------------------------------------
start = b.index(b'int USART2_IRQHandler(void)')
# the function ends at the first line that is exactly "}" followed by CRLF
end = b.index(b'\r\n}\r\n', start) + len(b'\r\n}\r\n')
print('ok  USART2_IRQHandler spans %d bytes' % (end - start))
```

新函数体（**纯 ASCII**，缩进用 Tab 以匹配本文件风格）：

```python
NEW_ISR = enc(r'''int USART2_IRQHandler(void)
{
	/* USART2 is the bus-servo line (LX/HTS 0x55 protocol, 115200 8N1).
	 *
	 * Everything here is byte-at-a-time on purpose: the ISR runs at NVIC
	 * preempt priority 5, which is inside the range FreeRTOS masks, so it
	 * must not call a single FreeRTOS API. It only feeds a state machine
	 * and publishes completed replies into LxRx.
	 *
	 * The old APP / Bluetooth parsers that used to live here are still in
	 * this file but are no longer wired up - servo replies would have been
	 * eaten by them.
	 */
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		u8 c = (u8)(USART2->DR & 0xFF);   /* reading DR clears RXNE */
		lx_rx_byte(c);
	}
	return 0;
}''')

b = b[:start] + NEW_ISR + b[end:]
open(p, 'wb').write(b)
print('ok  USART2_IRQHandler replaced')
```

- [ ] **Step 2: 在 `lx_servo.c` 追加收发胶水**

用 Edit 追加到 `/* ==== LX_LOGIC_END ==== */` 之后（新建文件是 LF，可以用 Edit）。

```c
/* ------------------------------------------------------------------
 * Bus glue. Hardware only - no host test covers this.
 *
 * TX is blocking byte polling, matching the style of usart1_send(). The
 * inter-frame gap uses delay_us(), NOT vTaskDelay(): the tick is 1 kHz
 * so vTaskDelay cannot express the sub-millisecond gap the servos want.
 * Busy-waiting with interrupts on is safe here - Balance_task runs at a
 * higher priority and still preempts us.
 * ------------------------------------------------------------------ */

#define LX_GAP_US   500u      /* inter-frame gap; bring-up item M8 */
#define LX_READ_TIMEOUT_MS  20u

LX_RX LxRx;

/* ---- byte level receiver, called from the USART2 ISR ---- */
static u8 s_rx_buf[12];
static u8 s_rx_idx;
static u8 s_rx_want;    /* total length we are collecting, 0 = hunting */

static void lx_rx_reset(void)
{
    s_rx_idx  = 0;
    s_rx_want = 0;
}

void lx_rx_byte(u8 c)
{
    if(s_rx_want == 0)
    {
        /* hunting for 55 55 */
        if(c == LX_HEAD)
        {
            if(s_rx_idx == 0) { s_rx_idx = 1; }
            else              { s_rx_idx = 2; s_rx_want = 0; }  /* need Length */
            if(s_rx_idx == 2)
            {
                s_rx_buf[0] = LX_HEAD;
                s_rx_buf[1] = LX_HEAD;
            }
        }
        else
        {
            s_rx_idx = 0;
        }
        return;
    }

    if(s_rx_idx == 2)
    {
        /* the Length byte: total = Length + 3 */
        if(c < LX_LEN_NOPRM || c > 12)
        {
            LxRx.errs++;
            lx_rx_reset();
            return;
        }
        s_rx_buf[2] = c;
        s_rx_idx   = 3;
        s_rx_want  = (u8)(c + 3);
        return;
    }

    s_rx_buf[s_rx_idx++] = c;
    if(s_rx_idx < s_rx_want) return;

    /* one frame complete - publish it */
    {
        u8 i;
        for(i = 0; i < s_rx_want; i++) LxRx.buf[i] = s_rx_buf[i];
        LxRx.len   = s_rx_want;
        LxRx.frames++;
        LxRx.ready = 1;                 /* publish flag written last */
    }
    lx_rx_reset();
}
```

- [ ] **Step 3: 追加发送与读接口**

```c
/* ---- TX ---- */
void lx_send_frame(const u8 *buf, u8 len)
{
    u8 i;
    for(i = 0; i < len; i++)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) { }
        USART2->DR = buf[i];
    }
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) { }
    delay_us(LX_GAP_US);
}

static void lx_send_cmd(u8 id, u8 cmd)
{
    u8 f[16];
    lx_send_frame(f, lx_build(f, id, cmd, 0, 0));
}

static void lx_send_prm1(u8 id, u8 cmd, u8 a)
{
    u8 p[1]; u8 f[16];
    p[0] = a;
    lx_send_frame(f, lx_build(f, id, cmd, p, 1));
}

static void lx_send_prm4(u8 id, u8 cmd, u8 a, u8 b, u8 c, u8 d)
{
    u8 p[4]; u8 f[16];
    p[0] = a; p[1] = b; p[2] = c; p[3] = d;
    lx_send_frame(f, lx_build(f, id, cmd, p, 4));
}

/* ---- convenience writers ---- */
void lx_move_time(u8 id, u16 pos, u16 t_ms)
{
    if(pos > 1000) pos = 1000;              /* clamp BOTH ends */
    lx_send_prm4(id, LX_MOVE_TIME_WRITE,
                 (u8)pos, (u8)(pos >> 8), (u8)t_ms, (u8)(t_ms >> 8));
}

void lx_move_wait(u8 id, u16 pos, u16 t_ms)
{
    if(pos > 1000) pos = 1000;
    lx_send_prm4(id, LX_MOVE_TIME_WAIT_WRITE,
                 (u8)pos, (u8)(pos >> 8), (u8)t_ms, (u8)(t_ms >> 8));
}

void lx_move_start_all(void) { lx_send_cmd(LX_BROADCAST, LX_MOVE_START); }
void lx_move_stop(u8 id)     { lx_send_cmd(id, LX_MOVE_STOP); }
void lx_torque(u8 id, u8 on) { lx_send_prm1(id, LX_LOAD_OR_UNLOAD_WRITE, on ? 1 : 0); }

void lx_set_mode(u8 id, u8 mode, s16 speed)
{
    lx_send_prm4(id, LX_OR_MOTOR_MODE_WRITE,
                 mode, 0, (u8)speed, (u8)(speed >> 8));
}

void lx_set_id(u8 old_id, u8 new_id)
{
    lx_send_prm1(old_id, LX_ID_WRITE, new_id);
}

void lx_set_limit(u8 id, u16 min_pos, u16 max_pos)
{
    lx_send_prm4(id, LX_ANGLE_LIMIT_WRITE,
                 (u8)min_pos, (u8)(min_pos >> 8),
                 (u8)max_pos, (u8)(max_pos >> 8));
}

/* ---- reads: send, then wait for the ISR to publish a reply ---- */
static u8 lx_wait_reply(u8 len_want, u16 timeout_ms)
{
    u32 t0 = getSysTickCnt();
    LxRx.ready = 0;
    while(LxRx.ready == 0)
    {
        if((u32)(getSysTickCnt() - t0) > timeout_ms) return 1;   /* timeout */
    }
    if(LxRx.len != len_want) return 2;
    return 0;
}

/* One read = send the query frame, then wait for the ISR to publish a
 * reply of the expected total length. */
#define LX_QUERY(id, cmd, resp_len)                        \
    do {                                                   \
        lx_send_cmd((id), (cmd));                          \
        if(lx_wait_reply((resp_len), LX_READ_TIMEOUT_MS))  \
            return 1;                                      \
    } while(0)

u8 lx_read_pos(u8 id, s16 *pos)
{
    LX_QUERY(id, LX_POS_READ, LX_RESP_POS);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *pos = lx_pos_from_resp((const u8 *)LxRx.buf);
    return 0;
}

u8 lx_read_vin(u8 id, u16 *mv)
{
    LX_QUERY(id, LX_VIN_READ, LX_RESP_VIN);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *mv = lx_u16_from_resp((const u8 *)LxRx.buf);
    return 0;
}

u8 lx_read_temp(u8 id, u8 *degc)
{
    LX_QUERY(id, LX_TEMP_READ, LX_RESP_TEMP);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *degc = lx_u8_from_resp((const u8 *)LxRx.buf);
    return 0;
}

u8 lx_read_id(u8 id, u8 *out)
{
    LX_QUERY(id, LX_ID_READ, LX_RESP_ID);
    if(lx_check((const u8 *)LxRx.buf, LxRx.len, id)) return 1;
    *out = lx_u8_from_resp((const u8 *)LxRx.buf);
    return 0;
}

/* Raw passthrough for the bring-up debug frame. Sends tx, then collects
 * whatever bytes appear within timeout_ms. Never blocks longer than that. */
u8 lx_raw_exchange(const u8 *tx, u8 tx_len, u8 *rx, u8 rx_max, u16 timeout_ms)
{
    u32 t0;
    u8  n = 0;

    LxRx.ready = 0;
    lx_send_frame(tx, tx_len);

    t0 = getSysTickCnt();
    while((u32)(getSysTickCnt() - t0) <= timeout_ms)
    {
        if(LxRx.ready && n < rx_max)
        {
            u8 i;
            for(i = 0; i < LxRx.len && n < rx_max; i++) rx[n++] = LxRx.buf[i];
            LxRx.ready = 0;
        }
    }
    return n;
}
```

- [ ] **Step 4: 把 `LX_RX` 与 `lx_rx_byte` 声明补进 `lx_servo.h`**

```c
typedef struct _LX_RX_
{
    volatile u8  buf[12];
    volatile u8  len;
    volatile u8  ready;      /* ISR sets 1 last; the task clears it    */
    volatile u32 frames;
    volatile u32 errs;
} LX_RX;

extern LX_RX LxRx;
void lx_rx_byte(u8 c);
```

- [ ] **Step 5: 编译验证宿主测试仍然通过**

Run: `python _ros_tooling/_gen_arm_test.py`
Expected: `ALL PASSED (0 failures)`，`exit=0`。
这一步验证新增的胶水代码没有破坏被抽取的纯逻辑段（生成器只抽 `LX_LOGIC_BEGIN/END` 之间的内容，胶水在标记之外，所以宿主构建里不会出现 STM32 符号）。

- [ ] **Step 6: 记录结果**

贴 `_gen_arm_test.py` 输出到 impl 文档的「执行记录」。
**上板验证（本任务无法自动验证，列入 Task 10）**：`USART2_IRQHandler` 真的被调用、`LxRx.frames` 会增长。Task 10 的 M-probe 脚本第一条就是发 `55 55 01 03 0E ED` 读 ID1。

---

### Task 3: ROS 帧 `0xAC`（下行）/ `0x56`（上行）

**Files:**
- Modify: `HARDWARE/usartx.h`（追加帧常量与结构声明，字节级）
- Modify: `HARDWARE/usartx.c`（追加 `0xAC` 解码状态机 + `0x56` 组帧发送）
- Create: `_ros_tooling/_apply_arm_task3.py`
- Modify: `_ros_tooling/_arm_tests.c`（追加 T3/T4 用例）
- Modify: `_ros_tooling/_gen_arm_test.py`（把 `0xAC` 解码器也抽进来）

**Interfaces:**
- Consumes: Task 2 的 `crc16_modbus()`（`usartx.c` 既有的）
- Produces:
  - `ROS_ARM RosArm`（`volatile s16 pos[6]`, `t_ms`, `flags`, `seq`, `rx_ready`, `frames`, `crc_err`）
  - `void ros_arm_byte(u8 c)` —— 在 `USART1_IRQHandler` 里追加调用
  - `void ros_send_arm_fb(void)` —— arm_task 调用，发 `0x56`
  - `ARM_ST_*` 状态位宏

- [ ] **Step 1: 写 `_ros_tooling/_apply_arm_task3.py`**

```python
# -*- coding: utf-8 -*-
"""Task 3: 0xAC arm command frame down, 0x56 arm feedback frame up.

usartx.h / usartx.c are GBK + CRLF. Byte level surgery only - never
decode(), the files carry byte sequences that are not valid in any
Unicode codec. Anchors must contain \r\n.
"""
CRLF = b'\r\n'

def enc(s):
    return s.replace('\n', '\r\n').encode('ascii')

# ------------------------------------------------------------- usartx.h --
p = 'HARDWARE/usartx.h'
b = open(p, 'rb').read()

anchor = enc('''#define ROS_SERVO_NUM        6
''')
assert b.count(anchor) == 1, 'usartx.h anchor = %d' % b.count(anchor)

block = enc('''
/**************************************************************************
 * Arm command frame  (third downlink frame type, header 0xAC)
 *
 *   0  header  0xAC
 *   1  len     0x14                  total frame length
 *   2  p[1]    int16  raw  0..1000    gripper      (servo id 1)
 *   4  p[2]    int16  raw  0..1000    wrist roll   (servo id 2)
 *   6  p[3]    int16  raw  0..1000    wrist pitch  (servo id 3)
 *   8  p[4]    int16  raw  0..1000    forearm      (servo id 4)
 *  10  p[5]    int16  raw  0..1000    upper arm    (servo id 5)
 *  12  p[6]    int16  signed -1000..+1000  base yaw  (servo id 6)
 *  14  t_ms    uint16 ms              travel time, 0 = firmware default
 *  16  flags   uint8  bit0 = re-home the base
 *  17  seq     uint8
 *  18  crc     uint16 little endian, CRC-16/MODBUS over bytes 0..17
 *
 * Units are raw servo counts on purpose: the ROS side already owns the
 * joint-angle <-> raw mapping (it has the URDF), and keeping the firmware
 * free of that layer is what makes it testable. Base field 0 is the centre
 * of the +-225 deg travel, so one count is 0.225 deg.
 *
 * One frame means "every joint walks to its target over t_ms". Streaming
 * is not a separate mode: the host simply sends these at 20-50 Hz with a
 * small t_ms, and point-to-point sends them rarely with a large one.
 **************************************************************************/
#define ROS_ARM_SIZE         20
#define ROS_ARM_HEAD         0xAC
#define ROS_ARM_LEN          0x14
#define ROS_ARM_CRC_IDX      18
#define ROS_ARM_NUM          6

#define ROS_ARM_FLAG_REHOME  0x01    /* bit0: re-run base homing */

typedef struct _ROS_ARM_
{
	volatile unsigned char rx_ready;
	volatile unsigned char flags;
	volatile unsigned char seq;
	volatile short         pos[ROS_ARM_NUM];  /* raw counts, see above    */
	volatile unsigned short t_ms;
	volatile unsigned int  frames;
	volatile unsigned int  crc_err;
	volatile unsigned int  head_err;
} ROS_ARM;

extern ROS_ARM RosArm;
void ros_arm_byte(unsigned char c);   /* called from the USART1 ISR */

/**************************************************************************
 * Arm feedback frame  (second uplink frame type, header 0x56)
 *
 *   0  header  0x56
 *   1  len     0x16                  total frame length
 *   2  a[1..5] int16  raw 0..1000    the five position joints, read back
 *  12  a[6]    int16  signed        base angle estimate, -1000..+1000
 *  14  pot_raw int16                base potentiometer raw reading
 *  16  status  uint8  ARM_ST_*
 *  17  base_st uint8  ARM_BASE_* enumerator
 *  18  seq     uint8
 *  19  rsvd    0
 *  20  crc     uint16 little endian, CRC-16/MODBUS over bytes 0..19
 *
 * A separate frame rather than an extension of the 28 byte 0x55 uplink:
 * that frame was verified on hardware and the host parser is known good,
 * so it stays byte for byte identical. Two uplink headers is also
 * symmetric with the two downlink headers.
 **************************************************************************/
#define ROS_ARMFB_SIZE       22
#define ROS_ARMFB_HEAD       0x56
#define ROS_ARMFB_LEN        0x16
#define ROS_ARMFB_CRC_IDX    20

/* status bits for ROS_ARMFB.status */
#define ARM_ST_ONLINE        0x01
#define ARM_ST_BASE_KNOWN    0x02
#define ARM_ST_BASE_BLIND    0x04
#define ARM_ST_HOMING        0x08
#define ARM_ST_BASE_FAULT    0x10
#define ARM_ST_OVERTEMP      0x20
#define ARM_ST_UNDERVOLT     0x40
#define ARM_ST_ESTOP         0x80

typedef struct _ROS_ARM_FB_
{
	short         pos[ROS_ARM_NUM];
	short         pot_raw;
	unsigned char status;
	unsigned char base_state;
} ROS_ARM_FB;

extern ROS_ARM_FB RosArmFb;
void ros_send_arm_fb(void);

''')

b = b.replace(anchor, anchor + block, 1)
open(p, 'wb').write(b)
print('ok  usartx.h arm frame block added')
```

- [ ] **Step 2: 追加 `0xAC` 解码状态机到 `usartx.c`**

用同一个脚本继续（锚点选在 `int USART1_IRQHandler(void)` 之前）：

```python
# ------------------------------------------------------------- usartx.c --
p = 'HARDWARE/usartx.c'
b = open(p, 'rb').read()

anchor = enc('''int USART1_IRQHandler(void)
''')
assert b.count(anchor) == 1, 'usartx.c ISR anchor = %d' % b.count(anchor)

decoder = enc('''/**************************************************************************
 * Arm frame decoder, header 0xAC.
 *
 * A THIRD independent state machine on the USART1 byte stream. 0xAA, 0xAB
 * and 0xAC are disjoint headers, so no decoder can lock onto another's
 * frame, and the two verified paths are untouched. Same shape as
 * ros_sv_byte() above on purpose - a reader who understands one
 * understands all three.
 **************************************************************************/
ROS_ARM RosArm;
ROS_ARM_FB RosArmFb;

static unsigned char s_arm_buf[ROS_ARM_SIZE];
static unsigned char s_arm_idx   = 0;
static unsigned char s_arm_state = 0;  /* 0 hunt, 1 length, 2 payload */

static void ros_arm_reset(void)
{
	s_arm_idx   = 0;
	s_arm_state = 0;
}

static void ros_arm_publish(void)
{
	unsigned char i;
	for(i = 0; i < ROS_ARM_NUM; i++)
		RosArm.pos[i] = (short)((unsigned short)s_arm_buf[2 + i * 2]
		                      | ((unsigned short)s_arm_buf[3 + i * 2] << 8));
	RosArm.t_ms    = (unsigned short)((unsigned short)s_arm_buf[14]
	                                | ((unsigned short)s_arm_buf[15] << 8));
	RosArm.flags   = s_arm_buf[16];
	RosArm.seq     = s_arm_buf[17];
	RosArm.frames++;
	RosArm.rx_ready = 1;
}

void ros_arm_byte(unsigned char c)
{
	unsigned short crc;
	unsigned char  k, n;

	switch(s_arm_state)
	{
		case 0:
			if(c == ROS_ARM_HEAD)
			{
				s_arm_buf[0] = c;
				s_arm_idx    = 1;
				s_arm_state  = 1;
			}
			break;

		case 1:
			if(c == ROS_ARM_LEN)
			{
				s_arm_buf[1] = c;
				s_arm_idx    = 2;
				s_arm_state  = 2;
			}
			else if(c == ROS_ARM_HEAD)
			{
				s_arm_buf[0] = c;
				s_arm_idx    = 1;
			}
			else
			{
				RosArm.head_err++;
				ros_arm_reset();
			}
			break;

		default:
			s_arm_buf[s_arm_idx++] = c;
			if(s_arm_idx < ROS_ARM_SIZE) break;

			crc = (unsigned short)s_arm_buf[ROS_ARM_CRC_IDX]
			    | ((unsigned short)s_arm_buf[ROS_ARM_CRC_IDX + 1] << 8);

			if(crc == crc16_modbus(s_arm_buf, ROS_ARM_CRC_IDX))
			{
				ros_arm_publish();
				ros_arm_reset();
			}
			else
			{
				RosArm.crc_err++;
				for(k = 1; k < ROS_ARM_SIZE; k++)
					if(s_arm_buf[k] == ROS_ARM_HEAD) break;

				if(k < ROS_ARM_SIZE)
				{
					n = 0;
					while(k < ROS_ARM_SIZE) s_arm_buf[n++] = s_arm_buf[k++];
					s_arm_idx   = n;
					s_arm_state = 1;
				}
				else
				{
					ros_arm_reset();
				}
			}
			break;
	}
}

''')

b = b.replace(anchor, decoder + anchor, 1)
print('ok  ros_arm_byte() added')
open(p, 'wb').write(b)
```

- [ ] **Step 3: 把 `ros_arm_byte(c)` 挂进 USART1 ISR**

```python
p = 'HARDWARE/usartx.c'
b = open(p, 'rb').read()
old = enc('''		ros_rx_byte(c);                   /* motion frame, 0xAA - unchanged */
		ros_sv_byte(c);                   /* servo frame,  0xAB - independent */
''')
assert b.count(old) == 1, 'ISR call site = %d' % b.count(old)
new = enc('''		ros_rx_byte(c);                   /* motion frame, 0xAA - unchanged */
		ros_sv_byte(c);                   /* servo frame,  0xAB - independent */
		ros_arm_byte(c);                  /* arm frame,    0xAC - independent */
''')
b = b.replace(old, new, 1)
open(p, 'wb').write(b)
print('ok  ros_arm_byte() wired into USART1_IRQHandler')
```

- [ ] **Step 4: 追加 `0x56` 组帧发送到 `usartx.c`**

放在 `ros_send_frame()` 之后（锚点用它的结尾 `for(i = 0; i < ROS_UP_SIZE; i++) usart1_send(buf[i]);`）。

```python
p = 'HARDWARE/usartx.c'
b = open(p, 'rb').read()
anchor = enc('''	for(i = 0; i < ROS_UP_SIZE; i++) usart1_send(buf[i]);
}
''')
assert b.count(anchor) == 1, 'ros_send_frame tail = %d' % b.count(anchor)

sender = enc('''/**************************************************************************
Function: Transmit the 0x56 arm feedback frame. Called from arm_task at
          20Hz - the arm owns this frame, not the chassis task.
 **************************************************************************/
void ros_send_arm_fb(void)
{
	unsigned char buf[ROS_ARMFB_SIZE];
	unsigned char i, seq;
	short         sx;
	/* transmit sequence for the 0x56 frame. Owned by arm_task, which is
	 * the only caller - hence no critical section. Kept separate from
	 * RosArm.seq, which the USART1 ISR overwrites with the RECEIVED
	 * downlink sequence on every accepted 0xAC frame. */
	static unsigned char s_armfb_seq = 0;

	seq = (unsigned char)(++s_armfb_seq);

	buf[0] = ROS_ARMFB_HEAD;
	buf[1] = ROS_ARMFB_LEN;
	for(i = 0; i < ROS_ARM_NUM; i++)
	{
		buf[2 + i * 2] = (unsigned char)((unsigned short)RosArmFb.pos[i] & 0xFF);
		buf[3 + i * 2] = (unsigned char)(((unsigned short)RosArmFb.pos[i] >> 8) & 0xFF);
	}
	sx = RosArmFb.pot_raw;
	buf[14] = (unsigned char)((unsigned short)sx & 0xFF);
	buf[15] = (unsigned char)(((unsigned short)sx >> 8) & 0xFF);
	buf[16] = RosArmFb.status;
	buf[17] = RosArmFb.base_state;
	buf[18] = seq;
	buf[19] = 0;
	ros_put_u16(&buf[ROS_ARMFB_CRC_IDX], crc16_modbus(buf, ROS_ARMFB_CRC_IDX));

	for(i = 0; i < ROS_ARMFB_SIZE; i++) usart1_send(buf[i]);
}

''')

b = b.replace(anchor, anchor + sender, 1)
open(p, 'wb').write(b)
print('ok  ros_send_arm_fb() added')
```

- [ ] **Step 5: 把 `0xAC` 解码器也纳入宿主测试**

改 `_gen_arm_test.py`：在 `tests = read(...)` 之前插入对 `usartx.c` 的两段切片 —— **真实的 `crc16_modbus()` 和真实的三段解码器**。

> ⚠️ **不要**在测试文件里手抄一份等价状态机。Task 2 已经证明"按标记逐字切片真实函数"是可行的，而手抄副本会与被测物漂移 —— 项目记忆里的教训正是"用例看不见接线"。这里连 CRC 都用真实的，顺便把 usartx.h 注释里给的**权威校验向量** `crc16_modbus("123456789") == 0x4B37` 也断言上。

```python
# ---- slice the real CRC and the real 0xAC decoder out of usartx.c ----
# usartx.c is GBK + CRLF; read() already gives a latin-1 byte round trip.
uc = read('HARDWARE/usartx.c')

# the CRC the firmware actually uses. Test frames are built with it too,
# so a wrong CRC implementation cannot pass by being wrong on both sides.
crc = slice_between(uc, 'u16 crc16_modbus(const u8 *buf, u16 len)',
                        '\n}\n', 'crc16_modbus')
crc = crc + '\n}\n'
must(crc, '0xA001', 'the reflected polynomial')

# reset + publish + byte machine, verbatim
dec = slice_between(uc, 'static void ros_arm_reset(void)',
                        '/* ==== ARM_TASK_BEGIN ==== */', 'arm decoder')
must(dec, 'ros_arm_publish', 'the publish hook')
must(dec, 'ROS_ARM_CRC_IDX', 'the crc offset')

# the frame constants the decoder and the layout test both need
uh = read('HARDWARE/usartx.h')
arm_defs = []
for line in uh.split('\n'):
    s = line.strip()
    if s.startswith('#define ROS_ARM') or s.startswith('#define ARM_ST_'):
        m = re.match(r'#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)\b', s)
        if m:
            arm_defs.append('#define %-22s %s' % (m.group(1), m.group(2)))
arm_defs = '\n'.join(arm_defs)
must(arm_defs, 'ROS_ARM_SIZE', '0xAC size')
must(arm_defs, 'ROS_ARMFB_CRC_IDX', '0x56 crc offset')
must(arm_defs, 'ARM_ST_ESTOP', 'status bits')
```

Host 模板再加两个占位：`@@CRC@@`、`@@ARMDEFS@@`、`@@ARMDEC@@`。

> ⚠️ 结束标记还不存在，**同时要求在 `usartx.c` 里 `ros_arm_byte()` 之后插入一行标记**作为切片右边界（否则上面第二段会一直切到文件尾，把后面无关的代码也拖进宿主构建）：

```python
p = 'HARDWARE/usartx.c'
b = open(p, 'rb').read()
anchor = enc('''				else
				{
					ros_arm_reset();
				}
			}
			break;
	}
}
''')
assert b.count(anchor) == 1, 'ros_arm_byte tail = %d' % b.count(anchor)
b = b.replace(anchor, anchor + enc('/* ==== ARM_TASK_BEGIN ==== */\n'), 1)
open(p, 'wb').write(b)
print('ok  slice boundary marker added')
```

然后把宿主模板改成也包含解码器，并追加 T3/T4 用例（见 Step 6）。

- [ ] **Step 6: 追加 T3/T4 用例到 `_ros_tooling/_arm_tests.c`**

```c
/* ------------------------------------------------------------------
 * T3: 0xAC arm frame decoder - CRC, resync after garbage, partial frames.
 * T4: 0x56 feedback frame layout round trip.
 *
 * The state machine under test is the REAL ros_arm_reset / ros_arm_publish
 * / ros_arm_byte, sliced out of usartx.c by the generator, and the CRC is
 * the real crc16_modbus. Only the mailbox struct and the two frame
 * constants are declared here - nothing is hand-copied, so this cannot
 * drift away from what ships.
 * ------------------------------------------------------------------ */

#define T_ROS_ARM_SIZE      20
#define T_ROS_ARM_HEAD      0xAC
#define T_ROS_ARM_LEN       0x14
#define T_ROS_ARM_CRC_IDX   18
#define T_ROS_ARM_NUM       6

/* The mailbox, declared exactly as usartx.h does, because the sliced
 * ros_arm_publish() writes these field names by hand. If Task 3's header
 * edit renames a field, this stops compiling - which is the point. */
typedef struct _ROS_ARM_
{
    volatile unsigned char  rx_ready;
    volatile unsigned char  flags;
    volatile unsigned char  seq;
    volatile short          pos[T_ROS_ARM_NUM];
    volatile unsigned short t_ms;
    volatile unsigned int   frames;
    volatile unsigned int   crc_err;
    volatile unsigned int   head_err;
} ROS_ARM;

ROS_ARM RosArm;

static void arm_feed(const u8 *f, int n)
{
    int i;
    for(i = 0; i < n; i++) ros_arm_byte(f[i]);
}

/* Clear the mailbox and the decoder's internal state between cases. The
 * decoder's statics are file-scope in usartx.c; ros_arm_reset() is the
 * sliced function that clears them, which is why it is part of the slice. */
static void arm_reset_all(void)
{
    memset(&RosArm, 0, sizeof(RosArm));
    ros_arm_reset();
}

static void build_arm_frame(u8 *f, s16 p1,s16 p2,s16 p3,s16 p4,s16 p5,s16 p6,
                            u16 t_ms, u8 flags, u8 seq)
{
    s16 p[6]; int i;
    p[0]=p1; p[1]=p2; p[2]=p3; p[3]=p4; p[4]=p5; p[5]=p6;
    f[0]=T_ROS_ARM_HEAD; f[1]=T_ROS_ARM_LEN;
    for(i=0;i<6;i++){ f[2+i*2]=(u8)((u16)p[i]&0xFF); f[3+i*2]=(u8)(((u16)p[i]>>8)&0xFF); }
    f[14]=(u8)(t_ms&0xFF); f[15]=(u8)((t_ms>>8)&0xFF);
    f[16]=flags; f[17]=seq;
    {
        /* the REAL crc, sliced from usartx.c. A test frame whose checksum
         * came from a second, hand-written implementation could pass while
         * both sides were wrong in the same way. */
        u16 c = crc16_modbus(f, T_ROS_ARM_CRC_IDX);
        f[T_ROS_ARM_CRC_IDX]   = (u8)(c & 0xFF);
        f[T_ROS_ARM_CRC_IDX+1] = (u8)(c >> 8);
    }
}

static void test_crc_vector(void)
{
    /* the check vector quoted in usartx.h's own header comment */
    static const u8 v[9] = { '1','2','3','4','5','6','7','8','9' };
    u16 got = crc16_modbus(v, 9);
    if(got != 0x4B37)
    { printf("FAIL crc16 vector 0x%04X != 0x4B37\n", got); fails++; }
    else printf("ok   crc16_modbus(\"123456789\") == 0x4B37\n");
}

static void test_arm_frame(void)
{
    u8 f[T_ROS_ARM_SIZE];

    /* a clean frame decodes, into the firmware's own mailbox */
    arm_reset_all();
    build_arm_frame(f, 226, 500, 177, 129, 408, 500, 1500, 0, 7);
    arm_feed(f, T_ROS_ARM_SIZE);
    if(RosArm.frames != 1 || RosArm.rx_ready != 1)
    { printf("FAIL arm frame not published\n"); fails++; }
    else if(RosArm.pos[0]!=226 || RosArm.pos[1]!=500 || RosArm.pos[2]!=177 ||
            RosArm.pos[3]!=129 || RosArm.pos[4]!=408 || RosArm.pos[5]!=500)
    { printf("FAIL arm positions wrong\n"); fails++; }
    else if(RosArm.t_ms != 1500 || RosArm.seq != 7)
    { printf("FAIL arm t/seq wrong\n"); fails++; }
    else printf("ok   arm frame decodes\n");

    /* the base field is signed - this is the whole point of the design */
    arm_reset_all();
    build_arm_frame(f, 500,500,500,500,500, (s16)-1000, 40, 0, 8);
    arm_feed(f, T_ROS_ARM_SIZE);
    if(RosArm.pos[5] != -1000)
    { printf("FAIL base -1000 -> %d\n", RosArm.pos[5]); fails++; }
    else printf("ok   base field is signed (-1000)\n");

    /* a bad CRC is rejected, counted, and NOT published */
    arm_reset_all();
    build_arm_frame(f, 1,2,3,4,5,6, 100, 0, 9);
    f[T_ROS_ARM_CRC_IDX] ^= 0xFF;
    arm_feed(f, T_ROS_ARM_SIZE);
    if(RosArm.frames != 0 || RosArm.rx_ready != 0 || RosArm.crc_err != 1)
    { printf("FAIL crc reject (frames=%u ready=%u err=%u)\n",
             RosArm.frames, RosArm.rx_ready, RosArm.crc_err); fails++; }
    else printf("ok   bad CRC rejected\n");

    /* garbage then a good frame: the decoder must resync */
    arm_reset_all();
    {
        u8 noise[7];
        int i;
        for(i = 0; i < 7; i++) noise[i] = (u8)(0x10 + i);
        arm_feed(noise, 7);
    }
    build_arm_frame(f, 226,500,177,129,408, 0, 2000, 0, 10);
    arm_feed(f, T_ROS_ARM_SIZE);
    if(RosArm.frames != 1 || RosArm.pos[5] != 0 || RosArm.t_ms != 2000)
    { printf("FAIL resync after garbage (frames=%u)\n", RosArm.frames); fails++; }
    else printf("ok   resync after garbage\n");

    /* half a frame, then a whole one: the decoder must not stay stuck */
    arm_reset_all();
    build_arm_frame(f, 226,500,177,129,408, 500, 1500, 0, 11);
    arm_feed(f, 9);
    build_arm_frame(f, 100,200,300,400,500, (s16)-500, 800, 0, 12);
    arm_feed(f, T_ROS_ARM_SIZE);
    if(RosArm.frames != 1 || RosArm.pos[0] != 100 || RosArm.pos[5] != -500)
    { printf("FAIL recovery from a truncated frame (frames=%u)\n",
             RosArm.frames); fails++; }
    else printf("ok   recovers from a truncated frame\n");

    /* every base value round trips, including both extremes */
    {
        int bad = 0;
        s16 v;
        for(v = -1000; v <= 1000; v += 7)
        {
            arm_reset_all();
            build_arm_frame(f, 500,500,500,500,500, v, 0, 0, 0);
            arm_feed(f, T_ROS_ARM_SIZE);
            if(RosArm.frames != 1 || RosArm.pos[5] != v) bad++;
        }
        if(bad) { printf("FAIL %d base values failed round trip\n", bad); fails++; }
        else printf("ok   base field round trips -1000..1000\n");
    }
}

static void test_frame_layout_invariants(void)
{
    /* ros_send_arm_fb() pushes bytes through usart1_send(), so it is not
     * host testable. What IS worth pinning are the layout invariants a
     * host parser depends on: they fail silently if a later edit changes
     * one of a pair and forgets the other. */
    if(ROS_ARMFB_LEN != ROS_ARMFB_SIZE)
    { printf("FAIL 0x56 len %u != size %u\n", ROS_ARMFB_LEN, ROS_ARMFB_SIZE); fails++; }
    else if(ROS_ARMFB_SIZE - ROS_ARMFB_CRC_IDX != 2)
    { printf("FAIL 0x56 crc offset %u\n", ROS_ARMFB_CRC_IDX); fails++; }
    else if(ROS_ARM_LEN != ROS_ARM_SIZE)
    { printf("FAIL 0xAC len %u != size %u\n", ROS_ARM_LEN, ROS_ARM_SIZE); fails++; }
    else if(ROS_ARM_SIZE - ROS_ARM_CRC_IDX != 2)
    { printf("FAIL 0xAC crc offset %u\n", ROS_ARM_CRC_IDX); fails++; }
    else printf("ok   frame layout invariants (0xAC and 0x56)\n");
}
```

- [ ] **Step 7: 在宿主 `main()` 里加上新用例**

```c
    test_crc_vector();
    test_arm_frame();
    test_frame_layout_invariants();
```

- [ ] **Step 8: 跑测试**

Run: `python _ros_tooling/_gen_arm_test.py`
Expected: 零警告，`ALL PASSED (0 failures)`，`exit=0`。

- [ ] **Step 9: 记录结果**

贴输出到「执行记录」。

---

### Task 4: `0xAD` 调试透传 + bring-up 探测脚本

**Files:**
- Modify: `HARDWARE/usartx.h`（追加 `0xAD`/`0x57` 常量与 `ROS_RAW` 结构）
- Modify: `HARDWARE/usartx.c`（追加 `0xAD` 变长解码器 + `0x57` 发送）
- Create: `_ros_tooling/_apply_arm_task4.py`
- Create: `_ros_tooling/_arm_probe.py`

**Interfaces:**
- Consumes: Task 1 的 `lx_raw_exchange()`；Task 2 的 `LX_GAP_US`
- Produces:
  - `ROS_RAW RosRaw`（`rx_ready`, `n`, `raw[16]`, `timeout_ms`）
  - `void ros_raw_byte(u8 c)`；`void ros_send_raw_fb(const u8 *rx, u8 n)`
  - `_arm_probe.py`：树莓派上跑的 bring-up 工具，命令行子命令 `id`/`pot`/`spin`/`cal`

> **为什么这一整个任务要先做**：`协议规范` 里 M1（电机模式下 `0x1C` 是否连续可读、盲区是截断还是回绕）**没有答案**，而它决定了 Task 7 的寻零策略选哪个分支。没有透传就只能靠人肉在 PC 上敲指令，M1 这种"每 20ms 读一次、连读 30 秒"的测量根本做不了。

- [ ] **Step 1: 写 `_apply_arm_task4.py`（追加 `0xAD` 解码器）**

> **备份命名规则（Task 2/3 已在用，后续所有任务照此执行）**：备份目录 `_ros_tooling/_backup_<任务名>/`，
> 目录内的文件名把原路径的 `/` 换成 `__`、保留扩展名 —— `HARDWARE/usartx.c` → **`HARDWARE__usartx.c`**、
> `HARDWARE/usartx.h` → `HARDWARE__usartx.h`、`_ros_tooling/_ws_tests.c` → `_ros_tooling___ws_tests.c`。
> 依据：`_backup_arm_task2/`、`_backup_arm_task3/`、`_backup_armfb_seq/`、`_backup_task2/` 全部是这个名字。
> Task 4 第一次落地时写成了裸 `usartx.c`，已重命名归位（内容未变，sha256 一致）。

```python
# -*- coding: utf-8 -*-
"""Task 4: raw passthrough frame, so bring-up measurements can be driven
from the Pi. 0xAD down / 0x57 up. Compile-time switchable, off in the
shipped build.
"""
CRLF = b'\r\n'
def enc(s): return s.replace('\n', '\r\n').encode('ascii')

# ---- usartx.h --------------------------------------------------------
p = 'HARDWARE/usartx.h'
b = open(p, 'rb').read()
anchor = enc('''#define ARM_ST_ESTOP         0x80
''')
assert b.count(anchor) == 1
block = enc('''
/**************************************************************************
 * Raw bus passthrough  (bring-up only, header 0xAD down / 0x57 up)
 *
 * USART2 is the servo bus, so a PC cannot be plugged into it any more.
 * These two frames let a script on the Pi drive the bus by proxy, which
 * is how the base joint has to be characterised (motor-mode pot
 * behaviour, speed calibration, inter-frame gap).
 *
 *   0xAD down:  0 hdr  1 len  2..3 timeout_ms u16  4 n u8
 *               5.. bytes  4+n.. crc u16
 *   0x57 up:    0 hdr  1 len  2 seq  3 status  4..5 n_rx u16
 *               6.. rx bytes  6+n_rx.. crc u16
 *
 * Guarded by ARM_DEBUG_PASSTHROUGH so it can be compiled out.
 **************************************************************************/
#define ARM_DEBUG_PASSTHROUGH   0    /* set to 1 for a bring-up build */

#define ROS_RAW_SIZE         26      /* 5 header + 16 payload + 2 crc + ... */
#define ROS_RAW_HEAD         0xAD
#define ROS_RAW_MIN_LEN      7       /* header..crc with zero payload      */
#define ROS_RAW_MAX_PAYLOAD  16

#define ROS_RAWFB_SIZE       24
#define ROS_RAWFB_HEAD       0x57

typedef struct _ROS_RAW_
{
	volatile unsigned char rx_ready;
	volatile unsigned char n;
	volatile unsigned char raw[ROS_RAW_MAX_PAYLOAD];
	volatile unsigned short timeout_ms;
	volatile unsigned int  frames;
	volatile unsigned int  crc_err;
} ROS_RAW;

extern ROS_RAW RosRaw;
void ros_raw_byte(unsigned char c);
void ros_send_raw_fb(const unsigned char *rx, unsigned char n);

''')
b = b.replace(anchor, anchor + block, 1)
open(p, 'wb').write(b)
print('ok  usartx.h passthrough block added')

# ---- usartx.c: decoder ----------------------------------------------
p = 'HARDWARE/usartx.c'
b = open(p, 'rb').read()
anchor = enc('/* ==== ARM_TASK_BEGIN ==== */\n')
assert b.count(anchor) == 1
dec = enc('''
/**************************************************************************
 * Raw passthrough decoder, header 0xAD. Variable length: the length byte
 * gives the frame total, exactly like the other three decoders.
 **************************************************************************/
ROS_RAW RosRaw;

static unsigned char s_raw_buf[ROS_RAW_SIZE];
static unsigned char s_raw_idx   = 0;
static unsigned char s_raw_want  = 0;
static unsigned char s_raw_state = 0;

void ros_raw_byte(unsigned char c)
{
	unsigned short crc;

	if(s_raw_state == 0)
	{
		if(c == ROS_RAW_HEAD) { s_raw_buf[0] = c; s_raw_idx = 1; s_raw_state = 1; }
		return;
	}
	if(s_raw_state == 1)
	{
		if(c < ROS_RAW_MIN_LEN || c > ROS_RAW_SIZE)
		{
			RosRaw.crc_err++;
			s_raw_idx = 0; s_raw_state = 0;
			if(c == ROS_RAW_HEAD) { s_raw_buf[0] = c; s_raw_idx = 1; s_raw_state = 1; }
			return;
		}
		s_raw_buf[1] = c;
		s_raw_want   = c;
		s_raw_idx    = 2;
		s_raw_state  = 2;
		return;
	}
	s_raw_buf[s_raw_idx++] = c;
	if(s_raw_idx < s_raw_want) return;

	crc = (unsigned short)s_raw_buf[s_raw_want - 2]
	    | ((unsigned short)s_raw_buf[s_raw_want - 1] << 8);
	if(crc == crc16_modbus(s_raw_buf, (unsigned short)(s_raw_want - 2)))
	{
		unsigned char i;
		RosRaw.timeout_ms = (unsigned short)s_raw_buf[2]
		                  | ((unsigned short)s_raw_buf[3] << 8);
		RosRaw.n = s_raw_buf[4];
		for(i = 0; i < RosRaw.n && i < ROS_RAW_MAX_PAYLOAD; i++)
			RosRaw.raw[i] = s_raw_buf[5 + i];
		RosRaw.frames++;
		RosRaw.rx_ready = 1;
	}
	else
	{
		RosRaw.crc_err++;
	}
	s_raw_idx = 0; s_raw_state = 0;
}

void ros_send_raw_fb(const unsigned char *rx, unsigned char n)
{
	unsigned char buf[ROS_RAWFB_SIZE];
	unsigned char i;
	static unsigned char seq = 0;

	if(n > (ROS_RAWFB_SIZE - 8)) n = (unsigned char)(ROS_RAWFB_SIZE - 8);

	buf[0] = ROS_RAWFB_HEAD;
	buf[1] = (unsigned char)(n + 8);
	buf[2] = seq++;
	buf[3] = 0;                       /* status: reserved, 0 = ok */
	buf[4] = n;
	buf[5] = 0;
	for(i = 0; i < n; i++) buf[6 + i] = rx[i];
	ros_put_u16(&buf[6 + n], crc16_modbus(buf, (unsigned short)(6 + n)));
	for(i = 0; i < (unsigned char)(8 + n); i++) usart1_send(buf[i]);
}
''')
b = b.replace(anchor, dec + anchor, 1)
open(p, 'wb').write(b)
print('ok  ros_raw_byte() / ros_send_raw_fb() added')

# ---- wire it into the ISR, guarded by the macro ----------------------
p = 'HARDWARE/usartx.c'
b = open(p, 'rb').read()
old = enc('''		ros_arm_byte(c);                  /* arm frame,    0xAC - independent */
''')
assert b.count(old) == 1
new = enc('''		ros_arm_byte(c);                  /* arm frame,    0xAC - independent */
#if ARM_DEBUG_PASSTHROUGH
		ros_raw_byte(c);                  /* bring-up passthrough, 0xAD       */
#endif
''')
b = b.replace(old, new, 1)
open(p, 'wb').write(b)
print('ok  passthrough wired into USART1_IRQHandler')
```

- [ ] **Step 2: 让 `arm_task` 执行透传请求**

在 Task 8 的 `arm_task` 里加一段（先在这里写出来，Task 8 会把它放进循环）：

```c
#if ARM_DEBUG_PASSTHROUGH
	/* ---- bring-up passthrough: proxy one raw frame onto the servo bus ---- */
	if(RosRaw.rx_ready)
	{
		u8 raw[ROS_RAW_MAX_PAYLOAD];
		u8 n, tmo;
		u16 tmo_ms;
		u8 rx[ROS_RAWFB_SIZE - 8];

		taskENTER_CRITICAL();
		n       = RosRaw.n;
		tmo_ms  = RosRaw.timeout_ms;
		for(tmo = 0; tmo < n; tmo++) raw[tmo] = RosRaw.raw[tmo];
		RosRaw.rx_ready = 0;
		taskEXIT_CRITICAL();

		tmo = (u8)lx_raw_exchange(raw, n, rx, (u8)sizeof(rx), tmo_ms);
		ros_send_raw_fb(rx, tmo);
	}
#endif
```

- [ ] **Step 3: 写 bring-up 探测脚本 `_ros_tooling/_arm_probe.py`**

**这个脚本在树莓派上跑**（连的是 `/dev/ttyAMA*` 或 USB 转串口到 STM32 的 USART1）。

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bring-up probe for the arm bus servos, driven from the Pi.

USART2 belongs to the servo bus, so the Pi cannot talk to the servos
directly. This script wraps raw 0x55 frames in the firmware's 0xAD
passthrough frame and unwraps the 0x57 replies. That is how the M1-M5
measurements in the design doc get taken.

Build the firmware with ARM_DEBUG_PASSTHROUGH = 1 first.

Subcommands
-----------
  id    [timeout]          read the ID of every servo 1..6
  pot   <id> <secs>        poll POS_READ at 20 Hz and print raw + degrees
  spin  <id> <speed> <secs> motor mode at `speed`, polling POS_READ throughout
  mode  <id> <0|1> <speed>  set the mode register
  torque <id> <0|1>         load / unload
  raw   <hexbytes...>       send arbitrary bytes, print the reply

NOTE: write commands get NO reply from the servo, so `raw` on a write will
just time out. That is correct behaviour, not a bug.
"""
import argparse, struct, sys, time

try:
    import serial
except ImportError:
    sys.exit('pyserial is required: pip3 install pyserial')

HEAD_DOWN, HEAD_UP = 0xAD, 0x57
LX_HEAD = 0x55


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def check_crc16():
    assert crc16_modbus(b'123456789') == 0x4B37, 'crc16 vector failed'


def lx_frame(servo_id: int, cmd: int, params: bytes = b'') -> bytes:
    """Build one 0x55 frame. Length = params + 3, checksum = ~sum & 0xFF."""
    length = (len(params) + 3) & 0xFF
    body = bytes([servo_id, length, cmd]) + params
    chk = (~sum(body)) & 0xFF
    return bytes([LX_HEAD, LX_HEAD]) + body + bytes([chk])


def wrap_raw(port, payload: bytes, timeout_ms: int) -> bytes:
    """Send `payload` over the passthrough frame; return the servo's bytes.

    The length byte is the TOTAL frame length (7 + payload), matching the
    decoder in usartx.c which collects `len` bytes in total and reads the
    CRC from buf[len-2..len-1]. An earlier draft said 6 + payload, which
    is one short - the decoder would then compute the CRC over a different
    span than the sender used and reject every single frame. Verified
    against the real decoder: 6/6 cases returned nothing before the fix.
    """
    frame = bytes([HEAD_DOWN, 7 + len(payload),
                   timeout_ms & 0xFF, (timeout_ms >> 8) & 0xFF,
                   len(payload)]) + payload
    frame += struct.pack('<H', crc16_modbus(frame))
    port.reset_input_buffer()
    port.write(frame)
    port.flush()

    buf = b''
    deadline = time.time() + (timeout_ms / 1000.0) + 0.25
    while time.time() < deadline:
        chunk = port.read(64)
        if chunk:
            buf += chunk
            # a complete 0x57 frame: header, len, seq, status, n_lo, n_hi, ...
            if len(buf) >= 6 and buf[0] == HEAD_UP and len(buf) >= buf[1]:
                n = struct.unpack('<H', buf[4:6])[0]
                return bytes(buf[6:6 + n])
        else:
            time.sleep(0.002)
    return b''


def lx_read_pos(port, servo_id: int) -> int:
    """Returns the raw pot reading, or None on timeout."""
    reply = wrap_raw(port, lx_frame(servo_id, 0x1C), 30)
    # 55 55 ID 05 1C posL posH CS
    if len(reply) != 8 or reply[0] != LX_HEAD or reply[1] != LX_HEAD:
        return None
    body = reply[2:7]
    if reply[7] != (~sum(body)) & 0xFF:
        return None
    return struct.unpack('<h', reply[5:7])[0]


def lx_read_id(port, servo_id: int):
    reply = wrap_raw(port, lx_frame(servo_id, 0x0E), 50)
    if len(reply) != 7 or reply[0] != LX_HEAD:
        return None
    return reply[5]


def lx_write(port, servo_id: int, cmd: int, params: bytes):
    wrap_raw(port, lx_frame(servo_id, cmd, params), 20)


# --------------------------------------------------------------- commands

def cmd_id(port, args):
    for sid in range(1, 7):
        got = lx_read_id(port, sid)
        print('servo %d -> %s' % (sid, 'NO REPLY' if got is None else 'ID %d' % got))


def cmd_pot(port, args):
    """M-1 helper: watch one servo's pot reading.

    Run this while the joint sits still to find the noise floor, and again
    while the base is in motor mode to see whether the reading tracks
    continuously (and whether it pins at a rail or wraps).
    """
    t0 = time.time()
    last = None
    jumps = 0
    while time.time() - t0 < args.secs:
        raw = lx_read_pos(port, args.id)
        now = time.time() - t0
        deg = None if raw is None else raw * 240.0 / 1000.0 - 120.0
        if raw is not None and last is not None and abs(raw - last) > 500:
            jumps += 1
            print('  %6.3fs  raw=%-5s  *** JUMP %+d  (wrap #%d) ***' %
                  (now, raw, raw - last, jumps))
        else:
            print('  %6.3fs  raw=%-5s  deg=%s' %
                  (now, raw, 'n/a' if deg is None else '%+.1f' % deg))
        last = raw
        time.sleep(0.05)
    print('---- %d wrap-sized jumps in %.1fs ----' % (jumps, args.secs))
    if jumps == 0:
        print('M1 verdict: no wrap seen -> likely TRUNCATING (set ARM_POT_WRAPS 0)')
    else:
        print('M1 verdict: %d wraps -> WRAPPING (set ARM_POT_WRAPS 1), '
              'check the jump spacing is a stable 240 deg' % jumps)


def cmd_spin(port, args):
    """M-3 helper: motor mode at a fixed speed, log the pot reading.

    Put the base inside the pot's valid window first, then read off the
    total angle change over `secs` to get deg/s per speed unit.
    """
    lx_write(port, args.id, 0x1F, bytes([1]))                 # torque on
    lx_write(port, args.id, 0x1D,
             bytes([1, 0]) + struct.pack('<h', args.speed))    # motor mode
    print('spinning id %d at %+d for %.1fs' % (args.id, args.speed, args.secs))
    t0 = time.time()
    first = last = None
    while time.time() - t0 < args.secs:
        raw = lx_read_pos(port, args.id)
        if raw is not None:
            if first is None:
                first = raw
            last = raw
        time.sleep(0.02)
    lx_write(port, args.id, 0x1D, bytes([1, 0]) + struct.pack('<h', 0))
    lx_write(port, args.id, 0x1F, bytes([0]))                  # torque off
    if first is None or last is None:
        print('no readings - is the servo in the pot window?')
        return
    delta_deg = (last - first) * 240.0 / 1000.0
    print('delta = %+.1f deg over %.2fs -> %.1f deg/s at speed %+d'
          % (delta_deg, args.secs, delta_deg / args.secs, args.speed))
    print('K = %.5f deg/s per speed unit' % (delta_deg / args.secs / args.speed))


def cmd_mode(port, args):
    lx_write(port, args.id, 0x1D, bytes([args.mode, 0]) + struct.pack('<h', args.speed))
    print('id %d mode -> %d speed -> %d (write has no reply, as expected)'
          % (args.id, args.mode, args.speed))


def cmd_torque(port, args):
    lx_write(port, args.id, 0x1F, bytes([args.on]))
    print('id %d torque -> %d' % (args.id, args.on))


def cmd_raw(port, args):
    payload = bytes(int(h, 16) for h in args.hexbytes)
    reply = wrap_raw(port, payload, args.timeout)
    print('sent    : ' + ' '.join('%02X' % x for x in payload))
    print('received: ' + (' '.join('%02X' % x for x in reply) or '(nothing)'))


def main():
    check_crc16()
    ap = argparse.ArgumentParser(description='arm bus-servo bring-up probe')
    ap.add_argument('-p', '--port', default='/dev/ttyAMA0')
    ap.add_argument('-b', '--baud', type=int, default=115200)
    sub = ap.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('id');     s.add_argument('timeout', type=float, default=0.5, nargs='?')
    s.set_defaults(func=cmd_id)

    s = sub.add_parser('pot');    s.add_argument('id', type=int); s.add_argument('secs', type=float, default=10.0)
    s.set_defaults(func=cmd_pot)

    s = sub.add_parser('spin');   s.add_argument('id', type=int); s.add_argument('speed', type=int)
    s.add_argument('secs', type=float, default=5.0)
    s.set_defaults(func=cmd_spin)

    s = sub.add_parser('mode');   s.add_argument('id', type=int); s.add_argument('mode', type=int)
    s.add_argument('speed', type=int, default=0)
    s.set_defaults(func=cmd_mode)

    s = sub.add_parser('torque'); s.add_argument('id', type=int); s.add_argument('on', type=int)
    s.set_defaults(func=cmd_torque)

    s = sub.add_parser('raw');    s.add_argument('hexbytes', nargs='+')
    s.add_argument('--timeout', type=int, default=50)
    s.set_defaults(func=cmd_raw)

    args = ap.parse_args()
    with serial.Serial(args.port, args.baud, timeout=0) as port:
        args.func(port, args)


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 验证脚本自身的 CRC 与组帧**

Run: `python -c "import sys; sys.path.insert(0,'_ros_tooling'); import _arm_probe as p; p.check_crc16(); print(' '.join('%02X'%x for x in p.lx_frame(1,0x0E))); print(' '.join('%02X'%x for x in p.lx_frame(1,0x01,bytes([0xF4,0x01,0xE8,0x03]))))"`
Expected 输出：`55 55 01 03 0E ED` 与 `55 55 01 07 01 F4 01 E8 03 16` —— 与 Task 1 的 T1 用例一致。这是**跨语言交叉校验**：Python 与 C 两条独立实现必须给出同一串字节。

- [ ] **Step 5: 记录结果**

贴输出到「执行记录」。

---

### Task 5: `arm.c` 骨架 + 上电状态机

**Files:**
- Create: `BALANCE/arm.h`
- Create: `BALANCE/arm.c`
- Create: `BALANCE/arm_cal.h`

**Interfaces:**
- Consumes: Task 1/2 的 `lx_*`；Task 3 的 `RosArm` / `RosArmFb` / `ARM_ST_*`
- Produces:
  - `void arm_task(void *pvParameters)` —— Task 8 会填满循环，本任务先建骨架 + 上电状态机
  - `ARM_BASE ArmBase;` 与 `ARM_CAL ArmCal;`（Task 6/7 用）
  - `const u8 ARM_JOINT_ID[6]` = `{1,2,3,4,5,6}`
  - `const u16 ARM_RESET_POSE[6]` = `{226,500,177,129,408,500}`

- [ ] **Step 1: 写 `BALANCE/arm_cal.h`**

**纯 ASCII + LF。** 这些数值全部来自上板实测，`ARM_CAL_MEASURED` 在 Task 10 回填后置 1。

```c
#ifndef __ARM_CAL_H
#define __ARM_CAL_H

/* ------------------------------------------------------------------
 * Base joint calibration. EVERY number here comes from a bring-up
 * measurement (M1..M8 in the design doc) - none of them are guesses that
 * should be trusted. ARM_CAL_MEASURED stays 0 until they have been
 * replaced with measured values, and arm_task refuses to close the loop
 * on the base while it is 0.
 * ------------------------------------------------------------------ */

#define ARM_CAL_MEASURED     0    /* Task 10 flips this to 1 */

/* ---- M1: which way does the pot behave outside the 240 deg window? ----
 * 0 = reading pins at a rail and stops moving  (truncating)
 * 1 = reading keeps moving and jumps by a full scale every 240 deg
 */
#define ARM_POT_WRAPS        0

/* ---- M2: the pot window, from _arm_probe.py pot + a protractor ---- */
#define ARM_POT_DEG_AT_RAW0  (-120.0f)  /* mechanical deg when raw == 0     */
#define ARM_POT_DEG_PER_RAW  (0.24f)    /* 240 deg over 1000 counts         */
#define ARM_POT_LO           60         /* usable window, raw counts        */
#define ARM_POT_HI           940

/* ---- M3: speed calibration, measured at the real pack voltage ---- */
#define ARM_SPEED_1000_DPS   150.0f     /* deg/s when the speed field is 1000 */

/* ---- M4: the ground lost to the servo's acceleration ramp ---- */
#define ARM_ACCEL_COMP_DEG   2.0f

/* ---- M5: does speed 0 hold the axis? if not, lower KP below ---- */
#define ARM_VEL_KP           20.0f
#define ARM_VEL_MAX          600        /* speed field clamp                */

/* ---- base travel ------------------------------------------------------ */
#define ARM_SOFT_LIMIT_DEG   225        /* the requirement                  */
#define ARM_SEARCH_SPEED     200        /* slow - this is a search           */

/* ---- search budget ----------------------------------------------------
 * Worst case distance from anywhere in +-225 to the pot window edge is
 *   soft_limit - pot_valid_half = 225 - 105.6 = 119.4 deg
 *
 * Two independent caps, because neither alone is sound:
 *
 * ARM_SEARCH_BOUND_DEG is a PROGRESS cap on the dead-reckoned travel. It
 *   has to exceed 119.4 with margin for a speed calibration that reads
 *   low, so 135 tolerates 13% error. M3 must confirm the calibration is
 *   inside that; if it is not, this number moves.
 *
 * ARM_SEARCH_MAX_MS is a TIME cap and is the real safety net. At the
 *   search speed the true travel is at most
 *   ARM_SEARCH_MAX_MS/1000 * ARM_SPEED_1000_DPS * 1.25
 *   regardless of how wrong the calibration is, so it bounds the
 *   excursion even in the total-pot-failure case:
 *   3.5 s * 150 deg/s * 1.25 = 656 ... times the speed fraction below.
 *   Recomputed in Task 10 once M3 gives a real ARM_SPEED_1000_DPS.
 *
 * ARM_SEARCH_REVERSE is 0 for a truncating pot: the rail already tells us
 * the side, so leg 1 always moves TOWARD the window and a second leg
 * would only ever be a mistake that walks away from it (224 + 135 + 280 =
 * 639 deg, straight past the cable budget). Set it to 1 only for a
 * wrapping pot, where the hint can be wrong and coming back matters.
 */
#define ARM_SEARCH_BOUND_DEG   135.0f
#define ARM_SEARCH_BOUND2_DEG  280.0f
#define ARM_SEARCH_MAX_MS      3500
#define ARM_SEARCH_REVERSE     0

/* ---- how close to the window edge the tracker starts probing again --- */
#define ARM_RESYNC_MARGIN_DEG  20.0f

/* ---- health limits ---- */
#define ARM_TEMP_MAX_C       70
#define ARM_VIN_MIN_MV       6400

/* Consecutive 0x1C read failures during base homing before giving up.
 * Faulting early is what keeps the dead-pot corner out of the cable
 * budget: a pot that never answers used to be skipped tick after tick
 * while the search ran to ARM_SEARCH_BOUND_DEG. Five ticks is 100 ms -
 * far longer than any real transient, far shorter than the ~4.6 s it
 * takes to spend the whole bound. */
#define ARM_HOME_READ_FAIL_MAX  5

#endif
```

- [ ] **Step 2: 写 `BALANCE/arm.h`**

```c
#ifndef __ARM_H
#define __ARM_H

#include "system.h"
#include "lx_servo.h"
#include "arm_cal.h"

/* ---- task parameters ---- */
#define ARM_TASK_PRIO     3
#define ARM_STK_SIZE      512
#define ARM_RATE_HZ       50

/* ---- joints: 0xAC field i drives servo ARM_JOINT_ID[i] ---- */
#define ARM_JOINTS        6
extern const u8  ARM_JOINT_ID[ARM_JOINTS];    /* {1,2,3,4,5,6} */
extern const u16 ARM_RESET_POSE[ARM_JOINTS];  /* spec 5.5.1    */
#define ARM_BASE_JOINT    5                   /* index of servo 6 */
#define ARM_GRIPPER_FIELD 0
#define ARM_GRIPPER_MAX   700                 /* stall protection */

/* ---- base tracker states (also reported in the 0x56 frame) ---- */
#define ARM_BASE_UNKNOWN  0
#define ARM_BASE_HOMING   1
#define ARM_BASE_OK       2
#define ARM_BASE_BLIND    3
#define ARM_BASE_FAULT    4

/* ---- base tracker (pure data, see the ARM_LOGIC block in arm.c) ---- */
typedef struct
{
    float deg;             /* angle estimate, degrees                    */
    float prev_v;          /* last commanded speed in DEG/S - degrees per
                            * second, not the raw speed field. Both the
                            * blind integration and the accel-compensation
                            * edge detect read it, so it has to be one
                            * unit everywhere.                          */
    float search_travel;   /* degrees travelled in the current leg       */
    u16   search_ms;       /* elapsed time in the current leg - the cap
                            * that holds even if the speed calibration is
                            * wrong                                          */
    s16   pot_raw;
    s16   prev_raw;
    u8    state;
    u8    primed;          /* has a previous reading been stored yet?    */
    s8    hint;            /* sign of the last trusted reading; only
                            * consulted when the pot wraps - a truncating
                            * pot tells us the side directly             */
    s8    search_dir;
    u8    search_reversed;
    u8    in_valid;
} ARM_BASE;

typedef struct
{
    float pot_deg_at_raw0;
    float pot_deg_per_raw;
    s16   pot_lo, pot_hi;
    s16   speed_1000_dps;
    float accel_comp_deg;
    s16   soft_limit_deg;
    s16   search_speed;
    float search_bound_deg;
    float search_bound2_deg;
    u16   search_max_ms;
    float resync_margin_deg;
    float vel_kp;
    s16   vel_max;
} ARM_CAL;

/* ---- pure logic (host testable, ARM_LOGIC block) ---- */
float arm_pot_deg(const ARM_CAL *c, s16 raw);
u8    arm_pot_valid(const ARM_CAL *c, s16 raw);
void  arm_base_init(ARM_BASE *b, s8 last_sign_hint);
u8    arm_homing_step(ARM_BASE *b, const ARM_CAL *c, s16 raw, float dt,
                      s16 *v_cmd_out);
u8    arm_track_step(ARM_BASE *b, const ARM_CAL *c, u8 have_raw, s16 raw,
                     float v_cmd_dps, float dt);
s16   arm_base_vel(const ARM_CAL *c, float err_deg, u16 t_ms);
float arm_field_to_deg(const ARM_CAL *c, s16 field);
s16   arm_deg_to_field(const ARM_CAL *c, float deg);
u8    arm_need_probe(const ARM_CAL *c, const ARM_BASE *b);

/* ---- task ---- */
extern ARM_BASE ArmBase;
extern ARM_CAL  ArmCal;
void arm_cal_load(void);
void arm_task(void *pvParameters);

#endif
```

- [ ] **Step 3: 写 `BALANCE/arm.c` 的骨架与上电状态机**

先只写非纯逻辑部分；`ARM_LOGIC` 段在 Task 6 填。

```c
#include "arm.h"

const u8  ARM_JOINT_ID[ARM_JOINTS]   = { 1, 2, 3, 4, 5, 6 };
const u16 ARM_RESET_POSE[ARM_JOINTS] = { 226, 500, 177, 129, 408, 500 };

ARM_BASE ArmBase;
ARM_CAL  ArmCal;

void arm_cal_load(void)
{
    ArmCal.pot_deg_at_raw0  = ARM_POT_DEG_AT_RAW0;
    ArmCal.pot_deg_per_raw  = ARM_POT_DEG_PER_RAW;
    ArmCal.pot_lo           = ARM_POT_LO;
    ArmCal.pot_hi           = ARM_POT_HI;
    ArmCal.speed_1000_dps   = (s16)ARM_SPEED_1000_DPS;
    ArmCal.accel_comp_deg   = ARM_ACCEL_COMP_DEG;
    ArmCal.soft_limit_deg   = ARM_SOFT_LIMIT_DEG;
    ArmCal.search_speed     = ARM_SEARCH_SPEED;
    ArmCal.search_bound_deg = ARM_SEARCH_BOUND_DEG;
    ArmCal.search_bound2_deg= ARM_SEARCH_BOUND2_DEG;
    ArmCal.search_max_ms    = ARM_SEARCH_MAX_MS;
    ArmCal.resync_margin_deg= ARM_RESYNC_MARGIN_DEG;
    ArmCal.vel_kp           = ARM_VEL_KP;
    ArmCal.vel_max          = ARM_VEL_MAX;
}

/* ==== ARM_LOGIC_BEGIN ==== */
/* (Task 6 fills this in) */
/* ==== ARM_LOGIC_END ==== */

/* ---- bring-up state machine ---------------------------------------- */
#define INIT_WAIT       0
#define INIT_TORQUE     1
#define INIT_PROBE      2
#define INIT_CONFIG     3
#define INIT_POSE       4
#define INIT_HOME       5
#define INIT_READY      6
#define INIT_ABSENT     7

void arm_task(void *pvParameters)
{
    u32 last = getSysTickCnt();
    u8  stage = INIT_WAIT;
    u8  retry = 0;
    u8  j     = 0;
    u8  home_read_fail = 0;      /* consecutive 0x1C failures while homing */
    u32 t0    = getSysTickCnt();

    arm_cal_load();
    ArmBase.state = ARM_BASE_UNKNOWN;
    RosArmFb.status = 0;

    while(1)
    {
        vTaskDelayUntil(&last, F2T(ARM_RATE_HZ));

        switch(stage)
        {
            case INIT_WAIT:
                if((u32)(getSysTickCnt() - t0) > 300u) { stage = INIT_TORQUE; j = 0; }
                break;

            case INIT_TORQUE:
                /* servo power comes up before the MCU; bring each joint up
                 * one at a time so a shorted winding cannot brown out the
                 * whole rail at once */
                if(j < ARM_JOINTS)
                {
                    lx_torque(ARM_JOINT_ID[j], 1);
                    j++;
                }
                else { stage = INIT_PROBE; retry = 0; }
                break;

            case INIT_PROBE:
            {
                /* reading ID1 is the presence test, same idea as the
                 * vendor firmware's bus-vs-PWM detection (spec 7.6) */
                u8 id = 0;
                if(lx_read_id(1, &id) == 0)
                {
                    RosArmFb.status |= ARM_ST_ONLINE;
                    stage = INIT_CONFIG;
                }
                else if(++retry >= 3)
                {
                    stage = INIT_ABSENT;
                }
                break;
            }

            case INIT_CONFIG:
                /* mode is NOT power-down saved - this must run every boot,
                 * and it must be speed 0 or the base takes off */
                lx_set_mode(6, LX_MODE_MOTOR, 0);
                stage = INIT_POSE;
                j = 0;
                break;

            case INIT_POSE:
                /* the five position joints walk to the factory pose
                 * together; the base is deliberately left alone */
                if(j == 0)
                {
                    u8 i;
                    for(i = 0; i < (ARM_JOINTS - 1); i++)
                        lx_move_wait(ARM_JOINT_ID[i], ARM_RESET_POSE[i], 2000);
                    lx_move_start_all();
                    j = 1;
                }
                else
                {
                    arm_base_init(&ArmBase, 1);
                    RosArmFb.status |= ARM_ST_HOMING;
                    stage = INIT_HOME;
                }
                break;

            case INIT_HOME:
                /* Task 7 drives this with real pot readings */
                break;

            case INIT_ABSENT:
            default:
                RosArmFb.status &= (u8)~ARM_ST_ONLINE;
                break;
        }

        /* Task 8 adds the runtime block here */
    }
}
```

- [ ] **Step 4: 编译验证**

Run: `python _ros_tooling/_gen_arm_test.py`
Expected: `ALL PASSED`（本任务的代码还没进宿主测试，这一步只是确认没有破坏生成器）。

- [ ] **Step 5: 记录结果**

贴输出到「执行记录」。**上板验证**：上电后 `INIT_PROBE` 能过（`RosArmFb.status` 的 bit0 置位）、5 个关节会一起走到复位姿态、底座**不动**。

---

### Task 6: 底座跟踪核心（纯逻辑 + 宿主测试 T5/T6）

**Files:**
- Modify: `BALANCE/arm.c`（填 `ARM_LOGIC` 段）
- Modify: `_ros_tooling/_arm_tests.c`（追加 T5/T6）
- Modify: `_ros_tooling/_gen_arm_test.py`（抽取 `arm.c` 的逻辑段）

**Interfaces:**
- Consumes: Task 5 的 `ARM_BASE` / `ARM_CAL`
- Produces: `arm_pot_deg`、`arm_pot_valid`、`arm_base_init`、`arm_homing_step`、`arm_track_step`、`arm_base_vel`、`arm_field_to_deg`、`arm_deg_to_field`、`arm_need_probe`

- [ ] **Step 1: 填 `ARM_LOGIC` 段**

替换 `arm.c` 里的占位行（新建文件是 LF，可以用 Edit）。

```c
/* ==== ARM_LOGIC_BEGIN ==== */
/*
 * Base joint tracking. Deliberately pure: no registers, no globals, no
 * clock, every input an argument. The whole reason this is host testable
 * is that its failure modes (bad direction hint, sitting exactly on the
 * window edge, a noisy reading at the boundary) take tens of minutes to
 * reproduce on the bench and about a microsecond here.
 */

float arm_pot_deg(const ARM_CAL *c, s16 raw)
{
    return c->pot_deg_at_raw0 + c->pot_deg_per_raw * (float)raw;
}

u8 arm_pot_valid(const ARM_CAL *c, s16 raw)
{
    return (u8)((raw >= c->pot_lo && raw <= c->pot_hi) ? 1 : 0);
}

/* Convert the 0xAC base field (-1000..+1000) to degrees and back. */
float arm_field_to_deg(const ARM_CAL *c, s16 field)
{
    return (float)field * (float)c->soft_limit_deg / 1000.0f;
}

s16 arm_deg_to_field(const ARM_CAL *c, float deg)
{
    float f = deg * 1000.0f / (float)c->soft_limit_deg;
    if(f >  1000.0f) f =  1000.0f;
    if(f < -1000.0f) f = -1000.0f;
    return (s16)f;
}

/* Start homing. The hint is the sign the base had the last time the pot
 * was trusted. It is only ever consulted for a WRAPPING pot - a
 * truncating pot reveals the side outright. search_dir stays 0 here and
 * is picked by arm_homing_step() on its first primed call. */
void arm_base_init(ARM_BASE *b, s8 last_sign_hint)
{
    b->deg            = 0.0f;
    b->prev_v         = 0.0f;
    b->search_travel  = 0.0f;
    b->search_ms      = 0;
    b->pot_raw        = 0;
    b->prev_raw       = 0;
    b->state          = ARM_BASE_HOMING;
    b->hint           = last_sign_hint;
    b->search_dir     = 0;
    b->search_reversed= 0;
    b->in_valid       = 0;
    b->primed         = 0;
}

/* Pick the first search direction.
 *
 * ⭐ With a TRUNCATING pot the reading itself says which side the base is
 * on: pinned at the top rail means the angle is positive, at the bottom
 * rail means negative. The search direction is therefore not a guess, and
 * a manual turn while the power was off cannot mislead it. This is the
 * case the user actually asked about, and it needs no hint at all.
 *
 * With a WRAPPING pot the reading carries no side information, so the
 * stored hint is the only evidence and it CAN be wrong. That is the case
 * the two-leg search and the travel caps exist for - see the budget note
 * on ARM_SEARCH_BOUND_DEG in arm_cal.h.
 */
static s8 arm_search_dir_pick(const ARM_CAL *c, s16 raw, s8 hint)
{
#if ARM_POT_WRAPS
    (void)c; (void)raw;
    return (s8)((hint >= 0) ? -1 : +1);
#else
    (void)hint;
    if(raw > c->pot_hi) return -1;   /* pinned high -> angle is positive */
    if(raw < c->pot_lo) return +1;   /* pinned low  -> angle is negative */
    return -1;                       /* mid window, should not happen    */
#endif
}

/* One homing step. Returns the new state; *v_cmd_out is the speed command
 * to write to the servo.
 *
 * Two ways the window announces itself, and which one applies is bring-up
 * item M1:
 *
 *   truncating (ARM_POT_WRAPS 0): outside the window the reading sits on
 *     a rail. arm_pot_valid() turning true is therefore already an
 *     absolute angle and no edge estimate is needed at all. Direction
 *     comes from the rail.
 *   wrapping (ARM_POT_WRAPS 1): the reading keeps moving and jumps by a
 *     full scale once per revolution. The jump frame is the landmark.
 *     Direction comes from the hint.
 *
 * The travel caps are what make this safe on a mechanism with no hard
 * stop, but they only have to carry the weight in the wrapping case:
 *
 *   truncating - the direction is derived from the physical reading, so
 *     leg 1 always moves TOWARD the window and |angle| shrinks
 *     monotonically. The caps are defence in depth only.
 *   wrapping - a wrong hint costs up to search_bound_deg of travel in the
 *     wrong direction before the reverse, so the cap has to satisfy
 *     soft_limit + 1.25 * bound <= 360 which needs bound <= 108. That is
 *     only sound once M3 has shown the speed error is inside 5%; until
 *     then the wrapping path is NOT proven safe and must not be trusted.
 *     T6 asserts both cases separately so the difference stays visible.
 */
u8 arm_homing_step(ARM_BASE *b, const ARM_CAL *c, s16 raw, float dt,
                   s16 *v_cmd_out)
{
    u8 found = 0;

    b->prev_raw = b->pot_raw;
    b->pot_raw  = raw;

    if(!b->primed)
    {
        /* The very first sample has no previous to difference against,
         * and it is the one that decides the direction. Without this the
         * wrapping detector would read pot_raw - 0 as a full scale jump
         * and declare a landmark that is not there. */
        b->primed     = 1;
        b->search_dir = arm_search_dir_pick(c, raw, b->hint);
        *v_cmd_out    = (s16)((b->search_dir > 0) ? c->search_speed
                                                 : -c->search_speed);
        b->state = ARM_BASE_HOMING;
        return ARM_BASE_HOMING;
    }

    if(arm_pot_valid(c, raw))
    {
        found = 1;
    }
#if ARM_POT_WRAPS
    else
    {
        s16 d = (s16)(raw - b->prev_raw);
        if(d > 500 || d < -500) found = 1;
    }
#endif

    if(found)
    {
        b->deg       = arm_pot_deg(c, raw);
        b->state     = ARM_BASE_OK;
        b->in_valid  = 1;
        b->search_travel = 0.0f;
        *v_cmd_out   = 0;
        return ARM_BASE_OK;
    }

    {
        s16   v   = (s16)((b->search_dir > 0) ? c->search_speed
                                              : -c->search_speed);
        float mag = (float)((v < 0) ? -v : v) / 1000.0f;

        b->search_travel += (float)c->speed_1000_dps * mag * dt;
        b->search_ms     += (u16)(dt * 1000.0f);

        /* two independent caps: the dead-reckoned distance, and elapsed
         * time. The time cap is the one that holds even if the speed
         * calibration is badly wrong or the pot has stopped answering. */
        if(b->search_travel >= c->search_bound_deg ||
           b->search_ms     >= c->search_max_ms)
        {
#if ARM_SEARCH_REVERSE
            if(!b->search_reversed)
            {
                b->search_reversed = 1;
                b->search_dir      = (s8)(-b->search_dir);
                b->search_travel   = 0.0f;
                b->search_ms       = 0;
            }
            else
#endif
            {
                b->state   = ARM_BASE_FAULT;
                *v_cmd_out = 0;
                return ARM_BASE_FAULT;
            }
        }
        *v_cmd_out = v;
    }

    b->state = ARM_BASE_HOMING;
    return ARM_BASE_HOMING;
}

/* Running tracker.
 * have_raw = 0 means "do not touch the pot this tick" - deep in the blind
 * zone we deliberately stop reading it, as specified. have_raw = 1 comes
 * from the probe window near the boundary. */
u8 arm_track_step(ARM_BASE *b, const ARM_CAL *c, u8 have_raw, s16 raw,
                  float v_cmd_dps, float dt)
{
    if(b->state == ARM_BASE_UNKNOWN || b->state == ARM_BASE_FAULT)
        return b->state;

    if(have_raw)
    {
        b->prev_raw = b->pot_raw;
        b->pot_raw  = raw;
        if(arm_pot_valid(c, raw))
        {
            b->deg      = arm_pot_deg(c, raw);   /* resync: error to zero */
            b->in_valid = 1;
            b->state    = ARM_BASE_OK;
            b->prev_v   = v_cmd_dps;
            return b->state;
        }
    }

    /* blind: integrate the commanded speed */
    if(b->prev_v == 0.0f && v_cmd_dps != 0.0f)
    {
        /* once per move, cover the ramp the servo never counts */
        b->deg += (v_cmd_dps > 0.0f) ? c->accel_comp_deg : -c->accel_comp_deg;
    }
    b->deg += v_cmd_dps * dt;
    b->prev_v   = v_cmd_dps;
    b->in_valid = 0;
    b->state    = ARM_BASE_BLIND;

    /* soft limit, applied to the ESTIMATE. The cable survives 360 deg, so
     * the estimate can be off by the integration error and still be safe. */
    if(b->deg >  (float)c->soft_limit_deg) b->deg =  (float)c->soft_limit_deg;
    if(b->deg < -(float)c->soft_limit_deg) b->deg = -(float)c->soft_limit_deg;

    return b->state;
}

/* Velocity loop. Point-to-point and streaming share it: a large t_ms
 * makes the feed-forward dominate and arrive on time, a small one clamps
 * it and leaves a plain proportional loop, which is what following wants. */
s16 arm_base_vel(const ARM_CAL *c, float err_deg, u16 t_ms)
{
    float v, ts;

    if(err_deg > -0.5f && err_deg < 0.5f) return 0;

    ts = (t_ms == 0) ? 0.5f : ((float)t_ms / 1000.0f);
    v  = err_deg / ts + c->vel_kp * err_deg;

    if(v >  (float)c->vel_max) v =  (float)c->vel_max;
    if(v < -(float)c->vel_max) v = -(float)c->vel_max;
    return (s16)v;
}

/* Should this tick spend a read on the pot? Only near the boundary -
 * deep in the blind zone the reading is worthless and the frame costs
 * bus time we would rather spend on the control loop. */
u8 arm_need_probe(const ARM_CAL *c, const ARM_BASE *b)
{
    float lo, hi;
    if(b->state == ARM_BASE_HOMING) return 1;   /* always, while homing */
    if(b->state != ARM_BASE_OK && b->state != ARM_BASE_BLIND) return 0;

    lo = arm_pot_deg(c, c->pot_lo) - c->resync_margin_deg;
    hi = arm_pot_deg(c, c->pot_hi) + c->resync_margin_deg;
    return (u8)((b->deg >= lo && b->deg <= hi) ? 1 : 0);
}
/* ==== ARM_LOGIC_END ==== */
```

- [ ] **Step 2: 让生成器也抽 `arm.c` 的逻辑段**

在 `_gen_arm_test.py` 里加：

```python
# ---- slice the base tracker out of arm.c ------------------------------
arm_c = read('BALANCE/arm.c')
arm_logic = slice_between(arm_c, '/* ==== ARM_LOGIC_BEGIN ==== */',
                                 '/* ==== ARM_LOGIC_END ==== */', 'arm logic')

# the calibration macros the logic is compiled against
cal_h = read('BALANCE/arm_cal.h')
cal_defs = []
for line in cal_h.split('\n'):
    s = line.strip()
    if s.startswith('#define ARM_'):
        m = re.match(r'#define\s+(ARM_\w+)\s+(.+?)\s*(?:/\*.*)?$', s)
        if m:
            cal_defs.append('#define %-24s %s' % (m.group(1), m.group(2).strip()))
cal_defs = '\n'.join(cal_defs)
must(cal_defs, 'ARM_POT_WRAPS', 'wrap switch')
must(cal_defs, 'ARM_SOFT_LIMIT_DEG', 'soft limit')
must(cal_defs, 'ARM_SEARCH_BOUND_DEG', 'search bound')
must(cal_defs, 'ARM_SEARCH_BOUND2_DEG', 'second search bound')

# the ARM_BASE / ARM_CAL struct definitions, taken from arm.h verbatim
arm_h = read('BALANCE/arm.h')
s0 = arm_h.index('typedef struct\n{\n    float deg;')
# ⚠️ the end anchor MUST be searched from s0, not from the top of the file.
# str.index() returns the FIRST match, so an unanchored search would find
# the literal inside any earlier comment or prose and slice to nothing -
# which then fails must(arm_types, 'search_bound2_deg') with no hint as to
# why. Task 5 hit exactly this while writing arm.h.
s1 = arm_h.index('} ARM_CAL;', s0) + len('} ARM_CAL;')
arm_types = arm_h[s0:s1]
assert s1 > s0, 'arm.h anchors are out of order'
must(arm_types, 'search_bound2_deg', 'ARM_CAL field')

# the state enumerators
state_defs = '\n'.join(
    '#define %-18s %s' % (n, v) for n, v in
    [('ARM_BASE_UNKNOWN','0'), ('ARM_BASE_HOMING','1'), ('ARM_BASE_OK','2'),
     ('ARM_BASE_BLIND','3'), ('ARM_BASE_FAULT','4')])
must(arm_h, '#define ARM_BASE_FAULT', 'state defines')
```

宿主模板再加两个占位：`@@CALDEFS@@` 和 `@@ARMTYPES@@` / `@@ARMLOGIC@@`。

- [ ] **Step 3: 追加 T5/T6 用例**

```c
/* ------------------------------------------------------------------
 * T5: the base tracker - resync, blind integration, soft limit.
 * T6: the homing safety argument. This one matters more than all the
 *     others: the mechanism has NO hard stop, so the only thing keeping
 *     the cable alive is the travel cap in arm_homing_step(). A future
 *     edit that loosens ARM_SEARCH_BOUND* or flips the direction logic
 *     has to fail here.
 * ------------------------------------------------------------------ */

static ARM_CAL  tc;
static ARM_BASE tb;

static void cal_default(void)
{
    memset(&tc, 0, sizeof(tc));
    tc.pot_deg_at_raw0   = ARM_POT_DEG_AT_RAW0;
    tc.pot_deg_per_raw   = ARM_POT_DEG_PER_RAW;
    tc.pot_lo            = ARM_POT_LO;
    tc.pot_hi            = ARM_POT_HI;
    tc.speed_1000_dps    = (s16)ARM_SPEED_1000_DPS;
    tc.accel_comp_deg    = ARM_ACCEL_COMP_DEG;
    tc.soft_limit_deg    = ARM_SOFT_LIMIT_DEG;
    tc.search_speed      = ARM_SEARCH_SPEED;
    tc.search_bound_deg  = ARM_SEARCH_BOUND_DEG;
    tc.search_bound2_deg = ARM_SEARCH_BOUND2_DEG;
    tc.search_max_ms     = ARM_SEARCH_MAX_MS;
    tc.resync_margin_deg = ARM_RESYNC_MARGIN_DEG;
    tc.vel_kp            = ARM_VEL_KP;
    tc.vel_max           = ARM_VEL_MAX;
}

static void test_pot_mapping(void)
{
    cal_default();
    /* raw 500 must be the centre of the window */
    if(arm_pot_valid(&tc, 500) != 1) { printf("FAIL 500 invalid\n"); fails++; }
    else printf("ok   raw 500 is inside the window\n");

    if(arm_pot_valid(&tc, 10) != 0) { printf("FAIL 10 should be outside\n"); fails++; }
    else printf("ok   raw 10 is outside the window\n");

    {
        float d = arm_pot_deg(&tc, 500);
        if(d < -0.05f || d > 0.05f) { printf("FAIL centre %.2f != 0\n", d); fails++; }
        else printf("ok   raw 500 -> 0.0 deg\n");
    }
    /* the field conversion is a pure scale of the soft limit */
    if(arm_deg_to_field(&tc, 225.0f) != 1000) { printf("FAIL deg_to_field\n"); fails++; }
    else if(arm_deg_to_field(&tc, -225.0f) != -1000) { printf("FAIL deg_to_field neg\n"); fails++; }
    else if(arm_deg_to_field(&tc, 9999.0f) != 1000) { printf("FAIL deg_to_field clamp\n"); fails++; }
    else printf("ok   field <-> deg scaling and clamp\n");

    if(arm_field_to_deg(&tc, -1000) != -225.0f) { printf("FAIL field_to_deg\n"); fails++; }
    else printf("ok   field -1000 -> -225 deg\n");
}

static void test_track_resync(void)
{
    cal_default();
    arm_base_init(&tb, 1);
    tb.state = ARM_BASE_BLIND;
    tb.deg   = 200.0f;                 /* deliberately wrong */

    /* a valid reading must snap the estimate back */
    {
        u8 st = arm_track_step(&tb, &tc, 1, 800, 0.0f, 0.02f);
        float want = tc.pot_deg_at_raw0 + tc.pot_deg_per_raw * 800.0f;
        if(st != ARM_BASE_OK) { printf("FAIL resync state %u\n", st); fails++; }
        else if(tb.deg < want - 0.01f || tb.deg > want + 0.01f)
        { printf("FAIL resync %.2f != %.2f\n", tb.deg, want); fails++; }
        else printf("ok   valid reading resyncs (200 -> %.1f)\n", tb.deg);
    }

    /* blind integration accumulates at the commanded rate */
    {
        float before = tb.deg;
        int i;
        for(i = 0; i < 50; i++) arm_track_step(&tb, &tc, 0, 0, 100.0f, 0.02f);
        /* 50 ticks * 20 ms * 100 deg/s = 100 deg, plus one accel comp */
        float got = tb.deg - before;
        float want = 100.0f + ARM_ACCEL_COMP_DEG;
        if(got < want - 0.5f || got > want + 0.5f)
        { printf("FAIL blind integrate %.1f != %.1f\n", got, want); fails++; }
        else printf("ok   blind integration %.1f deg (incl. accel comp)\n", got);
    }

    /* the soft limit clamps the estimate */
    {
        int i;
        for(i = 0; i < 500; i++) arm_track_step(&tb, &tc, 0, 0, 500.0f, 0.02f);
        if(tb.deg > (float)ARM_SOFT_LIMIT_DEG + 0.01f)
        { printf("FAIL soft limit %.1f\n", tb.deg); fails++; }
        else printf("ok   soft limit holds at %+.1f deg\n", tb.deg);
    }
}

/* ---- T6 ------------------------------------------------------------- */
/* A plant model of the base: a 240 deg pot that pins at the rails, and a
 * base that moves at exactly the speed it is told. Keeping the plant
 * ideal is deliberate - these cases are about the ALGORITHM. Speed
 * calibration error gets its own explicit case below.
 *
 * This block matters more than every other test in the plan. The
 * mechanism has NO hard stop, so the only thing keeping the cable alive
 * is the travel/time cap in arm_homing_step(). A later edit that loosens
 * ARM_SEARCH_BOUND* or ARM_SEARCH_MAX_MS, or that reinstates a reverse
 * leg for a truncating pot, has to fail here. */
typedef struct { float true_deg; float k_true; } SIM;

static s16 sim_pot(float true_deg)
{
    float rawf = (true_deg + 120.0f) / 240.0f * 1000.0f;
    if(rawf < 0.0f)    rawf = 0.0f;      /* pinned at the low rail  */
    if(rawf > 1000.0f) rawf = 1000.0f;   /* pinned at the high rail */
    return (s16)rawf;
}

/* One homing session against the plant.
 * k_true is the TRUE deg/s per speed unit while the firmware reckons
 * with ARM_SPEED_1000_DPS, so k_true != ARM_SPEED_1000_DPS models a
 * calibration error. Returns 1 if the window was reached; *peak is the
 * largest |true angle| the base visited. */
static int run_homing(float start_deg, s8 hint, float k_true,
                      float *peak, int *ticks)
{
    SIM   sim;
    float dt = 0.02f;
    int   n  = 0;

    sim.true_deg = start_deg;
    sim.k_true   = k_true;

    arm_base_init(&tb, hint);
    *peak = 0.0f;

    while(n < 8000)
    {
        s16 v  = 0;
        u8  st = arm_homing_step(&tb, &tc, sim_pot(sim.true_deg), dt, &v);

        sim.true_deg += (float)v * sim.k_true / 1000.0f * dt;
        /* clip only far outside anything the mechanism can reach, so a
         * runaway still shows up in *peak rather than being hidden */
        if(sim.true_deg >  2000.0f) sim.true_deg =  2000.0f;
        if(sim.true_deg < -2000.0f) sim.true_deg = -2000.0f;

        if(sim.true_deg >  *peak) *peak =  sim.true_deg;
        if(-sim.true_deg > *peak) *peak = -sim.true_deg;

        n++;
        if(st == ARM_BASE_OK)    { *ticks = n; return 1; }
        if(st == ARM_BASE_FAULT) { *ticks = n; return 0; }
    }
    *ticks = n;
    return 0;
}

static void test_homing_direction_from_rail(void)
{
    /* ⭐ The point of the truncating-pot design: the direction comes from
     * which rail the reading is pinned to, NOT from the stored hint. A
     * hint left stale by someone turning the base by hand while the power
     * was off therefore cannot send it the wrong way. Every start runs
     * with BOTH hints and must behave identically.
     *
     * |angle| only ever shrinks here, so the peak is just the starting
     * angle - the strongest statement available. */
    float starts[] = {   0.0f,  104.0f,  110.0f,  130.0f,  180.0f,  224.0f,
                      -104.0f, -110.0f, -130.0f, -180.0f, -224.0f };
    s8 hints[] = { +1, -1 };
    size_t i, h;
    float worst = 0.0f;
    int bad = 0;

    for(i = 0; i < sizeof(starts)/sizeof(starts[0]); i++)
    {
        for(h = 0; h < 2; h++)
        {
            float peak; int ticks;
            int ok = run_homing(starts[i], hints[h], ARM_SPEED_1000_DPS,
                                &peak, &ticks);
            if(!ok || peak > 225.0f)
            {
                printf("FAIL homing start %+7.1f hint %+d: ok=%d peak=%.1f\n",
                       starts[i], hints[h], ok, peak);
                bad++;
            }
            if(peak > worst) worst = peak;
        }
    }
    if(bad) fails += bad;
    else printf("ok   homing reaches the window from every start, both "
                "hints (worst peak %.1f deg)\n", worst);
}

static void test_homing_survives_speed_error(void)
{
    /* The progress cap is dead-reckoned, so ARM_SEARCH_BOUND_DEG has to
     * clear the 119.4 deg worst case even when the calibration reads low.
     * This asserts the margin that 135 was picked for. */
    float starts[] = { 224.0f, -224.0f, 130.0f, -130.0f };
    float errs[]   = { 0.8f, 1.0f, 1.25f };     /* true / calibrated */
    size_t i, e;
    int bad = 0;

    for(i = 0; i < sizeof(starts)/sizeof(starts[0]); i++)
    {
        for(e = 0; e < sizeof(errs)/sizeof(errs[0]); e++)
        {
            float peak; int ticks;
            int   ok;
            float k = ARM_SPEED_1000_DPS * errs[e];

            ok = run_homing(starts[i], (errs[e] > 1.0f) ? +1 : -1, k,
                            &peak, &ticks);
            if(!ok)
            {
                printf("FAIL homing start %+7.1f speed x%.2f: never reached "
                       "the window (peak %.1f) - raise ARM_SEARCH_BOUND_DEG\n",
                       starts[i], errs[e], peak);
                bad++;
            }
            else if(peak > 360.0f)
            {
                printf("FAIL homing start %+7.1f speed x%.2f: peak %.1f over "
                       "the cable budget - lower ARM_SEARCH_BOUND_DEG\n",
                       starts[i], errs[e], peak);
                bad++;
            }
        }
    }
    if(bad) fails += bad;
    else printf("ok   homing holds for 0.8x..1.25x speed error\n");
}

static void test_homing_dead_pot(void)
{
    /* If the pot stops answering, the search must give up rather than
     * walk to the end of the cable. The TIME cap is the net that catches
     * this - the distance cap cannot, because its own accounting is
     * exactly what a broken pot makes untrustworthy. */
    float dt   = 0.02f;
    float v    = 224.0f;            /* start at the far end */
    float peak = 224.0f;
    int   i;
    u8    st = ARM_BASE_HOMING;

    arm_base_init(&tb, 1);
    for(i = 0; i < 8000 && st != ARM_BASE_FAULT; i++)
    {
        s16 vc = 0;
        st = arm_homing_step(&tb, &tc, 0, dt, &vc);   /* stuck at raw 0 */
        v += (float)vc * ARM_SPEED_1000_DPS / 1000.0f * dt;
        if(v > peak) peak = v;
    }

    if(st != ARM_BASE_FAULT)
    { printf("FAIL dead pot never faulted\n"); fails++; }
    else if(peak > 360.0f)
    { printf("FAIL dead pot ran to %.0f deg - ARM_SEARCH_MAX_MS too large\n",
             peak); fails++; }
    else
    { printf("ok   dead pot faults at %.0f deg after %.1f s\n",
             peak, (float)i * dt); }
}

static void test_vel_loop(void)
{
    cal_default();
    if(arm_base_vel(&tc, 0.1f, 1000) != 0) { printf("FAIL deadband\n"); fails++; }
    else printf("ok   velocity loop deadband\n");

    {
        s16 v = arm_base_vel(&tc, 10.0f, 1000);      /* 10 deg in 1 s */
        if(v < 5 || v > 30) { printf("FAIL vel %.0f\n", (float)v); fails++; }
        else printf("ok   velocity loop 10 deg in 1s -> %.0f\n", (float)v);
    }
    {
        s16 v = arm_base_vel(&tc, 200.0f, 40);       /* streaming, big err */
        if(v != ARM_VEL_MAX) { printf("FAIL clamp %.0f\n", (float)v); fails++; }
        else printf("ok   velocity loop clamps to %d\n", ARM_VEL_MAX);
    }
    {
        s16 a = arm_base_vel(&tc, 10.0f, 0);         /* t_ms 0 -> 500ms default */
        s16 b = arm_base_vel(&tc, 10.0f, 500);
        if(a != b) { printf("FAIL t_ms 0 default %d != %d\n", a, b); fails++; }
        else printf("ok   t_ms 0 falls back to 500 ms\n");
    }
}

static void test_need_probe(void)
{
    cal_default();
    arm_base_init(&tb, 1);
    tb.state = ARM_BASE_OK;

    tb.deg = 0.0f;
    if(arm_need_probe(&tc, &tb) != 1) { printf("FAIL probe at centre\n"); fails++; }
    else printf("ok   probe wanted at centre\n");

    tb.deg = 200.0f;
    if(arm_need_probe(&tc, &tb) != 0) { printf("FAIL probe deep in blind\n"); fails++; }
    else printf("ok   no probe deep in the blind zone\n");

    tb.deg = 100.0f;    /* inside the window -> always probe */
    if(arm_need_probe(&tc, &tb) != 1) { printf("FAIL probe in window\n"); fails++; }
    else printf("ok   probe wanted inside the window\n");

    tb.state = ARM_BASE_HOMING;
    tb.deg   = 300.0f;
    if(arm_need_probe(&tc, &tb) != 1) { printf("FAIL probe while homing\n"); fails++; }
    else printf("ok   probe always wanted while homing\n");
}
```

- [ ] **Step 4: 在宿主 `main()` 里加新用例**

```c
    test_pot_mapping();
    test_track_resync();
    test_homing_direction_from_rail();
    test_homing_survives_speed_error();
    test_homing_dead_pot();
    test_vel_loop();
    test_need_probe();
```

- [ ] **Step 5: 跑测试**

Run: `python _ros_tooling/_gen_arm_test.py`
Expected: 零警告；`homing reaches the window from every start, both hints (worst peak 224.0 deg)`；`homing holds for 0.8x..1.25x speed error`；`dead pot faults at ~329 deg after 3.5 s`；`ALL PASSED (0 failures)`。

若 `test_homing_direction_from_rail` 有 `FAIL ... ok=0`，说明 `arm_search_dir_pick()` 的方向判反了 —— 检查 `raw > pot_hi` 对应的是"角度为正"这一前提。
若 `test_homing_survives_speed_error` 报 `never reached the window`，按 Step 1 的推导重算 `ARM_SEARCH_BOUND_DEG`（需要 ≥ 119.4 / 最低速比）。
若报 `peak ... over the cable budget`，说明时间上限或距离上限太松。

- [ ] **Step 6: 记录结果**

贴输出到「执行记录」。**特别记下三个 `peak`** —— 它们是线缆安全论证的实测数字。

---

### Task 7: 底座寻零与速度环接入 `arm_task`

**Files:**
- Modify: `BALANCE/arm.c`（填 `INIT_HOME` 分支 + 运行期底座块）

**Interfaces:**
- Consumes: Task 6 的全部纯函数
- Produces: 底座从 `HOMING` 走到 `OK/FAULT`；`RosArmFb.pot_raw` / `base_state` / `status` 开始有意义

- [ ] **Step 1: 填 `INIT_HOME` 分支**

替换 Task 5 里 `case INIT_HOME:` 的占位注释。

```c
            case INIT_HOME:
            {
                s16 raw = 0;
                s16 v   = 0;
                u8  st;

                if(lx_read_pos(6, &raw))
                {
                    /* A missing reading must NOT be silently skipped. The
                     * first version just "break"-ed and kept searching,
                     * which meant a dead pot walked all the way to the
                     * travel cap - and in the bad corner (dead pot AND a
                     * high speed calibration) that is 393 deg, past the
                     * 360 deg the cable survives. Fault out instead: a pot
                     * that cannot answer is not a pot whose blind zone we
                     * have any business driving through. */
                    if(++home_read_fail >= ARM_HOME_READ_FAIL_MAX)
                    {
                        lx_set_mode(6, LX_MODE_MOTOR, 0);
                        ArmBase.state = ARM_BASE_FAULT;
                        RosArmFb.status &= (u8)~ARM_ST_HOMING;
                        RosArmFb.status |=  ARM_ST_BASE_FAULT;
                        stage = INIT_READY;
                    }
                    break;
                }
                home_read_fail = 0;
                st = arm_homing_step(&ArmBase, &ArmCal, raw, 0.02f, &v);
                lx_set_mode(6, LX_MODE_MOTOR, v);

                RosArmFb.pot_raw    = raw;
                RosArmFb.base_state = ArmBase.state;

                if(st == ARM_BASE_OK)
                {
                    RosArmFb.status &= (u8)~ARM_ST_HOMING;
                    RosArmFb.status |= ARM_ST_BASE_KNOWN;
                    stage = INIT_READY;
                }
                else if(st == ARM_BASE_FAULT)
                {
                    /* the base is parked and holding torque; the five
                     * position joints stay usable */
                    RosArmFb.status &= (u8)~ARM_ST_HOMING;
                    RosArmFb.status |= ARM_ST_BASE_FAULT;
                    lx_set_mode(6, LX_MODE_MOTOR, 0);
                    stage = INIT_READY;
                }
                break;
            }
```

- [ ] **Step 2: 填 `INIT_READY` 的运行期底座块**

在 Task 8 会建好的运行期块里，底座这一段是这样（Task 8 会把它和其余部分拼在一起）：

```c
/* ---- base joint: track, then close the loop ---- */
if(ArmBase.state == ARM_BASE_OK || ArmBase.state == ARM_BASE_BLIND)
{
    u8  have_raw = 0;
    s16 raw      = 0;
    float err, v_dps;
    s16 v_cmd;

    if(arm_need_probe(&ArmCal, &ArmBase))
    {
        if(lx_read_pos(6, &raw) == 0) have_raw = 1;
        else raw = 0;
    }

    /* ArmBase.prev_v is the speed this loop commanded LAST tick, already
     * in deg/s (see the note on ARM_BASE). Dead reckoning integrates the
     * command that was actually in force over the elapsed interval, not
     * the one we are about to send - hence the read before the write. */
    v_dps = ArmBase.prev_v;
    arm_track_step(&ArmBase, &ArmCal, have_raw, raw, v_dps, 0.02f);

    err   = arm_field_to_deg(&ArmCal, ArmTargetPos) - ArmBase.deg;
    v_cmd = (ArmHold) ? 0 : arm_base_vel(&ArmCal, err, RosArm.t_ms);

    lx_set_mode(6, LX_MODE_MOTOR, v_cmd);

    /* remember what we just asked for; next tick integrates this */
    ArmBase.prev_v = (float)v_cmd * (float)ArmCal.speed_1000_dps / 1000.0f;

    RosArmFb.pot_raw    = have_raw ? raw : RosArmFb.pot_raw;
    RosArmFb.base_state = ArmBase.state;
}
```

> `ArmTargetPos` 与 `ArmHold` 是 arm_task 的文件级量，Task 8 定义（`ArmTargetPos` 来自 `RosArm.pos[5]`，`ArmHold` 来自急停/失联判定）。
>
> **顺序强制约束**：`arm_track_step()` 必须在 `lx_set_mode()` **之前**调用。反过来的话，积分用的就是这一拍刚发出去的速度，而它实际只生效了一个零头 —— 每次采样都会多算一拍，盲区里会系统性超前。T5 的 `blind integration` 用例正是钉住这个语义的。

- [ ] **Step 3: 跑宿主测试确认没破坏**

Run: `python _ros_tooling/_gen_arm_test.py`
Expected: `ALL PASSED`。

- [ ] **Step 4: 记录结果**

贴输出。**上板验证（Task 10 执行）**：底座上电后会自动慢慢转、找到有效区后停住；`RosArmFb.status` bit1（`ARM_ST_BASE_KNOWN`）置位；`pot_raw` 落在 `[60,940]` 内。

---

### Task 8: `arm_task` 运行期：5 轴下发、回读、健康、故障

**Files:**
- Modify: `BALANCE/arm.c`

**Interfaces:**
- Consumes: Task 3 的 `RosArm` / `RosArmFb`、Task 4 的 `RosRaw`、Task 7 的底座块
- Produces: 完整的 `arm_task`；`ros_send_arm_fb()` 在 20Hz 被调用

- [ ] **Step 1: 定义任务局部量并实现运行期块**

在 `arm_task` 的 `while(1)` 之前加：

```c
    u16 tick       = 0;
    u16 arm_fb_div = 0;
    u8  health_j   = 0;
    u8  probe_j    = 0;
    s16 last_p[ARM_JOINTS];
    u8  first_frame = 1;
    u8  i;

    for(i = 0; i < ARM_JOINTS; i++) last_p[i] = -1;
```

在 `while(1)` 的末尾（`switch` 之后、`}` 之前）替换 Task 5 留的 `/* Task 8 adds the runtime block here */`：

```c
        if(stage != INIT_READY) continue;      /* nothing to do yet */
        tick++;

        /* ------------------------------------------------------------
         * 1. take the newest 0xAC frame
         * ---------------------------------------------------------- */
        {
            u8 fresh = 0;
            if(RosArm.rx_ready)
            {
                taskENTER_CRITICAL();
                for(i = 0; i < ARM_JOINTS; i++) ArmPos[i] = RosArm.pos[i];
                ArmTime = RosArm.t_ms;
                ArmFlags = RosArm.flags;
                RosArm.rx_ready = 0;
                fresh = 1;
                taskEXIT_CRITICAL();
            }
            (void)fresh;
        }

        /* ------------------------------------------------------------
         * 2. TWO separate gates, and they are not the same set
         *
         * ArmHold       = stop moving. Set by estop, link loss, arm not
         *                 enabled, or a base fault.
         * ArmCanRehome  = allowed to run a homing SEARCH. Set by the
         *                 first three only.
         *
         * A base fault must hold the axis but must NOT block the one
         * mechanism that can clear it. Folding ARM_BASE_FAULT into the
         * re-home gate is a deadlock: a faulted base could never be
         * re-homed, so it would stay dead until a power cycle.
         * ---------------------------------------------------------- */
        ArmHold      = 0;
        ArmCanRehome = 1;
        if(RosFault & ROS_FAULT_ESTOP)   { ArmHold = 1; ArmCanRehome = 0; }
        if(RosFault & ROS_FAULT_TIMEOUT) { ArmHold = 1; ArmCanRehome = 0; }
        if(!(RosDown.flags & ROS_FLAG_ENABLE)) { ArmHold = 1; ArmCanRehome = 0; }
        if(ArmBase.state == ARM_BASE_FAULT)    { ArmHold = 1; }

        if(ArmHold) RosArmFb.status |=  ARM_ST_ESTOP;
        else         RosArmFb.status &= (u8)~ARM_ST_ESTOP;

        /* Re-home on request, gated on ArmCanRehome - NOT on ArmHold.
         *
         * EDGE triggered, deliberately. ArmFlags is a frame SNAPSHOT, so
         * a host that holds bit0 set would otherwise re-home again the
         * instant each search finished - an endless homing loop with the
         * base sweeping the whole budget every few seconds. The contract
         * is "request", not "level": one re-home per 0 -> 1 edge. A host
         * that wants another clears the bit and sets it again.
         *
         * A request that arrives while ArmCanRehome is 0 is dropped, not
         * latched: latching it would resurrect the level semantics we are
         * avoiding, and the host can simply re-issue. */
        if((ArmFlags & ROS_ARM_FLAG_REHOME) &&
           !(ArmFlagsPrev & ROS_ARM_FLAG_REHOME) && ArmCanRehome)
        {
            arm_base_init(&ArmBase, arm_base_last_sign());
            RosArmFb.status |= ARM_ST_HOMING;
            RosArmFb.status &= (u8)~ARM_ST_BASE_FAULT;
            stage = INIT_HOME;
            continue;
        }
        ArmFlagsPrev = ArmFlags;    /* consume the edge either way */

        ArmTargetPos = ArmPos[ARM_BASE_JOINT];

        /* ------------------------------------------------------------
         * 3a. PRESET the five position joints - only when a target
         *     actually moved, which is what keeps the bus free in
         *     point-to-point use. The trigger is deliberately NOT here:
         *     see 3b.
         * ---------------------------------------------------------- */
        {
            u8 changed = first_frame;
            for(i = 0; i < (ARM_JOINTS - 1); i++)
                if(ArmPos[i] != last_p[i]) changed = 1;

            ArmChanged = changed;

            if(changed && !ArmHold)
            {
                for(i = 0; i < (ARM_JOINTS - 1); i++)
                {
                    u16 p = (u16)ArmPos[i];
                    if(p > 1000) p = 1000;
                    if(i == ARM_GRIPPER_FIELD && p > ARM_GRIPPER_MAX)
                        p = ARM_GRIPPER_MAX;          /* stall protection */
                    lx_move_wait(ARM_JOINT_ID[i], p,
                                 ArmTime ? ArmTime : 500);
                }
            }
        }

        /* ------------------------------------------------------------
         * 3b. THE BASE SPEED GOES EXACTLY HERE - between the preset and
         *     the trigger. 协议规范 §7.9: 预置 5 轴 → 发底座速度 →
         *     广播触发.
         *
         * A motor-mode servo starts the instant its speed frame lands,
         * while the other five wait for MOVE_START. Sending the base
         * speed after the broadcast staggers the six axes by one mode
         * frame plus the broadcast - about 1.4 ms at 115200, visible on
         * a 450 deg slew.
         *
         * Task 7's ARM_BASE_TICK_BEGIN/END block already sits here and
         * must stay exactly here. It is a 50 Hz velocity loop: it runs
         * EVERY tick, and must not be moved inside the `changed` gate.
         * ---------------------------------------------------------- */
        /* ---- base joint: Task 7's ARM_BASE_TICK block ---- */

        /* ------------------------------------------------------------
         * 3c. TRIGGER the five position joints.
         * ---------------------------------------------------------- */
        if(ArmChanged && !ArmHold)
        {
            lx_move_start_all();
            for(i = 0; i < (ARM_JOINTS - 1); i++) last_p[i] = ArmPos[i];
            first_frame = 0;
        }
        else if(ArmHold)
        {
            for(i = 0; i < (ARM_JOINTS - 1); i++)
                lx_move_stop(ARM_JOINT_ID[i]);
            /* MOVE_STOP does nothing to a motor-mode servo - the base
             * needs an explicit speed 0, which 3b sends */
        }

        /* ------------------------------------------------------------
         * 4. readback: two position joints per tick, round robin
         * ---------------------------------------------------------- */
        {
            u8 k;
            for(k = 0; k < 2; k++)
            {
                u8 idx = probe_j;
                s16 p  = 0;
                if(idx < (ARM_JOINTS - 1) && lx_read_pos(ARM_JOINT_ID[idx], &p) == 0)
                    RosArmFb.pos[idx] = p;
                else if(idx < (ARM_JOINTS - 1))
                    RosArmFb.pos[idx] = 0;      /* 0 = "no reading" */
                probe_j = (u8)((probe_j + 1) % (ARM_JOINTS - 1));
            }
        }
        RosArmFb.pos[ARM_BASE_JOINT] = arm_deg_to_field(&ArmCal, ArmBase.deg);

        /* ------------------------------------------------------------
         * 5. health: one servo per second, round robin
         * ---------------------------------------------------------- */
        if((tick % ARM_RATE_HZ) == 0)
        {
            u16 mv = 0;
            u8  tc_ = 0;
            if(lx_read_vin(ARM_JOINT_ID[health_j], &mv) == 0)
            {
                if(mv < ARM_VIN_MIN_MV) RosArmFb.status |=  ARM_ST_UNDERVOLT;
                else                    RosArmFb.status &= (u8)~ARM_ST_UNDERVOLT;
            }
            if(lx_read_temp(ARM_JOINT_ID[health_j], &tc_) == 0)
            {
                if(tc_ > ARM_TEMP_MAX_C) RosArmFb.status |=  ARM_ST_OVERTEMP;
                else                     RosArmFb.status &= (u8)~ARM_ST_OVERTEMP;
            }
            health_j = (u8)((health_j + 1) % ARM_JOINTS);
        }

        /* ------------------------------------------------------------
         * 6. bring-up passthrough (Task 4 block goes here)
         * ---------------------------------------------------------- */

        /* ------------------------------------------------------------
         * 7. the 0x56 uplink, 20Hz
         * ---------------------------------------------------------- */
        if(++arm_fb_div >= (ARM_RATE_HZ / 20))
        {
            arm_fb_div = 0;
            ros_send_arm_fb();
        }
```

- [ ] **Step 2: 在 `arm.c` 顶部定义这些局部量**

```c
/* Latest command taken from the 0xAC frame, and the derived hold flag.
 * File scope rather than locals because the frame is published by an ISR
 * and copied under a critical section. */
static s16 ArmPos[ARM_JOINTS];
static u16 ArmTime;
static u8  ArmFlags;
static u8  ArmFlagsPrev;   /* previous frame's flags, so the re-home bit is
                            * read as an EDGE - see the note in the loop */
static s16 ArmTargetPos;
static u8  ArmHold;        /* stop moving      - see the note in the loop */
static u8  ArmCanRehome;   /* may run a SEARCH - deliberately wider than
                            * ArmHold, so a base fault can still be
                            * cleared by a re-home request */
static u8  ArmChanged;     /* the 5-axis preset ran this tick, so 3c must
                            * trigger; set in 3a, consumed in 3c, because
                            * the base block sits between them */

/* The sign the base had when the pot was last trusted. It is a HINT for
 * the wrapping-pot search and nothing else - with a truncating pot the
 * rail names the side outright, so this value is never consulted (see
 * arm_search_dir_pick).
 *
 * Kept in RAM, and updated on every resync below. That is enough while
 * ARM_POT_WRAPS is 0. If M1 flips the pot to wrapping, this must ALSO be
 * persisted across a power cut (a spare RTC backup register is the
 * obvious home) - otherwise a boot after a power loss hands the search a
 * fabricated sign. Leaving it unassigned entirely was worse still: it
 * was a constant 1 that looked like a real hint. */
static s8 arm_base_last_sign = 1;
```

并在 `arm.c` 里加：

```c
s8 arm_base_last_sign(void) { return arm_base_last_sign; }
```

（在 `arm.h` 里声明。）

**并在底座块里每次重同步后更新它**（紧接 `arm_track_step()` 之后、`lx_set_mode()` 之前）：

```c
            /* keep the hint honest: refreshed only where the reading is
             * actually trusted, which is the only place its sign means
             * anything */
            if(ArmBase.state == ARM_BASE_OK && have_raw)
                arm_base_last_sign = (s8)((ArmBase.deg >= 0.0f) ? 1 : -1);
```

- [ ] **Step 2b: 补上 `ARM_CAL_MEASURED` 闸门（改的是 Task 7 已完成的 `INIT_HOME`）**

> `arm_cal.h` 的注释一直写着"`ARM_CAL_MEASURED` 为 0 时 `arm_task` 拒绝闭合底座环"，但**全工程没有任何地方读它** —— 也就是说这句承诺是空的，固件今天就会拿未标定的占位常数去闭环，还会对外声称角度"已知"。把它变成真的。

在 `BALANCE/arm.c` 的 `case INIT_HOME:` **最开头**插入：

```c
            case INIT_HOME:
                /* The calibration is unmeasured until Task 10 replaces the
                 * placeholders in arm_cal.h. Closing the loop on them would
                 * produce a confident, wrong angle - the arm would go where
                 * it thinks it is. Hold the base at zero speed and say so
                 * instead. Task 10 flips the macro after M1..M8. */
                if(!ARM_CAL_MEASURED)
                {
                    lx_set_mode(6, LX_MODE_MOTOR, 0);
                    ArmBase.state = ARM_BASE_UNKNOWN;
                    RosArmFb.status &= (u8)~ARM_ST_HOMING;
                    RosArmFb.status |=  ARM_ST_BASE_FAULT;
                    stage = INIT_READY;
                    break;
                }
```

**注意**：这意味着在 Task 10 把 `ARM_CAL_MEASURED` 置 1 之前，**底座不会寻零、也不会闭环**；5 个位置关节照常工作。Task 9 之后的上板调试如果只想验证 5 轴与协议链路，这是期望行为；要试底座，先把宏置 1。

- [ ] **Step 2c: 确认 3a/3b/3c 的相对顺序已落实**

`arm.c` 里这三个小段必须是 §7.9 的顺序：**预置 5 轴（3a）→ 底座速度（3b，Task 7 的 `ARM_BASE_TICK` 块）→ 广播触发（3c）**。
写完后用与生成器同样的手法验证（剥注释后比 `index()`）：

```python
_bt = re.sub(r'/\*.*?\*/', '', arm_c, flags=re.S)
assert _bt.index('arm_need_probe') < _bt.index('lx_move_start_all'), \
    'the base speed is sent after the broadcast: the six axes start a frame apart'
```

- [ ] **Step 3: 跑宿主测试**

Run: `python _ros_tooling/_gen_arm_test.py`
Expected: `ALL PASSED`（`arm_task` 不在 `ARM_LOGIC` 段内，所以宿主构建里不会出现 FreeRTOS 符号）。

- [ ] **Step 4: 记录结果**

贴输出。**上板验证**：5 轴能跟着 `0xAC` 走、`0x56` 持续 20Hz 上报、`RosArm.frames` 增长。

---

### Task 9: 工程接入（字节级修改 + Keil 工程）

**Files:**
- Modify: `BALANCE/arm.c`（**补丁**：REHOME 边沿触发；见 Step 1a）
- Modify: `BALANCE/system.h`（加 `#include "arm.h"`）
- Modify: `USER/main.c`（加 `xTaskCreate`，**不 decode**）
- Modify: `USER/WHEELTEC.uvprojx`（加两个文件条目）
- Create: `_ros_tooling/_apply_arm_task9.py`

**Interfaces:**
- Consumes: Task 5 的 `arm_task` / `ARM_TASK_PRIO` / `ARM_STK_SIZE`
- Produces: 可编译的 Keil 工程

- [ ] **Step 1a: 补丁 —— 把 REHOME 改成边沿触发（改的是 Task 8 已完成的代码）**

> **为什么必须补**：`ArmFlags` 是 0xAC 帧的**快照**，主机若把 bit0 常置（很容易写错），底座会在每次寻零刚结束时立刻再寻一次 —— **无限寻零循环**，每几秒扫一遍整个搜索预算。今天被 `ARM_CAL_MEASURED = 0` 的闸门遮住，Task 10 一置 1 就会暴露。
> 契约是"**请求**"而不是"电平"：一次 0→1 边沿触发一次寻零。这不改主机契约（主机置一帧或常置都只寻一次），只是把语义写对。

在 `BALANCE/arm.c` 里做两处改动：

1. 文件级声明加 `static u8 ArmFlagsPrev;`（就放在 `ArmFlags` 旁边）。
2. 把 REHOME 判定改成边沿触发，并在 `if` 块之后无条件消费边沿：

```c
        /* EDGE triggered, deliberately. ArmFlags is a frame SNAPSHOT, so a
         * host that holds bit0 set would otherwise re-home again the
         * instant each search finished - an endless homing loop with the
         * base sweeping the whole budget every few seconds.
         *
         * ⚠️ THE EDGE IS CAPTURED AND CONSUMED BEFORE THE `if`, NOT AFTER.
         * The body ends in `continue`, which jumps to the top of while(1)
         * and SKIPS anything written after the block - so consuming the
         * edge there consumes nothing, the same 0->1 re-fires on the next
         * tick, and the loop the previous paragraph exists to prevent
         * survives intact. Measured: consuming after the `if` gives 91
         * searches per 1000 ticks, before it gives 1. A pulse of one
         * frame is enough to trigger it, so this is not hypothetical. */
        {
            u8 rehome = (u8)((ArmFlags & ROS_ARM_FLAG_REHOME) &&
                             !(ArmFlagsPrev & ROS_ARM_FLAG_REHOME));
            ArmFlagsPrev = ArmFlags;    /* consume the edge either way */

            if(rehome && ArmCanRehome)
            {
                arm_base_init(&ArmBase, arm_base_last_sign());
                RosArmFb.status |= ARM_ST_HOMING;
                RosArmFb.status &= (u8)~ARM_ST_BASE_FAULT;
                stage = INIT_HOME;
                continue;
            }
        }
```

并把这一语义写进 `HARDWARE/usartx.h` 的 `0xAC` 帧说明里：

```c
 *  16  flags   uint8  bit0 = re-home the base (EDGE: one search per
 *                           0->1 transition, not per frame)
```

`usartx.h` 是 GBK + CRLF，**必须走字节级脚本**（`assert count == 1`、`\r\n` 锚点、写入后断言无裸 LF）。

**上板验证**：把 bit0 常置发 0xAC，确认底座**只寻零一次**而不是反复寻。

- [ ] **Step 1: 写 `_apply_arm_task9.py`**

```python
# -*- coding: utf-8 -*-
"""Task 9: wire arm.c / lx_servo.c into the project.

system.h is GBK + CRLF. main.c is CRLF and NOT valid GBK (byte 439 breaks
it) - so this script never decodes anything, it only matches bytes.

WHEELTEC.uvprojx is ASCII with bare LF (CRLF count is 0), so its anchors
use \n, NOT \r\n. Getting that wrong silently finds nothing.
"""
CRLF = b'\r\n'
def enc(s): return s.replace('\n', '\r\n').encode('ascii')

# ---- 1. system.h: make arm.h visible to everything -------------------
p = 'BALANCE/system.h'
b = open(p, 'rb').read()
old = b'#include "imu_task.h"' + CRLF
assert b.count(old) == 1, 'system.h include anchor = %d' % b.count(old)
b = b.replace(old, old + b'#include "arm.h"' + CRLF, 1)
open(p, 'wb').write(b)
print('ok  system.h includes arm.h')

# ---- 2. main.c: start the task --------------------------------------
p = 'USER/main.c'
b = open(p, 'rb').read()
old = b'    vTaskDelete(StartTask_Handler);'
assert b.count(old) == 1, 'main.c anchor = %d' % b.count(old)
new = (b'    xTaskCreate(arm_task,       "arm_task",       ARM_STK_SIZE,      NULL, ARM_TASK_PRIO,       NULL);\r\n'
       b'    vTaskDelete(StartTask_Handler);')
b = b.replace(old, new, 1)
open(p, 'wb').write(b)
print('ok  main.c starts arm_task')

# ---- 3. uvprojx: Keil will not compile them otherwise ----------------
p = 'USER/WHEELTEC.uvprojx'
b = open(p, 'rb').read()          # ASCII, bare LF

def entry(fname, path):
    return ('            <File>\n'
            '              <FileName>%s</FileName>\n'
            '              <FileType>1</FileType>\n'
            '              <FilePath>%s</FilePath>\n'
            '            </File>\n' % (fname, path)).encode('ascii')

old = entry('usartx.c', '..\\HARDWARE\\usartx.c')
assert b.count(old) == 1, 'HARDWARE anchor = %d' % b.count(old)
b = b.replace(old, old + entry('lx_servo.c', '..\\HARDWARE\\lx_servo.c'), 1)

old = entry('imu_task.c', '..\\BALANCE\\imu_task.c')
assert b.count(old) == 1, 'Balance anchor = %d' % b.count(old)
b = b.replace(old, old + entry('arm.c', '..\\BALANCE\\arm.c'), 1)

open(p, 'wb').write(b)
print('ok  uvprojx has lx_servo.c and arm.c')
```

- [ ] **Step 2: 运行接入脚本**

Run: `python _ros_tooling/_apply_arm_task9.py`
Expected: 四行 `ok  ...`，无 assert 失败。

- [ ] **Step 3: 校验改动确实落盘且没破坏别的**

Run: `python -c "
b=open('USER/WHEELTEC.uvprojx','rb').read()
print('lx_servo.c entries:', b.count(b'lx_servo.c'))
print('arm.c entries     :', b.count(b'arm.c'))
c=open('USER/main.c','rb').read()
print('arm_task lines    :', c.count(b'xTaskCreate(arm_task'))
h=open('BALANCE/system.h','rb').read()
print('arm.h include     :', h.count(b'#include \"arm.h\"'))
"`
Expected: `1`、`1`（`arm.c` 只出现一次，不要被 `lcd_arm.c` 之类误匹配）、`1`、`1`。

- [ ] **Step 4: 用 Keil 编译**

在 Keil 里打开 `USER/WHEELTEC.uvprojx`，Rebuild。**必须零 error、零 warning。**

> 这一步是**上板验证**：宿主机 gcc 测不到 ARM 工具链特有的问题（内联汇编、`__IO` 限定、标准外设库头文件）。
> 若报 `undefined symbol`，检查 Step 3 的两个条目是否真的加进了正确的 `<Group>`。

- [ ] **Step 5: 记录结果**

贴 Keil 的 Build Output 末尾几行到「执行记录」。

---

### Task 10: 上板 bring-up（M1–M8 实测）与标定回填

**Files:**
- Modify: `BALANCE/arm_cal.h`（把 M1–M8 的实测值填进去，`ARM_CAL_MEASURED` 置 1）
- Create: `docs/plans/2026-09-26-arm-bringup-record.md`（实测记录）

**Interfaces:**
- Consumes: Task 4 的 `_arm_probe.py`、Task 1–9 的全部固件
- Produces: 一份填好的 `arm_cal.h`，以及可以解释每一次修改的实测记录

> **这是整个计划里唯一一个不能跳过的任务。** `arm_cal.h` 里现在每一个数字都是待验证的初值。其中 `ARM_POT_WRAPS` 直接决定 Task 6 的寻零走哪个分支 —— 它错了，底座会在盲区里永远找不到窗口然后报 `ARM_BASE_FAULT`。

- [ ] **Step 1: 编一个带透传的 bring-up 固件**

把 `HARDWARE/usartx.h` 里 `#define ARM_DEBUG_PASSTHROUGH 0` 改成 `1`，Keil 重新编译烧录。

- [ ] **Step 2: M6 — 确认 6 个舵机的实际 ID**

在树莓派上：
```bash
python3 _arm_probe.py -p /dev/ttyAMA0 id
```
Expected: 6 行 `servo N -> ID N`。若有 `NO REPLY` 或 ID 与序号不符，**停下来** —— 后面的关节表全部要按实际 ID 改。

- [ ] **Step 3: M2 — 找出电位器有效区与机械角的对应**

先把底座**手动**转到大致中位，然后：
```bash
python3 _arm_probe.py -p /dev/ttyAMA0 pot 6 20
```
在读数过程中用手机水平仪/量角器把底座从一端缓慢转到另一端，记录：
- `raw` 开始**线性变化**的角度区间 → `ARM_POT_LO` / `ARM_POT_HI`
- `raw = 500` 时底座相对车身正前方的角度 → `ARM_POT_DEG_AT_RAW0`

填回 `arm_cal.h`。

- [ ] **Step 4: M1 — 判定盲区是截断还是回绕**

```bash
python3 _arm_probe.py -p /dev/ttyAMA0 mode 6 1 200     # 电机模式，慢速 +200
python3 _arm_probe.py -p /dev/ttyAMA0 pot 6 60         # 连续转一分钟，每 50ms 读一次
```
看 Step 3 之前的脚本输出末尾的判定行：
- `no wrap seen -> likely TRUNCATING` → `ARM_POT_WRAPS 0`
- `N wraps -> WRAPPING` → `ARM_POT_WRAPS 1`，**并且**要确认跳变间隔是否稳定地等于 240°（不稳定就不能用回绕方案，必须回头加 AS5600）

结束时记得 `mode 6 1 0` 停转。

- [ ] **Step 5: M5 — 速度 0 是否锁轴**

```
python3 _arm_probe.py -p /dev/ttyAMA0 mode 6 1 0
```
手动轻推底座：不动 = 锁轴（放心）；会溜 = 需要机械阻尼，并且把 `ARM_VEL_KP` 调小（否则积分会来回过冲）。

- [ ] **Step 6: M3 — 速度标定**

把底座放在有效区中间，然后：
```bash
python3 _arm_probe.py -p /dev/ttyAMA0 spin 6 300 5
```
读输出的 `K = ... deg/s per speed unit`。`ARM_SPEED_1000_DPS = K * 1000`。
**在满电和半电各测一次** —— 两者差多少，就说明盲区积分的误差量级有多大。把两次结果都记进记录文档。

- [ ] **Step 7: M4 — 加速段补偿角**

同一速度下，从静止起转 vs 已经匀速，用 `pot` 子命令记录"从发指令到读数开始匀速变化"这段的总转角，即 `ARM_ACCEL_COMP_DEG`（预期 1–5°）。

- [ ] **Step 8: M7 — 确认 5 个位置关节的机械行程**

逐个把 ID1..5 用 `55 55 0X 07 01 <pos> 00 00 <t>` 慢慢推到 125 和 875，确认不撞机械限位。若某个关节行程更窄，把它的上限记进记录文档，并在 Task 8 的下发处按关节裁剪。

- [ ] **Step 9: M8 — 帧间间隔**

把 `lx_servo.c` 的 `LX_GAP_US` 从 500 逐步降到 100，每次跑一次 6 轴联动。出现丢帧/不动作就回退到上一档，记录最小可用值。

- [ ] **Step 10: 把 `ARM_CAL_MEASURED` 置 1，重编译烧录**

改 `arm_cal.h` 里 `#define ARM_CAL_MEASURED 1`。

- [ ] **Step 11: 端到端验收**

1. 上电：5 关节走到复位姿态；底座自动寻零并在找到有效区后停住
2. 树莓派发一个 `0xAC`：底座从 −225 一路转到 +225，观察 `0x56` 上报的 `base_state` 依次是 `OK → BLIND → OK`
3. 在盲区里停住不动 30 秒，确认 `base_state` 保持 `BLIND` 且没有漂移报警
4. 断开树莓派链路，确认底座在 200ms 内停住、`status` 的 bit7 置位，且机械臂**没有垮下来**（保持上力）
5. 重新接上链路，确认它**不会自动续动**，要等新的 `0xAC`

- [ ] **Step 12: 记录**

把所有实测值、判定结论、异常现象写进 `docs/plans/2026-09-26-arm-bringup-record.md`，并在 `arm_cal.h` 里为每个值标注它来自哪一步。

---

## 执行记录

> 执行者每完成一个 Task 就把运行输出贴到这里。**本工程不是 git 仓库**（实测 `git rev-parse --show-toplevel` → `fatal: not a git repository`），所以没有 commit 步骤，运行输出就是唯一的变更凭证。

### Task 1

```
$ python _ros_tooling/_gen_arm_test.py
gcc -std=c99 -O2 -Wall -Wextra -o _ros_tooling/_arm_host_test.exe _ros_tooling/_arm_host_test.c

ok   MOVE_TIME_WRITE id1 500/1000ms     55 55 01 07 01 F4 01 E8 03 16
ok   MOVE_TIME_WAIT_WRITE id1           55 55 01 07 07 F4 01 E8 03 10
ok   broadcast MOVE_START               55 55 FE 03 0B F3
ok   POS_READ id1                       55 55 01 03 1C DF
ok   TEMP_READ id1                      55 55 01 03 1A E1
ok   VIN_READ id1                       55 55 01 03 1B E0
ok   ID_READ id1                        55 55 01 03 0E ED
ok   MOVE_STOP id1                      55 55 01 03 0C EF
ok   LOAD id1                           55 55 01 04 1F 01 DA
ok   UNLOAD id1                         55 55 01 04 1F 00 DB
ok   motor mode id6 speed 0             55 55 06 07 1D 01 00 00 00 D4
ok   motor mode id6 speed +300          55 55 06 07 1D 01 00 2C 01 A7
ok   motor mode id6 speed -300          55 55 06 07 1D 01 00 D4 FE 02
ok   position mode id6                  55 55 06 07 1D 00 00 00 00 D5
ok   pos reply accepted
ok   pos decode 524
ok   negative pos decode -200
ok   bad header -> 1
ok   wrong id -> 2
ok   wrong length -> 3
ok   bad checksum -> 4
ok   LX_BROADCAST accepts any id
ok   no-param Length = 3, frame = 6
ok   all 254 ids round trip
ok   checksum wraps mod 256

ALL PASSED (0 failures)
exit=0
```

gcc stderr 0 字节（零警告）。

**做法**：`lx_build()` 首次运行即与协议文档 §5.5 的权威报文逐字节吻合，未发现逻辑错误。`_gen_arm_test.py` 从 `HARDWARE/lx_servo.c` 的
`/* ==== LX_LOGIC_BEGIN ==== */` … `/* ==== LX_LOGIC_END ==== */` 之间**逐字切片**（latin-1 读入，1783 字节）。
已独立核验：该切片在生成的 `_arm_host_test.c` 中逐字出现恰好 1 次 —— 是真实固件源码，不是手抄副本。

**偏离计划一处**：`test_checksum_edges()` 的 wrap 用例里 `n = lx_build(...)` 未使用，触发
`-Wunused-but-set-variable`。因零警告是硬约束且禁止 `-w`，改为 `if(n != 10)` 断言帧总长
（4 参数帧 Length=7、总长 10，正确），原校验和检查原样保留。**这是加强而非削弱**：多钉了一条帧长断言。
校验和的循环上界仍硬编码 `5 + 4`，刻意不从 `n` 推导，避免校验和检查依赖于正在被测的那个长度。

**新建文件**（全部纯 ASCII + 纯 LF，已核验 `CRLF=0, nonascii=0`）：
`HARDWARE/lx_servo.h`（94 行）、`HARDWARE/lx_servo.c`（72 行，本步只含 LX_LOGIC 段）、
`_ros_tooling/_arm_tests.c`（226 行）、`_ros_tooling/_gen_arm_test.py`（107 行）。
**未触碰任何既有文件**（mtime 核验：`usartx.*` 09-16/09-17、`system.*`/`main.c` 09-18、`uvprojx` 09-18，均早于今日）。

**遗留清理（审查时发现）**：`lx_servo.h` 原声明的 `lx_init(void *unused)` 全计划从未定义、从未调用，
已删除该声明并在原处留注 —— 避免留一个永远不存在的 API 给后续 Task 误用。

### Task 2

已改：`HARDWARE/usartx.c`（GBK + CRLF，字节级脚本 `_ros_tooling/_apply_arm_task2.py`）、`HARDWARE/lx_servo.c|h`（ASCII + LF，Edit）。

**Step 5 复跑宿主测试** `python _ros_tooling/_gen_arm_test.py`（gcc 命令行里的盘符路径被控制台编码打乱，实际就是 `_ros_tooling/_arm_host_test.exe`）：

```
ok   MOVE_TIME_WRITE id1 500/1000ms     55 55 01 07 01 F4 01 E8 03 16
ok   MOVE_TIME_WAIT_WRITE id1           55 55 01 07 07 F4 01 E8 03 10
ok   broadcast MOVE_START               55 55 FE 03 0B F3
ok   POS_READ id1                       55 55 01 03 1C DF
ok   TEMP_READ id1                      55 55 01 03 1A E1
ok   VIN_READ id1                       55 55 01 03 1B E0
ok   ID_READ id1                        55 55 01 03 0E ED
ok   MOVE_STOP id1                      55 55 01 03 0C EF
ok   LOAD id1                           55 55 01 04 1F 01 DA
ok   UNLOAD id1                         55 55 01 04 1F 00 DB
ok   motor mode id6 speed 0             55 55 06 07 1D 01 00 00 00 D4
ok   motor mode id6 speed +300          55 55 06 07 1D 01 00 2C 01 A7
ok   motor mode id6 speed -300          55 55 06 07 1D 01 00 D4 FE 02
ok   position mode id6                  55 55 06 07 1D 00 00 00 00 D5
ok   pos reply accepted
ok   pos decode 524
ok   negative pos decode -200
ok   bad header -> 1
ok   wrong id -> 2
ok   wrong length -> 3
ok   bad checksum -> 4
ok   LX_BROADCAST accepts any id
ok   no-param Length = 3, frame = 6
ok   all 254 ids round trip
ok   checksum wraps mod 256
ok   rx: id6 reply published, 8 bytes, pos 524
ok   rx: id1 reply accepted (ID is not the Length)
ok   rx: 7 byte single-param reply
ok   rx: two frames back to back
ok   rx: resync after leading noise
ok   rx: 55 55 55 resync
ok   rx: Length above LX_MAX_LEN (9) rejected

ALL PASSED (0 failures)
exit=0
```

零 gcc 警告（32 条 `ok`，0 条 `FAIL`）。另外用 Keil 那套 include 路径对 `lx_servo.c` 做了独立语法检查（`gcc -fsyntax-only -Wall -Wextra` + 工程宏），警告全部来自厂商头文件（`core_cm4.h` / `arm_math.h` 在 64 位宿主机上），`lx_servo.c|h` 自身零警告。

**NVIC 改动校验**（`usartx.c` 逐字节 diff；脚本自报 + 事后独立复核脚本）：

- (a) `USART2_IRQn` 之后 200 字节窗口内 `NVIC_IRQChannelPreemptionPriority=5 ;`（原为 1）
- (b) `USART1_IRQn` 那 220 字节上下文与改动前**逐字节相同**，preempt 仍为 5；`bytes[0:17908]` 只有 1 个字节变化
- (c) 改动恰好两处：第 3999 字节 `0x31→0x35`（就是那个 `1`→`5`），以及 ISR 区段 `[17908,21231)` → `[17908,22537)`（3323 → 4629 字节）；区段之后剩余的 5757 字节与原文件逐字节相同。整文件同偏移 naive diff = 8763（= 1 + ISR 区段长度变化导致偏移后移的 8762）
- 文件仍是 GBK + CRLF：CRLF = LF = 857，无裸 LF

**与计划的偏差**（每条都写进了代码注释）：

1. **计划 Step 2 的 `lx_rx_byte()` 状态机是坏的**，已改写。两个独立缺陷：①外层 `if(s_rx_want == 0)` 守卫生效期间分支里又把它设回 0，于是状态机永远停在"找 55 55"，永远进不到收帧分支；②它把 `55 55` 之后的字节当 **Length**，但协议是 `55 55 ID Length Cmd .. CS` —— 那一位是 **ID**（`lx_servo.h` 自己就写着 `LX_RESP_POS 8 /* 55 55 ID 05 1C posL posH CS */`）。实测（把计划原文抄进临时宿主程序喂真实应答）：id1 和 id6 的 POS_READ 应答 `frames` 都是 0，读回整条链路是死代码。现按真实帧格式实现为 ID → Length → 载荷，并新增 7 条断言钉住（含 id1 这种 ID 字节小于最小 Length 的、以及 id6 那种长度刚好少一个字节就悬住的）。
2. 计划把 Length 上界写成 `c > 12`；但 12 字节邮箱装的是 `Length + 3`，上界只能是 9，否则 Length=10 会越界写 `s_rx_buf[12..]` 和 `LxRx.buf[12..]`。已引入 `LX_MAX_LEN`（9），定义放在 `lx_servo.h` 的 Length 常量旁边（宿主测试要能看见它）。
3. `lx_wait_reply()` 计划里一遇长度不符就 `return 2` 让整次读失败；接收是纯字节级猜头，误报文常见，改成丢弃长度不符的报文继续等（超时仍然兜底）。这正是计划自己那句注释 "wait for the ISR to publish a reply of the expected total length" 的本意。
4. `lx_servo.c` 补了 `#include "delay.h"`：胶水用 `delay_us()` / `getSysTickCnt()`，而 `lx_servo.h` 只带 `sys.h`（不含 delay.h），不补的话 Keil 编译期隐式声明。
5. 位置双向裁剪：`lx_move_time` / `lx_move_wait` 统一走 `lx_clamp_pos()` —— 参数是 u16，先按 s16 重新解释，负数 → 0，>1000 → 1000。**`lx_read_pos` 的输出故意不裁剪**：它是读回值，底座在电机模式下原始电位器读数本来就可能落在 0..1000 之外，裁掉会毁掉 Task 7/10 的诊断依据。
6. `USART2_IRQHandler` 的旧函数体**逐字节原样保留**在替换区段内的 `#if 0` 块里（本任务要求"只关入口不删代码"）：`AT_Command_Capture` / `Turn_Flag` / `Flag_Direction` / `Flag_PID` / `RC_Velocity` 在文件里的出现次数与改动前完全一致，只是不再参与编译、不再挂在 ISR 上。
7. 宿主测试新增 T3（`_arm_tests.c` + `_gen_arm_test.py` 各加一段，切的是 `lx_servo.c` 里的真实 `lx_rx_byte()`）：被 ISR 喂的那台状态机本来没有任何测试覆盖，而它恰恰是本任务新增的东西。

**上板验证（Task 10，宿主测不了）**：`USART2_IRQHandler` 真的被调用、`LxRx.frames` 会增长。M-probe 第一条就是发 `55 55 01 03 0E ED` 读 ID1。

### Task 3

命令：`python _ros_tooling/_gen_arm_test.py`（先跑 `python _ros_tooling/_apply_arm_task3.py` 做字节级接入）

```
gcc -std=c99 -O2 -Wall -Wextra -o _ros_tooling/_arm_host_test.exe _ros_tooling/_arm_host_test.c
gcc: 0 warnings
ok   rx: 7 byte single-param reply
ok   rx: two frames back to back
ok   rx: resync after leading noise
ok   rx: 55 55 55 resync
ok   rx: Length above LX_MAX_LEN (9) rejected
ok   crc16_modbus("123456789") == 0x4B37
ok   arm frame decodes
ok   base field is signed (-1000)
ok   bad CRC rejected
ok   resync after garbage
ok   recovers after a truncated frame (loses 1, not stuck)
ok   resync adopts a header in the last byte
ok   base field round trips -1000..1000
ok   frame layout invariants (0xAC and 0x56)

ALL PASSED (0 failures)
exit=0
```

**字节级 diff（备份 `_ros_tooling/_backup_arm_task3/`，改动前 sha256）**

| 文件 | 字节 | 变化 | opcode | 非插入编辑 | 删掉插入段能否还原备份 |
|---|---|---|---|---|---|
| `HARDWARE/usartx.c` | 28294 → 32038 | +3744 (13.23%) | equal×4, insert×3 | **0** | **True**（还原后 sha256 与备份逐字节相同） |
| `HARDWARE/usartx.h` | 8890 → 12661 | +3771 (42.42%) | equal×2, insert×1 | **0** | **True** |

- `usartx.h` 备份 sha256 `7d82f9a1…f4ede871` → 现 `ca66c45a…0c6574a0`；公共头 7257 B + 公共尾 1633 B = 8890 B = 原长 → 是**一整段连续插入**，其余区域逐字节未动。
- `usartx.c` 备份 sha256 `5b2c7faafb…4a86f99f62` → 现 `c5502f6c…25b2f0c3`；三段插入分别为 `ros_arm_byte`+切片标记（106 行，位于 ISR 之前）、ISR 里的一行调用、`ros_send_arm_fb()`（34 行）。公共头 13517 B + 公共尾 10662 B 小于原长是**三处插入**所致；逐段判定见 opcode 列与"还原"列。
- 两个文件行尾全部是 CRLF（`usartx.c` 857→998、`usartx.h` 210→303），孤立 LF = 0，未引入任何 LF 行。

**与计划的偏差（3 处，均为有意）**

1. **计划 Step 5 的标记锚点是坏的**：它按"多缩进一层"写的 `else / { / ros_arm_reset(); …` 与实际插入代码的缩进差一个 tab，`count == 1` 断言必然失败。改为**把 `/* ==== ARM_TASK_BEGIN ==== */` 直接拼在 Step 2 那段解码器文本的末尾**一次写入 —— 标记一定落在 `ros_arm_byte()` 之后，且不再依赖一个易碎的缩进锚点。
2. **计划 Step 5 给出的截断帧断言与它自己的解码器矛盾**。实测（把计划里那段状态机逐字跑一遍）：先喂 9 字节残帧、再喂一整帧时，CRC 失败后的重扫虽然能在 `buf[9]` 找到新帧头，但压缩后 `s_arm_state = 1`（"还在等长度字节"），于是已经缓存的那个长度字节被丢掉，**这一帧必丢**；下一帧才恢复正常（实测 `frames=0`，而计划断言 `frames==1`）。这是 `ros_rx_byte()` / `ros_sv_byte()` 一直以来的形状，计划自己也写明"三个解码器同形，读懂一个就懂三个"，所以**没有改解码器**，而是把用例改成断言真实语义：损坏输入必须被计数（`crc_err + head_err != 0`）、下一整帧必须正确发布（`frames==1` 且 `pos/t_ms/seq` 全等于第三帧）、且**最多丢一帧、绝不错发**。代价是 CRC 错后一帧的延迟（20–50 Hz 流下 20–50 ms）。
3. **`usartx.h` 的插入点从 `#define ROS_SERVO_NUM        6` 挪到 0xAB 段之后**（`void ros_apply_servo(void);` 那行之后）。计划那个锚点会把 0xAB 的 `#define` 列表从中间劈开，读起来像 `ROS_SERVO_NUM` 属于机械臂块。C 语义无差别，Task 4 用的锚点（`#define ARM_ST_ESTOP`）也不受影响。

**宿主测试补强（计划外，各加一条）**

- 生成器新增 **警告硬失败**：`gcc -Wall -Wextra` 输出里出现 `warning:` 即 `exit(1)`，不再依赖人眼（本次 `gcc: 0 warnings`）。
- 用例新增一条"**重扫必须覆盖失败窗口的最后一个字节**"：把一帧的 CRC 打坏、再把 `0xAC` 放进它的第 19 字节，然后只发一整帧的 `[1..19]` 字节，解码器必须认领末尾那个 `0xAC` 并发布该帧。这条钉住了重扫的上界（`k < ROS_ARM_SIZE`），否则 `k < ROS_ARM_SIZE - 3 / -5` 这类改动无人能发现。

**变异测试（证明用例真的在跑出货代码，而不是空转）**：14 个变异，13 个被抓（运行时断言或编译失败），唯一幸存的是把重扫上界从 `k < ROS_ARM_SIZE` 改成 `k < ROS_ARM_SIZE - 1` —— 循环自然退出时 `k` 恰好等于 `ROS_ARM_SIZE - 1`，后面 `if(k < ROS_ARM_SIZE)` 仍成立并做**同样的 1 字节压缩**，属等价变异；两者唯一的差别是"最后一字节不是帧头时是丢弃还是留 1 字节"，而留下的那一字节不可能发出去（CRC 是闸门），也不影响下一帧重同步。

顺带记录一个**方向性**的覆盖事实：宿主测试里那份 `ROS_ARM` 结构体是按 `usartx.h` 的字段名手写的（切片进去的 `ros_arm_publish()` 按名字赋值，名字写错就编译不过 —— 这正是要的同步压力）。反过来"只改 `usartx.h` 的字段名"这个方向宿主测试**抓不到**（宿主有自己的结构体），它由固件自身的编译负责 —— 改完 `ros_arm_publish()` 就找不到该字段，Keil/gcc 立刻报错。方向不同，但两边都有人管。

### Task 4

已改：`HARDWARE/usartx.h`、`HARDWARE/usartx.c`（**GBK + CRLF 上做字节级插入**，脚本 `_ros_tooling/_apply_arm_task4.py`）；新建 `_ros_tooling/_arm_probe.py`（纯 ASCII + LF）。备份在 `_ros_tooling/_backup_arm_task4/`（改动前 sha256 与 Task 3 记录一致）。

**Step 5 宿主测试复跑** `python _ros_tooling/_gen_arm_test.py`

```
gcc -std=c99 -O2 -Wall -Wextra -o ..._ros_tooling/_arm_host_test.exe ..._ros_tooling/_arm_host_test.c
gcc: 0 warnings
ok   MOVE_TIME_WRITE id1 500/1000ms     55 55 01 07 01 F4 01 E8 03 16
...（41 条 ok，与 Task 3 完全相同的 41 条，未增未减）
ok   crc16_modbus("123456789") == 0x4B37
ok   arm frame decodes
ok   base field is signed (-1000)
ok   bad CRC rejected
ok   resync after garbage
ok   recovers after a truncated frame (loses 1, not stuck)
ok   resync adopts a header in the last byte
ok   base field round trips -1000..1000
ok   frame layout invariants (0xAC and 0x56)

ALL PASSED (0 failures)
exit=0
```

gcc stderr = **0 字节**，`warning:` 计数 0，FAIL 计数 0。生成的 `_arm_host_test.c` 里 `ros_raw_byte` / `ros_send_raw_fb` / `ROS_RAW` / `ARM_DEBUG_PASSTHROUGH` 出现次数均为 **0** —— 新的 0xAD 代码没有渗进宿主构建（切片边界仍然干净）。

**Step 4 跨语言交叉校验：Python `lx_frame()` vs C `lx_build()` 逐字节相同**

不是抽查两串，而是两个方向都做了：

1. **把真实 `lx_build()` 切片出来跑**：从 `HARDWARE/lx_servo.c` 的 `LX_LOGIC_BEGIN/END` 之间逐字切片（与生成器同一套切片代码），在宿主机上对 **7 个 ID × 23 条指令 × 7 种参数集 = 1127 个组合** 逐一打印帧，与 Python `_arm_probe.lx_frame()` 的输出比对 → **1127/1127 全等，0 处不符**；该 C 侧构建 gcc stderr 0 字节、0 警告。
2. **对 `_arm_tests.c` 自己断言的那 14 串 hex**（也就是宿主 C 测试正在通过的那些权威报文）逐一比对 → **14/14 逐字节相同**：

| 用例 | Python `lx_frame()` | C 测试断言的 hex | |
|---|---|---|---|
| MOVE_TIME_WRITE id1 500/1000ms | `55 55 01 07 01 F4 01 E8 03 16` | `55 55 01 07 01 F4 01 E8 03 16` | 相同 |
| MOVE_TIME_WAIT_WRITE id1 | `55 55 01 07 07 F4 01 E8 03 10` | `55 55 01 07 07 F4 01 E8 03 10` | 相同 |
| broadcast MOVE_START | `55 55 FE 03 0B F3` | `55 55 FE 03 0B F3` | 相同 |
| POS_READ id1 | `55 55 01 03 1C DF` | `55 55 01 03 1C DF` | 相同 |
| TEMP_READ id1 | `55 55 01 03 1A E1` | `55 55 01 03 1A E1` | 相同 |
| VIN_READ id1 | `55 55 01 03 1B E0` | `55 55 01 03 1B E0` | 相同 |
| ID_READ id1 | `55 55 01 03 0E ED` | `55 55 01 03 0E ED` | 相同 |
| MOVE_STOP id1 | `55 55 01 03 0C EF` | `55 55 01 03 0C EF` | 相同 |
| LOAD id1 | `55 55 01 04 1F 01 DA` | `55 55 01 04 1F 01 DA` | 相同 |
| UNLOAD id1 | `55 55 01 04 1F 00 DB` | `55 55 01 04 1F 00 DB` | 相同 |
| motor mode id6 speed 0 | `55 55 06 07 1D 01 00 00 00 D4` | `55 55 06 07 1D 01 00 00 00 D4` | 相同 |
| motor mode id6 speed +300 | `55 55 06 07 1D 01 00 2C 01 A7` | `55 55 06 07 1D 01 00 2C 01 A7` | 相同 |
| motor mode id6 speed -300 | `55 55 06 07 1D 01 00 D4 FE 02` | `55 55 06 07 1D 01 00 D4 FE 02` | 相同 |
| position mode id6 | `55 55 06 07 1D 00 00 00 00 D5` | `55 55 06 07 1D 00 00 00 00 D5` | 相同 |

另：`crc16_modbus('123456789')` 在 Python 侧 = **0x4B37**，与 `usartx.h` 注释里的权威向量一致（C 侧同一条断言由 T3 通过）。

计划 Step 4 的原命令输出与之吻合：

```
$ python -c "import sys; sys.path.insert(0,'_ros_tooling'); import _arm_probe as p; p.check_crc16(); print(' '.join('%02X'%x for x in p.lx_frame(1,0x0E))); print(' '.join('%02X'%x for x in p.lx_frame(1,0x01,bytes([0xF4,0x01,0xE8,0x03]))))"
55 55 01 03 0E ED
55 55 01 07 01 F4 01 E8 03 16
exit=0
```

**额外补做：0xAD / 0x57 全链路的宿主验证（计划里没有，但它是 Step 3 脚本唯一能被验证的部分）**

把 `usartx.c` 里**真实的** `ros_raw_byte()`、`ros_send_raw_fb()`、`ros_put_u16()`、`crc16_modbus()` 逐字切片到宿主机，用一个假串口喂 `_arm_probe.wrap_raw()` 真正写出去的那串字节，再把固件真的吐出来的 0x57 帧喂回 `wrap_raw()` 的解析逻辑（两个方向都不是手抄副本）：

```
0xAD/0x57 host build: gcc stderr 0 bytes / 0 warning(s)
down  case 0 n=6  tmo=30  payload==555501031CDF                       OK
down  case 1 n=6  tmo=30  payload==555506031CDA                       OK
down  case 2 n=6  tmo=50  payload==555501030EED                       OK
down  case 3 n=10 tmo=20  payload==5555010701F401E80316               OK
down  case 4 n=6  tmo=20  payload==5555FE030BF3                       OK
down  case 5 n=16 tmo=40  payload==000102030405060708090A0B0C0D0E0F   OK
      badlen frames=1 crc_err=1
      toolong frames=1 crc_err=2
      resync frames=1 ready=1 n=6
      trunc frames=0 n=0
up    case 0 0x57 57 0E 00 00 06 00 55 55 01 03 1C DF 5D 66                -> wrap_raw -> 555501031CDF                       OK
up    case 3 0x57 57 12 03 00 0A 00 55 55 01 07 01 F4 01 E8 03 16 9E 14    -> wrap_raw -> 5555010701F401E80316               OK
up    case 5 0x57 57 18 05 00 10 00 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 61 8A -> wrap_raw -> 000102030405060708090A0B0C0D0E0F   OK

0xAD/0x57 round trip: 0 mismatches
```

`trunc frames=0` 是**预期**：残帧吃掉半条、解码器在 CRC 失败后复位，跨在重同步窗口上的那一帧必丢（与 Task 3 记录的 0xAC 解码器语义完全一致，三个解码器同形），下一帧恢复正常。

**ISR 编译期开关验证**（`#if ARM_DEBUG_PASSTHROUGH` 真的在起作用）：把真实的 `USART1_IRQHandler` 切出来在宿主机按宏 = 0 / 1 各编译一次并运行 —— 两种配置 gcc stderr 均 0 字节 0 警告；宏 = 0 时 `raw=0`、宏 = 1 时 `raw=1`，即透传调用确实被编译掉了。

**字节级 diff 证据（纯插入，零删改）**

| 文件 | 字节 | 变化 | opcode | 非插入编辑 | 删掉插入段能否还原备份 |
|---|---|---|---|---|---|
| `HARDWARE/usartx.c` | 32273 → 35431 | +3158 (9.79%) | equal×3, insert×2 | **0** | **True**（还原后 sha256 与备份逐字节相同） |
| `HARDWARE/usartx.h` | 12661 → 14509 | +1848 (14.60%) | equal×2, insert×1 | **0** | **True** |

- `usartx.c` 两段插入：`new[16029]` len=3047（0xAD 解码器 + 0x57 发送，紧跟在 `/* ==== ARM_TASK_BEGIN ==== */` 之后）、`new[19466]` len=111（ISR 里的 `#if` 块）。
- `usartx.h` 一段插入：`new[10806]` len=1848（`#define ARM_ST_ESTOP` 之后）。
- 备份 `85172ba4d335662c…fdc9429a` → 现 `d8a7fcca484eeab0…f9a277b8`（usartx.c）；`ca66c45a8a2e830f…0c6574a0` → `751e56d55581aa94…b46492e6`（usartx.h）。
- **裸 LF 断言**：`usartx.c` CRLF 1000 → 1092、孤立 LF **0 → 0**；`usartx.h` CRLF 303 → 347、孤立 LF **0 → 0**。脚本里 `assert b.count(b'\n')-b.count(b'\r\n') == 0` 是硬失败（Task 3 那条 3 个裸 LF 的教训）。
- 非 ASCII 字节数两文件均**未变**（`usartx.c` 2076 → 2076、`usartx.h` 154 → 154），GBK 原文一字未动。

**与计划的偏差（3 处，全部有意，每条都写进了代码注释）**

1. **⭐ 计划 Step 1 把 0xAD 解码器插在 `/* ==== ARM_TASK_BEGIN ==== */` **之前**，这会把 Task 3 的切片边界弄脏。**该标记是 `_gen_arm_test.py` 切 0xAC 解码器的右边界**，插到前面之后那段切片会一直吃到 0xAD 解码器，而宿主构建里既没有 `ROS_RAW` 类型也没有 `ROS_RAW_*` 常量（生成器只抽 `ROS_ARM*`/`ARM_ST_*`）→ 编译不过。改为插在标记**之后**，标记语义不变，宿主测试的 41 条断言一条不动。这是"计划文本与既有切片机制冲突"的第三种形态（Task 2/3 各撞到一次）。
2. **⭐ 计划的下行帧长度字节算错一位。** 计划自己的 C 解码器按 `s_raw_want = len` 收满整条帧、CRC 取 `buf[len-2..len-1]`，反推出来 `len = 7 + n`；但计划给的 Python `wrap_raw()` 写的是 `6 + len(payload)`。两边差一个字节 ⇒ **CRC 每次必失败、透传 100% 不可用**（实测：按计划原样组帧喂真实 `ros_raw_byte()`，6/6 个用例全部拿不回载荷）。已在 `_arm_probe.py` 里改成 `7 + n_payload` 并加注释说明约定；同时把 `ROS_RAW_MIN_LEN` 从 6 改成 **7**（n=0 的最短合法帧就是 7 字节，6 会让解码器去读帧外的字节并把 CRC 低字节当成 `n`）。改完后 6/6 取回，见上面的 round-trip 表。
3. **解码器加了一行 `n` 钳位**：`if(RosRaw.n > s_raw_want - 6) RosRaw.n = s_raw_want - 6;`。载荷实际只有 `want-6` 字节，而 `n` 是帧里自报的；随机噪声撞对 CRC（1/65536）时 `n` 可能是任意值，原样会读到帧外的陈旧字节。钳位后读循环不可能越出本帧声明的范围（也仍在 26 字节缓冲内）。对任何合法帧行为不变。

**Step 2 未执行（`arm_task` 里的透传执行块）** —— 计划把它的代码写在 Task 4 Step 2，但落地位置是 `BALANCE/arm.c` 的 `arm_task`，而 `BALANCE/arm.c` **此刻还不存在**（Task 5 建、Task 8 填满循环）。实测 `ls BALANCE/arm.c` → `No such file or directory`。该代码块原样留在计划里，由 Task 8 放进循环，本任务不代写。

**上板验证项（宿主测不了，列入 Task 10）**

- `_arm_probe.py` 的串口路径**无法在本机验证**：`serial.Serial('/dev/ttyAMA0', ...)` 抛 `SerialException`（无误的硬件即失败）。argparse 的子命令 `id/pot/spin/mode/torque/raw` 全部注册正常、`--help` 正常、`python -m py_compile` 通过；纯函数 `crc16_modbus` / `lx_frame` 已按上面的 Step 4 在本机逐字节验证。
- 尚未验证、必须上板的：`ros_raw_byte()` 真的被 ISR 调用（宿主验证只能用假串口喂字节）、`arm_task` 真的把 0xAD 代理到 USART2、`lx_raw_exchange()` 的超时与 `LX_GAP_US`、以及透传实际往返延迟（Pi 侧 deadline 留了 `timeout_ms + 250ms` 余量）。
- 用前提醒：透传**只在 `ARM_DEBUG_PASSTHROUGH = 1` 的 bring-up 构建里存在**，默认 0，出厂构建不含该路径。

**补做（评审要求）：把 0xAD 透传的宿主验证固化进 `_gen_arm_test.py` / `_arm_tests.c`**

理由：透传是 Task 10 的 M1–M5 实测**唯一**通道，M1 决定底座算法能不能定稿；它一旦静默失效，项目会卡在最后一个任务且宿主侧毫无信号。原来那份验证在 `%TEMP%` 里，活不过一次清理。

改了三处：生成器新增切片（真实的 `ros_raw_byte()` / `ros_send_raw_fb()` / `ros_put_u16()` / `ROS_RAW*` 常量；`crc16_modbus` 早已在切）+ 新占位符 `@@RAWDEFS@@ @@PUT@@ @@RAWBOX@@ @@RAWCODE@@ @@PYFRAMES@@`；`_arm_tests.c` 追加 12 条断言；宿主模板补一个 `usart1_send` 捕获桩。

```
$ python _ros_tooling/_gen_arm_test.py
gcc: 0 warnings
...（53 条 ok：Task 4 原有的 41 条一条未动，新增 12 条）
ok   6 frames from _arm_probe.wrap_raw() decode byte for byte
ok   0xAD decodes a 6 byte payload, timeout 30ms
ok   0xAD decodes a zero length payload (Length = 7)
ok   0xAD decodes a 3 byte payload
ok   0xAD decodes a full 16 byte payload
ok   0xAD rejects a Length below the minimum
ok   0xAD rejects a Length above the frame buffer
ok   0xAD rejects a bad crc
ok   0xAD resyncs after garbage
ok   0x57 frame layout and crc
ok   0x57 seq advances
ok   0x57 clamps a payload larger than the frame

ALL PASSED (0 failures)
exit=0
```

exit=0，gcc stderr **0 字节**，`warning:` 计数 0。

**配对机制（能抓住"差一位"的就是这一条，做法说明）**：**不是手抄字节**。`_gen_arm_test.py` 直接 `import _arm_probe`，用一个假串口对象调用 **`wrap_raw()` 本身**，把它真正写出去的那串字节抓下来，先在**生成期**断言其满足固件侧契约（`hdr == 0xAD`、`len == 7 + n`、`timeout` 字段、`n` 字段、载荷逐字节、CRC 覆盖前 `5+n` 字节），再把它**生成**成 `PyProbe[]` 写进 `_arm_host_test.c`，由 `test_raw_matches_probe_framing()` 喂给真实的 `ros_raw_byte()`。于是：改 `wrap_raw()` → 生成期失败；改 `ros_raw_byte()` → C 断言失败。**两个方向都锁死，且没有任何手抄副本**（与 Self-Review 第 6 条一致）。

为此把 `_arm_probe.py` 的 `import serial` 改成惰性（模块级失败降级到 `main()`）：否则没有 pyserial 的机器连生成器都跑不起来，而 `sys.exit()` 在 import 期还会伪装成 `SystemExit`。纯函数可导入可单测，本来也该成立。

**变异测试（证明新用例不是空转）：3 个变异，3 个被抓，0 幸存**

| 变异 | 位置 | 结果 |
|---|---|---|
| `wrap_raw()` 长度字节 `7+n` → `6+n`（**就是计划原先那个 bug**） | `_arm_probe.py` | 生成器 exit=1：`AssertionError: _arm_probe.wrap_raw() length byte is 12 but the firmware decoder wants 7 + 6` |
| CRC 偏移 `want-2` → `want-3` | `HARDWARE/usartx.c` | exit=1：`FAIL 6 of 6 _arm_probe.wrap_raw() frames did not survive the decoder` 等 4 条 |
| 长度上界 `ROS_RAW_SIZE` → `ROS_RAW_SIZE + 1` | `HARDWARE/usartx.c` | exit=1：`FAIL 0xAD rejects a Length above the frame buffer` 等 |

每个变异跑完都把 `usartx.c` / `_arm_probe.py` 按字节还原（脚本用 `atexit` 兜底），还原后 sha256 与变异前相同，生成器复跑 `ALL PASSED`。

> **顺带查出一个隐蔽的坑（宿主测试设施的"假绿"来源）**：变异测试第一轮出现"文件已还原、生成器却仍按变异后的代码跑"。原因是 `_ros_tooling/__pycache__/_arm_probe.cpython-311.pyc` —— 变异把 `7 +` 改成 `6 +` **文件大小不变**，还原又落在**同一秒**，`.pyc` 里记的 (mtime 秒, size) 因而仍然"有效"，Python 直接加载了**变异后的字节码**（已反汇编确认 `wrap_raw` 的常量为 `6`）。已在生成器里修掉：`sys.dont_write_bytecode = True` + 导入前删除 `__pycache__/_arm_probe.*.pyc`，保证 `_arm_probe` 永远从源码加载。

**备份重命名（统一约定）**：Task 4 第一次落地的 `_backup_arm_task4/usartx.c|h` 已改名为 **`HARDWARE__usartx.c|h`**，与 `_backup_arm_task2/`、`_backup_arm_task3/`、`_backup_armfb_seq/`、`_backup_task2/` 一致（内容未变，sha256 一致）。命名规则已写进本任务 Step 1，后续任务照此执行。

**复核**：`HARDWARE/usartx.c|h` 自 Task 4 完成后 sha256 未变（`d8a7fcca…` / `751e56d5…`），"纯插入、零删改、零裸 LF"的结论复跑后依然成立（`delete = 0, replace = 0`，删掉插入段逐字节还原备份）；`ARM_DEBUG_PASSTHROUGH` 仍为 **0**，在 `usartx.c` 中只出现 1 次（ISR 的那个 `#if`）。解码器与发送器定义在该 `#if` **之外**（`usartx.c` 第 534 / 587 行，`#if` 在第 613–615 行、位于 `USART1_IRQHandler` 内），所以任何构建里都能切片、都能测 —— 这正是"不发售该功能也能测它"的前提。


### Task 5

新建 `BALANCE/arm_cal.h`、`BALANCE/arm.h`、`BALANCE/arm.c`（三个文件均 **纯 ASCII + 纯 LF**，注释全英文）。本任务**未改动任何既有文件** —— `HARDWARE/*`、`_ros_tooling/_gen_arm_test.py`、`_ros_tooling/_arm_tests.c` 一个字节都没动（计划里本任务就没有 Modify 项）。

**编码实测**（字节级）

| 文件 | bytes | CRLF | 裸 LF | 非 ASCII 字节 |
|---|---|---|---|---|
| `BALANCE/arm_cal.h` | 3530 | 0 | 77 | 0 |
| `BALANCE/arm.h` | 4202 | 0 | 107 | 0 |
| `BALANCE/arm.c` | 5505 | 0 | 153 | 0 |

**Step 4 宿主测试复跑** `python _ros_tooling/_gen_arm_test.py`

```
gcc -std=c99 -O2 -Wall -Wextra -o ..._ros_tooling/_arm_host_test.exe ..._ros_tooling/_arm_host_test.c
gcc: 0 warnings
ok   MOVE_TIME_WRITE id1 500/1000ms     55 55 01 07 01 F4 01 E8 03 16
...（53 条 ok，与 Task 4 完全相同的 53 条，未增未减：本任务按计划不新增宿主用例，arm.c 不在生成器的切片范围内）
ok   0x57 seq advances
ok   0x57 clamps a payload larger than the frame

ALL PASSED (0 failures)
exit=0
```

gcc stderr = **0 字节**，`warning:` 计数 0，FAIL 计数 0，ok 计数 **53**（Task 4 也是 53）。

**Task 6 Step 2 锚点自检（逐条实跑 Step 2 的表达式，不是目视核对）**

| 检查 | 表达式 / 断言 | 结果 |
|---|---|---|
| 结构体切片起点 | `arm_h.index('typedef struct\n{\n    float deg;')` | 偏移 **1867** |
| 结构体切片终点 | `arm_h.index('} ARM_CAL;')` | 偏移 **3382**，切片长 **1525 字节** |
| `must(arm_types, 'search_bound2_deg', ...)` | ARM_CAL 含该字段 | 通过 |
| `cal_defs` 抽取 | `re.match(r'#define\s+(ARM_\w+)\s+(.+?)\s*(?:/\*.*)?$')` | 抽出 **19** 条 `#define ARM_` |
| `must(cal_defs, 'ARM_POT_WRAPS' / 'ARM_SOFT_LIMIT_DEG' / 'ARM_SEARCH_BOUND_DEG' / 'ARM_SEARCH_BOUND2_DEG')` | 四条 | 全部通过 |
| `must(arm_h, '#define ARM_BASE_FAULT', ...)` | 状态宏存在 | 通过 |
| `ARM_LOGIC_BEGIN` / `_END` | 各出现 1 次、切片非空 | 通过 |
| 切片段能否在宿主编译 | 把 `arm_types` + `cal_defs` 单独喂 gcc `-std=c99 -O2 -Wall -Wextra` | **0 警告 0 错误**，`sizeof(ARM_BASE)=24`、`sizeof(ARM_CAL)=48` |

**执行中发现的一个坑（计划文本未提醒，已避免）**：`arm.h` 最初在说明注释里**原样引用了锚点字面量** `} ARM_CAL;`，于是 `arm_h.index('} ARM_CAL;')` 命中的是注释里那一处（偏移 **1126** < 1867），切片长度变成 **0**，`must(arm_types, 'search_bound2_deg')` 直接失败。`index()` 是从文件头找**第一处**匹配、且终点表达式没有起点约束（`arm_h.index(e2)` 而非 `arm_h.index(e2, s0)`），所以**任何在结构体之前出现该字面量的地方（注释、说明、宏名）都会静默把切片截空**。已把注释改写为不再复现任何锚点字面量，并在注释里写明这条约束。这个失败是**在写文件之后立刻实跑锚点表达式才发现的** —— 若只按计划把文件写出来不管，它会一直潜伏到 Task 6 生成器报 `missing ARM_CAL field`。

**与计划文本的偏差（1 处，功能等价）**：计划 Step 2 的 `arm.h` 把 `ARM_BASE.deg` 写成 `float deg;             /* angle estimate, degrees */`（同一行挂对齐注释）。本实现把该注释挪到了下一行，使 `float deg;` 成为**恰好四个空格 + 该行再无其它内容**的一行 —— 这是任务书对 Task 6 锚点的更严要求（`index` 本身是子串匹配，计划原文本其实也能命中，但严格写成整行更不容易被后续改动破坏）。除此之外 `arm.h` 与计划 Step 2 逐字一致。

**`arm.c` 的验证方式**：按硬约束**不在宿主编译**（它 include `system.h` → FreeRTOS/STM32）。替代验证 —— 把计划 Step 3 的代码块与 `BALANCE/arm.c` 都去掉注释后逐行 diff：**100 行 vs 100 行，完全一致**；括号配平 `{}` 14/14、`()` 27/27、`[]` 5/5。`arm.c` 引用但尚未定义的只有 `arm_base_init()`（Task 6 的 `ARM_LOGIC` 段提供），C99 下「已声明未定义」只影响链接、不影响编译，与计划一致。

**留给后续任务的接口（已固定，勿改）**：`ARM_BASE` / `ARM_CAL` 两个结构体在 `arm.h` 中相邻且顺序为 ARM_BASE 在前；`arm.c` 的 `/* ==== ARM_LOGIC_BEGIN ==== */ … /* ==== ARM_LOGIC_END ==== */` 目前只含一行占位注释；`INIT_HOME` 为空桩（Task 7 填），`arm_task` 主循环末尾的 `/* Task 8 adds the runtime block here */` 为空桩（Task 8 填）。

**上板验证（本任务无法自动验证，列入 Task 10）**：上电后 `INIT_PROBE` 能过（`RosArmFb.status` 的 `ARM_ST_ONLINE` 置位）、5 个位置关节一起走到复位姿态 `226/500/177/129/408`、底座**不动**（`INIT_POSE` 只对 `ARM_JOINT_ID[0..4]` 下发）。

### Task 6

**交付物**：`BALANCE/arm.c` 的 `ARM_LOGIC` 段填满（9 个纯函数 + `static arm_search_dir_pick`）、`_ros_tooling/_arm_tests.c` 追加 T5/T6、`_ros_tooling/_gen_arm_test.py` 增加三段切片与宿主模板占位。另新增 `_ros_tooling/_mutate_t6.py`（变异测试脚本，可重复运行）。

**宿主实测**（真实源码逐字切片，零改写；常数见文末「常数修订」）

```
$ python _ros_tooling/_gen_arm_test.py
gcc -std=c99 -O2 -Wall -Wextra -o ...\_arm_host_test.exe ...\_arm_host_test.c
gcc: 0 warnings
...
ok   homing reaches the window from all 11 starts, both hints, hint-independent (worst peak 224.0 deg)
ok   homing reaches from every start over 30 (start, speed error) pairs, 0.75x..1.5x
ok   dead pot faults at 384.2 deg after 5.36 s (268 ticks)
...
ALL PASSED (0 failures)
exit=0
```

`exit=0`、gcc stderr **0 字节**、`warning:` 计数 0、FAIL 计数 0、ok 计数 **86**（Task 5 为 53；本任务 +33）。

**T6 实测 peak**（定稿常数 `ARM_SEARCH_BOUND_DEG = 160.0f`、`ARM_SEARCH_MAX_MS = 5400`）

| 用例 | 计划 Expected | 实测 |
|---|---|---|
| `direction_from_rail`（11 起点 × 2 hint） | worst peak 224.0 | **224.0**；两个 hint 的 tick/peak 逐位相同（已断言） |
| `survives_speed_error`（6 起点 × 0.75/0.8/1.0/1.25/1.5 = 30 对） | 原计划 ±25% | **全部到达窗口**，peak 恒等于起点绝对值，最大 **224.0** |
| `dead_pot`（起点 +224，读数恒 0） | "~329 deg after 3.5 s" | **384.2 deg、268 tick、5.36 s**（= 224 + 160 预算 + 一拍） |
| `dead_pot` × k=1.25（模型推算，未进断言） | — | **424.3 deg**（224 + 160 × 1.25） |

线缆预算 720 deg 下两者都有约 300 deg 余量。

**缺陷 1：`ARM_SEARCH_MAX_MS = 3500` 把到达预算吃掉了（本任务修掉，定稿 5400）**

两个 cap 量的都是**指令行程**，所以有效上限只有一个数：`min(ARM_SEARCH_BOUND_DEG, ARM_SEARCH_MAX_MS × 0.03)` believed deg（20 ms/tick、0.6 believed deg/tick）。3500 ms 只买到 175 tick，而 135 believed deg 需要 225 tick → 有效上限被压到 **105 believed deg**，可达范围缩到 ±210.6 deg，±224 起点**永远寻不到零**（T6 实测 `never reached the window`）。这是任务书要求的 `ALL PASSED` 无法达成的根因：计划里 `ARM_SEARCH_BOUND_DEG` 的注释说 135 是到达预算，`ARM_SEARCH_MAX_MS` 的注释说 3.5 s 是安全网，两条注释互相矛盾，而代码里时间 cap 才是**更紧**的那条。
定稿 5400 = 270 tick > 267 tick（160 believed deg 的成本 = 160/(150 × 0.2 × 0.02) = 267 tick），距离 cap 重新成为唯一约束项。`_gen_arm_test.py` 现在**从宏本身推导**并断言这条关系（`min(BOUND, MAX_MS*0.03) == BOUND`），不再写死 `225*20`，所以以后调 `ARM_SEARCH_BOUND_DEG` 不会用一个过期的常数把同一个 bug 再种一次；变异 M1/M6/M7/M7b 都会撞上这道闸门。

**缺陷 2：计划的 Expected 文本自相矛盾** —— "从每个起点都能寻到零"需要 T ≥ 5300，「dead pot 停在 ~329 deg」却正是 3500 的产物，两者不能同时成立。

**缺陷 3：两个 cap 不是独立的**（同一条指令速度、同一个 dt）→ 有效安全上限只有 `min(BOUND, MAX_MS × 0.03)`；计划 Self-Review 说"时间网才是真正的安全网"与代码不符。5400 时 = min(160, 162) = 160，时间 cap 只是网：把它**调高**不改变任何行为（变异 M2 存活即此），把它**调低**会立刻缩短到达（M6/M7/M7b 全被抓）。

**缺陷 4：`arm_homing_step` 首次 priming 调用总是先动一下** —— `primed` 分支无条件输出 ±search_speed，起步已在窗口内时会先抖 0.6 deg，下一拍 resync。无害，但 Task 7 接线时值得知道。

**缺陷 5：计划对反向腿的算术是错的** —— Self-Review #5 说两腿搜索会走到 224+135+280 = 639°，但 `arm_homing_step` 反向时重置行程并翻转方向，第二腿是**回撤**：峰值不增加，只是把 268 tick 的行程翻倍到 535 tick、把 FAULT 推迟到 10.7 s。所以 `ARM_SEARCH_REVERSE=1` 的危险是"延迟报警"而非"扯断线缆"，T6 因此除 peak 外还钉 tick 数。

**未覆盖项（有意为之）**：dead pot × k_high 的组合没有进断言 —— 宿主只跑 k=1 的 dead pot，k=1.25 的 424.3 deg 是模型推算。720 预算下即使按 k=1.5 算（224 + 240 = 464）仍有 256 deg 余量，这是修正预算后新获得的余量，早先的 ±360 假设把它吃光了。

**变异测试**（`python _ros_tooling/_mutate_t6.py`；每次改字节、跑、再逐字节还原并用 sha256 校验）

| 变异 | 结果 | 被哪条断言抓住 |
|---|---|---|
| M1 `ARM_SEARCH_BOUND_DEG` 160 → 400 | **CAUGHT** | 生成器的预算闸门（`_gen_arm_test.py` 拒绝生成：有效到达预算 ≠ 声明的 160） |
| M1b 同 M1，但临时放宽预算闸门 | **SURVIVOR** | 无 —— 见下 |
| M2 `ARM_SEARCH_MAX_MS` 5400 → 12000 | **SURVIVOR** | 无 —— 距离 cap 仍先触发（缺陷 3） |
| M3 `ARM_SEARCH_REVERSE` 0 → 1 | **CAUGHT** | `dead pot took 535 ticks (10.70 s) - the search is no longer one leg` |
| M3b 同 M3 + 换向拍改用新方向 | **CAUGHT** | 同上（535 tick），证明 tick 断言是活的（峰值断言看不见回撤腿） |
| M4 翻转 `arm_search_dir_pick` 的导轨规则 | **CAUGHT** | `homing start +110.0: never reached the window (peak 270.2)` |
| M5 去掉 `arm_track_step` 的软限幅 | **CAUGHT** | `FAIL soft limit 5176.0`（T5） |
| M6 `ARM_SEARCH_MAX_MS` 5400 → 2000 | **CAUGHT** | `homing start +180.0: never reached the window` |
| M7 `ARM_SEARCH_MAX_MS` 5400 → 4600（上一版取值） | **CAUGHT** | `homing start +224.0 speed x0.80: never reached the window` —— 有效预算 162 → 138，**只有 0.80x 那条带边界看得见它** |
| M7b `ARM_SEARCH_MAX_MS` 5400 → 3500（计划原值） | **CAUGHT** | `homing start +224.0: never reached the window` |
| M8 `ARM_SEARCH_BOUND_DEG` 160 → 100 | **CAUGHT** | `homing start +224.0: never reached the window`（预算闸门放行：162 ≥ 100，所以是 T6 抓的） |

**两个 survivor 都是真发现，不是被改掉的测试，如实记录：**

- **M1b（放宽闸门后 `ARM_SEARCH_BOUND_DEG` 160 → 400 存活）**：720 预算下把 bound 调大**不再违反任何安全性质** —— 有效上限变成 `min(400, 162) = 162` believed deg，dead pot 峰值 386 deg 仍远在 720 内，到达反而更宽，所以 T6 不报错是**正确**的。真实的缺口是**常数漂移**：宿主用例只钉不变量（peak ≤ 720、能到达、单腿），不钉 `ARM_SEARCH_BOUND_DEG` 的具体数值，所以"160 被改成 400"这种事故只能靠生成器那道闸门拦住 —— 而它确实拦住了（M1）。若要求用例本身也钉住这个数，需要断言"dead pot 峰值 ≈ 224 + BOUND"，那会把常数值写死进测试、Task 10 重标时必然要改，本任务没有加。
- **M2（`ARM_SEARCH_MAX_MS` 5400 → 12000 存活）**：同缺陷 3 —— 两个 cap 量同一条指令行程，距离 cap 更紧时向上放松时间 cap 在数学上是空操作。不是测试没看见，是被测物没有发生变化。

M3 / M6 / M7 / M7b / M1b 需要**临时放宽生成器自己的一道闸门**才能对到 gcc：生成器默认拒绝编译与出厂形态不同的 `ARM_POT_WRAPS` / `ARM_SEARCH_REVERSE`，以及预算关系被破坏的 `ARM_SEARCH_MAX_MS`；这道闸门比 T6 更早触发（脚本已注明，还原时一并校验）。

还原后三个文件 sha256 前缀：`arm.c 0b9f8b24321c`、`arm_cal.h f6297b93a8c8`、`_gen_arm_test.py 420e1142ee48`，与基线**逐字节相同**。

**宿主构建编译的是出厂形态（用 `gcc -E` 实查，不是目视）**

生成器抽出的 `#define` 实测为 `ARM_POT_WRAPS 0`、`ARM_SEARCH_REVERSE 0`、`ARM_SEARCH_BOUND_DEG 160.0f`、`ARM_SEARCH_MAX_MS 5400`；预处理后 `if(d > 500 || d < -500)`（回绕判据）与 `b->search_reversed = 1`（反向腿）**各出现 0 次**，两条 `#if` 分支都被编译掉。取这两个值是因为它们就是出厂值（M1 判定电位器为截断型 → `ARM_POT_WRAPS 0`；截断型的导轨已指明方向 → `ARM_SEARCH_REVERSE 0`，本轮**未改动**），且生成器把这两个值写成**硬断言**：出厂值一变而宿主模板没跟着改，生成器直接失败，而不是默默测一份不再出厂的代码。

**切片自检（本任务新增，全部是断言而非目视）**

- `ARM_LOGIC_BEGIN` / `_END` 各恰好 1 次；终点锚点**从起点偏移搜**（`arm_c.index(end, i)`）+ `assert j > i`。
- 切片非空且内容正确：`must()` 逐条钉住 `arm_homing_step` / `arm_search_dir_pick` / `arm_track_step` 的完整签名、`c->search_bound_deg`、`c->search_max_ms`；`assert len > 4000`（实测 9350 字节）。
- 纯度：切片内出现 `#include` / `USART` / `vTask` / `delay_` / `getSysTickCnt` / `ARM_JOINT_ID` 任一即失败（实测全部不出现），所以不会出现"宿主构建失败于未知标识符"。
- `arm.h` 结构体切片：`index('} ARM_CAL;', s0)` + `assert s1 > s0` + `must(arm_types, 'search_bound2_deg' / 'float search_travel;' / 'u8    primed;')` + 两个结构体各恰好闭合一次。
- 状态枚举不再手抄：从 `arm.h` 正则切 `ARM_BASE_*` 五条（Task 5 记录里的手抄字面量已删），数量断言为 5。
- 宿主模板占位符替换后统一 `assert '@@...@@' not in src`，防止模板与替换表漂移。
- 宿主 typedef 补 `s8`（原文有 u8/u16/u32/s16/s32）；`<string.h>` 原本就有。

**计划文本与实际不符之处（除上文 1–5 外）**

- Task 6 Step 3 的 `test_vel_loop` 断言 `arm_base_vel(&tc, 10.0f, 1000)` 落在 `5..30`，但 `ARM_VEL_KP = 20` 时该值 = `10/1 + 20×10 = 210` —— 计划里这条内联断言与 `arm_cal.h` 的 `ARM_VEL_KP` 互相矛盾（写的时候按小 KP 估的）。改为断言**契约**：命令速度必须够在 `t_ms` 内走完误差（除非已饱和），并补上符号、单调性、`t_ms` 缩放、双侧限幅。未动 `ARM_VEL_KP`（它是 M5 实测项）。
- Task 6 Step 2 的 `must(cal_defs, 'ARM_SEARCH_BOUND2_DEG', ...)` 只证明宏存在；`ARM_SEARCH_REVERSE=0` 时 `search_bound2_deg` 是死字段，已在报告里点明，未删除（截断型/回绕型共用一份 `ARM_CAL`）。
- 计划 Step 3 的 `run_homing()` 把 `*peak` 初值设为 0，漏掉了**起点本身**；起点是底座真实到过的位置，漏掉它会让"全程没动"的用例报 peak = 0。已改为以 `|start_deg|` 初始化（这也是"worst peak 224.0"能成立的前提）。
- 计划 Step 3 的 T6 用例依赖前一个用例留下的 `tc`（未调 `cal_default()`），用例顺序一变就会静默用错标定值。已让每个 T5/T6 用例各自调 `cal_default()`。

**不写宿主测试（上板项，Task 10）**：真实舵机在 motor 模式下的速度响应、电位器在盲区/导轨附近的实际读数形态、`ARM_SPEED_1000_DPS` 与 `ARM_ACCEL_COMP_DEG` 的真值（M3/M4）。T6 里的 `k_true` 是**模型**出来的速度误差，不是实测。

**常数修订（两轮，留档以便追溯）**

| 常数 | 计划原值 | 第一轮（按 ±360 预算） | 第二轮（定稿，按 ±720 预算） | 一行理由 |
|---|---|---|---|---|
| `ARM_SEARCH_BOUND_DEG` | `135.0f` | `135.0f` | **`160.0f`** | 到达是唯一约束：`bound ≥ 119.4 / k_low`，160 ⇒ k_low ≥ 0.746，覆盖 ±25% 标定带；720 预算下 excursion 384 / 424 deg 绰绰有余 |
| `ARM_SEARCH_MAX_MS` | `3500` | `4600` | **`5400`** | 必须高于 `ARM_SEARCH_BOUND_DEG` 的 round trip（160 believed deg = 267 tick = 5333 ms），否则时间 cap 变成到达预算 —— 3500 时只剩 105 believed deg，±224 起点回不了零 |

**注意：早先一版把线缆预算记成 ±360 deg，并据此写下"标定带只能 −12%/+0.7%"与"dead pot × 1.25x = 393 deg 超标"两条结论 —— 那个 ±360 是错的，真实预算是 ±720，上述两条结论已作废，不要按旧文本重新推导。** 两轮修订里 135 → 160 与 4600 → 5400 都是围绕真实预算重做的取值。

### Task 7

**交付物**

| 文件 | 动作 | 状态 |
|---|---|---|
| `BALANCE/arm_cal.h` | health-limits 块加 `#define ARM_HOME_READ_FAIL_MAX 5` | 完成。**Task 5 落地时漏了这条** —— 计划第 2153 行有、Task 5 的实现没写进文件（实测 `grep` 命中 0），Task 7 的 `INIT_HOME` 要用它，所以本任务补回 |
| `BALANCE/arm.c` | `case INIT_HOME:` 填实；加 `home_read_fail` / `prev_stage` 局部量；加 `(void)pvParameters;` | 完成 |
| `BALANCE/arm.c` | 文件级 `static s16 ArmTargetPos;` / `static u8 ArmHold = 1;` | 完成（Task 8 会在这两行旁边补 `ArmPos`/`ArmTime`/`ArmFlags`，**别再重复声明这两个**） |
| `BALANCE/arm.c` | 运行期底座块，夹在 `/* ==== ARM_BASE_TICK_BEGIN ==== */ … /* ==== ARM_BASE_TICK_END ==== */` 之间 | 完成 |
| `_ros_tooling/_gen_arm_test.py` | 新增"底座块调用顺序"断言（**不新增 ok 用例**，ok 计数仍 86） | 完成 |
| `BALANCE/arm.h`、`_ros_tooling/_arm_tests.c` | 未改动 | — |

**sha256**

| 文件 | before | after |
|---|---|---|
| `BALANCE/arm.c` | `0b9f8b24321c2a1bbaf2501360fcedfc64cf37fd0d2cda278ae2edda0edfb129` | `ea2299b9ade7564f1c169ded4bbf2a053e47878b5cb7e66d1bdf6322d4478fad` |
| `BALANCE/arm_cal.h` | `f6297b93a8c8f7074cae1c45699c468841115c45d43117128f68d362715021af` | `81968a1e84cec8eb2b68eda85bfb1b6466a1dc2604b9daabd5d157939519e494` |
| `BALANCE/arm.h` | `831367b3133da717f37243ed5546ea0d8b9113899e14b715ce9e5d33362a646f` | 不变 |
| `_ros_tooling/_gen_arm_test.py` | `420e1142ee4889695984a13d906bb6b3908c68ea3bbb8223caee405711b7d9a8` | `dfc7e0254eac0981c764f2c206b292aaa88a47323a26b59df1f4335fb5f66f0a` |

生成器的 before 值不是"改之前记下来的"，而是**从 after 版本里删掉本次新增那一段后重算**得到的，结果与 Task 6 记录里的前缀 `420e1142ee48` 逐位相同 —— 即本次对生成器的改动确实只有那一段。

**`arm.c` 改动的区域（逐处）**

1. 文件级新增两行静态量（第 316-317 行）与 `ArmHold` 初值的理由注释。
2. `arm_task()` 开头：`home_read_fail`、`prev_stage` 两个局部量；`(void)pvParameters;`。
3. `arm_task()` `while(1)` 内、`switch` **之前**：INIT_HOME 入口边沿块。
4. `case INIT_HOME:` 整段（原为占位注释）。
5. 主循环末尾：`if(stage != INIT_READY) continue;` + 运行期底座块（带两个切片标记）+ 两处给 Task 8 留的挂钩注释，替换原来的 `/* Task 8 adds the runtime block here */`。

**宿主测试** `python _ros_tooling/_gen_arm_test.py`

```
gcc -std=c99 -O2 -Wall -Wextra -o ...\_ros_tooling\_arm_host_test.exe ...\_arm_host_test.c
gcc: 0 warnings
ok   MOVE_TIME_WRITE id1 500/1000ms     55 55 01 07 01 F4 01 E8 03 16
ok   MOVE_TIME_WAIT_WRITE id1           55 55 01 07 07 F4 01 E8 03 10
...
ok   probe always wanted while homing
ok   no probe while faulted
ok   no probe while unknown

ALL PASSED (0 failures)
exit=0
```

gcc stderr = **0 字节**，`warning:` 计数 **0**，FAIL 计数 **0**，ok 计数 **86**（Task 6 也是 86；本任务按计划不新增宿主 ok 用例，理由见下）。

**`arm.c` 无法宿主编译 —— 替代验证（`-fsyntax-only`）**

```
gcc -fsyntax-only -std=c99 -O2 -Wall -Wextra \
    -ICORE -ISYSTEM/delay -ISYSTEM/sys -ISYSTEM/usart -IUSER -IHARDWARE/LED \
    -IFWLIB/inc -IFreeRTOS/include -IFreeRTOS/portable/RVDS/ARM_CM4F \
    -IBALANCE -IHARDWARE -IHARDWARE/ICM20948 \
    -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP -DARM_MATH_CM4 \
    BALANCE/arm.c
```

include 路径与宏取自 `USER/WHEELTEC.uvprojx`（`IncludePath` 与 `Define` 两处，逐字提取，只去掉 Keil 专用的 `__CC_ARM`）。

**过滤方法（两步，第二步是关键）**

1. 按**诊断归属文件**分类：gcc 的每条 `file:line:col: severity:` 都标出源文件，用正则把 `error|warning|fatal error` 连同归属文件抽出来计数（`note:` 是上下文，单独排除）。
   第一次运行：**167 条诊断，其中 21 条归属 `arm.c`**（`arm.h` 0 条）。但这 21 条是**级联假象** —— 首因在 `FreeRTOS/portable/RVDS/ARM_CM4F/portmacro.h:209-286`：Keil 专有的 `__asm { msr basepri, ulBASEPRI }` 内联汇编块在 gcc 上不是合法 C，解析器从那一点起失步，之后每个正常声明都被报成 `static declaration of 'xxx' follows non-static declaration`、`declaration of 'ARM_JOINT_ID' with no linkage follows extern declaration`、`expected declaration or statement at end of input`（这条最终在文件末尾连报 5 次）。`BALANCE/system.h` 的 49 条、`BALANCE/balance.h` 的 21 条同理。
2. 因此第二步把**那 5 个 Keil `__asm {}` 块**中和掉（把 RVDS `portmacro.h` 复制到临时目录，仅把 `__asm { … }` 替换成 `{ }`，把该临时目录放在 `-I` 最前；**工程文件一个字节都没动，临时目录用完即删**），命令行其余部分逐字不变。
   第二次运行：**8 条诊断，其中 `arm.c` / `arm.h` 0 条**。剩下 8 条全部来自 `CORE/`，是纯粹的 64 位宿主噪声：`core_cm4.h` 的 `__FPU_USED` 重定义 1 条、`core_cmFunc.h` 的未用参数 1 条、`arm_math.h` 的指针↔整数宽度转换 6 条。

**结论：归属 `arm.c` / `arm.h` 的诊断数为 0。**

> 中途这一项并非一开始就是 0：第二次运行曾剩 1 条 `arm.c:337: warning: unused parameter 'pvParameters' [-Wunused-parameter]` —— 这是**真诊断、不是级联**（`-Wextra` 下必然出现）。已按惯用法加 `(void)pvParameters;` 消掉，而不是靠过滤把它藏起来。Keil 默认不报这条，所以加它不影响工程。

**括号配平（`arm.c`）**

| 计数 | 原始字节 | 去掉注释与字符串字面量后 |
|---|---|---|
| `{` / `}` | 43 / 43 | 43 / 43 |
| `(` / `)` | 155 / 155 | 140 / 140 |
| `[` / `]` | 6 / 6 | 5 / 5 |

另做了**嵌套深度**检查：从 0 开始逐字符累计 `{}` 深度，过程中从未变负、结束时为 0 —— 即不存在"多一个右括号"也凑出相等数量的情况。（原始计数含注释里的括号，所以两列的 `()` 数不同；结论以第二列为准。）

**宿主可测性：Task 7 有一处，且它恰好是唯一一处无法宿主编译又属"正确性"的东西**

底座块本身不可宿主编译（`arm.c` → `system.h` → FreeRTOS/STM32，且它直接调 `lx_set_mode()` / `lx_read_pos()`）。但块里有一条**调用顺序**是正确性而非接线：

- `arm_track_step()` 必须在 `lx_set_mode()` **之前**。反过来的话，盲区积分积的是这一拍刚发出去、实际只生效了一个零头的速度，**每次采样都多算一整拍**，盲区里系统性超前。
- `ArmBase.prev_v` 的回写必须在 `lx_set_mode()` **之后**（写的是"刚发出去的那一拍"，下一拍才该积它）。

这两条是**调用点的性质**：`arm_track_step()` 自己的 T5 用例（`blind integration`）钉的是"积分入参、并把入参存进 `prev_v`"，把它单独测一万遍也看不见调用点写反。所以本任务把底座块用 `/* ==== ARM_BASE_TICK_BEGIN/END ==== */` 夹起来，在 `_gen_arm_test.py` 里**对真实字节**做结构断言（不是新增 ok 用例，ok 计数仍是 86）：

- 先做冒烟断言（切片非空、含 `arm_need_probe` / `arm_field_to_deg` / `arm_base_vel` / `ArmHold` / 状态门），这是 "切片必须非空且内容正确" 那条老教训。
- **先剥注释再判位置**：否则后续任何一次"在注释里提了一句 `lx_set_mode`"都会让位置断言误判。断言读的是剥掉 `/* … */` 之后的代码。
- 三条位置断言：`arm_track_step` 在 `lx_set_mode` 之前；存在 `ArmBase.prev_v = (float)v_cmd` 这个回写；回写在 `lx_set_mode` 之后。

**变异测试（改字节 → 跑生成器 → 逐字节还原后 sha256 校验）**

| 变异 | 结果 | 被哪条断言抓住 |
|---|---|---|
| M1 在 `arm_track_step` 之前插一次 `lx_set_mode` | **CAUGHT** | `the base tick commands the servo BEFORE it integrates` |
| M2 把 `prev_v` 回写挪到 `lx_set_mode` 之前 | **CAUGHT** | `the base tick records the commanded speed before sending it` |
| M3 整行删掉 `prev_v` 回写 | **CAUGHT** | `the base tick never records the speed it commanded` |
| M4 删 `ARM_BASE_TICK_BEGIN` 标记 | **CAUGHT** | `the base tick begin marker (a slice boundary) is gone from arm.c` |
| M5 删 `ARM_BASE_TICK_END` 标记 | **CAUGHT** | `the base tick end marker (a slice boundary) is gone from arm.c` |
| M6 把 `arm_need_probe` 门改成 `if(0)` | **CAUGHT** | `missing the probe gate` |

还原后 `arm.c` sha256 = `ea2299b9ade7564f1c169ded4bbf2a053e47878b5cb7e66d1bdf6322d4478fad`，与改动后基线**逐字节相同**。

**上板验证（Task 10 执行，本任务无法自动验证）**：底座上电后自动慢转、找到有效区后停住；`RosArmFb.status` 的 bit1（`ARM_ST_BASE_KNOWN`）置位；`pot_raw` 落在 `[60,940]` 内；`pot_raw` 一直读不到时 5 拍（约 100 ms）后转 `ARM_ST_BASE_FAULT` 并停转（用拔掉 ID6 的信号线复现）。

**与计划文本的偏差（4 处，逐条给理由）**

1. **补了 INIT_HOME 的入口边沿处理** —— 计划 Step 1 的代码**不可重入**：`home_read_fail` 是函数局部量、且只在"读成功"时清零，Task 8 的 `ROS_ARM_FLAG_REHOME` 把 `stage` 掰回 `INIT_HOME` 时它还带着上一次把状态机打进 FAULT 的那个计数（5），于是**重寻零会在第一拍就再次 FAULT** —— 这正是"整条路径静默失效"那一类。同时计划里没有任何地方在重新寻零时清 `ARM_ST_BASE_KNOWN`，`0x56` 会在寻零期间谎报"底座角度已知"。处理：在 `switch` 之前用 `prev_stage` 做边沿检测，进入 `INIT_HOME` 的那一拍重置 `home_read_fail`、置 `ARM_ST_HOMING`、清 `ARM_ST_BASE_KNOWN|ARM_ST_BASE_FAULT`。`prev_stage` **在 `switch` 之前采样**（写在之后的话记下的是刚跳过去的新 stage，边沿永远不触发）。`ArmBase` 由调用方 `arm_base_init()` 重新初始化（`INIT_POSE` 与 Task 8 的重寻零路径都调了），入口块**故意不代它决定方向提示**。
2. **`ArmHold` 初值写成 1**（计划是裸 `static u8 ArmHold;`）—— Task 8 的运行时块每拍都会先 `ArmHold = 0;` 再按急停/失联置位，所以初值**进不了最终固件**；但在 Task 7 落地、Task 8 未落地的中间态里，初值 0 会让刚寻完零的底座立刻朝全零的 `ArmTargetPos` 闭环（也就是朝 field 0 猛转）。取 1 表示"保持"，语义不变。
3. **加了 `(void)pvParameters;`** —— 见上，为把 `-Wextra` 的真诊断清掉。
4. **补了 `arm_cal.h` 那句注释里的时长** —— 计划写的是 "~4.6 s"（bound 135/4600 时代的数字），按定稿 160 预算走完是 267 tick = 5333 ms，已改为 "~5.3 s" 并注明 267 tick。同一类"注释里的常数过期"在本计划里已经出过多次。

**计划文本的问题（Task 7 范围内发现的，其中 2 条要 Task 8 定夺）**

- **P1 `ARM_HOME_READ_FAIL_MAX` 缺失**（见交付物表）：Task 5 的计划文本有、实现里没有，说明"计划文本 → 文件"这一步漏抄了一个宏，而它只在 Task 7 才被引用，所以潜伏了两个任务。
- **P2 `INIT_HOME` 不可重入**（见偏差 1）。计划把"可重入"当成需要执行者自己发现的事项，建议直接写进计划文本。
- **⭐P3 Self-Review 第 4 条最后一句与它自己引用的 §7.9 规则矛盾。** 该句写"底座块（Task 7）**必须放在 5 轴广播之后**"，紧接着引用的规则却是「预置 5 轴 → **发底座速度** → 广播触发」。Task 8 Step 1 的代码把 `lx_move_start_all()` 放在第 3 段（`changed` 块内）、底座块在其后 —— 即**底座速度发在触发之后**。底座是电机模式：它的速度帧一到就开始动，另外 5 轴要等 `MOVE_START`，所以按 Task 8 现在的排法 6 轴会差约 1.4 ms（10 字节模式帧 + 6 字节广播，115200 下 ≈ 1.4 ms）起步。**本任务没有替 Task 8 改动它的代码**（Task 8 的运行时块此刻还不存在），只在底座块上方写明了 §7.9 的顺序要求、并把底座块放在运行期区域；**Task 8 执行者需要把广播从 `changed` 块里拆出来，把底座块夹在"预置"和"触发"之间，同时让底座块每拍都跑**（它不能只在目标变化时跑）。
- **P4 `ARM_ST_BASE_FAULT` 一旦置位就再也清不掉（Task 8 的问题）。** Task 8 Step 1 第 2 段把 `ArmBase.state == ARM_BASE_FAULT` 也算进 `ArmHold`，而重寻零的前置条件是 `&& !ArmHold` —— 于是**唯一能重寻零的机制，恰好在最需要它的时候被自己关掉**：底座一 FAULT 就只能断电重启。ESTOP / 失联 / 未使能这三条确实应该挡住重寻零，底座故障这条需要 Task 8 单独定夺（例如让 `ROS_ARM_FLAG_REHOME` 越过底座故障保持，但仍受前三者约束）。
- **P5 `arm_base_last_sign` 永远不会被写（Task 8 的问题）。** Task 8 Step 2 声明了它并给了 `arm_base_last_sign()` 访问器，注释说"读数有效时记录进 RTC 备份寄存器"，但**全计划没有任何一处给它赋值**。今天无害（`ARM_POT_WRAPS 0` 时 `arm_search_dir_pick()` 根本不看 hint），可一旦 M1 实测翻成"回绕型"，这个方向提示就是一个恒等于 1 的假值 —— 应当在寻零成功或运行期 resync 成功时写入，或者干脆删掉。
- **P6（次要）底座块读的是 `RosArm.t_ms` 而不是快照 `ArmTime`。** Task 8 在临界区里把一帧拷进 `ArmPos`/`ArmTime`/`ArmFlags`，5 轴路径用的是 `ArmTime ? ArmTime : 500`，底座块却直接用 `RosArm.t_ms`。两者都是一条指令的 u16 读（不会撕裂），但"新帧恰好在两次读之间落地"时，底座会拿**另一帧**的 `t_ms` 配这一帧的 `ArmTargetPos`。本任务保留计划写法（`ArmTime` 要到 Task 8 才存在，现在引用它会编不过），建议 Task 8 顺手改成 `ArmTime`。
- **P7 `ARM_CAL_MEASURED` 是"声明了但不读"的假闸门。** `arm_cal.h` 开头写"arm_task 在它为 0 时拒绝给底座闭环"，但全工程（含计划全文）只有 `#define`、没有任何读取处 —— 也就是说**现在就会用没实测过的标定给底座闭环**。Task 10 会把它置 1，所以功能上没损失，但这句话与代码不符，属于同一类"文档承诺、代码没有"的缺陷。建议要么实现（`stage = INIT_ABSENT` 或至少不上电寻零），要么把这句删掉。

### Task 8

**交付物**

| 文件 | 动作 | 状态 |
|---|---|---|
| `BALANCE/arm.c` | 运行期块：1 取帧（临界区快照）→ 2 两道闸门 + 重寻零 → 3a 预置 → 3b 底座速度 → 3c 广播触发 → 4 回读 → 5 健康 → 6 透传 → 7 上行 | 完成 |
| `BALANCE/arm.c` | 文件级 `ArmPos` / `ArmTime` / `ArmFlags` / `ArmCanRehome` / `ArmChanged` / `s_arm_base_last_sign` + 访问器 `arm_base_last_sign()`（`ArmTargetPos` / `ArmHold` 沿用 Task 7 的声明，**未重复声明、未移动底座块**） | 完成 |
| `BALANCE/arm.c` | Step 2b：`case INIT_HOME:` 顶部的 `ARM_CAL_MEASURED` 闸门（**改的是 Task 7 的代码**，计划要求如此） | 完成 |
| `BALANCE/arm.c` | 底座块内 `RosArm.t_ms` → 快照 `ArmTime`（Task 7 的 P6）；resync 成功后刷新 `s_arm_base_last_sign`（Task 7 的 P5） | 完成 |
| `BALANCE/arm.h` | 加 `s8 arm_base_last_sign(void);` 声明 | 完成 |
| `_ros_tooling/_gen_arm_test.py` | 新增 3a/3b/3c 顺序断言 + 单点计数自检（**不新增 ok 用例**，ok 计数仍 86） | 完成 |
| `_ros_tooling/_arm_tests.c`、`HARDWARE/lx_servo.c`、`HARDWARE/usartx.c`、`HARDWARE/usartx.h` | 未改动（本任务一个字节都没碰；三个驱动的 sha256 见下，供后续任务比对） | — |

**sha256**

| 文件 | before | after |
|---|---|---|
| `BALANCE/arm.c` | `ea2299b9ade7564f1c169ded4bbf2a053e47878b5cb7e66d1bdf6322d4478fad` | `03008ecd3a1050e5029205f8178183faf847c535939009345c35d652f1425037` |
| `BALANCE/arm.h` | `831367b3133da717f37243ed5546ea0d8b9113899e14b715ce9e5d33362a646f` | `1b8708ecc017d53b734a1380470146a7b3085933e1d5d99504e025940e6ecaa7` |
| `BALANCE/arm_cal.h` | `81968a1e84cec8eb2b68eda85bfb1b6466a1dc2604b9daabd5d157939519e494` | 不变 |
| `_ros_tooling/_gen_arm_test.py` | `dfc7e0254eac0981c764f2c206b292aaa88a47323a26b59df1f4335fb5f66f0a` | `4915cd094bae7019d5b8f82d0c83e66cb5915ab8a7d2c299afe926241d0ed513` |

`arm.c` 的 before 不是"改动前记下来的"，而是**把本次 8 处编辑逐条反向还原后重算**得到的：还原脚本对每一对（现在的串 → 原来的串）做 `count == 1` 断言，还原结果再过 13 项结构自检（纯 ASCII、纯 LF、无 `static s16 ArmPos` / `static u16 ArmTime` / `arm_base_last_sign`、Task 7 的 `Task 8 declares ArmPos / ArmTime /` 注释仍在、两条 Task 8 挂钩注释仍在、`case INIT_HOME:` 后紧跟 `{`、两个静态量仍落在第 316–317 行、`ARM_BASE_TICK` 两标记仍在、`arm_base_vel(&ArmCal, err, RosArm.t_ms)` 已还原）。结果与 Task 7 记录里的 after `ea2299b9…` **逐位相同** —— 即交到 Task 8 手上的 `arm.c` 与 Task 7 留下的字节一致。生成器的 before 同样是从 after 里删掉本次新增的 36 行重算，与 Task 7 记录的 `dfc7e025…` 逐位相同。

**未触碰文件的当前 sha256**（Task 8 前后一致，供后续任务比对）：

| 文件 | sha256 |
|---|---|
| `HARDWARE/lx_servo.c` | `24d4b2e959eb9732f0ed195499087d208ebefcf86c314474879246a503a6392f` |
| `HARDWARE/usartx.c` | `d8a7fcca484eeab018a38c75f678c7cc516187170b961372a213dbf7f9a277b8` |
| `HARDWARE/usartx.h` | `751e56d55581aa949b6f23d3258f3ad88b60330c706e34ad436f9745b46492e6` |
| `_ros_tooling/_arm_tests.c` | `388c7d285a3a827ba5b3d331ccf1f3ca02832515be34802de3d1a86a42e321ff` |

**arm.c 改动区域（全文件 8 个 hunk，`diff` 逐块核对过）**

1. 文件级静态量：在 Task 7 的 `ArmTargetPos` / `ArmHold` 两侧补 `ArmPos` / `ArmTime` / `ArmFlags` / `ArmCanRehome` / `ArmChanged` / `s_arm_base_last_sign`，并加访问器。
2. `arm_task()` 开头：`tick` / `arm_fb_div` / `health_j` / `probe_j` / `last_p[]` / `first_frame` / `i`，以及 `last_p[] = -1` 的初值。
3. `case INIT_HOME:` 顶部插入 `ARM_CAL_MEASURED` 闸门（Step 2b）。
4. 运行期块前半：`tick++` / 1 取帧 / 2 两道闸门 + 重寻零 / `ArmTargetPos` 取值 / 3a 预置。
5. 底座块（`ARM_BASE_TICK_BEGIN … END` 之间）：`ArmTime` 替换 `RosArm.t_ms`；resync 成功后刷新方向提示。**块的字节位置未动**。
6. 底座块正上方：加 3b 的说明注释；底座块自身注释里那句"Task 8 has to split…"改写为已落实的陈述。
7. 运行期块后半：3c / 4 / 5 / 6 / 7。
8. `ArmHold` 初值那段注释改写成成品固件语义（原注释写的是 Task 8 落地前的过渡语义，已过期）。

**底座块相对 3a / 3c 的字节位置（Step 2c）**

以 `arm.c` **剥掉 `/* … */` 之后**的文本为参照：

| 位置 | 绝对偏移 | 相对运行期块 |
|---|---|---|
| 运行期块起点 `if(stage != INIT_READY) continue;` | 10971 | 0 |
| 3a `lx_move_wait(` | 12961 | +1990 |
| 3b `arm_need_probe(&ArmCal`（底座块内） | 13338 | +2367 |
| 3c `lx_move_start_all()` | 14281 | +3310 |

即 `ARM_BASE_TICK_BEGIN/END` 整块**夹在预置与广播之间**，且不在 `changed` 门内 —— 每拍都跑（50Hz 速度环）。三个调用点在运行期块内各只出现一次（生成器里的 `count() == 1` 断言守着这一点）。

**计划 Step 2c 给的那行检查是永真的（本次最重要的发现）**

计划原文：`assert _bt.index('arm_need_probe') < _bt.index('lx_move_start_all')`。实跑：

```
plan check    : index(arm_need_probe)=5436  index(lx_move_start_all)=8639  -> PASS
                (offset 5436 is the DEFINITION of arm_need_probe inside
                 ARM_LOGIC, not the 3b call site)
runtime block : 5541 bytes, from offset 10971
  3a lx_move_wait(          @ abs  12961  (rel   1990)
  3b arm_need_probe(&ArmCal @ abs  13338  (rel   2367)
  3c lx_move_start_all()    @ abs  14281  (rel   3310)
order         : 3a < 3b < 3c -> PASS
```

`arm_need_probe` 在全文件里第一次出现是**它自己的定义**（`ARM_LOGIC` 段内，偏移 5436），`lx_move_start_all` 第一次出现是 `INIT_POSE` 里那次（8639）—— 两者都在运行期块（10971）之前，所以这个断言**无论 3b 挪到哪里都成立**。已把 3a/3b/3c 的顺序检查限定在运行期块内（`arm_need_probe(&ArmCal` 用带参前缀锚住调用点而不是定义），并加单点计数，写进 `_gen_arm_test.py` 作为硬断言（不打印 ok 行，ok 计数仍 86）。

**宿主测试** `python _ros_tooling/_gen_arm_test.py`

```
gcc -std=c99 -O2 -Wall -Wextra -o ...\_ros_tooling\_arm_host_test.exe ...\_arm_host_test.c
gcc: 0 warnings
ok   MOVE_TIME_WRITE id1 500/1000ms     55 55 01 07 01 F4 01 E8 03 16
ok   MOVE_TIME_WAIT_WRITE id1           55 55 01 07 07 F4 01 E8 03 10
...
ok   probe always wanted while homing
ok   no probe while faulted
ok   no probe while unknown

ALL PASSED (0 failures)
exit=0
```

gcc stderr = **0 字节**，`warning:` 计数 **0**，FAIL 计数 **0**，ok 计数 **86**（与 Task 6/7 相同；本任务按计划不新增宿主 ok 用例 —— `arm_task` 在 `ARM_LOGIC` 段之外，宿主编译不到它，能覆盖的只有下面这类结构断言）。

**`arm.c` 无法宿主编译 —— 替代验证（`gcc -fsyntax-only`，沿用 Task 7 的配方）**

命令行逐字沿用 Task 7 记录（include 路径与宏取自 `USER/WHEELTEC.uvprojx`），差别只有临时头文件。三次运行：

| 运行 | 临时头 | 诊断总数 | 按归属文件 | `arm.c` / `arm.h` |
|---|---|---|---|---|
| A | `portmacro.h`（5 个 `__asm{}` 置空） | 16 | arm_math.h 6、core_cm4.h 1、core_cmFunc.h 1、**portmacro.h 8** | **0** |
| B | 再加 `portFORCE_INLINE __forceinline` → `__inline__` | 11 | arm_math.h 6、core_cm4.h 1、core_cmFunc.h 1、portmacro.h 3 | **0** |
| C | B + 临时 `usartx.h` 把 `ARM_DEBUG_PASSTHROUGH` 置 1 | 11 | 同 B | **0** |

运行 A 的 8 条 CORE/ 噪声与 Task 7 记录里那 8 条**逐条相同**（`__FPU_USED` 重定义 1、`fpscr` 未用参数 1、`arm_math.h` 指针↔整数 6）。

**过滤方法（两步，第二步是关键）**

1. 按**诊断归属文件**归类：用 `^(?P<file>(?:[A-Za-z]:)?[^:]+):(?P<line>[0-9]+):(?P<col>[0-9]+): (error|warning|fatal error):` 逐条抽取，`note:` 行（无归属文件）排除。
   > ⚠️ **Task 7 记录里的"第二次运行 8 条"少算了 8 条，原因是它用的正则 `^([^:]+):` 匹配不了 Windows 绝对路径** —— `C:\Users\…\portmacro.h` 的盘符冒号会被当成字段分隔符，凡是带绝对路径打印的诊断**全部被静默丢掉**。那 8 条恰好是相对路径打印的（`CORE/…`、`BALANCE/…`），所以两边的"8"能对上。本轮把盘符写进正则后，临时 `portmacro.h` 里那 8 条（运行 A）/ 3 条（运行 B）才现形。
2. 那 8 条（运行 A）**不是 arm.c 的问题**，是 Keil 专有语法在 gcc 上的报错，归属临时头文件：`static portFORCE_INLINE` 展开成 `static __forceinline`（gcc 不认 `__forceinline`）→ 5 条 `multiple storage classes`；把 `__asm{}` 置空后函数体为空 → `ulBASEPRI` 未用参数 1 条、`ulNewBASEPRI` 未用变量 2 条。运行 B 把 `portFORCE_INLINE` 一并映射掉后只剩 3 条未用告警，运行 C 证明**打开透传后 `arm.c` 仍然 0 诊断**（`#if 0` 里那段代码平时连编译器都碰不到，靠 C 才验到）。

**结论：三次运行里归属 `arm.c` / `arm.h` 的诊断数都是 0。** 临时目录用完即删，工程文件一个字节未动（`portmacro.h`、`usartx.h` 的 sha256 与运行前一致）。

**括号配平（`arm.c`）**

| 计数 | 原始字节 | 去掉注释后 |
|---|---|---|
| `{` / `}` | 64 / 64 | 64 / 64 |
| `(` / `)` | 247 / 247 | 221 / 221 |
| `[` / `]` | 32 / 32 | 29 / 29 |

嵌套深度检查：从 0 起逐字符累计 `{}` 深度，过程中最小值 0、结束值 0 —— 不存在"多一个右括号凑出相等数量"的情况。（`arm.c` 无 `//` 注释也无字符串字面量，所以剥注释即等于剥掉全部非代码文本。）文件 618 行 → 881 行。

**变异测试（改字节 → 跑生成器 → 逐字节还原后 sha256 校验）**

| 变异 | 结果 | 被哪条断言抓住 |
|---|---|---|
| M1 广播 `lx_move_start_all()` 挪到底座速度之前（就是那 1.4ms 错位） | **CAUGHT** | `the base speed is sent after the broadcast: …` |
| M2 底座块整块挪到 3a 预置之前 | **CAUGHT** | `the five joints are preset AFTER the base speed is sent, …` |
| M3 广播复制成两处 | **CAUGHT** | `the runtime block holds 2 sites for 3c, …` |

还原后 `arm.c` sha256 = `03008ecd3a1050e5029205f8178183faf847c535939009345c35d652f1425037`，与改动后基线逐字节相同。

另外用 M2 同一份变异体直接对比**计划给的那行检查**与生成器里的加强检查：

```
mutated arm.c: the base speed now runs BEFORE the five joint preset
   plan=True   strengthened=False
   generator rc=1, caught by: AssertionError: the five joints are preset
   AFTER the base speed is sent, so the base starts moving before the joints
   it is meant to move with
```

即顺序真的被破坏时，计划那行检查仍然 PASS。**这条变异体是本次"测试必须能红"自检的核心证据。**

**上板验证（Task 10 执行，本任务无法自动验证）**：5 轴能跟着 `0xAC` 走、`0x56` 持续上报（实际约 25Hz，见 P3）、`RosArm.frames` 增长；`ARM_CAL_MEASURED` 为 0 时底座一动不动且 `status` 带 `ARM_ST_BASE_FAULT`；把宏置 1 后底座才寻零。急停/失联/未使能时 5 轴收 `MOVE_STOP`、底座收速度 0。

**与计划文本的偏差（5 处，逐条给理由）**

1. **`arm_base_last_sign` 变量改名 `s_arm_base_last_sign`（计划原文编译不过）。** 计划 Step 2 同时给出 `static s8 arm_base_last_sign = 1;` 与 `s8 arm_base_last_sign(void) { … }`。C 的函数与对象共用一个命名空间，这是 `error: 'arm_base_last_sign' redeclared as different kind of symbol`。访问器名被调用点 `arm_base_init(&ArmBase, arm_base_last_sign())`（计划 Step 1 第 2 段）钉住，所以改的是变量名，并在注释里写明原因。
2. **3a 的负值裁剪。** 计划写的是 `u16 p = (u16)ArmPos[i]; if(p > 1000) p = 1000;`。`ArmPos[i]` 是 s16（0xAC 帧里就是 int16），`-1` 先转 u16 变成 65535、再被上限裁成 **1000** —— 负值会跑到行程的另一端。Global Constraints 与设计文档 §12.2 第 6 条都要求"双向裁剪 0..1000"，而 `lx_move_wait()` 的形参是 `u16`，负值根本传不进去，**负的那一侧只能在 `arm.c` 的调用点裁**。已改为先判负再转：`if(ArmPos[i] < 0) p = 0; else p = (u16)ArmPos[i];`，随后照旧裁上限与夹爪 700。
3. **`ArmHold` 初值那段注释重写。** Task 7 写的是"Task 8 落地前初值 0 会怎样"，Task 8 落地后每拍都会重算，原话已过期。
4. **底座块注释里"Task 8 has to split its broadcast out…"改写为已落实的陈述**，并写明 `_gen_arm_test.py` 现在守着这个顺序（同一类"注释承诺、代码没做"的过期反向版本）。
5. **把 3a/3b/3c 的顺序检查从"计划给的那一行"换成限定运行期块的加强版**，理由见上（原检查永真）。顺带说明：`ArmTime == 0` 时 5 轴用 500ms、底座走 `arm_base_vel()` 内部的 `ts = 0.5f`，两条默认路径恰好同为 0.5 s —— 这一点是核对过的，不是巧合到需要改。

**计划文本的问题（本任务范围内发现的）**

- **⭐P1 Step 2c 的顺序检查永真（见上）。** `arm_need_probe` 与 `lx_move_start_all` 在 `arm.c` 里各有更早的出现点（前者是 `ARM_LOGIC` 段内的定义本身，后者是 `INIT_POSE` 里那次），所以 `index()` 比的两个偏移都在运行期块之前。这正是第 10 条自己命名的"文档/注释承诺了一件事，代码没做"的同族问题：**给某个量写了"必须/只有"这类断言，就必须有一条真会红的检查去守它**；写在计划里的一行 `assert` 如果不先证明它会红，等于没写。
- **P2 Step 2 的 `arm_base_last_sign` 命名冲突**（计划原文无法编译，见偏差 1）。
- **P3 `(ARM_RATE_HZ / 20)` 到不了 20Hz。** `ARM_RATE_HZ = 50` 时整数除法得 2 → 每 2 拍发一次 = **25Hz**，而设计文档 §4.4/§8 与计划行内注释都写"20Hz"。整数分频从 50Hz 出发不可能得到 20Hz（需要 2.5）。本任务按计划原文保留该表达式，只把真实频率写进注释；若确实要 20Hz，要么改 `arm_fb_div` 为计数器（每 5 拍发 2 次），要么调 `ARM_RATE_HZ`。
- **P4 `ROS_ARM_FLAG_REHOME` 只有电平语义，没有边沿语义。** `ArmFlags` 是"最新一帧的快照"，在下一次 `0xAC` 到来前一直有效，而重寻零的消耗只是把 `stage` 掰回 `INIT_HOME`。所以**只要主机把 bit0 一直置着，每次寻零结束后都会再寻一次**（寻零期间 `stage != INIT_READY`，不会叠加）。计划与设计文档都没说这一位要不要脉冲。当前 `ARM_CAL_MEASURED` 为 0，走到闸门即回 `INIT_READY`，所以今天无害；Task 10 置 1 之后若主机常置 bit0，底座会反复寻零。**本任务没有替它加边沿检测**（那会改主机契约），只在此记录。
- **P5 透传块平时不在编译器的视野里。** `#if ARM_DEBUG_PASSTHROUGH` 为 0，gcc 根本不解析那 20 行，所以"编译干净"这个证据对它是空的；而且 Task 4 Step 2 的代码把 `tmo` 同时当循环计数与 `lx_raw_exchange()` 返回值用（能跑，但读起来像笔误）。本任务用运行 C（临时把宏置 1）补上了编译验证，**没有改动那段代码**。

---

### Task 9

**状态：Step 1a、1、2、3 完成；Step 4（Keil Rebuild）未做 —— 本机没有 Keil，它是仍待人工执行的唯一一步，见下面「未完成项」。**

备份（首次写入之前）：`_ros_tooling/_backup_arm_task9/`，命名 `DIR__file`，共 5 个文件
`BALANCE__arm.c` / `BALANCE__system.h` / `USER__main.c` / `USER__WHEELTEC.uvprojx` / `HARDWARE__usartx.h`。
脚本：`_apply_arm_task9_rehome.py`（Step 1a 的 usartx.h 半边）、`_apply_arm_task9.py`（Steps 1–3）、
`_verify_arm_task9.py`（Step 3 的字节级证据）、`_syntax_arm_task9.py`（`-fsyntax-only` + 更正后的过滤器）、
`_rehome_edge_probe.c`（偏差 1 的控制流探针）、`_check_t9_anchors.py`（写入前的锚点预检）、
`_check_t9_uvprojx_nl.py`（uvprojx 锚点对 LF/CRLF 敏感的变异自检）。

**Step 1a 的 usartx.h 半边（字节级脚本，GBK + CRLF）**

```
ok  usartx.h documents 0xAC bit0 as edge triggered
    CRLF=348 LF=348 (no bare LF)
```

锚点 `assert count == 1` 通过；写入后 `assert new.count(b'\n') == new.count(b'\r\n')` 通过（348/348）。
裸 LF 断言做了变异自检：把新行写成裸 LF 后 `LF=347 CRLF=346`，断言确实变红。

**⭐ 偏差 1：计划 Step 1a 给的 snippet 不能修好它要修的那个 bug（已改写）**

计划原文把消费边沿的赋值放在 `if` **之后**，而 `if` 体内是 `continue`：

```c
        if((ArmFlags & ROS_ARM_FLAG_REHOME) && !(ArmFlagsPrev & ROS_ARM_FLAG_REHOME) && ArmCanRehome)
        {
            ...
            continue;                       /* <-- 跳到 while(1) 顶部，下面这行永远不执行 */
        }
        ArmFlagsPrev = ArmFlags;    /* consume the edge either way */
```

`continue` 跳回 `while(1)` 顶部，而顶部有 `if(stage != INIT_READY) continue;` —— 寻零期间根本走不到这里；
寻零一结束 `stage` 回到 `INIT_READY`，`ArmFlagsPrev` 仍是 0 而 `ArmFlags` 仍是快照里的 bit0=1
（没有任何东西会把它清掉），于是**同一个边沿再次触发**。这正是该补丁要消灭的无限寻零循环，
只是从"每次寻零结束立刻再寻"变成"仍然是每次寻零结束立刻再寻"。而且**主机只发一帧脉冲也一样**：
`ArmFlags` 是快照，一帧之后 bit0 一直是 1。

控制流探针 `_ros_tooling/_rehome_edge_probe.c`（`arm_task` 无法宿主编译，所以是同一循环骨架，不是固件副本）实测：

```
plan's literal ordering (consume after the if): 91 searches in 1000 ticks
consume before the if                        : 1 searches in 1000 ticks
RED  : the plan's ordering re-homes in a loop
```

**已改为**：把边沿判定与消费都提到 `if` 之前，用一个嵌套块保持 C89 的声明位置习惯
（`arm.c` 里所有声明都在块首，Keil 也按那个风格读）：

```c
        {
            u8 rehome = (u8)((ArmFlags & ROS_ARM_FLAG_REHOME) &&
                             !(ArmFlagsPrev & ROS_ARM_FLAG_REHOME));
            ArmFlagsPrev = ArmFlags;    /* consume the edge either way */

            if(rehome && ArmCanRehome)
            {
                ... stage = INIT_HOME; continue; ...
            }
        }
```

语义与计划声明的一致（"请求"而非"电平"，一次 0→1 边沿一次寻零，"consume either way"），只是把顺序摆对。
`_verify_arm_task9.py` 里加了一条**能红**的顺序断言（在真实字节上比 `ArmFlagsPrev = ArmFlags;` 与
其后第一个 `continue;` 的偏移）；变异自检：按计划原文重排后 `consume @28441 continue @28405` → **RED**。

> 与 Self-Review 第 11 条同族，但更重：11 条是"断言永真"，这条是**修复本身永假**。
> 共同点都是**没先证明它会红就写进计划**。

**Step 2（`python _ros_tooling/_apply_arm_task9.py`）**

```
ok  system.h includes arm.h            insert @1539, +18 bytes, 0 delete / 0 replace
ok  main.c starts arm_task             insert @2442, +104 bytes, 0 delete / 0 replace
ok  uvprojx gains lx_servo.c           insert @16143, +180 bytes, 0 delete / 0 replace
ok  uvprojx gains arm.c                insert @23057, +169 bytes, 0 delete / 0 replace
ok  all three files written; arm.c / usartx.h were done separately
```

`report()` 的"纯插入"判据是**可失败的**，已做变异自检：插入 → ok；等长替换、删除 → 各自变红（2/2 抓到）。
（该判据本轮改过一次：初版写的是 `after.count(before) == 1`，对整文件字节串恒为假，跑第一次就红了。）

**Step 3 的字节级证据（`python _ros_tooling/_verify_arm_task9.py`）**

> **⭐ 偏差 3：计划给的 `_apply_arm_task9.py` 不是幂等的，重跑一次会静默写重复条目。**
> 三个锚点（`#include "imu_task.h"`、`vTaskDelete`、`usartx.c` / `imu_task.c` 的 `<File>`）在**成功之后依然存在**，
> 所以第二次运行会**再加一份**：本轮实测 `system.h` 的 `arm.h` include 变 2 份、`main.c` 的 `xTaskCreate(arm_task` 变 2 份、
> uvprojx 的 `<File>` 44 → **48**（`lx_servo.c` / `arm.c` 各两条）。Keil 会把同一个 .c 编译两遍，链接期报重定义。
> **已从备份整体回滚并重做一次**，然后在脚本里加了三道"已存在就拒绝"的断言（`assert b.count(...) == 0`），
> 第二次运行现在以 `AssertionError: system.h already includes arm.h` 退出 1 且**不写盘**。
> 回滚重做后四个插入偏移与首次逐字节相同（@1539 / @2442 / @16143 / @23057），可证回滚干净。
> 教训：这类**一次性接入脚本**天然不幂等，而"再跑一遍看看"是执行者最自然的动作 —— 脚本必须自己守住，
> 不能指望执行者记得只跑一次。

| 文件 | 编码 | delta | insert | delete | replace | 行尾 |
|---|---|---|---|---|---|---|
| `BALANCE/arm.c` | ASCII + LF | +1423 | 10 | 0 | **1** | LF=905 CRLF=0（按设计纯 LF） |
| `BALANCE/system.h` | GBK + CRLF | +18 | 1 | 0 | 0 | LF=161 CRLF=161，裸 LF=0 |
| `USER/main.c` | CRLF（任何编码都解不开） | +104 | 1 | 0 | 0 | LF=71 CRLF=71，裸 LF=0 |
| `USER/WHEELTEC.uvprojx` | ASCII + 裸 LF | +349 | 2 | 0 | 0 | LF=672 CRLF=0（按设计纯 LF） |
| `HARDWARE/usartx.h` | GBK + CRLF | +84 | 1 | 0 | 0 | LF=348 CRLF=348，裸 LF=0 |

**纯增加（0 delete / 0 replace）的是 4 个，不是 3 个**：`system.h`、`main.c`、`uvprojx`，
外加 `usartx.h` —— 因为新注释是把旧行当**前缀**接长（`…re-home the base` → `…re-home the base (EDGE: …)`），
diff 出来是纯插入。只有 `arm.c` 有 1 个 replace opcode，**2 字节**：

```
context before : b'_ESTOP;\n\n        /* re-home on request, gated on ArmCanRehome - NOT on ArmHold */\n        '
OLD replaced   : b'if'
NEW replaced   : b'{\n            u8 rehome = (u8)'
context after  : b'((ArmFlags & ROS_ARM_FLAG_REHOME) && Arm'
```

即被改动的既有字节只有 `if` 这两个字符（连同它后面那行注释改成多行注释仍算 insert），其余全是新增。

**uvprojx XML（Keil 构建的替代检查）**

```
ok  xml.etree.ElementTree.parse() accepted the file
    <File> entries: 44 -> 46  (delta +2, want +2)
    lx_servo.c   in <Group> HARDWARE   path=..\HARDWARE\lx_servo.c  ok
    arm.c        in <Group> Balance    path=..\BALANCE\arm.c  ok
```

> 两个坑：① Keil 把条目嵌在 `Group > Files > File`，直接 `group.findall('File')` **找不到任何东西**，
> 会静默报"missing"（本人先踩了一次）；② 条目结构本身就是 `GroupName` + `Files`。

**⭐ 偏差 2：计划 Step 3 的期望值是错的。** 计划写 `b.count(b'lx_servo.c')` 期望 **1**、`b.count(b'arm.c')` 期望 **1**。
但一个正常条目会把文件名写两遍（`<FileName>` 与 `<FilePath>`），所以正确加入后是 **2**，缺失时是 **0** ——
**1 是不可能的值**。这条 grep 无法区分"加对了"和"没加"，作者按预期的 1 去核会对不上而误以为是缺陷。
已改为 `<FileName>lx_servo.c</FileName>` 计数 == 1（可区分 0/1/2），并加 `<File>` 总数 44→46 的 delta 断言。

**宿主测试仍然全过（`python _ros_tooling/_gen_arm_test.py`）**

```
gcc: 0 warnings
ok   MOVE_TIME_WRITE id1 500/1000ms     55 55 01 07 01 F4 01 E8 03 16
...
ok   no probe while unknown

ALL PASSED (0 failures)
exit=0
```

gcc stderr = **0 字节**，`warning:` 计数 **0**，FAIL 计数 **0**，**ok 计数 86**。
`arm.c` 的补丁没有扰动它：`ARM_LOGIC` 段在补丁区域之外；Step 2c 加强版顺序自检的锚点
`if(stage != INIT_READY) continue;` 之后仍各只有一处 `lx_move_wait(` / `arm_need_probe(&ArmCal` / `lx_move_start_all()`，
三个 `count() == 1` 全部成立。

**`gcc -fsyntax-only`（Task 7 配方 + Task 8 的更正的过滤器）**

```
temp portmacro.h: 5 asm blocks neutralised, 1 __forceinline, 7 portFORCE_INLINE (project files untouched)
exit=0

diagnostics, corrected regex (drive letter in the file group): 11
     6  C:\...\CORE/arm_math.h
     3  C:\Users\29544\AppData\Local\Temp\_t9_syntax_74iss9dr/portmacro.h
     1  C:\...\CORE/core_cm4.h
     1  C:\...\CORE/core_cmFunc.h
diagnostics, Task 7 regex `^([^:]+):`                     : 0  <- silently drops absolute paths

arm.c / arm.h diagnostics: 0
SYNTAX CHECK OK: zero diagnostics attributed to arm.c / arm.h
```

11 条的归属与 Task 8 运行 B 的表**逐条一致**（arm_math.h 6、core_cm4.h 1、core_cmFunc.h 1、portmacro.h 3）。
**⭐ 计划 Self-Review / Task 8 记录里那条教训本轮再次复现，而且更极端**：Task 7 的正则
在本轮的输入上给出 **0** 条诊断 —— 不是少算 8 条，而是**把 11 条全丢了**，于是"arm.c 零诊断"这个结论
可以在一份实际有 11 条诊断的日志上被读出来。过滤器本身必须能证明它会红，否则它就是第 11 条那种永真断言的反面。
临时 `portmacro.h` 用完即删；工程文件未动（本轮只增加临时目录，不覆盖任何工程文件）。

**括号配平（`arm.c`，补丁后）**

| 计数 | 原始字节 | 去掉注释后 |
|---|---|---|
| `{` / `}` | 65 / 65 | 65 / 65 |
| `(` / `)` | 252 / 252 | 224 / 224 |
| `[` / `]` | 32 / 32 | 29 / 29 |

嵌套深度：结束 0、过程中最小 0。文件 881 行 → **905 行**。

**未完成项（必须由人做）**

- **Step 4：Keil Rebuild 没有做。** 执行本任务的环境**没有 Keil**，也没有 ARM 工具链。
  `gcc -fsyntax-only` 只能证明 C 语法与声明没问题，**证明不了** ARM 侧特有的问题
  （`__IO` 限定、标准外设库头文件、`__CC_ARM` 专有扩展、内联汇编、链接期符号可见性）。
  在 Keil 里打开 `USER/WHEELTEC.uvprojx` 并 **Rebuild，要求零 error 零 warning**，是**唯一**能验证
  工具链那一侧的动作，**仍是未执行的下一步**。若报 `undefined symbol`，先查 Step 3 那两个条目是否
  真的落在正确的 `<Group>`（上面的 XML 检查已确认落位）。
- **Step 1a 的上板验证**：把 bit0 常置发 `0xAC`，确认底座**只寻零一次**（列在 Task 10）。


### Task 10

```
2026-09-27 完成（当日多轮：实测 → 锚定方案会诊与执行 → 现场排障 → 验收 → 跟随扩展）。
完整记录与主机对接契约：docs/plans/2026-09-26-arm-bringup-record.md、
docs/plans/2026-09-27-arm-firmware-handoff.md（坑册）、docs/plans/2026-09-27-acceptance-checklist.md（验收）。
要点：M1–M7 实测回填（WRAPS=1、中位 500=车头、0.2308°/raw、KP=18、夹爪 600、SOFT_LIMIT=360）；
缺陷 #13/#14/#15/#16/#17 全部修复入档；探针 wrap_raw 修复 + --keepalive；
上电自锚（全周期）+ 自动回中 + 复位时长自适应；断链滑行与错圈残余为已接受行为（人工修复）。
宿主测试与双值语法全绿；M8 与 flash 持久化为可选遗留；后续工作在 ROS/任务编排端。
```

---

## Self-Review

**1. Spec coverage** —— 逐条对照设计文档：

| 设计文档条目 | 落在哪个 Task |
|---|---|
| §3 模块划分（lx_servo / arm / usartx / system / main / uvprojx） | Task 1、5、9 |
| §3.2 USART2 ISR 改为舵机专用 | Task 2 Step 1 |
| §4.1 下行 `0xAC`（20B，字段与偏移） | Task 3 Step 1 |
| §4.2 量纲（全 raw，底座有符号 ±1000） | Task 3 Step 1（`ROS_ARM` 注释）、Task 6（`arm_field_to_deg`/`arm_deg_to_field`） |
| §4.3 单帧语义（无模式标志位） | Task 3 Step 1；Task 8 Step 1 第 3 段（只在目标变化时广播） |
| §4.4 上行 `0x56`（22B） | Task 3 Step 1、Step 4 |
| §4.5 调试透传 `0xAD`/`0x57` | Task 4 |
| §5 舵机驱动层（帧格式、指令、收发模型、参数顺序、双向裁剪） | Task 1（组帧与 T1/T2）、Task 2（胶水） |
| §6 上电初始化（WAIT/TORQUE/PROBE/CONFIG/POSE/HOME） | Task 5 Step 3、Task 7 Step 1 |
| §7.1 底座状态机 | Task 5（枚举与 `arm_base_init`）、Task 6 Step 1 |
| §7.2 标定参数 | Task 5 Step 1（`arm_cal.h`）、Step 3（`arm_cal_load`） |
| §7.3 寻零算法 + 安全证明 | Task 6 Step 1（`arm_homing_step`）、Task 6 Step 3（T6） |
| §7.4 运行期跟踪（重同步 / 盲区积分 / 距边界才读） | Task 6 Step 1（`arm_track_step` / `arm_need_probe`）、Task 6 Step 3（T5） |
| §7.5 底座速度环（前馈+P，流式退化） | Task 6 Step 1（`arm_base_vel`）、Task 6 Step 3 |
| §7.6 与 5 轴的同步顺序（预置→底座速度→广播） | Task 8 Step 1 的 3a/3b/3c 三段 + Step 2c 的**加强版**顺序自检（见自检第 10.b、11 条：旧说法"底座块在广播之后"已于执行中更正） |
| §8 调度与时序预算（50Hz 任务、底座 50Hz、其余 20Hz） | Task 8 Step 1 |
| §9 故障处理（门槛、保持上力、不自动续动） | Task 8 Step 1 第 2 段、Task 8 Step 1 第 5 段 |
| §10 测试方案 T1–T6 | Task 1（T1/T2）、Task 3（T3/T4）、Task 6（T5/T6） |
| §11 待实测清单 M1–M8 | Task 10 Step 2–9 |
| §12.2 工程注意（双向裁剪、夹爪 ≤700、只关入口不删代码） | Task 2 Step 3（裁剪）、Task 8 Step 1 第 3 段（夹爪）、 Task 2 Step 1（ISR 注释） |

**未覆盖项（有意为之，不补）**：
- §12.1 的电气红线（电压、共地、J8 跳线、改 ID 时只接一个舵机）是**接线纪律**，不是代码，已在设计文档里，计划里不重复。
- §13 的"AS5600 外部编码器""动作组回放""云台 PWM 清理"都是明确的本轮非目标。

**2. Placeholder scan** —— 无 `TBD` / `TODO` / "类似 Task N" / "加适当的错误处理"。代码步骤全部给了完整代码块。唯一带 `(pending)` 的是「执行记录」小节，那是设计上要求执行者填写的运行输出，不是计划缺内容。

**3. Type consistency** —— 逐项核对跨任务的签名：

| 名字 | 定义处 | 使用处 | 一致？ |
|---|---|---|---|
| `lx_build(u8*,u8,u8,const u8*,u8)` | Task 1 Step 1 | Task 1 Step 2、Task 2 Step 3（经 `lx_send_prm1/4`） | ✅ |
| `lx_check(const u8*,u8,u8)` | Task 1 Step 1 | Task 2 Step 3（`lx_read_*`） | ✅ |
| `lx_pos_from_resp(const u8*)` → `s16` | Task 1 Step 1 | Task 2 Step 3 | ✅ |
| `LX_RX` / `LxRx` / `lx_rx_byte` | Task 2 Step 4 | Task 2 Step 2（ISR）、Step 3（读路径） | ✅ |
| `LX_QUERY(id,cmd,resp_len)` | Task 2 Step 3 | Task 2 Step 3 四个读函数 | ✅ |
| `ROS_ARM` / `RosArm` / `ros_arm_byte` | Task 3 Step 1 | Task 3 Step 2、Step 3（ISR）、Task 8 Step 1 | ✅ |
| `ROS_ARM_FB` / `RosArmFb` / `ros_send_arm_fb` | Task 3 Step 1 | Task 3 Step 4、Task 8 Step 1 第 7 段 | ✅ |
| `ARM_ST_*` 八个位 | Task 3 Step 1 | Task 5/7/8 各处 | ✅ |
| `ROS_RAW` / `RosRaw` / `ros_raw_byte` / `ros_send_raw_fb` | Task 4 Step 1 | Task 4 Step 2、Task 8 | ✅ |
| `ARM_BASE` 字段（`deg`/`prev_v`(deg/s)/`search_travel`/`search_ms`/`primed`/`hint`/`search_dir`(s8)/`search_reversed`/`in_valid`） | Task 5 Step 2 | Task 6 Step 1、Task 6 Step 3（T5/T6）、Task 7 Step 2 | ✅ |
| `ARM_CAL` 字段（含 `search_bound2_deg`/`search_max_ms`） | Task 5 Step 2 | Task 5 Step 3（`arm_cal_load`）、Task 6 Step 1、Task 6 Step 3（`cal_default`） | ✅ |
| `arm_homing_step/arm_track_step/arm_base_vel/arm_need_probe/arm_field_to_deg/arm_deg_to_field/arm_pot_deg/arm_pot_valid/arm_base_init` | Task 5 Step 2（声明）、Task 6 Step 1（定义） | Task 7 Step 1/2、Task 8 Step 1 | ✅ |
| `arm_search_dir_pick(c, raw, hint)` | Task 6 Step 1（static，`ARM_LOGIC` 段内） | Task 6 Step 1 的 `arm_homing_step`、Task 6 Step 3（`test_homing_direction_from_rail` 间接验证） | ✅ |
| `ARM_POT_WRAPS` / `ARM_SEARCH_REVERSE` / `ARM_SEARCH_MAX_MS` | Task 5 Step 1（`arm_cal.h`） | Task 6 Step 1（`#if`）、Task 6 Step 3（`cal_default`） | ✅ |
| `ARM_JOINT_ID` / `ARM_RESET_POSE` / `ARM_BASE_JOINT` / `ARM_GRIPPER_FIELD` / `ARM_GRIPPER_MAX` | Task 5 Step 2 | Task 5 Step 3、Task 8 Step 1 | ✅ |
| `ArmPos` / `ArmTime` / `ArmFlags` / `ArmTargetPos` / `ArmHold` | Task 8 Step 2 | Task 7 Step 2（底座块引用 `ArmTargetPos`/`ArmHold`）、Task 8 Step 1 | ✅ |
| `arm_base_last_sign()` | Task 8 Step 2 | Task 8 Step 1 第 2 段 | ✅ |
| `ros_put_u16` | `usartx.c` 既有 | Task 3 Step 4、Task 4 Step 1 | ✅ |
| `crc16_modbus` | `usartx.c` 既有 | Task 3/4 全部 | ✅ |
| `t_crc16`（宿主桩） | 宿主模板既有 | Task 3 Step 6 | ✅ |
| `getSysTickCnt()` / `F2T()` / `delay_us()` | 工程既有 | Task 2 Step 3、Task 5 Step 3 等 | ✅ |

**自检中发现并已在计划内修正的五处**（这些是写计划时算出来的，不是执行期风险）：

1. **Task 2 Step 3** 初稿引用了不存在的 `lx_wait_reply_after()`，已改成 `LX_QUERY` 宏。
2. **Task 6** 初稿的 `ARM_BASE.search_dir` 是 `u8`，`-search_dir` 会截断回原值（搜索永不反向），已改为 `s8`。
3. **Task 6** 初稿的 `arm_homing_step()` 用 `prev_raw` 初值 0 与第一帧读数做差 —— 回绕模式下会凭空判出一个跳变、把错误的角度当成绝对基准。已加 `primed` 标志。
4. **Task 7** 初稿把 `ArmBase.prev_v` 当速度字段用、且在发送后没有回写 —— 盲区积分会永远用上一拍的陈旧值，且 `prev_v` 在 Task 6 里是 deg/s、在 Task 7 里被当成 raw。已统一为 deg/s 并补上回写，同时把"积分必须在发指令之前"写成强制约束。
5. **⭐ Task 6 的搜索预算原本是错的，这是最重要的一处。** 初稿让截断型也走"首段 —— 反向"两腿搜索，推导下来矛盾：够得着窗口需要首段 ≥119.4°（标定偏低时还要更多），而不超 ±360° 线缆预算要求首段 ≤108°，在 ±25% 速度误差下**无解**；更糟的是若电位器彻底不响应，两腿搜索会把底座带到 224+135+280=**639°**，直接扯断线。
   修正基于一个原先被漏掉的物理事实：**截断型电位器的读数本身就指明了在哪一侧**（贴 1000 ⇒ 角度为正 ⇒ 往负搜）。于是
   - 方向不再依赖那个可能被手动掰坏的标志位 —— 这正是用户担心的场景，现在**从原理上不可能出错**；
   - 截断型下取消反向腿（`ARM_SEARCH_REVERSE 0`），搜不到就 `FAULT`；
   - 加一条与标定误差无关的**时间上限**（`ARM_SEARCH_MAX_MS`）作为真正的安全网，`T6` 的 `dead pot` 用例专门钉它。

   设计文档 §7.3 里"用标志位定方向"的说法已被这条更强的规则取代，已在设计文档中加注说明。

6. **Task 3 Step 5/6 原本是个反模式，Task 2 执行后才发现。** 初稿让 T3 在测试文件里"重写一份形状相同的状态机"（原文写着 *a trimmed re-issue of the firmware state machine*）。这正是项目记忆里警告的"副本会与被测物漂移"—— 测试通过不代表固件正确。Task 2 的 subagent 实际采用了"按标记逐字切片真实函数"的做法并证明可行，因此 T3 已改写为**切片真实的 `crc16_modbus()` + `ros_arm_reset/ros_arm_publish/ros_arm_byte`**，连 CRC 都用真的，并断言 `usartx.h` 注释里给出的权威向量 `crc16_modbus("123456789") == 0x4B37`。同时把 `test_arm_feedback_layout`（原本只是自造一帧自校验，信息量为零）换成**帧布局不变式断言**。

   > ⚠️ **Task 3 执行前注意**：`_ros_tooling/_gen_arm_test.py` 是 Task 1 建的，Task 2 改过一次。Task 3 要在其基础上继续加切片，先读现状再改，不要按计划文本里的旧片段覆盖。

7. **Task 3 的 `ros_send_arm_fb()` 抢字段（审查时由 subagent 发现并已修复）。** 初稿写的是

   ```c
   seq = (unsigned char)(RosArm.seq + 1);
   RosArm.seq = seq;
   ```

   但 `RosArm.seq` 是**下行**邮箱的字段，而 `ros_arm_publish()`（USART1 ISR）在每收到一个合法 `0xAC` 时就写 `RosArm.seq = s_arm_buf[17]`。**两个写者为两个不同目的抢同一个字节** —— 上报的 seq 既不单调、也不是宿主的 seq 回显，主机**根本无法用它判断反馈帧丢失**，也就是这个字段白放了。
   已改为 `ros_send_arm_fb()` 自持一个 `static unsigned char s_armfb_seq`（唯一调用者是 `arm_task`，无需临界区），ISR 继续把 `RosArm.seq` 当"收到的下行 seq"用。
   修正脚本 `_ros_tooling/_fix_armfb_seq.py`，并把**"裸 LF 检测"写成脚本里的断言**（见下条）。

   > 附带教训：该脚本第一版在三行注释结尾误用了 `\n` 而非 `\r\n`，往 `usartx.c` 里引入了 3 个裸 LF。已从备份回滚重做，并把 `assert new.count(b'\n') == new.count(b'\r\n')` 加进脚本作为硬失败 —— **这类"行尾污染"肉眼极难发现，必须靠断言而不是靠检查**。

8. **⭐ Task 4 的 `0xAD` 长度字节差一，整条 bring-up 透传 100% 不工作（执行时发现，最致命的一处）。**
   计划里的 C 解码器语义是"收 `len` 个字节、CRC 取 `buf[len-2..len-1]`"，即 **`len` = 帧总长**；但计划里的 Python `wrap_raw()` 组帧写的是 `6 + len(payload)`，**比实际帧长少 1**。
   已独立复现：payload 取 0/3/16 字节时，解码器全部判定 **CRC 失败 → 丢弃**。

   后果的严重性：M1–M5 那批实测（电机模式下电位器行为、速度标定、加速补偿）**只能**经这条透传链路跑，而 M1 直接决定底座算法能不能定稿。这条链路不通 = 整个项目卡在 Task 10。

   已修正：Python 侧 `6 + n` → **`7 + n`**，`ROS_RAW_MIN_LEN` **6 → 7**（最短合法帧是 7 字节；取 6 时解码器会读到帧外并把 CRC 低字节当成 `n`）。修正后 6/6 往返成功。

   > **教训**：协议的双向实现（发端/收端）必须用**同一份字节**做交叉验证，而不是两处各自"看起来对"。Task 4 Step 4 的跨语言交叉校验（Python 组帧 vs C 组帧）抓不到这个 —— 因为它只比了发端对发端。真正抓住它的是"把发端的字节喂进真实收端"。**后续任何新增帧类型，都要用「真实收端吃真实发端字节」的方式验证，而不是各测各的。**

9. **Task 6 的切片结束锚点没有带起始偏移（Task 5 执行时发现）。** 初稿写的是

   ```python
   s0 = arm_h.index('typedef struct\n{\n    float deg;')
   s1 = arm_h.index('} ARM_CAL;') + len('} ARM_CAL;')     # ← 无 offset
   ```

   `str.index()` 返回**第一个**匹配。`} ARM_CAL;` 这个字面量只要在结构体**之前**的任何注释/散文里出现一次，`s1` 就会落在 `s0` 之前，切片变成 **0 字节**，然后在 `must(arm_types, 'search_bound2_deg')` 上失败 —— **错误信息完全指不出原因**。Task 5 写 `arm.h` 时正好中招（它在一段解释性注释里引用了这个字面量），靠"写完后实跑一遍锚点"才发现。

   已改为 `arm_h.index('} ARM_CAL;', s0)` 并加 `assert s1 > s0`。

   > **教训（通用）**：**结束锚点必须相对起始锚点搜**，否则切片边界会随无关内容的增减而漂移。更一般地：**任何从源码里切片出来的测试资产，都必须有一条"切片非空且内容正确"的自检**，并且这条自检要在写入的当下就跑，而不是等下游 Task 来发现。上面 6 条里已经有两条（#6 手抄副本、#9 空切片）属于这个类别。

10. **Task 7 执行时发现的三处（两处在 Task 8 的文本里，已在计划内修正）。**

    a) **⭐ `ARM_ST_BASE_FAULT` 死锁。** Task 8 初稿把 `ArmBase.state == ARM_BASE_FAULT` 也算进 `ArmHold`，而重寻零的前置条件是 `&& !ArmHold` —— **唯一能救回故障底座的机制，恰好在最需要它的时候被自己关掉**，底座一 FAULT 就只能断电重启。
    已把闸门拆成两个**不同集合**：`ArmHold`（停止运动）= 急停 ∪ 失联 ∪ 未使能 ∪ 底座故障；`ArmCanRehome`（允许搜索）= 前三条。底座故障保持轴但放行重寻零。

    b) **⭐ 6 轴起步不同步。** Self-Review 自己写着"底座块必须在 5 轴广播**之后**"，而同段引用的 §7.9 规则是「预置 5 轴 → **发底座速度** → 广播触发」。底座是电机模式，速度帧一到就动，5 轴要等 `MOVE_START` —— 照旧文本写，6 轴差约 **1.4ms** 起步。
    已把 Task 8 Step 1 的第 3 段拆成 **3a 预置 / 3b 底座速度 / 3c 广播触发**，并加了一条"剥注释后比 `index()`"的顺序自检。

    c) **`ARM_CAL_MEASURED` 是一句空承诺。** `arm_cal.h` 注释写着"为 0 时 `arm_task` 拒绝闭合底座环"，但**全工程没有一处读它** —— 固件会拿未标定的占位常数闭环，还会对外声称角度"已知"。
    已在 Task 8 加 Step 2b，把它变成真的编译期闸门。

    > **共同模式（这是第 10 条了，值得命名）**：这三条都不是"算错"，而是**文档/注释承诺了一件事，代码没做**，或者**两条各自正确的规则拼在一起就矛盾**。单看任何一处都读不出问题，只有把 `ArmHold` 的定义和 REHOME 的前置条件放在一起、或者把 Self-Review 那句话和它引用的规范放在一起，矛盾才显形。
    > **对策**：凡是给某个量写了"必须/只有/拒绝"这类断言，就应该有一条**可执行的检查**去守它 —— 要么是断言，要么是测试。**只写在散文里的约束，等于没有约束。**

11. **⭐ Task 8 执行时发现：我 Step 2c 给的顺序检查是永真的（这条最值得记）。**

    原文写的是 `assert _bt.index('arm_need_probe') < _bt.index('lx_move_start_all')`。但实测：
    - `arm_need_probe` 的**首个出现 @5436 是 ARM_LOGIC 段里的函数定义**
    - `lx_move_wait` 首个出现 @8563 在 `INIT_POSE` 里
    - `lx_move_start_all` 首个出现 @8639

    三者**全部在运行期块之前**，所以这个比较**与底座块的位置无关，恒为真** —— 把底座块整块挪到预置之前，它照样 PASS。**我写了一条永远不会红的断言，而它的全部目的就是抓那一件事。**

    已替换为**加强版**：限定在运行期块的字节区间内比较，且每个调用点 `count() == 1`。Task 8 执行者对其做了变异自检（广播挪到底座速度之前 / 底座块挪到预置之前 / 广播复制成两处），**3/3 全部被抓**。

    同族问题还有：`(ARM_RATE_HZ / 20)` 整数除法得 2 → 实际上行 **25Hz** 而非文档写的 20Hz（保留表达式，文档已更正）；`arm_base_last_sign` 变量与访问器同名（C 里函数与对象共用命名空间，**原文编译不过**），已把变量改名 `s_arm_base_last_sign`。

    > **教训（与项目记忆"测试必须能红"同源）**：**写在计划里的一行 assert，如果不先证明它会红，等于没写。** 而且这类"永真断言"比没有断言更坏 —— 它占据了一个本该由真断言把守的位置，让后来的人以为那里已经被覆盖了。
    > **通用做法**：任何新写的检查，**先制造它应该抓到的那种缺陷，确认它变红，再撤掉缺陷**。Task 4/6/7/8 的执行者都在做变异自检，这一条应该从"我要求的"升级为"计划里默认要求的"。

12. **⭐⭐ Task 9 执行时发现：我 Step 1a 的补丁自己废掉了自己的修复（本轮最精妙的一处）。**

    我给的写法是：

    ```c
    if(rehome_condition && ArmCanRehome)
    {
        ...
        stage = INIT_HOME;
        continue;              /* ← 跳到 while(1) 顶部，跳过下面那句 */
    }
    ArmFlagsPrev = ArmFlags;   /* ← 永远到不了 */
    ```

    `continue` **跳过**了消费边沿的赋值。而 `ArmFlags` 是 0xAC 帧的快照、没有任何人清它，于是同一个 0→1 边沿在**下一拍**再次成立 —— **我加这段代码要消灭的那个无限寻零循环，原封不动地活了下来**。

    实测（探针 `_ros_tooling/_rehome_edge_probe.exe`，可复跑）：

    ```
      plan's literal ordering (consume after the if): 91 searches in 1000 ticks
      consume before the if                        : 1 searches in 1000 ticks
    ```

    **一个单帧脉冲就够触发**。已把边沿捕获与消费移到 `if` **之前**的嵌套块里（保持声明在块首，C89/Keil 友好），并对 `arm.c` 的真实字节加了一条会变红的顺序断言 —— 它对计划里的字面写法变红。

    > **教训**：`continue` / `break` / `return` / 异常是**"语句可达性"的断点**。凡把"清理/记账/消费状态"写在控制流跳转**之后**，它就可能永不执行 —— 而这类错误**读起来完全正常**，因为代码的字面顺序就是你以为的那个顺序。它没有编译错误、没有警告、静态审查也看不出来。
    > **通用做法**：**状态消费要写在跳转之前**，或干脆做成一个独立表达式块（如上）。且这类错误**只有真的跑一遍才会现形** —— 这正是"可执行探针"存在的理由。

    ---

    > **⭐ 至此计划共 12 处缺陷，其中 5 处有一个共同点：它们都不会报错。**
    > #6 手抄副本（漂移了也不红）、#9 空切片（`index()` 返回 -1 之外的静默空集）、#11 永真断言（永远不会红）、#12 不可达消费（不报错不警告）、#10 散文承诺（文档说了代码没做）。
    > **这类"静默失效"是这种逐任务执行方式的主要风险** —— 每一处单独看都合理，拼起来才错，而编译器、静态检查、人眼审查全都拦不住。
    > **唯一可靠的对策是让每一处约束都有一个会变红的可执行检查**，并且在写入的当下就制造缺陷验证它确实会红。这条应该作为本计划后续任何扩展的默认要求。

**4. 已知的执行期风险**（不是计划缺陷，但执行者会撞到）：

- `_apply_arm_task2.py` 的 NVIC 优先级替换用正则限定在 `USART2_IRQn` 后 200 字节内，**必须靠 `assert` 命中数确认**再看结果。这个文件里 `USART1_IRQn` 的写法相同，正则写松了会同时改掉 USART1 的优先级 —— 那会直接破坏已验证的底盘链路。
- `ros_arm_byte()` 之后插入的 `/* ==== ARM_TASK_BEGIN ==== */` 标记是给生成器当切片边界的。**它必须留在 `usartx.c` 里**，删掉会让 Task 3 Step 5 的生成器报 `markers not found`。
- Task 8 的 `arm_task` 里底座块（Task 7）**必须夹在"预置 5 轴"和"广播触发"之间**，而不是放在广播之后。
  `协议规范` §7.9 要求的顺序是「预置 5 轴 → **发底座速度** → 广播触发」：底座是电机模式，速度帧一到就开始动，另外 5 轴要等 `MOVE_START`。按旧文本把底座放在广播之后，6 轴差约 **1.4ms** 起步（10 字节模式帧 + 6 字节广播，115200 下）。Task 8 Step 1 的第 3 段已拆成 3a/3b/3c 三小段修正了这一点。
  **且底座块必须每拍都跑** —— 它是 50Hz 速度环，不能只在目标变化时跑。

