# WS2811 灯带 C542 跑通记录（NUCLEO-C542RC）

任务：把车上 F407 固件里的 WS2811 灯效引擎原样搬到 **NUCLEO-C542RC**
（STM32C542RCT6）上，换成 **12 V、120 LED/m、2 m（240 灯珠 / 80 像素）**
的新灯带先跑通，之后再移植回 F407。

> 当前状态：固件已编译、烧录并通过 SWD 自检。
> `ws_tx_ok` 持续递增、`ws_tx_err = 0`；SPI 帧内容已用调试器读回并与
> `ws_frame` → 亮度 → LUT 的期望值逐字节比对，**完全一致**。
> 还没接灯带，等你接上 12 V 后看实际颜色。

---

## 1. 接线（PA7 方案）

| 灯带 | 接到 |
|---|---|
| `DIN`（数据） | **PA7 = Arduino D11（CN5 排针）**，串一个 **330–470 Ω** 电阻靠灯带一端 |
| `GND`（12 V 电源负极） | **Nucleo 的 GND**（必须共地！） |
| `+12V` | 独立 12 V 电源正极（**不要**从 Nucleo 的 5 V/3.3 V 取电） |

建议在灯带电源入口并一个 **1000 µF / 25 V** 电解电容，热插拔时保命。
2 m / 240 灯全白约 4 A；固件 `WS_BRIGHT_MAX = 80`（约 31 % 亮度），
实际约 1.2 A，12 V / 3 A 电源足够。

**3.3 V 数据电平**：12 V WS2811 的 DIN 高门限有时接近 5 V 系统的 0.7×VDD，
3.3 V 驱动属于“能用但看批次”。如果出现首灯乱闪、颜色随机、偶尔整条不亮，
在 PA7 和灯带 DIN 之间加 **74HCT245 / 74AHCT125（5 V 供电）** 做电平转换，
5 V 可从 Nucleo 的 5V 引脚取（只给逻辑芯片，不给灯带）。

---

## 2. 引脚选择（都支持 SPI，均在排针上）

| 引脚 | Arduino 位置 | SPI 功能 | 说明 |
|---|---|---|---|
| **PA7** | **D11（CN5）** | SPI1_MOSI AF5 | **推荐/当前使用**，和旧 F407 固件同引脚，旁边就是 GND |
| PA4 | A2（CN8） | SPI1_MOSI AF4 | 备选 |
| PC1 | D4（CN9） | SPI2_MOSI AF5 | 备选，需要改用 SPI2 |
| PB15 | D0（CN9） | SPI1_MOSI AF6 / SPI2_MOSI AF5 | 备选 |
| PB5 | ST morpho 排针 | SPI1_MOSI AF5 | 备选 |

> **PB7 被固件占用为 SPI1_SCK（AF6）**。C5 的 SPI 主机必须先给 SCK
> 分配一个真实引脚才会真正把 FIFO 移出去；灯带不用 SCK，PB7 悬空即可。
> 这也是本次调试踩到的唯一大坑：只配 MOSI 时 `HAL_SPI_Transmit()` 会
> 一直等 TXP，50 ms 后超时，灯带全黑。

---

## 3. 运行现象

默认上电/复位后**不跑自检**，直接进入 **DEMO 0：2 秒青色呼吸**
（ROS 已连接、静止状态；亮度 5%↔45%，`WS_IDLE_BREATHE 1`；改成 0 就是
常亮）。需要重新检查接线/颜色顺序时，把 `ws2811.h` 里的 `WS_BOOT_TEST`
改回 1 再烧录，就会先跑 4 s 自检：

1. 整条 **红** 0.6 s
2. 整条 **绿** 0.6 s
3. 整条 **蓝** 0.6 s
4. **RGB PROBE** 2 s：灯带分三段，分别送“红/绿/蓝”
5. 熄灭 0.2 s，然后进入 **待机青色呼吸**

之后：

- **蓝色用户按键 B1（PC13）**：按住约 0.3 s 再松开，切到下一个演示档
- **绿色用户灯 LD1（PA5）**：1 Hz 心跳
- **ST-LINK 虚拟串口**：`115200 8N1`，每次切档打印一行，例如
  `[DEMO 8] TURN LEFT`

> **B1 消抖（2026-11-25 修正）**：本板 B1 是 **高电平有效、只有 10 kΩ
> 下拉、没有硬件 RC**（UM3615 §7.9）。台架上 12 V 灯带的电流脉冲会耦合到
> 地，旧的“单次上升沿”逻辑被毛刺触发，档位会自己乱跳（VCP 里可看到
> `DEMO 5 → 6 → 8 → 9 …`），灯带于是不停换效果/爆闪。现在要求电平
> **稳定 200 ms** 才认一次按键，并且必须稳定松开 200 ms 后才接受下一次；
> 常高/常低的毛刺最多只触发一次。按一下没反应就按住 0.3 s。

演示档位（与 F407 固件完全一致）：

| 档 | 名称 | 说明 |
|---|---|---|
| 0 | DEMO OFF | 真实输入：台架上固定为“ROS 已连接、静止”→ 青色呼吸 |
| 1 | IDLE CYAN | 待机青色呼吸 |
| 2 | WAIT ROS | 黄色呼吸（等上位机） |
| 3 | ROS LOST | 紫色闪烁 + 彗星绕圈 |
| 4 | LOW BAT | 橙色慢呼吸、限亮 |
| 5 | SPEED RAMP | 8 s 速度 0→0.75 m/s 往返，颜色随速度变化 |
| 6 | CRUISE GRN | 0.25 m/s 巡航色 |
| 7 | MAX RED | 0.75 m/s 满速红 |
| 8 / 9 | TURN LEFT / RIGHT | 琥珀色转向流光 |
| 10 | BRAKE | 红色快闪 |
| 11 | E-STOP | 8 Hz 红色爆闪 |
| 12 | REVERSE | 白色彗星倒车流 |
| 13 | NAV PROG | 深蓝进度 + 青色扫描头 |
| 14 | CALIBRATE | 一个白点沿灯带走 |
| 15 | ALL WHITE | 全白 |
| 16 | RGB PROBE | 三段红/绿/蓝，用于判定真实字节序 |

---

## 4. 颜色顺序判定

**2026-11-25 实测：这条新灯带是 RBG**（固件里 `WS_ORDER_RBG 1`，
和旧 60 灯/m 灯带同一族）。

判据是用 BGR 固件跑 16 档时观察到的整体错位：待机青色变紫、WAIT ROS 黄色
变青、ROS LOST 紫色变黄、MAX RED 红色变绿、RGB PROBE 显示绿/蓝/红——这些
正是 RBG 灯带收到 BGR 数据的结果。改成 RBG 后，PROBE 应显示 **红/绿/蓝**。

| 实际显示（前→中→后） | 真实字节序 | 改 `ws2811.h` |
|---|---|---|
| 红 → 绿 → 蓝 | **GRB** | `WS_ORDER_GRB 1`，其余改 0 |
| 蓝 → 红 → 绿 | **RBG** | 当前实测值，`WS_ORDER_RBG 1` |
| 绿 → 红 → 蓝 | **RGB** | `WS_ORDER_RGB 1`，其余改 0 |
| 红 → 蓝 → 绿 | **BRG** | 需要新增 BRG 分支 |
| 蓝 → 绿 → 红 | **GBR** | 需要新增 GBR 分支 |
| 绿 → 蓝 → 红 | **BGR** | `WS_ORDER_BGR 1`，其余改 0 |

---

## 5. 编译与烧录

工具链是本机 CubeMX2 自带的（无需装 Keil）：

```bash
# cbuild / arm-none-eabi-gcc / cmake / ninja 都在 CubeMX2 的 bundle 里
export PATH="/d/zaku/CubeMx2/resources/stm32cubemx-application/1.2.0/bundles/open-cmsis-toolbox/2.11.0+st.1/toolbox/bin:$PATH"
export PATH="/c/Users/29544/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH"
export PATH="/c/Users/29544/AppData/Local/stm32cube/bundles/cmake/4.2.3+st.1/bin:$PATH"
export PATH="/c/Users/29544/AppData/Local/stm32cube/bundles/ninja/1.13.2+st.1/bin:$PATH"
export GCC_TOOLCHAIN_14_3_1="C:/Users/29544/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin"

cd F:/STM32G542/LED_open-cmsis
cbuild LED.csolution.yml --context LED.debug_GCC+STM32C542RCT6 -j 8
```

烧录：

```bash
"C:/Users/29544/AppData/Local/stm32cube/bundles/programmer/2.22.0+st.1/bin/STM32_Programmer_CLI.exe" \
  -c port=SWD -w build/LED.debug_GCC/LED.elf -v -rst
```

看 VCP 日志：设备管理器里的 **STLink Virtual COM Port (COM16)**，
115200 8N1。每次复位会打印启动横幅和 `[DEMO 0] DEMO OFF`。

---

## 6. 实现要点

- **协议**：一个 WS2811 位 = 3 个 SPI 位，SPI 2.25 MHz（PCLK2 144 MHz / 64）
  - `0` → `100`：高 0.444 µs / 低 0.889 µs（规范 0.40 / 0.85）
  - `1` → `110`：高 0.889 µs / 低 0.444 µs（规范 0.80 / 0.45）
  - 帧周期 1.333 µs，比 1.25 µs 慢 6.7 %，在 WS2811 容差内
- 每像素 24 bit → 9 个 SPI 字节；80 像素 = 720 字节，再加 100 字节
  全 0 复位（356 µs 低电平，远大于 50 µs 要求），一帧 820 字节
- 台架版用 **阻塞发送**（`HAL_SPI_Transmit`，一帧约 2.9 ms），40 ms
  效果 tick 里完全看不出来；省掉了 C5 LPDMA 的 bring-up 风险
- **PA7/PB7 的 GPIO slew 用 MEDIUM，不要用 HIGH**：2026-11-25 实测，HIGH
  边沿在灯带 DIN 上振铃，WS2811 会把 13/255 的暗白读成很亮/乱闪；改成
  MEDIUM 后 5% 固定白立刻稳定。见下面手册分析
- 效果引擎（HSV、速度映射、状态机、idle/转向/刹车/急停/倒车/导航、
  demo 档位）与 F407 固件同一份代码，放在 `WS_LOGIC_BEGIN/END` 之间
- 为适配 240 灯珠，把按长度相关的常量按比例放大：
  - `WS_TURN_SIDE = WS_PIXELS / 4`（40 像素时是 10）
  - `WS_ORBIT_TAIL = WS_PIXELS * 3 / 40`（40 像素时是 3）
- 顺手修了旧代码里非 GRB 顺序时的淡入淡出通道映射（RBG 现在也正确）

---

## 7. 移植回 F407 的清单

> **已被第 9 节取代（2026-11-25）**：F407 不再直接驱动灯带，改为通过
> USART3 把"要什么灯"发给 C542，由 C542 本地渲染并输出。下面的清单仅
> 在坚持让 F407 直接驱动时参考。

硬件层不用动：旧固件 `HARDWARE/ws2811.c` 的 SPI1 PA7 + DMA2_Stream5 +
100 字节复位都是好的。逻辑层只改：

1. `WS_PIXELS`：40 → **80**（2 m × 120 LED/m ÷ 3 灯/IC）
2. `WS_TURN_SIDE`：10 → **20**（保持占整条 1/4）
3. `WS_ORBIT_TAIL`：3 → **6**（保持同样物理长度）
4. `WS_ORDER_*`：按新灯带实测顺序设置（和 C542 上测出来的一致）
5. 把本工程 `ws2811.c` 里 `WS_LOGIC_BEGIN..END` 的代码覆盖过去
   （包含 RBG 淡入淡出修正）；`ws2811_task()` 仍用 FreeRTOS 版本
6. 旧的 `ws_collect_overlay()`（轮速/电压/ROS 状态/转向/刹车）保持不变

> 注意：本工程为了台架方便加了启动自检、VCP 日志和 B1 切档，这些**不要**
> 带到车上；F407 上仍由 `show.c` / OLED 和真实状态驱动。

---

## 8. 注意事项

- 本工程的 `project/LED.cproject.yml`、`project/main.c` 和
  `project/generated/hal/stm32c5xx_hal_conf.h` 有**手工修改**。如果再用
  STM32CubeMX2 重新生成，这些修改可能被覆盖；重新生成后需要把
  GPIO/SPI 组件和 `USE_HAL_GPIO_MODULE` / `USE_HAL_SPI_MODULE` 等宏加回来。
- `ws_last_status / ws_last_err / ws_last_sr / ws_last_cfg1/cfg2 /
  ws_last_cr1/cr2 / ws_last_tx_ms` 是给 SWD 调试看的诊断变量，正常时
  `ws_last_status = 0xEAEAEAEA (HAL_OK)`、`ws_last_tx_ms ≈ 3`。
- 亮度上限 `WS_BRIGHT_MAX` 在 `ws2811.h`；当前为 **255/255（100%）**，
  台架实测稳定。全白约 **80 IC × 3 通道 × 16 mA ≈ 3.8 A @12 V**，
  12 V 电源建议 5 A 以上；长时间全白请确认电源、地线和灯带散热。
- 灯带 12 V 与 Nucleo 必须共地；DIN 先串 330–470 Ω，电源入口加电容。
- **WS2811 手册（Worldsemi V2.2）关键值**：
  - VDD 3.5–5.7 V；VIH ≥ 0.55×VDD（5 V 时 2.75 V）；VIL ≤ 0.3×VDD
    （5 V 时 1.5 V）；回差 0.35 V。
  - 3.3 V 数据只比 2.75 V 高 0.55 V，边沿振铃/过冲一掉下去就被误读，
    所以 HIGH slew 会乱闪、MEDIUM slew 就正常。
  - 时序：T0H 220–380 ns、T1H 580 ns–1 µs、T0L 580 ns–1 µs、
    T1L 580 ns–1 µs、RES > 280 µs、800 kHz。
  - 本工程 2.25 MHz / 3-bit：T0H≈444 ns（超上限）、T1H≈889 ns、
    T0L≈889 ns、T1L≈444 ns（低于下限）。芯片目前能容忍，但余量小；
    要更高亮度/更长线，建议加 74HCT245/74AHCT125（5 V）把高电平抬到
    5 V，并串 330–470 Ω。
- 演示档 10（BRAKE，150 ms 红闪）和 11（E-STOP，8 Hz 红爆闪）是**故意**
  的快闪效果，不是故障；待机档 0 是 2 s 周期的青色慢呼吸。
- 如果待机时仍然快速乱闪：先确认 12 V 负极和 Nucleo GND 真的连通，再检查
  DIN 的 3.3 V 电平是否够（首灯附近串电阻、必要时加 74HCT245/74AHCT125
  5 V 电平转换），以及灯带电源入口的 1000 µF 电容。

---

## 9. 架构调整：C542 作为独立灯控从机（2026-11-25）

F407 上的 WS2811 实时时序已经证明走不通（TIM1/TIM8 被电机 PWM 占用、
没有空闲的 DMA2 定时器、SPI2 只有 2.625 MHz）。现在把 C542 做成独立的
灯带控制器：**F407 只发语义状态，C542 本地跑效果引擎并输出 SPI**。

### 9.1 接线

| 信号 | F407 | C542 |
|---|---|---|
| 数据 F407→C542 | USART3_TX = **PC10** (AF7) | USART1_RX = **PA10 / D2** |
| 状态 C542→F407 | USART3_RX = **PC11** (AF7) | USART1_TX = **PA9 / D8** |
| 地 | GND | GND（必须） |

- 115200 8N1，3.3 V 直连，不用电平转换；建议每根信号线串 100–330 Ω，
  和 GND 双绞，远离电机线。
- F407 的 USART3 **不能**用默认的 PB10/PB11：那两个脚是 IMU 的软件 I2C。
- C542 的 USART2（PA2/PA3）继续做 ST-LINK VCP：日志照常输出，同时它的
  RX 也接同一个解析器，台架上可以用 PC 直接发帧（见 9.4）。
- C542 装在灯带入口附近，让 **UART 走长线，灯带数据线尽量短**。

### 9.2 协议（`project/ws_link.h`）

```
AA 55 | ver(0x01) | type | len | seq | payload[len] | crc8
crc8 = poly 0x07, init 0x00, 覆盖 ver..payload
```

STATE（F407 → C542，len=7，建议 25–40 Hz）：

| 字节 | 含义 |
|---|---|
| 0 | flags：b0 急停 / b1 低电 / b2 刹车 / b3 倒车 / b4-5 ROS 状态(0 等待,1 连接,2 丢失) / b6 左转 / b7 右转 |
| 1 | 亮度 0–255（受本机 `WS_BRIGHT_MAX` 硬上限约束） |
| 2-3 | 速度 cm/s，s16 小端（倒车为负） |
| 4-5 | 导航进度 0–1000‰，0xFFFF = 关闭 |
| 6 | 模式：0 = 全灭，1 = 正常 |

STATUS（C542 → F407，len=4，5 Hz）：`mode, link_ok, crc_err, ver`。

失效安全：**500 ms 没有有效 STATE 帧** → C542 不冻结最后一帧，直接进入
引擎自带的“ROS LOST”紫色跑马灯；B1 可以随时把控制权抢回台架演示模式。

### 9.3 台架行为

- 上电无主机：和以前完全一样，青色呼吸待机，B1 切 16 档演示。
- 收到第一帧 STATE：主机接管，B1 暂时无效，亮度用帧里的值。
- 主机停止 500 ms：紫色 LOST 提示，VCP 打印 `[LINK] lost`。
- 主机恢复：VCP 打印 `[LINK] up`，继续按主机状态渲染。
- 主机在时按 B1 无效；主机不在时按 B1 回到演示档。

### 9.4 用 PC 测试（不需要 F407）

`F:\STM32G542\tools\ws_link_test.py` 通过 COM16 发送 STATE 帧，依次演示
待机、转向、刹车、急停、全灭、掉线提示。直接运行：

```
python tools/ws_link_test.py COM16 115200
```

### 9.5 已知不确定项

- USART1 在 PA9/PA10 上按 **AF7** 配置（和 USART2/PA2/PA3 一致）。如果接
  F407 后收不到字节，把 `ws2811.c` 里的 `WS_LINK_USART1_AF` 改成
  `HAL_GPIO_AF_4` 再试（HAL 同时定义了 AF4 和 AF7 两种 USART1 路由）。
- F407 侧 PC10/PC11 是否引到排针需要按车上实际板子确认；备选 PD8/PD9。
