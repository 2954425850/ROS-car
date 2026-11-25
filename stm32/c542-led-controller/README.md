# C542 LED 控制器（灯带从机）

2026-11-25 起，车上的 WS2811 灯带不再由 F407 直接驱动，而是由这块
NUCLEO-C542RC 独立控制：

- F407 只通过 **USART3** 把"要什么灯"发过来（速度、转向、刹车、急停、
  低电、ROS 状态、亮度），C542 本地跑效果引擎并输出 SPI 时序。
- 协议：`project/ws_link.h`（AA 55 帧头 + CRC8，STATE 7 字节 / STATUS 4 字节）。
- 接线：F407 **PC10 → C542 PA10 (D2)**；F407 **PC11 ← C542 PA9 (D8)**；共地。
- **烧 F407 时必须断开 D8**：F407 的 ROM bootloader 同时启用 USART3
  (PC10/PC11)，C542 的 5Hz STATUS 帧会破坏 stm32flash 的自动波特率。

## 目录说明

- `project/` 和 `LED.csolution.yml` 是源码快照，和实际编译目录一致。
- **实际编译/烧录在** `F:\STM32G542\LED_open-cmsis`
  （STM32CubeMX2 + cbuild + ST-LINK，见 README_WS2811_C542.md）。
- 改动后请把源码同步回本目录，避免再出现"改的是旧固件"的情况。
- 台架自测：`tools/ws_link_test.py COM16`，通过 ST-LINK VCP 发 STATE 帧。

## 关键实现

- 灯带数据：PA7 / SPI1_MOSI（AF5），2.25 MHz，3 SPI bit = 1 WS bit。
- 颜色顺序：RBG（实测）。
- 像素：80（2 m × 120 LED/m ÷ 3 LED/IC）。
- 主机链路：USART1 PA9(TX)/PA10(RX) 115200，D8/D2。
- 失效安全：500 ms 没有 STATE 帧 → "ROS LOST" 紫色跑马灯；B1 可抢回演示。
- 亮度：取 STATE 帧里的值，硬上限 `WS_BRIGHT_MAX`。
