# stm32 —— L150Pro 底盘 + 机械臂固件

WHEELTEC L150Pro 底盘与 6 轴 LX 总线舵机臂的 **STM32F407VE** 固件源码
（Keil MDK 工程，目标 `FreeRTOS`，输出名 `WHEELTEC`）。

这块板子是「底盘」这一端：它自己跑轮速环、姿态环，并把机械臂舵机总线挂在身上。
Pi（`pi/l150pro_ws/src/l150pro_driver`）通过串口发 `cmd_vel` / `servo_cmd` 给它，
它回里程计与舵机回读。**固件和 Pi 侧驱动是同一套协议的上下游，改了这边要同步看那边。**

## 版本

| 目录 | 版本 | 说明 |
|---|---|---|
| `v2.0-arm/` | 固件V2.0（机械臂版） | 当前在用。机械臂保持(急停/上位机断线)只在进入保持的前 3 拍发 `MOVE_STOP` |

（`固件V1.0（云台版）` 是云台版，没有进这个仓库。）

## 目录

```
BALANCE/      控制主体：balance.c 底盘、arm.c 机械臂、imu_task.c、
              robot_select_init.c 车型选择、show.c OLED、system.c 主循环
HARDWARE/     外设驱动：motor / encoder / adc / I2C / bsp_siic、
              ICM20948 IMU、lx_servo LX 总线舵机、usartx 串口、
              pstwo PS2 遥控、oled、key、lidar、stmflash、ws2811 灯、KD_Tree
SYSTEM/       delay / sys / usart（正点原子风格的基础层）
CORE/         CMSIS core + startup_stm32f40_41xxx.s
FWLIB/        STM32F4xx 标准外设库
FreeRTOS/     FreeRTOS 内核（RVDS 移植）
USER/         main.c、stm32f4xx_it.c、Keil 工程 WHEELTEC.uvprojx
docs/plans/   设计/实施/验收/交接文档（人写的）
更新记录.txt   固件改动记录
```

## 编译与烧录

- 用 **Keil MDK** 打开 `USER/WHEELTEC.uvprojx` 直接编译（器件 STM32F407VE）。
- `keilkilll.bat` 是清理编译中间产物的脚本。
- 编译产物落在 `OBJ/`——**不入库**（见下）。
- 烧录后记得确认 `Robot_Select()` 钉住了车型与几何参数；「自洽但错」的标定看里程计是发现不了的。

## 不入库的东西

`.gitignore` 里钉了（判据与仓库其他地方一致：**脚本/IDE「写」的不入库，人「写」的入库**）：

- `OBJ/`、`*.o`、`*.crf`、`*.d`、`*.lst`、`*.axf`、`*.hex` —— Keil 编译产物
- `*.uvguix.*` —— 各人 Keil 窗口布局（每台机器一份，没有共享价值）
- `.fang_cloud_tmp_dir/` —— 某网盘客户端的临时目录

另外**没有进来**的两块（2026-10-10 入库时按需排除，还留在原目录
`Desktop\raspot小车\3.STM32固件\固件V2.0（机械臂版）`）：

- `_ros_tooling/` —— 开发脚手架：27 个 `_apply_*.py` 补丁脚本、约 20 个 `_backup_*/`
  快照、7 个 `_*.exe` 宿主机测试、`monitor.py`。它记录的是「怎么改出来的」，不是固件本身。
- `*.uvguix.*` 与 `OBJ/`（同上）
