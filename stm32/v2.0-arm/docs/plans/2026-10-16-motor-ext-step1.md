# 第一步：外接驱动板 + 复用 6 个舵机口，让车能被动起来

日期：2026-10-16　　范围：固件 V2.0（机械臂版）
状态：代码已改完、已用 armcc 全量编译通过（0 error / 0 warning）；**尚未上电实测**

## 1. 目标与边界

第一步只要求一件事：**上位机发速度指令，车能按指令动起来**。所以

- 走原来的 ROS 链路不动（USART1 0xAA 速度帧 → `ros_apply_command()` → `Drive_Motor()`）。
- **开环**：`MOTOR_x.Target`（m/s）直接换算成占空比，不经过编码器 PI —— 编码器正在重新接线，PI 参数也是旧电机的，闭环现在只会把输出顶到满。
- 编码器读数、编码器供电、PID 标定全部留给第二步。

## 2. 接线（本轮固化在 `HARDWARE/motor_ext.h`）

| 舵机口 | MCU 引脚 | 驱动板信号 | 车轮 |
|--------|----------|------------|------|
| J1 | PC6 | PWM1（TIM8_CH1） | 右轮 |
| J2 | PC7 | INA1（GPIO） | 右轮 |
| J4 | PC8 | INB1（GPIO） | 右轮 |
| J5 | PC9 | PWM2（TIM8_CH4） | 左轮 |
| J6 | PB14 | INA2（GPIO） | 左轮 |
| J7 | PB15 | INB2（GPIO） | 左轮 |

J3 未接驱动板。驱动器 1 路 = 右轮、2 路 = 左轮。

驱动板逻辑（符号-幅值）：`INA=L INB=H` 正转、`INA=H INB=L` 反转、`L,L` 制动、`H,H` 脱机。
所以 **占空比 0 = 停**，而"停"在代码里是制动（两个 IN 都拉低），不是脱机。

## 3. PWM 配置

- TIM8，PSC=1 → 计数器 84MHz，ARR=4199 → **20.00 kHz**（驱动板要求 18–60kHz，且在听阈之上）。
- 两个 PWM 脚都在 TIM8（J1=CH1、J5=CH4），一套时基同时驱动。TIM12 的时钟也是 84MHz，所以万一将来有 PWM 脚要挪到 TIM12，同样的 PSC/ARR 还是 20kHz、同一套占空比刻度。
- 占空比刻度 0…4199 = 0…100%，软限幅停在 **4199 的 95%（3989）**，给驱动板自举留余量。

## 4. 改了什么

| 文件 | 改动 |
|------|------|
| `HARDWARE/motor_ext.c/.h` | **新增**：6 个口的引脚表、TIM8 20kHz 初始化、方向 + 占空比输出、上电斜坡、Brake/Coast |
| `BALANCE/system.c` | `Servo_PWM_Init()` / `Servo_PWM12_Init()` 改为 `MotorExt_Init()`（旧调用留在 `#else` 里） |
| `BALANCE/balance.c` | 控制拍改开环 `MotorExt_SetTargets(MOTOR_C.Target, MOTOR_A.Target)`；"不允许运动"时 `MotorExt_Brake()`；0xAB 舵机帧解绑；`Set_Pwm()` 不再写 `Servo_PWM` |
| `HARDWARE/motor.c` | `Servo_Set()` 变空操作（它写的 TIM8->CCR1/CCR4 就是两个 PWM 输出） |
| `HARDWARE/motor.h` | 舵机宏处注明 6 个脚已复用 |
| `USER/WHEELTEC.uvprojx` | 加入 `motor_ext.c` |

全部由 `MOTOR_EXT_ENABLE`（在 `motor_ext.h`，=1）控制，置 0 就整体回到出厂路径。
改动脚本：`_ros_tooling/_apply_motor_ext_step1.py`；每个被改文件都留了 `.bak-motorext` 备份（此目录不在版本控制下）。

**必须一起改的原因**：`Set_Pwm()` 每拍写 `Servo_PWM`（=TIM8->CCR4=PC9=左轮 PWM），`ros_apply_servo()` 每拍写 6 路 CCR。不处理的话，100Hz 上会把左轮占空比反复冲成 0/脉宽值。

## 5. 可调旋钮（都在 `motor_ext.h`）

| 宏 | 含义 |
|----|------|
| `MOTOREXT_DUTY_PER_MPS` | 4000：0.25 m/s → 24% 占空比，1.0 m/s 触到 95% 限幅。车太慢就调大，一冲就窜就调小 |
| `MOTOREXT_R_FORWARD_IS_POS` / `_L_` | 某个轮正转指令下反转 → 置 0 |
| `MOTOREXT_SLEW` | 每个 100Hz 拍的斜坡上限（420 = 10%/10ms → 0→满 100ms）。只限制加速，减速和停车立即执行，反向必经 0（先制动再反向） |

左右接反了不用改宏，改 `balance.c` 里 `MotorExt_SetTargets(MOTOR_C.Target, MOTOR_A.Target)` 两个实参的顺序即可（`FourWheel_Car` 下 A/B 带 `Vx - Vz` 项、C/D 带 `Vx + Vz` 项）。

## 6. 上电实测步骤（轮子离地）

1. 编译下载。上电瞬间两个通道就是**制动**状态（两个 IN 都低、占空比 0），车不会窜。
2. 上位机起链路，发一个很小的 `vx`（比如 0.1 m/s）。前提和以前一样：0xAA 帧里带使能位、使能开关 ON、电压 > 10V，否则 `ros_apply_command()` 会把速度清零。
3. 期望：两个轮子同向、向前（车头方向）转。
   - 只有一侧转 → 查那一侧的 INA/INB/PCB 接线（方向对但不动 = PWM 脚没接对）。
   - 某轮反转 → 把对应 `MOTOREXT_*_FORWARD_IS_POS` 置 0。
   - 车转向反了（`wz` 打正却右转）→ 换 `MotorExt_SetTargets()` 两个实参的顺序。
   - 完全不动 → OLED 第 0 行显示 `LOST` 就是链路超时（没收到 0xAA 帧），其余故障位（失能 / 欠压 / IMU）在 uplink 帧里。
4. 确认方向后逐步加 `vx` 到 0.3~0.5 m/s，听声音、看有没有堵转；不合适就调 `MOTOREXT_DUTY_PER_MPS`。
5. 断电前松手即可，松手就是制动。

## 7. 已知取舍 / 第二步

- OLED 上显示的 `MOTOR_A/B.Motor_Pwm` 在开环下恒为 0（那两个字段只有闭环才写）——想看实际占空比的话第二步顺手接上。
- 停车策略选的是**制动**而不是脱机：轮子被抱住、不会溜坡，代价是松手时有一声"顿"。要改成滑行就把 `MotorExt_Brake()` 换成 `MotorExt_Coast()`（`motor_ext.c` 里已经有）。
- 第二步：编码器回来后把 PI 接回 `Balance_task`，把它的输出（±16800 量程）换算成占空比送进驱动板，而不是直接从 `Target` 开环换算 —— 接口位置就是现在 `MotorExt_SetTargets()` 那一行。

## 8. 调试记录（2026-10-16 晚）：目标速度正常但六个脚量不到电平

**现象**：上位机链路正常（OLED 第 0 行有帧计数、`O N`、故障码 `--`、电压正常、第 2 行目标速度非零），
但驱动板 M+/M- 恒 0V，六个控制输入端在板子侧量也全是 0V。

**已经排除的**（都查到了证据，不是"可能"）：

| 检查 | 证据 |
|------|------|
| 编译产物是不是我们改的 | `OBJ/WHEELTEC.build_log.htm` 22:11 全量 Rebuild，有 `compiling motor_ext.c...`；`WHEELTEC.map` 里 `balance.o(i.Balance_task) refers to motor_ext.o(i.MotorExt_SetTargets)`、`system.o(i.systemInit) refers to motor_ext.o(i.MotorExt_Init)`；`Removing balance.o(i.Set_Pwm) / Incremental_PI_A..D / Limit_Pwm` 说明旧闭环确实被整段换掉了 |
| `MotorExt_Init()` 到底调没调 | 反汇编 `systemInit`（0x08008d82）：`Encoder_Init_TIM5` 之后**无条件** `BL MotorExt_Init`，位置和原来的 `Servo_*_Init` 一致 |
| 初始化代码对不对 | 反汇编 `MotorExt_Init`：`RCC_APB2PeriphClockCmd(2,1)`=TIM8、`RCC_AHB1PeriphClockCmd(6,1)`=GPIOB|GPIOC、MODER/AFR/PUPDR 逐字节核对、TIM8 PSC=1 ARR=4199、`TIM_CtrlPWMOutputs`(MOE) ✓ |
| 输出代码对不对 | 反汇编 `MotorExt_SetTargets` / `motor_ext_out` / `motor_ext_slew`：斜坡 420、限幅 3989、方向对（INB 拉高=正转）、CCR 写入 ✓ |
| 有没有别的模块抢这 6 个脚 | 全工程 GPIO 配置点清单里，除 `motor_ext.c` 只有 `motor.c` 的两个 Servo_init（已被链接器删除）；`system_stm32f4xx.c` 里那段写整个 `GPIOC->MODER` 的代码在 `SystemInit_ExtMemCtl()` 里，而 `DATA_IN_ExtSRAM` 是注释掉的，**不会被调用** |
| 任务是不是卡死 | `usart1_send` 有 100000 次 guard，不会永久阻塞；arm_task 每拍 `vTaskDelayUntil` 让出 CPU |

**结论**：代码（按 22:11 那份二进制）在"门限放开 + 目标速度非零"时不可能出现"六个脚全 0V"。
所以只剩两种可能：**(A) 板子里跑的不是这份镜像**；**(B) 六个脚被外部钉在低电平**（线插在 GND 那一排 / 短路），
因为拉低后 MCU 侧和驱动板侧量出来都是 0V。

**为此这次改动加了两件事**（都是为了让固件自己回答，而不是继续靠万用表）：

1. **去掉 `s_ch[]` 指针表**：原来 6 个脚的端口/引脚是运行时从 `.data` 初值里取的间接表，是唯一无法远程核对的一环。
   现在每个脚都像本工程其它硬件一样**字面写死**（`GPIO_ResetBits(GPIOC, GPIO_Pin_7)` 这种），表没了，少一类风险。
2. **开机自检 + OLED 显示**：`MotorExt_Init()` 在配置引脚之前，把 6 个脚逐个**拉高并读回自己的电平**
   （输出模式下 IDR 仍然反映引脚真实电平），结果锁存在 `MotorExt_PinTest()`；
   OLED 第 6 行在故障码和 `O N` 之间多显示 `EX nn`：`EX` = 确认跑的是这版固件，`nn` = 6 个脚的回读位图（十进制 0..63，63 = 六个都跟得上）。
   自检期间所有组合都是停机态（两个 IN 都低=制动；IN 拉高时占空比仍是 0），车不会动。

**怎么读 `EX nn`**：

| OLED | 含义 | 下一步 |
|------|------|--------|
| 没有 `EX` | 板子里不是这版镜像 | 重新 Rebuild + 下载（Keil ST-Link，或确认 FlyMcu 指向 `OBJ\WHEELTEC.hex`） |
| `EX 63` | 六个脚都能被 MCU 拉到高 | MCU 侧完全正常，问题在线材/驱动板输入（驱动板可能要 5V 逻辑、有 EN/STBY、或进了保护） |
| `EX` 其它值 | **为 0 的那些位就是被外部拉低的脚**（接线插到了 GND 那一排，或短路） | 按位反查是哪几个口，把那几根线挪到信号针上；一位都不用表 |
