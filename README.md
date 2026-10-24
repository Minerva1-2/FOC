# FOC 无感电机控制工程

> - 平台：STM32G431RBT6（Cortex-M4F @ 170 MHz）
> - 控制：20 kHz 电流环 + 10 kHz 速度环 + SMO 滑模观测器 + PLL 锁相环
> - 启动：电流闭环强拖（I/F）→ 观测器接管 → 速度/电流双闭环
> - 应用：波轮（洗衣机）类负载，电位器 0~825 RPM 调速，USART1 + VOFA+ 实时波形

本工程是一个**不依赖转子位置传感器**的 PMSM/BLDC 磁场定向控制（FOC）实现。整条链路从
ADC 注入采样、Clarke/Park、电流 PI、SVPWM，到滑模观测器、锁相环、开环强拖与无感闭环切换、
梯形加减速、故障保护和上位机遥测均已实现，并在实机上完成测试（见提交记录
`550e6fa 完整功能已经实现，测试通过...`）。

---

## 目录

- [1. 项目概览](#1-项目概览)
- [2. 已实现功能](#2-已实现功能)
- [3. 架构详解](#3-架构详解)
- [4. 关键参数速查](#4-关键参数速查)
- [5. 编译、烧录与运行](#5-编译烧录与运行)
- [6. 移植教程](#6-移植教程)
- [7. 调试与观测](#7-调试与观测)
- [8. 已知限制与后续扩展](#8-已知限制与后续扩展)
- [9. 术语表](#9-术语表)

---

## 1. 项目概览

### 1.1 硬件平台

| 项目 | 配置 |
| --- | --- |
| MCU | STM32G431RBT6，LQFP64，Cortex-M4F，128 KB Flash / 32 KB SRAM |
| 系统时钟 | HSI 16 MHz → PLL（M=4，N=85，R=2）→ **170 MHz** |
| PWM | TIM1 CH1/CH2/CH3 → PC0/PC1/PC2（AF2），中央对齐，20 kHz |
| 电流采样 | ADC2 注入组：CH3(PA6)=A 相，CH17(PA4)=C 相；5 mΩ 下桥采样电阻 + 10 倍运放 + 1.65 V 偏置 |
| 母线电压 | ADC2 规则组 CH11(PC5)，分压系数 0.0449 |
| 调速输入 | ADC2 规则组 CH12(PB2)，电位器 0~3.3 V → 0~825 机械 RPM |
| 温度采样 | ADC2 规则组 CH15(PB15)，10 kΩ 分压 + NTC（B=3434） |
| 驱动使能 | PA0、PA1、PA2、PB0（MOTOR_A~D） |
| 状态灯 | PC13、PC14、PC15（当前上电常亮，状态指示预留） |
| 调试串口 | USART1 PB6(TX)/PB7(RX)，115200 8N1 |
| 电流环周期 | **50 µs（20 kHz）** |

> 注意：TIM1 的 `DeadTime = 0`，死区由外部栅极驱动芯片/驱动板负责。若换成需要 MCU 产生死区的
> 半桥驱动，务必在 CubeMX 中配置 `DeadTime`，否则存在直通风险。

### 1.2 控制指标

| 指标 | 数值 |
| --- | --- |
| 电流环频率 | 20 kHz（`CONTROLER_TIME = 50 µs`） |
| 速度环频率 | 10 kHz（`FOC_SPEED_DIV = 2`） |
| 额定调速范围 | 0 ~ 825 机械 RPM（0 ~ 5775 电 RPM，0 ~ 96.25 Hz 电频率） |
| 载波/电频率比 | 约 208 个控制周期/电周期 |
| 电流分辨率 | 约 0.0161 A/LSB（3.3 V/4096 ÷ (5 mΩ × 10)） |
| 母线电压量程 | 0 ~ 73.5 V（3.3 V ÷ 0.0449） |
| 保护阈值 | 母线 10~40 V，相电流 12 A |

---

## 2. 已实现功能

### 2.1 控制算法

- **无感 FOC（`MOTOR_CONTROLER = FOC_SENSORLESS`）**
  - Clarke / Park / 反 Park 变换
  - q 轴电流闭环（d 轴给定恒为 0，即 `Id = 0` 控制）
  - 速度环 + 电流环双闭环（速度环 10 kHz 分频执行）
  - SVPWM：逆 Clarke + 零序分量注入 + 占空比限幅 + CCR 映射
- **滑模观测器（SMO）**
  - 基于电机电压方程的前向欧拉电流观测器
  - 饱和函数（边界层 `SMO_PHI = 1.0`）抑制抖振
  - 反电动势一阶低通滤波
  - 滑模增益每拍按 `Vbus/√3` 自适应，母线变化时仍能覆盖反电势幅值
- **锁相环（PLL）**
  - 鉴相器 + PI 环路滤波器 + 压控振荡器
  - 估计转速一阶低通滤波
  - 支持方向系数 `Dir`，正反转均可跟踪
- **梯形加减速（TAccDac）**
  - 按 `TRA_ACC_SPEED = 1000 RPM/s` 对目标转速做斜坡限制
  - 输出加速/匀速/减速状态，供强拖电流策略和遥测使用
  - 角度发生器按电角速度积分出连续电角度，并做 `[0, 2π)` 归一化

### 2.2 启动与模式切换

- **上电零位校准**：驱动封锁状态下累计 1024 次 ADC 注入采样，求 A/C 相电流零点（约 51.2 ms）
- **开环强拖（I/F）**：角度由发生器给定，q 轴给固定电流拖动转子；同时并行运行 SMO + PLL，
  只观测不参与控制
- **强拖电流自适应**：根据转速、加减速状态、观测器收敛程度在 0.05~3.0 A 之间动态调整，
  低速段保证力矩、高速段降低电流
- **开环 → 闭环切换**：
  - 观测速度与给定速度同向且误差 < 15%，持续累计 `CheckCnt`
  - `CheckCnt > 40`、开环速度 > 4200 电 RPM、观测速度 > 2940 电 RPM 时切入闭环
  - 闭环后退回开环的阈值为 3000 电 RPM，与 4200 构成**滞环**，避免阈值附近抖动
- **切换平滑**：切入闭环瞬间记录“开环角 − 观测角”的偏差 `ThetaOffset`，之后每拍乘 0.995
  （时间常数约 10 ms），数十毫秒内平滑收敛到观测角；速度环积分预置为当前强拖电流，避免切换冲击
- **失步检测与自动重启**：开环匀速/减速阶段，若给定 > 800 电 RPM 而观测不到给定 30%，
  持续 100 ms 判定失步，清零斜坡与观测器状态后重新拉起
- **闭环异常保护**：闭环运行 1 s 后转速仍低于 2000 电 RPM，或观测转速与给定方向相反且幅值 > 200，
  判定观测器失效 → 清零斜坡与观测器状态、退回开环；5000 拍（250 ms）后清除故障标志，
  斜坡从 0 重新按加速度拉起

### 2.3 保护

- 母线欠压（< 10 V）、过压（> 40 V）
- 三相过流（任一项 > 12 A）
- 任一保护触发：立即封锁驱动使能、清零全部 PID/PLL 积分、强制回到开环
- 故障消失后自动重新使能；重新使能只在状态变化时写一次 GPIO

### 2.4 采样与信号处理

- 双电阻下桥电流采样（A、C 相），B 相由 `Ib = -(Ia + Ic)` 重构
- 上电零点校准（1024 次平均）
- 母线电压、电位器给定、NTC 温度换算
- 温度计算做 1000 次回调节流，避免每 50 µs 调用 `logf`
- 温度使用 B 值公式：`1/T = 1/T25 + ln(Rntc/R25)/B`

### 2.5 调试与可观测性

- **VOFA+ JustFloat 协议**，15 通道 float，10 ms 一帧，非阻塞逐字节发送
  （帧组装与推流在 `sysControl.c` 的 `DebugStream_Poll()`，`DBG_CH_NUM/DBG_FRAME_LEN` 在
  `userDef.h`，由 `main()` 主循环轮询调用）
- `printf` 重定向到 USART1（支持 MicroLIB，也提供非 MicroLIB 的半主机桩函数）
- 中断内逐拍累加电角度 `theta_acc`，解决主循环 10 ms 采样率不足导致的相位展开失败

### 2.6 预留/未实现

- `FOC_SENSOR`（有感算法）分支为空，仅保留宏和状态机入口
- 三颗 LED 仅在上电时点亮，尚未接入运行状态指示
- 单电阻采样、三电阻采样未实现（当前为 A/C 双电阻）
- 弱磁、MTPA、参数在线辨识未实现（表贴式电机，`Ld = Lq`，`Id = 0` 即可）

---

## 3. 架构详解

### 3.1 分层结构

```
┌────────────────────────────────────────────────────────────────┐
│  应用 / 调度层  sysControl.c                                   │
│    - 全局 motor 实例与参数初始化                               │
│    - 50 µs FOC ISR：保护、模式分派、强拖/闭环切换              │
│    - 状态机、切换策略、HAL 回调                                │
│    - VOFA+ 调试帧组装与非阻塞推流                              │
├────────────────────────────────────────────────────────────────┤
│  算法层  motorControl/algorithm/                               │
│    - foc.c       Clarke / Park / PID / 反 Park / SVPWM         │
│    - observer.c  SMO 滑模观测器 / PLL 锁相环                   │
│    - TAccDac.c   梯形加减速 / 电角度发生器                     │
├────────────────────────────────────────────────────────────────┤
│  硬件抽象层  motorControl/hardware/                            │
│    - driver.c    驱动使能 / LED / PWM 比较值写入               │
│    - sample.c    相电流 / 母线 / 电位器 / NTC 采样与换算       │
├────────────────────────────────────────────────────────────────┤
│  STM32Cube 层  Core/                                           │
│    - main.c / tim.c / adc.c / dma.c / gpio.c / usart.c / it.c  │
│    - 时钟、外设初始化、printf 重定向                           │
└────────────────────────────────────────────────────────────────┘
```

设计原则：

1. **配置与算法分离**：所有参数（`Ts`、电机参数、引脚、PID 增益）集中在 `sysControl.c` 的
   `motor` 全局实例初始化器中；算法模块只持有运行时状态和函数表。
2. **ops 函数表注册**：每个模块通过 `*_register()` 绑定一张 `const` 函数表，应用层通过
   `motor.xxx.ops->yyy(&motor.xxx)` 调用，便于替换算法实现（例如换用不同的观测器）。
3. **单一电机实例**：`Motor_t motor` 是全局唯一实例，所有模块数据挂在它下面；中断与主循环
   共享该实例（当前按“中断写、主循环读”设计，未加锁）。

### 3.2 目录结构

```
FOC/
├── FOC.ioc                     # STM32CubeMX 工程配置
├── Core/
│   ├── Inc/                    # HAL 配置、外设句柄声明
│   └── Src/
│       ├── main.c              # 时钟、外设初始化、主循环轮询调试推流
│       ├── adc.c               # ADC2 规则组 + 注入组 + DMA 配置
│       ├── tim.c               # TIM1 中央对齐 PWM 配置
│       ├── dma.c               # DMA1_Channel1 配置
│       ├── gpio.c              # 驱动使能、LED 引脚
│       ├── usart.c             # USART1 + printf 重定向
│       ├── stm32g4xx_it.c      # 中断向量
│       └── stm32g4xx_hal_msp.c
├── Drivers/                    # STM32G4 HAL + CMSIS
├── MDK-ARM/
│   ├── FOC.uvprojx             # Keil MDK 工程（ARMCLANG 6.15）
│   ├── .eide/                  # EIDE（VS Code）工程配置
│   └── FOC/                    # 构建产物（.axf/.hex/.map）
└── motorControl/
    ├── Inc/
    │   ├── userDef.h           # 全部可调参数、引脚映射、类型别名
    │   └── sysControl.h        # Motor_t、StrongDragToObs_t、模式枚举
    ├── Src/
    │   └── sysControl.c        # FOC ISR、状态机、切换策略、HAL 回调、VOFA+ 推流
    ├── algorithm/
    │   ├── Inc/                # foc.h / observer.h / TAccDac.h
    │   └── Src/                # foc.c / observer.c / TAccDac.c
    └── hardware/
        ├── Inc/                # driver.h / sample.h
        └── Src/                # driver.c / sample.c
```

### 3.3 ops 注册机制

每个模块在头文件中定义一张函数表，在源文件中定义 `const` 实例，并提供 `*_register()`：

| 模块 | 函数表 | 注册函数 | 入口 |
| --- | --- | --- | --- |
| FOC | `FOC_OPS_t`：`clarke`、`park`、`pid`、`speedPid`、`antiPark`、`svpwm` | `foc_register(&motor.foc)` | `motor.foc.ops->clarke(...)` |
| 观测器 | `SMO_OPS_t`：`SMO`；`PLL_OPS_t`：`PLL` | `smo_register(&motor.smo, &motor.pll)` | `motor.smo.ops->SMO(...)` |
| 斜坡/角度 | `TAccDac_OPS_t`：`TAccDac`；`Angle_OPS_t`：`AngleGenerator` | `TAccDac_register(&motor.tra, &motor.angle)` | `motor.tra.ops->TAccDac(...)` |
| 驱动 | `DRIVER_OPS_t`：`motorEnable`、`motorDisable`、`LedOn`、`LedOff`、`SetPWM` | `driver_register(&motor.driver)` | `motor.driver.ops->SetPWM(...)` |
| 采样 | `SAMPLE_OPS_t`：`MotorCurrent`、`MotorVbus`、`Washer`、`MotorTemp` | `sample_register(&motor.sample)` | `motor.sample.ops->MotorCurrent(...)` |

注册失败返回 `REGISTER_ERROR (-1)`，成功返回 `REGISTER_SUCCESS (0)`；`motorInit()` 会在
串口打印失败模块名并返回。

### 3.4 控制时序（核心）

```
TIM1 更新事件（20 kHz，50 µs）
        │
        ├─ HAL_TIM_PeriodElapsedCallback()
        │      └─ HAL_ADCEx_InjectedStart_IT(&hadc2)   // 软件触发注入组
        │
        ▼
ADC2 注入组转换完成（A 相 + C 相）
        │
        ├─ HAL_ADCEx_InjectedConvCpltCallback()
        │      1. 读 rank1/rank2 → g_adc_buf[0..1]
        │      2. MotorCurrent()：零位校准 / 换算 Ia、Ic、Ib
        │      3. 未校准完成 → 直接返回（驱动保持封锁）
        │      4. motorFocISR()：
        │           ├─ 保护检查（母线、过流）
        │           │     └─ 故障 → 封锁驱动、清积分、回开环、return
        │           ├─ 驱动使能（状态变化时写 GPIO）
        │           └─ 按 GeneralMode 分派：
        │                 OPEN_LOOP  → Strong_Drag_Current_Close_Loop()
        │                 CLOSE_LOOP → Strong_Drag_Smo_Speed_Current_Loop()
        │
        ▼
┌──────────────────── 开环强拖（OPEN_LOOP） ─────────────────────┐
│  TAccDac → 角度发生器 → theta = 开环角                         │
│  相电流 → Clarke → SMO → PLL（仅观测/判据）                    │
│  电流环：Id*=0，Iq*=OpenCurr（自适应 0.05~3 A）                │
│  Park → 电流 PI → 反 Park → SVPWM → CCR → TIM1                 │
└────────────────────────────────────────────────────────────────┘

┌──────────────────── 无感闭环（CLOSE_LOOP） ────────────────────┐
│  相电流 → Clarke → SMO → PLL                                   │
│  theta = theta_hat + ThetaOffset                               │
│  速度环（10 kHz）：speed_ref → Iq*                             │
│  电流环：Id*=0，Iq*=速度环输出                                 │
│  Park → 电流 PI → 反 Park → SVPWM → CCR → TIM1                 │
└────────────────────────────────────────────────────────────────┘
```

后台并行运行（不占用 FOC 中断）：

- **ADC 规则组**：连续转换 + DMA 循环，完成回调里更新母线电压、电位器、温度
- **主循环**：轮询 `DebugStream_Poll()`，每 10 ms 组一帧 VOFA+ 数据并逐字节非阻塞发送
- **SysTick**：1 ms，HAL 时基（优先级 15，低于 FOC 中断）

中断优先级：`TIM1_UP_TIM16_IRQn`、`ADC1_2_IRQn`、`DMA1_Channel1_IRQn` 均为 0（抢占优先级 0），
SysTick 为 15。FOC 链路（TIM 更新 → ADC 注入完成）实际由两个同级中断串行完成。

### 3.5 状态机

```
               上电
                 │
                 ▼
      ┌────────────────────┐
      │    MOTOR_CALIB     │  驱动封锁、零位校准
      └──────────┬─────────┘
                 │  1024 次采样完成
                 ▼
      ┌────────────────────┐
      │     OPEN_LOOP      │  电流强拖（开环角度）
      └──────────┬─────────┘
                 │  CheckCnt > 40 且速度达标
                 ▼
      ┌────────────────────┐
      │     CLOSE_LOOP     │  SMO + 速度/电流双闭环
      └──────────┬─────────┘
                 │  速度 < 3000 电 RPM 或观测器失效
                 └──────────────▶  退回 OPEN_LOOP
```

- `motor.GeneralMode` 是应用层模式，校准完成后同步到 `motor.StrongDragToObs.GeneralMode`
- 运行期由 `Strong_Drag_To_Observer_Cal()` 在 `OPEN_LOOP` / `CLOSE_LOOP` 之间切换
- `LastGeneralMode` 在每拍末尾更新，用于检测“刚刚切换”的边沿，做角度/积分预置

### 3.6 算法链路详解

#### 3.6.1 电流采样与零位校准

`sample.c`：

```c
Ia = (raw_a - offset_a) * SAMPLE_I_PER_LSB;
Ic = (raw_c - offset_c) * SAMPLE_I_PER_LSB;
Ib = -(Ia + Ic);
```

- `SAMPLE_I_PER_LSB = (3.3/4096) / (0.005 × 10) ≈ 0.0161 A/LSB`
- 上电后驱动封锁，累计 `SAMPLE_CALI_TIMES = 1024` 次注入采样求平均，得到 `offset_a/offset_c`
- `raw - offset` 在 `u16` 提升为 `int` 后相减，负电流可正确表示
- 母线：`Vbus = adc/4096 × 3.3 / 0.0449`
- 电位器：`washer_vlaue = adc/4096 × 3.3 × SPEED_K`，`SPEED_K = 250` → 0~825 机械 RPM
- 温度：每 1000 次规则组回调计算一次，避免高频 `logf`

#### 3.6.2 Clarke / Park / 反 Park

```c
Iα = Ia
Iβ = (Ia + 2·Ib) / √3

Id =  Iα·cosθ + Iβ·sinθ
Iq = -Iα·sinθ + Iβ·cosθ

Vα = Vd·cosθ - Vq·sinθ
Vβ = Vd·sinθ + Vq·cosθ
```

角度 `theta` 在开环时来自角度发生器，闭环时来自 `pll.theta_hat + ThetaOffset`。

#### 3.6.3 电流环 PI

位置式 PI，积分与输出分别限幅：

```c
err       = target - feedback;
integral += Ki * err * Ts;              // 积分限幅 ±output_limit
output    = clamp(Kp * err + integral); // 输出限幅 ±output_limit
```

- d 轴：`target = 0`，反馈 `Id_LPF`
- q 轴：开环时 `target = OpenCurr`，闭环时 `target = 速度环输出`
- `Id_LPF / Iq_LPF` 为一阶滤波，系数 `0.1`
- **输出电压限幅每拍按 `Vbus/√3` 覆盖**（SVPWM 线性调制区上限）；
  `PID_ID_LIMIT / PID_IQ_LIMIT` 只是上电初值
- 当前 `Kp = 0.2 V/A`、`Ki = 40`（已按 `Ts = 50 µs` 折算）

#### 3.6.4 速度环 PI

- 每 `FOC_SPEED_DIV = 2` 个电流周期执行一次（10 kHz，`Ts_speed = 100 µs`）
- 给定与反馈统一为**电 RPM**：
  - 给定：`tra.speedOut`
  - 反馈：`pll.omega_hat_PLF / 2π × 60`
- 输出为 q 轴电流给定，限幅 6 A
- 只在 `CLOSE_LOOP` 中调用；开环强拖阶段不执行，避免积分空累积
- 切入闭环瞬间，速度环积分/输出被预置为当前强拖电流 `OpenCurr`

#### 3.6.5 SVPWM

```c
Va = Vα;
Vb = -0.5·Vα + (√3/2)·Vβ;
Vc = -0.5·Vα - (√3/2)·Vβ;

Voffset = -0.5·(Vmax + Vmin);           // 零序分量注入
Va += Voffset; Vb += Voffset; Vc += Voffset;

duty = 0.5 + V/Vbus;                    // 钳位到 [0, 1]
CCR  = ccr_min + duty × (ARR - 2×ccr_min);
CCR  = ARR - CCR;                       // PWM2 模式取补
```

- `ARR = 4250`，`ccr_min = 350`，有效调制区间对称，`duty = 0.5` 时 CCR 正好为 `ARR/2`
- `Duty_To_Ccr()` 用 `!(duty > 0.0f)` 兜住 `NaN`，避免未定义行为
- `+0.5f` 四舍五入，消除截断带来的 -0.5 LSB 系统性偏置
- TIM1 配置为 **PWM 模式 2**，所以 CCR 需要取补；若改用 PWM 模式 1，把 `PWM_INVERTED` 置 0

#### 3.6.6 梯形加减速与角度发生器

`TAccDac.c` 实际是一个**转速斜坡限制器**（不是位置规划器）：

```c
speedTargetIncrement = TargetSpeed / 60 × Ts;          // 目标：每拍转过的机械圈数
speedIncrement       = accSpeed    / 60 × Ts²;         // 每拍允许的速度增量

// 当前增量向目标增量靠拢（加速/减速）
// state = 增量变大 ? ACCELERATE : (变小 ? DECELERATE : UNIFORM)
speedOutDir  = (speedChangeIncrement > 0) ? -1 : 1;    // 供 PLL 方向归一化
speedOut     = speedChangeIncrement / Ts × 60 × MOTOR_PAIRS;  // 电 RPM
```

角度发生器：

```c
theta += ElectricalAngleSpeedSet × Ts / 60 × 2π;       // 电角度积分
// 归一化到 [0, 2π)
```

> `TargetSpeed` 运行时每拍被 `sample.washer_vlaue` 覆盖，`TRA_TARGET_SPEED` 只是初值。
> 负的目标转速在代码中受支持，但当前电位器输入只产生 0~825 RPM。

#### 3.6.7 滑模观测器（SMO）

```c
// 1. 电流观测（前向欧拉）
i_hat += Ts × ( -Rs/Ld × i_hat + (U - e_hat_plf) / Ld );

// 2. 滑模切换（饱和函数，边界层 phi）
if      (i_hat - I >  phi) e_hat =  gain;
else if (i_hat - I < -phi) e_hat = -gain;
else                       e_hat = (i_hat - I) × gain;

// 3. 反电势低通滤波
e_hat_plf = α·e_hat + (1-α)·e_hat_plf;    // α = SMO_E_PLF_FACTOR = 0.1
```

- `gain` 每拍取 `Vbus/√3`，保证高于最高转速下的反电势幅值（`SMO_GAIN` 仅为初值）
- `U_alpha / U_beta` 使用上一拍的 `V_alpha / V_beta`，存在一拍延迟，属于正常现象
- `SMO_FC` 与 `wc` 当前未参与运算，滤波由 `SMO_E_PLF_FACTOR` 完成
- 表贴式电机按 `Ld = Lq` 处理

#### 3.6.8 锁相环（PLL）

```c
theta_err = Dir × (cos(theta_hat)·e_alpha + sin(theta_hat)·e_beta);
integral += Ki × theta_err;                       // 位置式，未乘 Ts
omega_hat = Kp × theta_err + integral;
omega_hat_PLF = β·omega_hat + (1-β)·omega_hat_PLF;
theta_hat += omega_hat × Ts;                      // 必须累加，不能赋值
// 归一化到 [0, 2π)
```

- `Dir` 每拍由 `tra.speedOutDir` 写入（正转速对应 -1，负转速对应 +1），用于匹配反电势与
  角度的符号约定；`PLL_DIR` 只是上电初值
- `Ki` 未乘 `Ts`（与参考工程写法一致），换控制周期时需重新整定
- 估计角速度经一阶滤波后用于速度反馈与遥测；角度积分使用未滤波的 `omega_hat`

#### 3.6.9 开环强拖与切换策略

`Strong_Drag_To_Observer_Cal()` 每拍执行：

| 步骤 | 内容 |
| --- | --- |
| 1 | `ThetaErr = ThetaRef - ThetaObs`，归一化到 [-π, π]（仅调试观测） |
| 2 | `SpeedErr = OpenSpeed - CloseSpeed`，一阶滤波 |
| 3 | 取开环/闭环速度绝对值 |
| 4 | 若 `\|OpenSpeed\| > 100` 且速度误差 < 15%，`CheckCnt++`，否则递减 |
| 5 | 闭环状态下累加 `CloseRunTime`（上限 40000） |
| 6 | 动态调整强拖电流 `OpenCurr`（见下） |
| 7 | 按开环速度方向确定电流符号 |
| 8 | 故障检测 + 切换判据 |
| 9 | `CheckCnt` 限幅到 [0, 41] |

强拖电流 `OpenCurr` 策略（初始 0.3 A，上限 3.0 A）：

| 情况 | 条件 | 动作 |
| --- | --- | --- |
| 1 | 非减速 && 开环速度 > 4200 && `CheckCnt >= 30` | 减小电流，下限 0.5 A |
| 2 | 非减速 && (开环速度 < 4200 或 `CheckCnt < 30`) | 增大电流，每拍 +0.01 A |
| 3 | 减速 && 2000 < 开环速度 < 4205 | 略微增大，防失步 |
| 4 | 减速 && 开环速度 < 2000 | 略微减小，下限 1.2 A（保证低速力矩） |

切换判据：

| 条件 | 判据 | 动作 |
| --- | --- | --- |
| A | 闭环 && 运行 > 1 s && 观测速度 < 2000 | `Errflag = 1` |
| A2 | 闭环 && 给定与观测方向相反 && 观测 > 200 | `Errflag = 1` |
| B | `CheckCnt > 40` && 开环速度 > 4200 && 同向 && 观测 > 2940 | 切入 `CLOSE_LOOP` |
| C | 开环速度 < 3000 | 退回 `OPEN_LOOP`（与 4200 构成滞环） |

> 用“观测速度是否跟得上给定速度”而不是角度差作为收敛判据，是因为开环强拖时转子必然滞后
> 给定磁场一个功角（实测 20°~80°），再叠加观测相位偏差，角度差会稳定停在 -60°~-80°，
> 永远无法收敛。速度观测精度高（实测误差 < 1%），更适合作为“观测器可信”的判据。

#### 3.6.10 故障、失步与恢复

- `Errflag != 0`（首次）：
  - 保存当前**目标**转速 `LastTargetSpeed`
  - `TargetSpeed / speedOut / 增量` 全部清零
  - 观测器复位，`theta_hat` 对齐开环角，`omega/integral` 清零
  - 退回 `OPEN_LOOP`
  - 5000 拍（250 ms）后清除 `Errflag` 并恢复 `TargetSpeed`
  - 注意：`TargetSpeed` 在每拍开头会被电位器值重新写入，因此实际表现是“斜坡从 0 重新
    按 1000 RPM/s 拉起”，而不是严格静默 250 ms；`Errflag` 期间模式保持在开环
- 开环失步：
  - 非加速阶段、给定 > 800 电 RPM、观测 < 给定 30%，持续 2000 拍（100 ms）
  - 清零斜坡输出、观测器、电流环积分，从 0 重新拉起（保留电位器目标）
- 保护：
  - 母线/过流故障立即封锁驱动、清积分、强制开环；故障消失后自动恢复
- 速度给定接近 0（`|speedOut| < 10` 电 RPM）：
  - 电流给定清零、全部积分清零、观测器反电势/速度/角度复位，避免静止通电发热

#### 3.6.11 调试遥测

`sysControl.c` 中的 `DebugStream_Poll()` 每 10 ms 组一帧 15 通道 JustFloat 数据
（由 `main()` 的主循环反复调用）：

| 通道 | 含义 | 单位 |
| --- | --- | --- |
| 0 | `tra.TargetSpeed` 目标转速（电位器阶跃） | 机械 RPM |
| 1 | `tra.speedOut / MOTOR_PAIRS` 斜坡输出 | 机械 RPM |
| 2 | `pll.omega_hat_PLF / 2π × 60 / MOTOR_PAIRS` 观测转速 | 机械 RPM |
| 3 | `tra.motor_state` 0=加速 1=匀速 2=减速 | — |
| 4 | `sample.Vbus` | V |
| 5 | `foc.Iq_LPF` | A |
| 6 | `sample.washer_vlaue` 电位器给定 | 机械 RPM |
| 7 | `StrongDragToObs.GeneralMode` 1=开环 2=闭环 | — |
| 8 | `StrongDragToObs.CheckCnt` | — |
| 9 | `StrongDragToObs.ThetaErr` | rad |
| 10 | `pll.omega_hat_PLF` | rad/s（电角速度） |
| 11 | `sample.motor_temp` | °C |
| 12 | `foc.duty_a` | 0~1 |
| 13 | `tra.speedChangeIncrement × 1e6` | 圈/拍 ×1e6 |
| 14 | `theta_acc` 中断内累加电角度 | rad |

帧格式：15 × float（小端，60 字节）+ 帧尾 `00 00 80 7F`（+Inf），共 **64 字节**。
115200 8N1 下约 5.6 ms 发完，10 ms 周期约有 4.4 ms 余量。

> 电角度不能在主循环里用 10 ms 采样点做差分解卷：825 机械 RPM（5775 电 RPM）时每 10 ms
> 转过约 6.05 rad > π，超过奈奎斯特极限，会展开失败（表现为“角度几乎不变”）。因此在 20 kHz
> 中断里逐拍累加，每拍仅约 0.03 rad，可以正确展开。

---

## 4. 关键参数速查

所有可调参数集中在 `motorControl/Inc/userDef.h`；切换阈值在 `sysControl.c` 的
`StrongDragToObs` 初始化器中。

### 4.1 电机与运行

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_CONTROLER` | `FOC_SENSORLESS` | 控制算法类型（`FOC_SENSOR` 分支为空） |
| `MOTOR_PAIRS` | 7 | 电机极对数 |
| `CONTROLER_TIME` | `0.00005f` | 控制周期 50 µs（20 kHz） |
| `MOTOR_RS` | `0.18f` Ω | 定子相电阻（实测） |
| `MOTOR_LD` | `0.00007f` H | 定子相电感（实测 70 µH） |
| `SPEED_K` | 250 | 电位器换算系数（0~3.3 V → 0~825 机械 RPM） |

### 4.2 PWM

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `PWM_ARR` | `4250u` | TIM1 ARR，170 MHz / (2×4250) = 20 kHz |
| `PWM_CCR_MIN` | `350u` | CCR 两端裕量，最小/最大占空比约 8.2% |
| `PWM_INVERTED` | `1u` | 1 = PWM 模式 2（CCR 取补） |

### 4.3 观测器与 PLL

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `SMO_GAIN` | `14.0f` | 初值；运行时每拍被 `Vbus/√3` 覆盖 |
| `SMO_PHI` | `1.0f` | 饱和函数边界层 |
| `SMO_FC` | `0.0f` | 当前未参与运算（滤波用 `SMO_E_PLF_FACTOR`） |
| `SMO_E_PLF_FACTOR` | `0.1f` | 反电势一阶滤波系数 |
| `PLL_DIR` | `1` | 初值；运行时由 `tra.speedOutDir` 覆盖 |
| `PLL_KP` | `1200.0f` | PLL 比例增益 |
| `PLL_KI` | `100.0f` | PLL 积分增益（不加 `Ts`） |
| `PLL_PLF_FACTOR` | `0.01f` | 观测角速度一阶滤波系数 |

### 4.4 PID

| 宏 | 值 | 说明 |
| --- | --- | --- |
| `PID_ID_KP / PID_IQ_KP` | `0.2f` | 电流环比例（V/A） |
| `PID_ID_KI / PID_IQ_KI` | `40.0f` | 电流环积分（已按 50 µs 折算） |
| `PID_ID_LIMIT / PID_IQ_LIMIT` | `6.0f` V | 初值；运行时被 `Vbus/√3` 覆盖 |
| `PID_SPEED_KP` | `0.0003f` | 速度环比例（A/rpm） |
| `PID_SPEED_KI` | `0.005f` | 速度环积分（已按 100 µs 折算） |
| `PID_SPEED_LIMIT` | `6.0f` A | 速度环输出（q 轴电流）限幅 |
| `FOC_SPEED_DIV` | `2u` | 速度环分频比 |

### 4.5 保护与斜坡

| 宏/字段 | 值 | 说明 |
| --- | --- | --- |
| `OVER_CURRENT` | `12.0f` A | 相电流保护阈值 |
| `BUS_VOLTAGE_MIN / MAX` | `10 / 40` V | 母线保护阈值 |
| `TRA_ACC_SPEED` | `1000` RPM/s | 梯形斜坡加速度（0→825 RPM 约 0.82 s） |
| `TRA_TARGET_SPEED` | `0` | 初值；运行时由电位器覆盖 |
| `VF_DRAG_TEST` | `0` | 置 1 启用 V/F 开环强拖硬件通路测试 |
| `OpenToCloseSwitchSpeed` | `4200` 电 RPM | 开环 → 闭环阈值 |
| `CloseToOpenSwitchSpeed` | `3000` 电 RPM | 闭环 → 开环阈值（滞环） |
| `CloseMinSpeed` | `2000` 电 RPM | 允许闭环的最低转速 |
| `OpenCurr / OpenCurrMax` | `0.3 / 3.0` A | 强拖初始/最大电流 |

### 4.6 采样硬件

| 宏（`sample.h`） | 值 | 说明 |
| --- | --- | --- |
| `SHUNT_RESISTOR_OHM` | `0.005f` | 下桥采样电阻 |
| `AMP_GAIN` | `10u` | 运放增益 |
| `I_ZERO_VOLT` | `1.65f` | 电流零位偏置 |
| `VBUS_DIV_RATIO` | `0.0449f` | 母线分压系数 |
| `VOLTAGE_DIVID_RESISTOR` | `10000.0f` | NTC 分压上电阻 |
| `NTC_B_VALUE` | `3434.0f` | NTC B 值 |
| `SAMPLE_CALI_TIMES` | `1024u` | 零位校准次数 |
| `ADC_BUF_LEN` | `5` | `g_adc_buf` 长度 |

---

## 5. 编译、烧录与运行

### 5.1 Keil MDK

1. 打开 `MDK-ARM/FOC.uvprojx`
2. 目标器件：`STM32G431RBTx`，编译器：ARMCLANG 6.15（AC6）
3. 宏定义：`USE_HAL_DRIVER, STM32G431xx`
4. 包含路径（已配置）：
   ```
   ../Core/Inc
   ../motorControl
   ../motorControl/Inc
   ../motorControl/algorithm/Inc
   ../motorControl/hardware/Inc
   ../Drivers/STM32G4xx_HAL_Driver/Inc
   ../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
   ../Drivers/CMSIS/Device/ST/STM32G4xx/Include
   ../Drivers/CMSIS/Include
   ```
5. Build → 下载。产物：`MDK-ARM/FOC/FOC.axf`、`FOC.hex`

### 5.2 VS Code + EIDE

`MDK-ARM/.eide/eide.yml` 已配置好源文件与目录结构，`.vscode/tasks.json` 提供
`build` / `flash` / `build and flash` / `rebuild` / `clean` 任务。

### 5.3 上电预期

1. 串口打印 `function register success, TIM && ADC enabled, System init done...`
2. 三颗 LED 点亮，驱动处于封锁状态
3. 约 51 ms 完成电流零位校准
4. 电位器在 0 附近时电机静止、无电流
5. 顺时针拧电位器：目标转速上升 → 开环强拖 → 观测速度收敛 → 切入闭环 → 稳定运行
6. VOFA+ 中可见梯形给定、观测转速、模式、角度等曲线

---

## 6. 移植教程

把本工程迁移到另一块 STM32G4（或同类 Cortex-M4F）板子、另一台电机时，按下面顺序做。
先让硬件通路转起来，再让观测器收敛，最后才调闭环参数——不要一开始就同时调所有环。

### 6.0 移植清单

| 序号 | 工作 | 主要文件 |
| --- | --- | --- |
| 1 | 确认功率级与采样硬件 | 原理图 |
| 2 | CubeMX 配置 TIM1 / ADC2 / DMA / USART1 / GPIO | `FOC.ioc`、`Core/` |
| 3 | 把 `motorControl/` 加入工程与头文件路径 | 工程文件 |
| 4 | 改引脚、电机参数、PWM 参数 | `userDef.h` |
| 5 | 改采样硬件系数 | `sample.h` |
| 6 | 绑定 ADC/TIM 句柄 | `sysControl.c`、`sysControl.h` |
| 7 | 适配驱动使能极性与相序 | `driver.c`、`userDef.h` |
| 8 | 分阶段验证（V/F → 电流强拖 → 闭环） | `VF_DRAG_TEST` |
| 9 | 整定观测器、PLL、电流环、速度环 | `userDef.h` |
| 10 | 用 VOFA+ 观察并记录波形 | `sysControl.c`、`userDef.h` |

### 6.1 硬件准备

移植目标板至少需要：

- 三相逆变桥（6 个 MOSFET/IGBT 或智能功率模块）
- **A、C 两相下桥采样电阻 + 运放**（本工程约定：A 相 → 注入 rank1，C 相 → 注入 rank2）
- 母线电压分压电路（分压后不超过 3.3 V）
- 驱动芯片使能引脚（当前用 4 路：PA0/PA1/PA2/PB0，按实际驱动板增减）
- 可选：电位器调速、NTC 测温、调试串口

> 如果目标板是**三电阻采样**或**单电阻采样**，需要重写 `sample.c` 的 `MotorCurrent()`，
> 并相应修改 ADC 注入组配置；SVPWM 与观测器部分无需改动。

### 6.2 CubeMX 配置

#### TIM1（PWM + 控制时基）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| Clock Source | Internal Clock | 170 MHz |
| Prescaler | 0 | |
| Counter Mode | Center Aligned 1 | 中央对齐，降低谐波与开关噪声 |
| Period (ARR) | `f_TIM1 / (2 × f_PWM)` | 170 MHz / 40 kHz = **4250** |
| Repetition Counter | 1 | 中央对齐下每两个计数事件产生一次更新，得到 20 kHz 更新中断 |
| Auto-reload Preload | Enable | |
| CH1/CH2/CH3 | PWM Generation CH1/2/3 | PC0/PC1/PC2（按板子改） |
| PWM Mode | **PWM mode 2** | 与 `PWM_INVERTED = 1` 对应 |
| Pulse | 0 | 运行时写入 |
| Dead Time | 0（或按驱动需求） | 当前由外部驱动负责 |
| CH4 | 可保留 | 当前未使用，可作 ADC 触发/测速输入 |

> 一定要**逐个通道**启动 PWM（`HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1/2/3)`）。
> HAL 的 `TIM_CHANNEL_ALL` 在 `TIM_CCxChannelCmd` 中会算出 `1 << 28`，落在 CCER 保留位，
> CH1~CH3 的使能位一个都不会置位——表现是“CCR 在变、引脚却没波形”。代码注释里已记录该坑。
>
> 也**不要**用 `HAL_TIM_Base_Start_IT()` 开更新中断：`HAL_TIM_PWM_Start()` 已把
> `htim->State` 置为 `HAL_TIM_STATE_BUSY`，`Base_Start_IT` 会直接返回 `HAL_ERROR`。
> 正确做法是 `__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE)`。

#### ADC

| 组 | 通道 | 引脚 | 用途 | 配置 |
| --- | --- | --- | --- | --- |
| 注入 rank1 | CH3 | PA6 | A 相电流 | 2.5 cycles，软件触发 |
| 注入 rank2 | CH17 | PA4 | C 相电流 | 2.5 cycles，软件触发 |
| 规则 rank1 | CH11 | PC5 | 母线电压 | 92.5 cycles |
| 规则 rank2 | CH12 | PB2 | 电位器 | 92.5 cycles |
| 规则 rank3 | CH15 | PB15 | NTC 温度 | 92.5 cycles |

- ADC 时钟：`SYSCLK / 4 = 42.5 MHz`，12 bit
- 规则组：`ContinuousConvMode = Enable`、`DMAContinuousRequests = Enable`、扫描模式
- DMA：`DMA1_Channel1`，外设→内存，半字，循环模式，内存自增
- 注入组：`InjectedNbrOfConversion = 2`，`ExternalTrigInjecConv = Software`
- 使能 `ADC1_2_IRQn`

> 采样时刻：注入转换由 TIM1 更新事件触发，与 PWM 同步。移植到不同功率级时，请用示波器确认
> 注入转换落在**下桥臂导通窗口中央**（避开开关沿与振铃）。若不合适，可改用 TIM1 CH4/TRGO
> 触发、调整计数模式或 `RCR`，但必须同步修改 `CONTROLER_TIME` 与所有按 `Ts` 折算的增益。

#### USART

- PB6 = TX，PB7 = RX，115200 8N1，无流控
- 用于 `printf` 和 VOFA+ 推流

#### GPIO

- 驱动使能：PA0、PA1、PA2、PB0，推挽输出
- LED：PC13、PC14、PC15，推挽输出
- 所有未使用引脚保持默认

### 6.3 把控制代码接入工程

1. 复制 `motorControl/` 整个目录到新工程
2. 把以下源文件加入编译：
   - `motorControl/Src/sysControl.c`
   - `motorControl/algorithm/Src/foc.c`
   - `motorControl/algorithm/Src/observer.c`
   - `motorControl/algorithm/Src/TAccDac.c`
   - `motorControl/hardware/Src/driver.c`
   - `motorControl/hardware/Src/sample.c`
3. 加入头文件路径：
   - `motorControl/Inc`
   - `motorControl/algorithm/Inc`
   - `motorControl/hardware/Inc`
4. 在 `main.c` 中：
   ```c
   #include "userDef.h"
   #include "sysControl.h"
   // 外设初始化完成后
   motorInit();
   // 主循环中轮询调试推流（函数实现在 sysControl.c）
   while (1)
   {
       DebugStream_Poll();
   }
   ```
   若要增删调试通道，修改 `userDef.h` 的 `DBG_CH_NUM`，并同步修改`sysControl.c` 的 `DebugStream_Load()` 与上位机配置。
5. 确保 `stm32g4xx_it.c` 中的 `TIM1_UP_TIM16_IRQHandler`、`ADC1_2_IRQHandler` 会调用`HAL_TIM_IRQHandler(&htim1)`、`HAL_ADC_IRQHandler(&hadc2)`（CubeMX 生成的代码已包含）

### 6.4 修改 `userDef.h`

按顺序改：

1. **控制方式**：`MOTOR_CONTROLER = FOC_SENSORLESS`
2. **电机参数**：
   - `MOTOR_PAIRS`：极对数（务必数对，速度与角度都依赖它）
   - `MOTOR_RS`、`MOTOR_LD`：用万用表/LCR 表测相电阻、相电感；表贴式取 `Ld = Lq`
3. **控制周期**：`CONTROLER_TIME = 1 / f_PWM`
4. **PWM 参数**：
   - `PWM_ARR = f_TIM1 / (2 × f_PWM)`（中央对齐）
   - `PWM_CCR_MIN`：按驱动自举/死区需求留裕量
   - `PWM_INVERTED`：PWM 模式 2 填 1，模式 1 填 0
5. **引脚**：
   - `MOTOR_A~D_PORT/PIN`：驱动使能引脚
   - `LED_A~C_PORT/PIN`：状态灯
6. **保护阈值**：按母线电压与功率级调整 `BUS_VOLTAGE_MIN/MAX`、`OVER_CURRENT`
7. **初始增益**：先保留默认值，进入整定阶段再改

> 如果目标 MCU 的 TIM1 时钟不是 170 MHz，`PWM_ARR` 与 `PWM_CCR_MIN` 必须同步换算；
> 否则实际开关频率与 `CONTROLER_TIME` 不一致，所有积分增益都会失配。

### 6.5 修改 `sample.h`

| 宏 | 改为 |
| --- | --- |
| `SHUNT_RESISTOR_OHM` | 实际采样电阻 |
| `AMP_GAIN` | 实际运放增益 |
| `I_ZERO_VOLT` | 实际零位偏置（通常 Vref/2） |
| `VBUS_DIV_RATIO` | 分压电阻比 |
| `VOLTAGE_DIVID_RESISTOR` | NTC 分压上电阻 |
| `NTC_B_VALUE` | 实际 NTC B 值 |
| `SPEED_K`（在 `userDef.h`） | 电位器满量程对应的机械 RPM |

若没有电位器/温度采样，可以保留函数，在 `Washer()` 中把 `washer_vlaue` 固定为某个目标转速，或使用串口指定目标速度。

### 6.6 修改外设绑定

`sysControl.c` 顶部的 `motor` 初始化器中：

```c
Motor_t motor = {
    .name = NULL,
    .GeneralMode = MOTOR_CALIB,
    .hadc = &hadc2,        // ← 改成目标 ADC 句柄
    .adc_base = ADC2,      // ← 对应的 ADC 实例
    ...
    .driver = {
        ...
        .htim = &htim1,    // ← 改成目标 PWM 定时器句柄
    },
};
```

同时检查三个 HAL 回调：

```c
HAL_ADC_ConvCpltCallback()          // 规则组：母线/电位器/温度
HAL_ADCEx_InjectedConvCpltCallback()// 注入组：相电流 + FOC ISR
HAL_TIM_PeriodElapsedCallback()     // TIM1 更新：启动注入转换
```

前两个回调用 `motor.adc_base` 做实例判断；TIM 回调用 `motor.tim_base` 做实例判断，换用其他定时器时需要同步修改这一行。

### 6.7 适配驱动使能与相序

- `driver.c` 的 `motorEnable()` 用 `GPIO_PIN_SET` 使能。若驱动芯片是**低电平使能**，把 `motorEnable()`/`motorDisable()` 的 `SET`/`RESET` 对调。
- 若上电后电机反转或强拖方向与预期相反：
  1. 对调任意两相电机线（A/C 互换），或
  2. 在代码中反转 `TAccDac` 输出的方向，或
  3. 调整 `PLL.Dir` 的符号约定
- 相电流符号：若 `Iq` 恒为负、力矩方向相反，通常是 A/C 相采样通道接反或相序接反；先用 `VF_DRAG_TEST` 确认旋转方向，再检查电流符号。

### 6.8 分阶段验证（强烈建议）

#### 阶段 1：V/F 开环硬件通路测试

```c
#define VF_DRAG_TEST  1
```

此时 `Strong_Drag_Current_Close_Loop()` 只做“固定电压矢量随角度旋转”：

- 绕过电流环、SMO、PLL、切换判据
- `Vd = 0`，`Vq = 0.35 V`
- 用来单独验证：驱动使能、PWM 输出、相序、角度方向、ADC 采样

检查项：

1. 示波器看 PC0/PC1/PC2 是否有 20 kHz、占空比随角度变化的 PWM
2. 电机是否能被缓慢拖动（电位器从小往大拧）
3. 三相电流是否平衡、无明显直流偏置
4. 若电机不转但电流很大，立即断电检查相序/角度方向

> 注意：V/F 模式下没有电流闭环，务必使用**限流电源**（代码注释建议 2 A 限流）。

#### 阶段 2：电流闭环强拖

```c
#define VF_DRAG_TEST  0
```

保持开环角度，但 q 轴电流闭环（`Iq* = OpenCurr`）。检查：

1. `Iq_LPF` 是否跟随 `OpenCurr`
2. 电流是否稳定、无高频振荡
3. 电位器变化时，`tra.speedOut` 是否按 1000 RPM/s 斜坡上升
4. 电机能否从静止可靠启动

#### 阶段 3：观测器与闭环

观察 VOFA+ 通道：

- 通道 1（斜坡输出）与通道 2（观测转速）应基本重合
- 通道 8（`CheckCnt`）应从 0 逐渐增长到 41
- 通道 7（模式）应从 1（开环）变为 2（闭环）
- 通道 9（`ThetaErr`）在开环时允许有 20°~80° 的稳定功角，闭环后应接近 0
- 通道 5（`Iq`）在切换瞬间不应有大幅冲击（`ThetaOffset` 与速度环预置应生效）

若 `CheckCnt` 一直为 0：

1. 检查 `pll.omega_hat_PLF` 是否与给定同向、幅值是否接近
2. 检查 `MOTOR_RS`、`MOTOR_LD`、`MOTOR_PAIRS` 是否准确
3. 尝试加大 `SMO_GAIN`（当前每拍被 `Vbus/√3` 覆盖，可改 `Foc_Observer()` 中的公式）
4. 检查 `PLL.Dir` 符号是否与旋转方向匹配
5. 降低 `OpenToCloseSwitchSpeed`，或提高强拖电流上限

### 6.9 参数整定顺序

1. **电流环**（开环强拖阶段）
   - 先 `Ki = 0`，增大 `Kp` 直到阶跃响应刚好不振荡，再取 50%~70%
   - 加 `Ki` 消除静差；`Ki` 已按 `Ts` 折算，换周期要重算
   - 输出限幅保持 `Vbus/√3`
2. **SMO**
   - 滑模增益由 `Vbus/√3` 自适应，一般无需改
   - `SMO_PHI` 越小抖振越大、收敛越快；`1.0` 是实测折中
   - `SMO_E_PLF_FACTOR` 越小反电势越平滑但相位滞后越大；先用 `0.1`
3. **PLL**
   - 先 `Ki = 0`，增大 `Kp` 让观测角跟上；再加 `Ki` 消除稳态速度误差
   - `PLL_KI` 未乘 `Ts`，换控制周期时必须重算
   - `PLL_PLF_FACTOR` 越小速度反馈越平滑但滞后越大
4. **速度环**
   - 从 `Kp` 很小开始，逐步增大到转速阶跃无明显超调
   - `Ki` 与 `Kp` 同比例调整；速度环周期是电流环的 `FOC_SPEED_DIV` 倍
5. **切换策略**
   - 先保证能稳定切入闭环，再逐步提高 `OpenToCloseSwitchSpeed`
   - 切换抖动时增大滞环（`4200` vs `3000`）
   - 低速失步时提高情况 4 的电流下限（当前 1.2 A）

### 6.10 常见问题排查

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| PWM 引脚无波形 | 用了 `TIM_CHANNEL_ALL` | 改为逐通道启动 CH1/CH2/CH3 |
| PWM 无输出、`CCR` 正常变化 | MOE 未使能 / 引脚 AF 错 / 驱动未使能 | 检查 `HAL_TIM_PWM_Start`、GPIO AF、使能极性 |
| 更新中断不进 | 用 `HAL_TIM_Base_Start_IT` 返回错误 | 改用 `__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE)` |
| 电机不转、电流很大 | 相序错 / 角度方向反 / 电流零位未校准 | 对调两相；确认零位校准期间驱动封锁 |
| 强拖能转，切闭环就停 | 观测角符号错 / `MOTOR_RS/LD` 不准 / 切换太快 | 检查 `Dir`、参数；降低切换速度 |
| 反复在开环/闭环之间跳 | 滞环太小 / 观测速度抖动 | 增大 4200/3000 差值；减小 `PLL_PLF_FACTOR`（加强滤波）或降低 `PLL_KP/PLL_KI` |
| 转速上不去 | 母线电压低 / 电流限幅 / 反电势超出观测能力 | 检查 `Vbus`、母线压降、`SPEED_K` 与观测速度 |
| 静止时发热 | 速度给定未归零 / 电流环积分未清 | 检查 `fabsf(speedOut) < 10` 分支 |
| 串口无输出 | 未勾选 MicroLIB 且缺少半主机桩 / TX 引脚错 | 保留 `usart.c` 中的桩函数，或勾选 MicroLIB |
| 中断里 `printf` 卡死 | `fputc` 阻塞轮询 | 不要在 FOC/ADC 中断里打印，改用 VOFA+ |

---

## 7. 调试与观测

### 7.1 VOFA+ 配置

1. 打开 VOFA+，选择串口，115200
2. 协议选择 **JustFloat**
3. 通道数设为 **15**
4. 按第 3.6.11 节的通道表添加曲线

推荐同时观察：

- 通道 0/1/2：目标、斜坡、实际转速（三条曲线量级一致，直接对比）
- 通道 7/8：模式与 `CheckCnt`（判断切换时机）
- 通道 5/10：`Iq` 与电角速度（判断电流环与观测器）
- 通道 14：电角度累加（斜率即电角速度，用于检查角度连续性）

### 7.2 `printf` 使用注意

- `fputc` 是**阻塞轮询**，115200 下一个字符约 87 µs
- 不要在 FOC 电流环、ADC 注入中断里调用 `printf`
- `printf` 不可重入，中断与主循环同时打印会导致字符交错
- 当前工程使用 MicroLIB（`useUlib = 1`）；若关闭 MicroLIB，`usart.c` 已提供
  `__use_no_semihosting` 与 `_sys_exit/_ttywrch/_sys_open` 桩函数

### 7.3 中断负载

- 20 kHz 中断里 Park / 反 Park / PLL 各调用一对 `sinf/cosf`（共 6 次），再加 SVPWM 与观测器
- Cortex-M4F 带单精度 FPU，50 µs 周期内余量充足
- 若提高开关频率或增加算法，建议：
  - 用查表法/递推法替代 `sinf/cosf`
  - 使用 STM32G4 的 CORDIC 加速三角函数、FMAC 加速滤波
  - 把 `printf`、遥测等非实时工作留在主循环

---

## 8. 已知限制与后续扩展

### 限制

1. **单电机实例**：`Motor_t motor` 是全局单例，HAL 回调也直接引用它；多电机需重构为
   按句柄分发的实例数组。
2. **有感模式为空**：`FOC_SENSOR` 分支只有宏和状态机入口，未实现编码器/Hall 接口。
3. **低速性能受 SMO 限制**：调速范围上限 825 机械 RPM 是按反电势与观测增益估算的；低速段
   （< 100 电 RPM）观测器信噪比不足，仍依赖开环强拖。
4. **`SMO_FC` / `wc` 未参与运算**：反电势滤波由 `SMO_E_PLF_FACTOR` 完成。
5. **`PLL_KI` 不含 `Ts`**：换控制周期时必须重新整定，不能直接照搬。
6. **无死区配置**：TIM1 `DeadTime = 0`，依赖外部驱动；换成裸半桥必须补死区。
7. **调试帧注释与实际长度不一致**：`sysControl.c` 注释写“44 字节”，但 `DBG_CH_NUM = 15`
   （定义在 `userDef.h`）时实际帧长为 64 字节；以代码计算为准。
8. **主循环与中断共享数据无保护**：当前依赖 32 位对齐读写的原子性，复杂数据类型需加临界区。

### 可扩展方向

- 实现 `FOC_SENSOR` 分支（Hall/编码器），与无感模式共用电流环和 SVPWM
- 增加单电阻/三电阻采样与相电流重构
- 增加弱磁控制、MTPA（凸极电机）
- 增加参数在线辨识（Rs、Ld、磁链）
- 用 CORDIC/FMAC 优化中断耗时
- 增加 LED 状态指示（校准/开环/闭环/故障）
- 增加 Flash 参数存储与上位机参数下发
- 增加看门狗与故障码记录

---

## 9. 术语表

| 术语 | 含义 |
| --- | --- |
| FOC | Field Oriented Control，磁场定向控制 |
| SMO | Sliding Mode Observer，滑模观测器 |
| PLL | Phase Locked Loop，锁相环 |
| SVPWM | Space Vector PWM，空间矢量脉宽调制 |
| I/F | 电流-频率开环强拖（固定电流 + 旋转角度） |
| V/F | 电压-频率开环强拖（固定电压矢量 + 旋转角度） |
| 电 RPM | 电角速度折算的每分钟转数 = 机械 RPM × 极对数 |
| 机械 RPM | 转子实际每分钟转数 |
| 功角 | 开环强拖时转子位置滞后给定磁场的角度 |
| `Ts` | 控制周期，本工程为 50 µs |
| CCR | 定时器比较寄存器值，决定 PWM 占空比 |
| ARR | 定时器自动重装载值，决定 PWM 周期 |

---

> 本文档基于当前工作区代码（含未提交的 ADC 句柄参数化改动）整理。若后续修改了外设、
> 引脚或控制周期，请同步更新第 1、4、6 节中的表格。
