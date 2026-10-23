/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-10 22:09:56
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-20 11:30:29
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\Src\sysControl.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <stdio.h>
#include "sysControl.h"
#include "dma.h"
#include "adc.h"
#include "tim.h"
#include "stm32g4xx_it.h"
/*************************************************************************************************************************/
/*********************************************************电机实例********************************************************/
/*************************************************************************************************************************/
/* 说明：所有模块的数据（算法参数、Ts、硬件引脚）都在此处一次性初始化；
 *       底层模块只提供算法与 ops 函数表，不持有任何配置数据。
 *       ops 指针在 motorInit() 中通过各 *_register() 绑定。 */
Motor_t motor = {
    .foc = {
        .Ts            = CONTROLER_TIME,
        .V_bus         = 0.0f,
        .theta         = 0.0f,
        .Id_ref        = 0.0f,
        .Iq_ref        = 0.0f,
        .speed_ref     = 0.0f,
        .speed_fb      = 0.0f,
        .speed_div     = FOC_SPEED_DIV,
        .speed_div_cnt = 0u,
        .IdLPFFactor   = 0.1f,
        .IqLPFFactor   = 0.1f,
        /* PWM 输出：ARR=4250（20kHz 中央对齐）、两端各留 350 计数、TIM1 为 OCMODE_PWM2 需取补 */
        .pwm_cycle     = PWM_ARR,
        .ccr_min       = PWM_CCR_MIN,
        .pwm_inverted  = PWM_INVERTED,
        .pid_id    = {
            .Kp = PID_ID_KP, .Ki = PID_ID_KI, .Kd = PID_ID_KD,
            .output_limit = PID_ID_LIMIT,
        },
        .pid_iq    = {
            .Kp = PID_IQ_KP, .Ki = PID_IQ_KI, .Kd = PID_IQ_KD,
            .output_limit = PID_IQ_LIMIT,
        },
        .pid_speed = {
            .Kp = PID_SPEED_KP, .Ki = PID_SPEED_KI, .Kd = PID_SPEED_KD,
            .output_limit = PID_SPEED_LIMIT,
        },
    },
    .smo = {
        .Ts            = CONTROLER_TIME,
        .Rs            = MOTOR_RS,
        .Ld            = MOTOR_LD,
        .phi           = SMO_PHI,
        .fc            = SMO_FC,
        .wc            = TWO_PI * SMO_FC,
        .e_hat_PLF_factor = SMO_E_PLF_FACTOR,
    },
    .pll = {
        .Dir           = PLL_DIR,
        .Ts            = CONTROLER_TIME,
        .Kp            = PLL_KP,
        .Ki            = PLL_KI,
        .omega_hat_PLF_factor = PLL_PLF_FACTOR,
    },
    .tra = {
        .Ts            = CONTROLER_TIME,
        .TargetSpeed   = TRA_TARGET_SPEED,
        .accSpeed      = TRA_ACC_SPEED,
    },
    .angle = {
        .Ts            = CONTROLER_TIME,
    },
    .StrongDragToObs = {
        .CheckCnt = 0, .CloseRunTime = 0, .CloseSpeed = 0,
        .Curr_change_uint = 0, .EleCloseSpeedAbs = 0, .EleOpenSpeedAbs = 0,
        .ErrCnt = 0, .Errflag = 0, .ErrTimes = 0,
        /* 强拖 -> 观测器切换参数，取自参考工程 motor_publicdata.c:145-161。
         * 单位：OpenCurr/OpenCurrMax 为 A，两个阈值速度与 speedOut、omega_hat 换算出的电 RPM 同量纲 */
        .OpenCurr = 0.3f, .OpenCurrLast = 0.0f,
        /* 启动阶段必须有足够力矩：0.5A 在 0.18Ω 上只有 0.09V，转子根本拖不动。
         * 高速段想安静，靠"情况1"在速度达标后把电流降到 0.5A，而不是压低这个上限。 */
        .OpenCurrMax = 3.0f, .OpenSpeed = 0,
        .OpenToCloseSwitchSpeed = 4200.0f,   /* 开环 -> 闭环切换速度阈值 */
        .CloseToOpenSwitchSpeed = 3000.0f,   /* 闭环 -> 开环退回阈值，与 4200 构成滞环 */
        .CloseMinSpeed = 2000.0f,            /* 允许闭环的最低速度 */
        .SpeedErr = 0, .SpeedErrFlt = 0, .ThetaErr = 0, .ThetaObs = 0, .ThetaRef = 0,
    },
    .driver = {
        .motor_a   = MOTOR_A_PORT, .motor_pin_a = MOTOR_A_PIN,
        .motor_b   = MOTOR_B_PORT, .motor_pin_b = MOTOR_B_PIN,
        .motor_c   = MOTOR_C_PORT, .motor_pin_c = MOTOR_C_PIN,
        .motor_d   = MOTOR_D_PORT, .motor_pin_d = MOTOR_D_PIN,
        .led_a     = LED_A_PORT,   .led_pin_a   = LED_A_PIN,
        .led_b     = LED_B_PORT,   .led_pin_b   = LED_B_PIN,
        .led_c     = LED_C_PORT,   .led_pin_c   = LED_C_PIN,
        .htim      = &htim1,
    },
    .sample = {
        .cali_flag = false,
    },
    
    .GeneralMode = MOTOR_CALIB,

    .name = NULL,
};
/*************************************************************************************************************************/
/********************************************************初始化接口*******************************************************/
/*************************************************************************************************************************/
int motorInit(void)
{
    /* 1. 绑定各模块的算法函数表（配置数据已在上面的初始化器中静态就绪） */
    if (foc_register(&motor.foc) != REGISTER_SUCCESS)
    {
        printf("%s register failed...\r\n", motor.foc.ops->name);
        return REGISTER_ERROR;
    }
    if (smo_register(&motor.smo, &motor.pll) != REGISTER_SUCCESS)
    {
        printf("%s && %s register failed...\r\n", motor.smo.ops->name, motor.pll.ops->name);
        return REGISTER_ERROR;
    }
    if (TAccDac_register(&motor.tra, &motor.angle) != REGISTER_SUCCESS)
    {
        printf("%s && %s register failed...\r\n", motor.tra.ops->name, motor.angle.ops->name);
        return REGISTER_ERROR;
    }
    if (driver_register(&motor.driver) != REGISTER_SUCCESS)
    {
        printf("%s register failed...\r\n", motor.driver.ops->name);
        return REGISTER_ERROR;
    }
    if (sample_register(&motor.sample) != REGISTER_SUCCESS)
    {
        printf("%s register failed...\r\n", motor.sample.ops->name);
        return REGISTER_ERROR;
    }
    /* 2. 上电初始化：先关断驱动，等待采样零点校准完成后再使能 */
    motor.driver.ops->motorDisable(&motor.driver);
    motor.driver.ops->LedOn(&motor.driver);
    // 开启中断
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&motor.sample.g_adc_buf[2], 3);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
    /* 使能 TIM1 更新中断（20kHz）：此后每个 PWM 周期由 HAL_TIM_PeriodElapsedCallback
     * 触发一次 ADC 注入转换，注入完成中断里跑一整套 FOC，构成 50us 的控制周期。
     * 注意不能改用 HAL_TIM_Base_Start_IT()：HAL_TIM_PWM_Start() 已把 htim1.State 置为
     * HAL_TIM_STATE_BUSY，Base_Start_IT 会直接返回 HAL_ERROR，更新中断根本开不起来 */
    /* PWM 输出必须逐个通道启动！
     * TIM_CHANNEL_ALL 的值是 0x3C，HAL 的 TIM_CCxChannelCmd 里做的是
     *   tmp = TIM_CCER_CC1E << (Channel & 0x1F) = 1 << 28
     * 28 位落在 CCER 的保留位上，CH1/CH2/CH3 的 CCxE 一个都不会被置位，
     * 于是 MOE 虽然开了、CCR 也在变，但三个引脚上根本没有波形（电机不可能转）。
     * 参考工程 ShengHaoFOC 也是分三次调用 CH1/CH2/CH3。 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    /* 启动 PWM 输出、启动 ADC-DMA 采样（本层框架不接管外设启动） */
    printf("function register success, TIM && ADC enabled, System init done...\r\n");

    return REGISTER_SUCCESS;
}
/*************************************************************************************************************************/
/*******************************************************FOC 控制周期******************************************************/
/*************************************************************************************************************************/
#if (MOTOR_CONTROLER == FOC_SENSORLESS)     // 无感算法
    static void Strong_Drag_To_Observer_Cal(StrongDragToObs_t *p)
    {
        // 1.计算角度误差（开环给定角 - 观测角），并归一化到 [-π, π]
        p->ThetaErr = p->ThetaRef - p->ThetaObs;
        if (p->ThetaErr > PI)
            p->ThetaErr -= TWO_PI;
        if (p->ThetaErr < -PI)
            p->ThetaErr += TWO_PI;
        // 2.计算速度误差
        p->SpeedErr = p->OpenSpeed - p->CloseSpeed;
        p->SpeedErrFlt += 0.1f * (p->SpeedErr - p->SpeedErrFlt);
        // 3.取开环速度以及闭环速度的绝对值
        p->EleOpenSpeedAbs = fabsf(p->OpenSpeed);
        p->EleCloseSpeedAbs = fabsf(p->CloseSpeed);
        // 4.更新收敛计数器：改用"观测速度是否跟得上给定速度"作为判据
        //   不能再用角度差：开环强拖时转子必然滞后给定磁场一个功角（本机实测 20°~80°），
        //   再叠加 SMO/PLL 的相位偏差，ThetaErr 会稳定停在 -60°~-80°，永远过不了角度门槛，
        //   于是 CheckCnt 一直是 0、永远切不到闭环。
        //   而速度观测精度很高（实测误差 <1%），用它判断"观测器是否可信"更可靠。
        //   切换瞬间的角度冲击已经由 Foc_Observer 里的 theta_hat 对齐消除，不需要靠角度门槛兜。
        if ((fabsf(p->OpenSpeed) > 100.0f) &&
            (fabsf(p->OpenSpeed - p->CloseSpeed) < (fabsf(p->OpenSpeed) * 0.15f)))
            p->CheckCnt++;
        else if (p->CheckCnt > 0u)   /* CheckCnt 是 u16，递减前判零避免下溢成 65535 */
            p->CheckCnt--;
        // 5.若当前处于闭环状态，则累加计数器
        if (p->GeneralMode == CLOSE_LOOP)
        {
            p->CloseRunTime++;
            p->CloseRunTime = Sat(p->CloseRunTime, 0, 40000);
        }
        // 6.根据电机状态动态更新电流大小
        p->Curr_change_uint = p->OpenCurrMax * 0.0003f;
        /* 情况一：电机未减速 && 开环速度绝对值 > 切换速度 && 角度已经收敛(CheckCnt >= 30)
            此时逐渐减小强拖电流，让观测器平滑接管。
            注意条件必须是 CheckCnt >= 30，不能是 < 30：
            突加转速时速度会很快冲过 4200，而此刻角度还没收敛(CheckCnt=0)，
            若按 < 30 去减电流，电流会被一路压到 0.08A → 力矩消失 → 失步 → 电机停转，
            表现就是"电位器一下拧到底电机不转"。 */
        if ((p->state != DECELERATE) && (p->EleOpenSpeedAbs > p->OpenToCloseSwitchSpeed) && (p->CheckCnt >= 30))
        {
            /* 速度达标、角度也收敛了 → 可以把强拖电流降下来（高速段安静），
             * 闭环切换只差最后几拍，电流小一点不影响 */
            p->OpenCurr = Sat(fabsf(p->OpenCurr) - p->Curr_change_uint, 0.5f, p->OpenCurrMax);
        }
        /* 情况2：需要加大电流的两种情况（二者并集）：
            ① 速度还没到切换阈值 —— 正常加速段；
            ② 速度到了但角度还没收敛(CheckCnt < 30) —— 说明电流不足以维持同步，
               必须继续加，否则电流会永远停在初始值(0.3A)，功角越来越大直到失步。
            这是"突加转速后卡在开环、CheckCnt 永远为 0"的死锁根源：
            情况1 要求 CheckCnt>=30 才减、情况2 原来只要求速度<4200，
            速度一旦冲过 4200 而角度没收敛，两个分支都不触发，电流就冻结了。 */
        if ((p->state != DECELERATE) &&
            ((p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed) || (p->CheckCnt < 30)))
        {
            p->OpenCurr = Sat(fabsf(p->OpenCurr) + 0.01f, 0.05f, p->OpenCurrMax);
        }
        /* 情况3：电机减速 && 闭环速度最小值 < 开环速度绝对值 < 切换速度 + 5
            此时略微电流，防止失步
        */
        if ((p->state == DECELERATE) && (p->EleOpenSpeedAbs > p->CloseMinSpeed) && (p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed + 5))
        {
            p->OpenCurr = Sat(fabsf(p->OpenCurr) + 0.001f, 0.05f, p->OpenCurrMax);
        }
        /* 情况4：电机减速 && 开环速度绝对值 < 闭环速度最低值
            略微减小电流，但下限必须保持足够力矩（1.2A）：
            原来减到 0.08A，一旦电机在这个转速段失步就再也拖不动，
            表现就是"低速时静止不动/被吸住"，而且速度给定还在、角度还在空转。
        */
        if ((p->state == DECELERATE) && (p->EleOpenSpeedAbs < p->CloseMinSpeed))
        {
            p->OpenCurr = Sat(fabsf(p->OpenCurr) - 0.001f, 1.2f, p->OpenCurrMax);
        }
        // 7.根据开环速度的方向，确定开环电流的符号
        if (p->OpenSpeed < 0)
        {
            p->OpenCurr = -fabsf(p->OpenCurr);
        }
        // 8.切换条件与错误检测
        /**
         * 条件A:闭环状态 && 闭环运行周期 > 20000 && 闭环速度 < 闭环速度最小值
         * 此时在闭环条件下速度过低，容易引发观测器失效或者失步，触发错误标志
         */
        if ((p->GeneralMode == CLOSE_LOOP) && (p->CloseRunTime > 20000) && (p->EleCloseSpeedAbs < p->CloseMinSpeed))
        {
            p->Errflag = 1;
            p->CloseRunTime = 0;
            p->ErrCnt++;
        }
        /**
         * 条件A2:闭环状态 && 观测速度方向与给定相反 && 幅值不太小 → 判定为失步(反转)
         * 突加转速时若切换冲击导致 PLL 锁到反向，转速会与给定方向相反，
         * 原来的条件A(速度过低)永远不成立，就会一直反向转下去。这里补一道方向保护。
         */
        if ((p->GeneralMode == CLOSE_LOOP) && (p->OpenSpeed * p->CloseSpeed < 0.0f) &&
            (p->EleCloseSpeedAbs > 200.0f))
        {
            p->Errflag = 1;
            p->CloseRunTime = 0;
            p->ErrCnt++;
        }
        /**
         * 条件B:角度计数器 > 40 && 开环速度绝对值 > 切换速度 && 观测速度也已经跟上
         * 后两条是"观测器可信"的校验：只看速度给定就切到闭环，会在转子还没跟上的
         * 时候强行切换（快速拧电位器时尤其明显），切换后立刻失步甚至被"吸住"。
         */
        if ((p->CheckCnt > 40) &&
            (p->EleOpenSpeedAbs > p->OpenToCloseSwitchSpeed) &&
            (p->OpenSpeed * p->CloseSpeed > 0.0f) &&
            (p->EleCloseSpeedAbs > (p->OpenToCloseSwitchSpeed * 0.7f)))
        {
            p->Errflag = 0;
            p->GeneralMode = CLOSE_LOOP;
        }
        /**
         * 条件C:开环速度绝对值 < 闭环退回开环阈值（3000）
         * 此时速度过低，强制切换为开环模式。与 4200 一起构成滞环，避免阈值附近反复切换
         */
        if (p->EleOpenSpeedAbs < p->CloseToOpenSwitchSpeed)
        {
            p->Errflag = 0;
            p->GeneralMode = OPEN_LOOP;
        }
        // 9.限制计数器范围
        if (p->CheckCnt > 40)   p->CheckCnt = 41;
        else if (p->CheckCnt < 0)   p->CheckCnt = 0;
    }
    /*********************************************************************************************************************/
    /***********************************************各运行模式共用的算法段************************************************/
    /*********************************************************************************************************************/
    /* 说明：开环强拖和滑膜闭环有 4 段完全相同的处理，抽出来避免两处维护；
     *       两个模式函数只保留"角度从哪来、电流给定给谁"的差异。 */

    /** 相电流采样与 Clarke 变换 */
    static void Foc_Sample(void)
    {
        motor.foc.Ia = motor.sample.Ia;
        motor.foc.Ib = motor.sample.Ib;
        motor.foc.Ic = motor.sample.Ic;
        motor.foc.ops->clarke(&motor.foc);
    }
    /** 梯形速度曲线 + 开环电角度发生器 */
    static void Foc_SpeedRamp(void)
    {
        /* 电位器调速：washer_vlaue 已由 sample.c 换算成机械 RPM（0~SPEED_K*3.3 范围） */
        motor.tra.TargetSpeed = motor.sample.washer_vlaue;
        motor.tra.ops->TAccDac(&motor.tra);
        motor.angle.ElectricalAngleSpeedSet = motor.tra.speedOut;
        motor.angle.ops->AngleGenerator(&motor.angle);
    }
    /**
     * 滑膜观测器 + 锁相环，以及开环/闭环切换判据
     * 注意：开环强拖阶段也必须跑这一段。切换判据里的 ThetaObs、CheckCnt 都来自这里，
     *       不跑就永远不知道角度有没有收敛，也就永远切不到闭环。
     */
    static void Foc_Observer(void)
    {
        motor.smo.gain = motor.sample.Vbus * INV_SQRT3;
        /* 电流环输出限幅跟随母线（对齐参考工程 Motor_System_Run 的做法）：
         * Vbus/√3 是 SVPWM 线性调制区的相电压上限，写死固定值会在母线变化时
         * 过大（占空比饱和到 0/1）或过小（出力不足） */
        motor.foc.pid_id.output_limit = motor.sample.Vbus * INV_SQRT3;
        motor.foc.pid_iq.output_limit = motor.sample.Vbus * INV_SQRT3;
        motor.smo.Rs = MOTOR_RS;
        motor.smo.Ld = MOTOR_LD;
        /* SMO 用的是上一拍的 I_alpha/I_beta 与 V_alpha/V_beta（一拍延迟，属正常） */
        motor.smo.I_alpha = motor.foc.I_alpha;
        motor.smo.I_beta = motor.foc.I_beta;
        motor.smo.U_alpha = motor.foc.V_alpha;
        motor.smo.U_beta = motor.foc.V_beta;
        motor.smo.ops->SMO(&motor.smo);

        motor.pll.Dir = motor.tra.speedOutDir;
        motor.pll.A_in = motor.smo.e_alpha_hat_plf;
        motor.pll.B_in = motor.smo.e_beta_hat_plf;
        motor.pll.ops->PLL(&motor.pll);

        /* 速度单位统一为电 RPM：给定来自梯形曲线，反馈来自 PLL 观测角速度 */
        motor.foc.pid_speed.target = motor.tra.speedOut;
        motor.foc.pid_speed.feedback = motor.pll.omega_hat_PLF / TWO_PI * 60.0f;

        motor.StrongDragToObs.OpenSpeed = motor.tra.speedOut;
        motor.StrongDragToObs.CloseSpeed = motor.foc.pid_speed.feedback;
        motor.StrongDragToObs.ThetaRef = motor.angle.ElectricalAngleThetaOut;
        motor.StrongDragToObs.ThetaObs = motor.pll.theta_hat;
        motor.StrongDragToObs.state = motor.tra.motor_state;
        Strong_Drag_To_Observer_Cal(&motor.StrongDragToObs);

        /* 刚切入闭环的那一拍：记下"开环角 − 观测角"的差值，之后每拍按比例衰减。
         * 不能直接把 theta_hat 强行改成开环角（那样会打断 PLL 的连续跟踪，
         * 鉴相器突然看到几十度误差 → omega 突变 → 速度环抖动 → "颠一下"）。
         * 也不能什么都不做（开环时转子滞后给定磁场一个功角，直接用 theta_hat
         * 会让电流矢量瞬间跳变几十度，同样有冲击）。
         * 平滑过渡：切换瞬间 theta = theta_hat + offset 正好等于原来的开环角（角度连续），
         * 之后 offset 逐拍衰减到 0，theta 就平滑地收敛到观测角。 */
        if ((motor.StrongDragToObs.GeneralMode == CLOSE_LOOP) &&
            (motor.StrongDragToObs.LastGeneralMode != CLOSE_LOOP))
        {
            motor.StrongDragToObs.ThetaOffset = motor.angle.ElectricalAngleThetaOut - motor.pll.theta_hat;
            while (motor.StrongDragToObs.ThetaOffset > PI)  motor.StrongDragToObs.ThetaOffset -= TWO_PI;
            while (motor.StrongDragToObs.ThetaOffset < -PI) motor.StrongDragToObs.ThetaOffset += TWO_PI;
        }

        if (motor.StrongDragToObs.Errflag != 0)
        {
            /* 观测器失效：撤掉速度给定、退回开环，静默 5000 拍后再恢复原目标 */
            if (motor.StrongDragToObs.ErrTimes == 0)
            {
                /* 保存的是"目标值"而不是当前输出：突加转速时如果 Errflag 在低速段触发，
                 * speedOut 还很小，存下来恢复后就等于没给目标，电机就再也起不来了 */
                motor.StrongDragToObs.LastTargetSpeed = motor.tra.TargetSpeed;
                motor.tra.TargetSpeed = 0;
                motor.tra.speedOut = 0;
                motor.tra.speedChangeIncrement = 0;
                motor.tra.speedTargetIncrement = 0;

                /* 退回开环时把观测器一起复位并对齐角度，避免下一次切换再被冲击一次 */
                motor.pll.theta_hat = motor.angle.ElectricalAngleThetaOut;
                motor.pll.omega_hat = 0;
                motor.pll.omega_hat_PLF = 0;
                motor.pll.integral = 0;

                motor.StrongDragToObs.GeneralMode = OPEN_LOOP;
            }
            motor.StrongDragToObs.ErrTimes++;

            if (motor.StrongDragToObs.ErrTimes > 5000)
            {
                motor.StrongDragToObs.ErrTimes = 0;
                motor.StrongDragToObs.Errflag = 0;
                motor.tra.TargetSpeed = motor.StrongDragToObs.LastTargetSpeed;
            }
        }
        else if (motor.StrongDragToObs.GeneralMode == CLOSE_LOOP)
        {
            /* 闭环后让角度发生器跟着观测角走，便于异常退回开环时位置连续 */
            motor.angle.ElectricalAngleThetaOut = motor.pll.theta_hat;
        }

        /* ---- 开环强拖失步检测与自动重启 ----
         * 失步时角度发生器还在按给定"空转"（speedOut、CheckCnt 看起来都正常），
         * 但转子不转、电流拖不动，没有这个检测就会永远卡住（就是"被吸住"）。
         * 判据：已经不在加速阶段(匀速/减速)、给定速度 >800 电 RPM、
         *       而观测速度不到给定的 30%，持续 100ms。
         * 注意必须排除 ACCELERATE：加速时转子滞后 + PLL 收敛慢是正常现象，
         * 若在加速阶段检测会误判失步、反复清速度，导致永远切不到闭环。 */
        if (motor.StrongDragToObs.GeneralMode == OPEN_LOOP)
        {
            static u16 stall_cnt = 0u;

            if ((motor.StrongDragToObs.state != ACCELERATE) &&
                (motor.StrongDragToObs.EleOpenSpeedAbs > 800.0f) &&
                (motor.StrongDragToObs.EleCloseSpeedAbs < (motor.StrongDragToObs.EleOpenSpeedAbs * 0.3f)))
            {
                if (stall_cnt < 2000u) stall_cnt++;
            }
            else
            {
                stall_cnt = 0u;
            }

            if (stall_cnt >= 2000u)
            {
                /* 只清速度斜坡输出（TargetSpeed 保留电位器给定），下一拍从 0 重新拉起；
                 * 同时复位观测器，避免带着失步期间攒下的假速度再切一次 */
                stall_cnt = 0u;
                motor.tra.speedOut = 0;
                motor.tra.speedChangeIncrement = 0;
                motor.tra.speedTargetIncrement = 0;
                motor.pll.theta_hat = motor.angle.ElectricalAngleThetaOut;
                motor.pll.integral = 0;
                motor.pll.omega_hat = 0;
                motor.pll.omega_hat_PLF = 0;
                motor.foc.pid_id.integral = 0;
                motor.foc.pid_iq.integral = 0;
            }
        }
    }
    /** 反 Park + SVPWM + 写比较寄存器（所有模式输出都走这里） */
    static void Foc_Output(void)
    {
        motor.foc.ops->antiPark(&motor.foc);
        motor.foc.V_bus = motor.sample.Vbus;
        motor.foc.ops->svpwm(&motor.foc);

        motor.driver.ccr_a = motor.foc.ccr_a;
        motor.driver.ccr_b = motor.foc.ccr_b;
        motor.driver.ccr_c = motor.foc.ccr_c;
        motor.driver.ops->SetPWM(&motor.driver);

        /* ---- 逐拍累加电角度（供调试观测）----
         * 放在 20kHz 中断里做差分累加，每拍只转 0.03 rad，远小于 π，
         * 因此可以正确展开；而主循环 10ms 才取一次，每拍转 5.97 rad > π，
         * 已经超过奈奎斯特极限，展开会失败（表现出来的就是"角度几乎不变"）。 */
        {
            static float theta_prev = 0.0f;
            float dth = motor.pll.theta_hat - theta_prev;

            if (dth >  PI) dth -= TWO_PI;
            if (dth < -PI) dth += TWO_PI;
            theta_prev = motor.pll.theta_hat;

            motor.theta_acc += dth;
            if (motor.theta_acc >  100000.0f) motor.theta_acc -= 100000.0f;
            if (motor.theta_acc < -100000.0f) motor.theta_acc += 100000.0f;
        }
    }
    /**
     * 电流开环强拖
     */
    static void Strong_Drag_Current_Open_Loop(void)
    {
        // 获取目标速度
        motor.tra.TargetSpeed = motor.sample.washer_vlaue;
        motor.tra.ops->TAccDac(&motor.tra);
        // 将梯形速度曲线的输出即电角速度作为电角度发生器的输入，生成电角度
        motor.angle.ElectricalAngleSpeedSet = motor.tra.speedOut;
        motor.angle.ops->AngleGenerator(&motor.angle);
        // 将电角度发生器的输出值即电角度作为FOC的角度值进行反Park变换，更新母线电压后进行零序注入
        motor.foc.theta = motor.angle.ElectricalAngleThetaOut;
        motor.foc.ops->antiPark(&motor.foc);
        motor.foc.V_bus = motor.sample.Vbus;
        motor.foc.ops->svpwm(&motor.foc);
        // 把比较值写给定时器，否则 CCR 不会更新，PWM 没有输出
        motor.driver.ccr_a = motor.foc.ccr_a;
        motor.driver.ccr_b = motor.foc.ccr_b;
        motor.driver.ccr_c = motor.foc.ccr_c;
        motor.driver.ops->SetPWM(&motor.driver);
    }
    /**
     * 开环强拖（电流闭环形式）：角度由角度发生器开环给定，q 轴给固定电流拖动转子
     * 同时并行运行 SMO+PLL，只用于判断什么时候可以切闭环
     */
    static void Strong_Drag_Current_Close_Loop(void)
    {
        Foc_SpeedRamp();                        /* 梯形曲线 + 开环角度发生器 */

		#if (VF_DRAG_TEST != 0)
			/* ---- V/F 开环强拖测试（userDef.h 里 VF_DRAG_TEST 置 1 时启用）----
			 * 只保留"角度开环旋转 + 反 Park + SVPWM"：
			 *   Vd/Vq 固定幅值 → V_alpha/V_beta 随 theta 旋转 → 三相电压矢量跟着转，
			 *   转子被这个旋转电压矢量拖着走（最原始的 V/F 拖动）。
			 * 完全绕过电流环、SMO、PLL、切换判据，用来单独验证
			 *   驱动使能 / PWM 输出 / 相序 / 角度方向 这条硬件通路。 */
			motor.foc.theta = motor.angle.ElectricalAngleThetaOut;
			motor.foc.Vd = 0.0f;
			motor.foc.Vq = 0.35f;    /* 0.35V / 0.18Ω ≈ 1.9A 峰值，配限流电源 2A 使用 */
			Foc_Output();
		#else
        Foc_Sample();                           /* 相电流 -> clarke */
        Foc_Observer();                         /* SMO + PLL + 切换判据 */

        motor.foc.theta = motor.angle.ElectricalAngleThetaOut;   /* 开环：用发生器给的角度 */
        motor.foc.ops->park(&motor.foc);

        motor.foc.Id_LPF = motor.foc.Id * motor.foc.IdLPFFactor + motor.foc.Id_LPF * (1 - motor.foc.IdLPFFactor);
        motor.foc.Iq_LPF = motor.foc.Iq * motor.foc.IqLPFFactor + motor.foc.Iq_LPF * (1 - motor.foc.IqLPFFactor);

        if (fabsf(motor.foc.pid_speed.target) < 10)
        {
            /* 速度给定接近 0：撤掉电流、复位观测器，避免静止时通电发热 */
            motor.foc.pid_id.target = 0;
            motor.foc.pid_iq.target = 0;
            motor.foc.pid_id.integral = 0;
            motor.foc.pid_iq.integral = 0;
            motor.foc.pid_speed.integral = 0;
            motor.foc.pid_speed.output = 0;
            motor.smo.e_alpha_hat_plf = 0;
            motor.smo.e_beta_hat_plf = 0;
            motor.pll.omega_hat_PLF = 0;
            motor.pll.omega_hat = 0;
            motor.pll.integral = 0;     /* 位置式 PI 的积分必须一起清，否则会把错误速度"抱住" */
            motor.pll.theta_hat = 0;
        }
        else
        {
            motor.foc.pid_id.target = 0;
            motor.foc.pid_iq.target = motor.StrongDragToObs.OpenCurr;   /* q 轴建力矩 */
        }
        motor.foc.pid_id.feedback = motor.foc.Id_LPF;
        motor.foc.pid_iq.feedback = motor.foc.Iq_LPF;
        motor.foc.ops->pid(&motor.foc);

        motor.foc.Vd = motor.foc.pid_id.output;
        motor.foc.Vq = motor.foc.pid_iq.output;
        Foc_Output();
#endif
    }
    /**
     * 滑膜观测器 + 速度电流双闭环
     * 角度取自 PLL 观测角；q 轴电流给定来自速度环输出
     */
    static void Strong_Drag_Smo_Speed_Current_Loop(void)
    {
        Foc_SpeedRamp();                    /* 梯形曲线 + 角度发生器（闭环时被观测角接管） */
        Foc_Sample();                       /* 相电流 -> clarke */
        Foc_Observer();                     /* SMO + PLL + 切换判据 */

        /* 闭环角度 = 观测角 + 切换补偿；补偿每拍衰减 0.5%，约 10ms 平滑收敛到 0。
         * 切换瞬间 theta 与开环角完全连续，之后逐渐转到观测角，避免"颠一下"。 */
        motor.foc.theta = motor.pll.theta_hat + motor.StrongDragToObs.ThetaOffset;
        motor.StrongDragToObs.ThetaOffset *= 0.995f;
        motor.foc.ops->park(&motor.foc);

        motor.foc.Id_LPF = motor.foc.Id * motor.foc.IdLPFFactor + motor.foc.Id_LPF * (1 - motor.foc.IdLPFFactor);
        motor.foc.Iq_LPF = motor.foc.Iq * motor.foc.IqLPFFactor + motor.foc.Iq_LPF * (1 - motor.foc.IqLPFFactor);

        /* 速度环：只在闭环执行（对齐参考工程，开环强拖阶段不跑速度环，避免积分空累积） */
        motor.foc.ops->speedPid(&motor.foc);

        /* 开环切闭环的瞬间，把速度环积分/输出预置成当前强拖电流，避免切换冲击。
         * 必须放在 speedPid() 之后，否则本拍刚算出的输出会被覆盖成旧值 */
        if (motor.StrongDragToObs.LastGeneralMode == OPEN_LOOP)
        {
            motor.foc.pid_speed.integral = motor.StrongDragToObs.OpenCurr;
            motor.foc.pid_speed.output = motor.StrongDragToObs.OpenCurr;
        }

        motor.foc.pid_id.target = 0;
        motor.foc.pid_iq.target = motor.foc.pid_speed.output;
        motor.foc.pid_id.feedback = motor.foc.Id_LPF;
        motor.foc.pid_iq.feedback = motor.foc.Iq_LPF;
        motor.foc.ops->pid(&motor.foc);

        motor.foc.Vd = motor.foc.pid_id.output;
        motor.foc.Vq = motor.foc.pid_iq.output;
        Foc_Output();
    }
    /**
     * 应用层组装：一个控制周期只按当前运行模式走一条路径
     * 状态取自 StrongDragToObs.GeneralMode：校准完成时由 motor.GeneralMode 同步过来，
     * 之后由 Strong_Drag_To_Observer_Cal() 在 OPEN_LOOP / CLOSE_LOOP 之间切换。
     * LastGeneralMode 在这里统一更新，供下一次进入模式函数时判断是否刚刚切换过。
     */
    static void motorFocISR(void)
    {
        /* ---- 系统保护：母线欠压/过压 + 相过流，任一成立立刻封锁驱动并清掉闭环状态 ----
         * 与参考工程 Motor_System_Run() 里的 MOTOR_ERROR 分支等价：
         * 断电(Vbus=0)、母线异常、或任一相过流时，宁可停机也不能继续输出电压 */
        static u8 drv_enabled = 0u;

        if ((motor.sample.Vbus < BUS_VOLTAGE_MIN) || (motor.sample.Vbus > BUS_VOLTAGE_MAX) ||
            (fabsf(motor.sample.Ia) > OVER_CURRENT) ||
            (fabsf(motor.sample.Ib) > OVER_CURRENT) ||
            (fabsf(motor.sample.Ic) > OVER_CURRENT))
        {
            if (drv_enabled)
            {
                motor.driver.ops->motorDisable(&motor.driver);
                drv_enabled = 0u;
            }
            /* 清掉所有积分，避免故障恢复的瞬间带着旧积分冲击 */
            motor.foc.pid_id.integral = 0;
            motor.foc.pid_iq.integral = 0;
            motor.foc.pid_speed.integral = 0;
            motor.foc.pid_id.output = 0;
            motor.foc.pid_iq.output = 0;
            motor.foc.pid_speed.output = 0;
            motor.pll.integral = 0;
            motor.StrongDragToObs.GeneralMode = OPEN_LOOP;
            motor.StrongDragToObs.LastGeneralMode = OPEN_LOOP;
            return;
        }
        /* 条件恢复后重新使能（只在状态变化时写一次 GPIO） */
        if (!drv_enabled)
        {
            motor.driver.ops->motorEnable(&motor.driver);
            drv_enabled = 1u;
        }

        switch (motor.StrongDragToObs.GeneralMode)
        {
            case OPEN_LOOP:
                Strong_Drag_Current_Close_Loop();       /* 开环强拖 */
                break;
            case CLOSE_LOOP:
                Strong_Drag_Smo_Speed_Current_Loop();   /* 滑膜 + 速度电流闭环 */
                break;
            case MOTOR_CALIB:
            default:
                break;                                  /* 校准期只采样，不输出 */
        }
        motor.StrongDragToObs.LastGeneralMode = motor.StrongDragToObs.GeneralMode;
    }
#elif (MOTOR_CONTROLER == FOC_SENSOR)       // 有感算法

#endif
/*************************************************************************************************************************/
/*********************************************************中断函数********************************************************/
/*************************************************************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        motor.sample.ops->MotorVbus(&motor.sample);
        motor.sample.ops->MotorTemp(&motor.sample);
        motor.sample.ops->Washer(&motor.sample);
    }
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        motor.sample.g_adc_buf[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        motor.sample.g_adc_buf[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
        motor.sample.ops->MotorCurrent(&motor.sample);
        // 零点校准成功
        if (motor.sample.cali_flag)
        {
            if (motor.GeneralMode == MOTOR_CALIB)
            {
                motor.GeneralMode = OPEN_LOOP;      // 进入开环强拖
                motor.StrongDragToObs.GeneralMode = motor.GeneralMode;
            }
            motorFocISR();      // 应用层：保护检查 + 驱动使能 + 模式分派
        }
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_ADCEx_InjectedStart_IT(&hadc2);
    }
}