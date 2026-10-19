/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-10 22:09:56
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-19 17:43:25
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\Src\sysControl.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
        .gain          = SMO_GAIN,
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
};
/*************************************************************************************************************************/
/********************************************************初始化接口*******************************************************/
/*************************************************************************************************************************/
static int motorInit(void)
{
    /* 1. 绑定各模块的算法函数表（配置数据已在上面的初始化器中静态就绪） */
    if (foc_register(&motor.foc)                   != REGISTER_SUCCESS) return REGISTER_ERROR;
    if (smo_register(&motor.smo, &motor.pll)       != REGISTER_SUCCESS) return REGISTER_ERROR;
    if (TAccDac_register(&motor.tra, &motor.angle) != REGISTER_SUCCESS) return REGISTER_ERROR;
    if (driver_register(&motor.driver)             != REGISTER_SUCCESS) return REGISTER_ERROR;
    if (sample_register(&motor.sample)             != REGISTER_SUCCESS) return REGISTER_ERROR;

    /* 2. 上电初始化：先关断驱动，等待采样零点校准完成后再使能 */
    motor.driver.ops->motorDisable(&motor.driver);
    motor.driver.ops->LedOn(&motor.driver);
    // 开启中断
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&motor.sample.g_adc_buf[2], 3);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
    /* 启动 PWM 输出、启动 ADC-DMA 采样（本层框架不接管外设启动） */
    return REGISTER_SUCCESS;
}
/*************************************************************************************************************************/
/*******************************************************FOC 控制周期******************************************************/
/*************************************************************************************************************************/
#if (MOTOR_CONTROLER == FOC_SENSORLESS)
//  无感算法
    static void Strong_Drag_To_Observer_Cal(StrongDragToObs_t *p)
    {
        // 1.计算角度误差
        p->ThetaErr = p->ThetaRef - p->ThetaRef;
        p->ThetaErr = p->ThetaErr > PI ? p->ThetaErr - PI : p->ThetaErr;
        p->ThetaErr = p->ThetaErr < -PI ? p->ThetaErr + PI : p->ThetaErr;
        // 2.计算速度误差
        p->SpeedErr = p->OpenSpeed - p->CloseSpeed;
        p->SpeedErrFlt += 0.1f * (p->SpeedErr - p->SpeedErrFlt);
        // 3.取开环速度以及闭环速度的绝对值
        p->EleOpenSpeedAbs = fabsf(p->OpenSpeed);
        p->EleCloseSpeedAbs = fabsf(p->CloseSpeed);
        // 4.更新角度误差检测计数器
        p->CheckCnt = fabsf(p->ThetaErr) <= (PI / 10.0f) ? p->CheckCnt++ : p->CheckCnt--;
        // 5.若当前处于闭环状态，则累加计数器
        if (p->GeneralMode == CLOSE_LOOP)
        {
            p->CloseRunTime++;
            p->CloseRunTime = Sat(p->CloseRunTime, 0, 40000);
        }
        // 6.根据电机状态动态更新电流大小
        p->Curr_change_uint = p->OpenCurrMax * 0.0003f;
        /* 情况一：电机未减速 && 开环速度绝对值 > 切换速度 && 角度收敛周期 < 30
            此时逐渐减小电流以适应闭环切入
        */
        if ((p->state != DECELERATE) && (p->EleOpenSpeedAbs > p->OpenToCloseSwitchSpeed) && (p->CheckCnt < 30))
        {
            p->OpenCurr = Sat(fabsf(p->OpenCurr) - p->Curr_change_uint, 0.08f, p->OpenCurrMax);
        }
        /* 情况2：电机未减速 && 开环速度绝对值 < 切换速度
            此时逐渐增大电流，确保有足够力矩切换到闭环
        */
        if ((p->state == DECELERATE) && (p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed))
        {
            p->OpenCurr = Sat(fabsf(p->OpenCurr) + p->Curr_change_uint, 0.03f, p->OpenCurrMax);
        }
        /* 情况3：电机减速 && 闭环速度最小值 < 开环速度绝对值 < 切换速度 + 5
            此时略微电流，防止失步
        */
        if ((p->state == DECELERATE) && (p->EleOpenSpeedAbs > p->CloseMinSpeed) && (p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed + 5))
        {
            p->OpenCurr = Sat(fabsf(p->OpenCurr) + 0.001f, 0.05f, p->OpenCurrMax);
        }
        /* 情况3：电机减速 && 闭环速度最小值 > 开环速度绝对值
            此时减小电流电流，避免低速下大电流导致电机抖动
        */
        if ((p->state == DECELERATE) && (p->EleOpenSpeedAbs < p->CloseMinSpeed))
        {
            p->OpenCurr = Sat(fabsf(p->OpenCurr) - 0.001f, 0.08f, p->OpenCurrMax);
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
         * 条件B:角度计数器 > 40（即角度误差持续小于0.05超过40个周期） && 开环速度绝对值 > 切换速度
         * 此时满足切换条件，切入到闭环控制
         */
        if ((p->CheckCnt > 40) && (p->EleOpenSpeedAbs > p->OpenToCloseSwitchSpeed))
        {
            p->Errflag = 0;
            p->GeneralMode = CLOSE_LOOP;
        }
        /**
         * 条件C:开环速度绝对值 < 闭环速度最低值
         * 此时速度过低，强制切换为开环模式
         */
        if (p->EleOpenSpeedAbs < p->CloseMinSpeed)
        {
            p->Errflag = 0;
            p->GeneralMode = OPEN_LOOP;
        }
        // 9.限制计数器范围
        if (p->CheckCnt > 40)   p->CheckCnt = 41;
        else if (p->CheckCnt < 0)   p->CheckCnt = 0;
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
    }
    /**
     * 电流闭环强拖
     */
    static void Strong_Drag_Current_Close_Loop(void)
    {
        // 强拖需要建立一定的磁场，给定一个固定电流参考值1A
        motor.foc.Id_ref = 1.0f;
        // 获取目标速度
        motor.tra.TargetSpeed = motor.sample.washer_vlaue;
        motor.tra.ops->TAccDac(&motor.tra);
        // 将梯形速度曲线的输出即电角速度作为电角度发生器的输入，生成电角度
        motor.angle.ElectricalAngleSpeedSet = motor.tra.speedOut;
        motor.angle.ops->AngleGenerator(&motor.angle);
        // 将电角度发生器的输出值即电角度作为FOC的角度值进行clarke以及Park变换
        motor.foc.theta = motor.angle.ElectricalAngleThetaOut;
        motor.foc.Ia = motor.sample.Ia;
        motor.foc.Ic = motor.sample.Ic;
        motor.foc.ops->clarke(&motor.foc);
        motor.foc.ops->park(&motor.foc);
        // 电流一阶低通滤波
        motor.foc.Id_LPF = motor.foc.Id * motor.foc.IdLPFFactor + motor.foc.Id_LPF * (1 - motor.foc.IdLPFFactor);
        motor.foc.Iq_LPF = motor.foc.Iq * motor.foc.IqLPFFactor + motor.foc.Iq_LPF * (1 - motor.foc.IqLPFFactor);
        // 滤波后的电流作为电流环的反馈值，计算出Vd, Vq
        motor.foc.pid_id.feedback = motor.foc.Id_LPF;
        motor.foc.pid_iq.feedback = motor.foc.Iq_LPF;
        motor.foc.ops->pid(&motor.foc);
        motor.foc.ops->pid(&motor.foc);
        // 反park变换以及svpwm调制
        motor.foc.ops->antiPark(&motor.foc);
        motor.foc.V_bus = motor.sample.Vbus;
        motor.foc.ops->svpwm(&motor.foc);
    }
    static void Strong_Drag_Smo_Speed_Current_Loop(void)
{
    motor.smo.gain = motor.sample.Vbus * INV_SQRT3;

    motor.tra.TargetSpeed = motor.sample.washer_vlaue;
    motor.tra.ops->TAccDac(&motor.tra);

    motor.angle.ElectricalAngleSpeedSet = motor.tra.speedOut;
    motor.angle.ops->AngleGenerator(&motor.angle);

    motor.smo.Rs = MOTOR_RS;
    motor.smo.Ld = MOTOR_LD;
    motor.smo.I_alpha = motor.foc.I_alpha;
    motor.smo.I_beta = motor.foc.I_beta;
    motor.smo.U_alpha = motor.foc.V_alpha;
    motor.smo.U_beta = motor.foc.V_beta;
    motor.smo.ops->SMO(&motor.smo);

    motor.pll.Dir = motor.tra.speedOutDir;
    motor.pll.A_in = motor.smo.e_alpha_hat_plf;
    motor.pll.B_in = motor.smo.e_beta_hat_plf;
    motor.pll.ops->PLL(&motor.pll);

    motor.StrongDragToObs.OpenSpeed = motor.tra.speedOut;
    motor.StrongDragToObs.CloseSpeed = motor.pll.omega_hat_PLF / TWO_PI * 60.0f;
    motor.StrongDragToObs.ThetaRef = motor.angle.ElectricalAngleThetaOut;
    motor.StrongDragToObs.ThetaObs = motor.pll.theta_hat;
    motor.StrongDragToObs.state = motor.tra.motor_state;
    Strong_Drag_To_Observer_Cal(&motor.StrongDragToObs);

    if (motor.StrongDragToObs.Errflag != 0)
    {
        if (motor.StrongDragToObs.ErrTimes == 0)
        {
            motor.StrongDragToObs.LastTargetSpeed = motor.tra.speedOut;
            motor.tra.TargetSpeed = 0;
            motor.tra.speedOut = 0;
            motor.tra.speedChangeIncrement = 0;
            motor.tra.speedTargetIncrement = 0;

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
    else
    {
        if (motor.StrongDragToObs.GeneralMode == CLOSE_LOOP)
        {
            motor.angle.ElectricalAngleThetaOut = motor.pll.theta_hat;
        }
    }

    motor.foc.speed_ref = motor.tra.speedOut;
    motor.foc.speed_fb = motor.pll.omega_hat_PLF / TWO_PI * 60.0f;
    if (motor.StrongDragToObs.GeneralMode == CLOSE_LOOP)
    {
        motor.foc.ops->pid(&motor.foc);
        if (motor.StrongDragToObs.LastGeneralMode == OPEN_LOOP)
        {
            motor.foc.pid_speed.integral = motor.StrongDragToObs.OpenCurr;
            motor.foc.pid_speed.output = motor.StrongDragToObs.OpenCurr;
        }
    }
    motor.StrongDragToObs.LastGeneralMode = motor.StrongDragToObs.GeneralMode;

    motor.foc.Ia = motor.sample.Ia;
    motor.foc.Ib = motor.sample.Ib;
    motor.foc.Ic = motor.sample.Ic;
    motor.foc.ops->clarke(&motor.foc);

    if (motor.StrongDragToObs.GeneralMode == OPEN_LOOP)
        motor.foc.theta = motor.angle.ElectricalAngleThetaOut;
    else if (motor.StrongDragToObs.GeneralMode == OPEN_LOOP)
        motor.foc.theta = motor.pll.theta_hat;
    motor.foc.ops->park(&motor.foc);

    motor.foc.Id_LPF = motor.foc.Id * motor.foc.IdLPFFactor + motor.foc.Id_LPF * (1 - motor.foc.IdLPFFactor);
    motor.foc.Iq_LPF = motor.foc.Iq * motor.foc.IqLPFFactor + motor.foc.Iq_LPF * (1 - motor.foc.IqLPFFactor);

    if (motor.StrongDragToObs.GeneralMode == OPEN_LOOP)
    {
        if (fabsf(motor.foc.pid_speed.target) < 10)
        {
            motor.foc.pid_speed.integral = 0;
            motor.foc.pid_speed.output = 0;
            motor.foc.pid_speed.target = 0;
            motor.smo.e_alpha_hat_plf = 0;
            motor.smo.e_beta_hat_plf = 0;
            motor.pll.omega_hat_PLF = 0;
        }
        else
        {
            motor.foc.pid_id.target = motor.StrongDragToObs.OpenCurr;
            motor.foc.pid_iq.target = 0;
        }
    }
    else
    {
        motor.foc.pid_iq.target = motor.foc.pid_speed.output;
        motor.foc.pid_id.target = 0;
    }
    motor.foc.pid_id.feedback = motor.foc.Id_LPF;
    motor.foc.pid_iq.feedback = motor.foc.Iq_LPF;
    motor.foc.ops->pid(&motor.foc);

    motor.foc.Vd = motor.foc.pid_id.output;
    motor.foc.Vq = motor.foc.pid_iq.output;
    motor.foc.ops->antiPark(&motor.foc);

    motor.foc.V_bus = motor.sample.Vbus;
    motor.foc.ops->svpwm(&motor.foc);
}
#elif (MOTOR_CONTROLER == FOC_SENSOR)
// 有感算法
#endif

static void motorFocISR(void)
{
    Strong_Drag_Smo_Speed_Current_Loop();

    motor.driver.ccr_a = motor.foc.ccr_a;
    motor.driver.ccr_b = motor.foc.ccr_b;
    motor.driver.ccr_c = motor.foc.ccr_c;
    motor.driver.ops->SetPWM(&motor.driver);
}
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
                motor.driver.ops->motorEnable(&motor.driver);   // 使能PWM
                motor.GeneralMode = OPEN_LOOP;      // 进入开环强拖
            }
            motorFocISR();
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