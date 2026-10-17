/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-10 22:09:56
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-15 10:09:57
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\Src\sysControl.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-26 17:08:38
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-15 09:55:30
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC\motorControl\Src\motor.c
 * @Description: 系统控制层：统一初始化所有模块结构体的数据，并编排一个 FOC 控制周期
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
        .motor_state   = MOTOR_UNIFORM,
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
    
    .state = MOTOR_CALIB,
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
    /* TODO: 启动 PWM 输出、启动 ADC-DMA 采样（本层框架不接管外设启动） */
    return REGISTER_SUCCESS;
}
/*************************************************************************************************************************/
/*******************************************************FOC 控制周期******************************************************/
/*************************************************************************************************************************/
/**
 * 电流开环强拖
 */
static void Strong_Drag_Current_Open_Loop(void)
{
    if (motor.state == MOTOR_DEFAULT)
    {
        motor.driver.ops->motorDisable(&motor.driver);
        return;
    }
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
    if (motor.state == MOTOR_DEFAULT)
    {
        motor.driver.ops->motorDisable(&motor.driver);
        return;
    }
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
    if (motor.state == MOTOR_DEFAULT)
    {
        motor.driver.ops->motorDisable(&motor.driver);
        return;
    }
    // 梯形加减速
    motor.tra.TargetSpeed = motor.sample.washer_vlaue;
    motor.tra.ops->TAccDac(&motor.tra);
    // 电角度发生器
    motor.angle.ElectricalAngleSpeedSet = motor.tra.speedOut;
    motor.angle.ops->AngleGenerator(&motor.angle);
    // 滑膜观测器
    motor.smo.I_alpha = motor.foc.I_alpha;
    motor.smo.I_beta = motor.foc.I_beta;
    motor.smo.U_alpha = motor.foc.V_alpha;
    motor.smo.U_beta = motor.foc.V_beta;
    motor.smo.ops->SMO(&motor.smo);
    // 锁相环
    motor.pll.A_in = motor.smo.e_alpha_hat_plf;
    motor.pll.B_in = motor.smo.e_beta_hat_plf;
    motor.pll.Dir = motor.tra.speedOutDir;
    motor.pll.ops->PLL(&motor.pll);
    // foc控制
    motor.foc.Ia = motor.sample.Ia;
    motor.foc.Ib = motor.sample.Ib;
    motor.foc.Ic = motor.sample.Ic;
    motor.foc.V_bus = motor.sample.Vbus;
    motor.foc.theta = motor.pll.theta_hat;
    motor.foc.speed_ref = motor.tra.speedOut;
    motor.foc.speed_fb = motor.pll.omega_hat_PLF / TWO_PI * 60.0f;  // 转换为电角速度
    motor.foc.ops->clarke(&motor.foc);
    motor.foc.ops->park(&motor.foc);
    motor.foc.ops->pid(&motor.foc);
    motor.foc.ops->antiPark(&motor.foc);
    motor.foc.ops->svpwm(&motor.foc);
    // pwm输出
    motor.driver.ccr_a = motor.foc.ccr_a;
    motor.driver.ccr_b = motor.foc.ccr_b;
    motor.driver.ccr_c = motor.foc.ccr_c;
    motor.driver.ops->SetPWM(&motor.driver);
}
static void motorFocISR(void)
{

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
            if (motor.state == MOTOR_CALIB)
            {
                motor.driver.ops->motorEnable(&motor.driver);   // 使能PWM
                motor.state = MOTOR_ALIGN;      // 进入开环强拖
            }
            motorFocISR();
        }
    }
}