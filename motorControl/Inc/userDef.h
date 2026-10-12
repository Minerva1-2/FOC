/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-24 20:42:33
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-10 22:27:16
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC\motorControl\userDef.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __USERDEF_H
#define __USERDEF_H

#include <stdint.h>

#define __constrain__(value, min, max)    ((value) > (max) ? (max) : ((value) < (min) ? (min) : (value)))
/*************************************************************************************************************************/
/*****************************************************控制算法类型定义*****************************************************/
/*************************************************************************************************************************/
#define FOC_SENSORLESS          0       // 无感算法
#define FOC_SENSOR              1       // 有感算法

#define MOTOR_CONTROLER         FOC_SENSORLESS
/*************************************************************************************************************************/
/*********************************************************基本参数*********************************************************/
/*************************************************************************************************************************/
#define MOTOR_PAIRS             7                       // 电机极对数
#define CONTROLER_TIME          0.000005f               // FOC运行时间
#define PI                      3.1415926f
#define TWO_PI                  6.2831852f
#define MOTOR_RS                0.0f        // TODO: 定子相电阻 (Ω)
#define MOTOR_LD                0.0f        // TODO: 定子相电感 (H)
/*******************************************************滑膜观测器(SMO)****************************************************/
#define SMO_GAIN                0.0f        // TODO: 滑膜增益
#define SMO_PHI                 1.0f        // 滑膜边界层（饱和函数阈值）
#define SMO_FC                  0.0f        // TODO: 反电势低通滤波截止频率 (Hz)
#define SMO_E_PLF_FACTOR        0.0f        // TODO: 反电势一阶低通滤波系数 (0~1)
/*********************************************************锁相环(PLL)******************************************************/
#define PLL_DIR                 1           // 旋转方向：+1 / -1
#define PLL_KP                  0.0f        // TODO: PLL 比例增益
#define PLL_KI                  0.0f        // TODO: PLL 积分增益
#define PLL_PLF_FACTOR          0.1f        // TODO: 估算角速度一阶低通滤波系数 (0~1)
/********************************************************电流环PID*********************************************************/
#define PID_ID_KP               0.0f        // TODO: d 轴电流环比例增益
#define PID_ID_KI               0.0f        // TODO: d 轴电流环积分增益
#define PID_ID_KD               0.0f        // d 轴电流环微分增益
#define PID_ID_LIMIT            0.0f        // TODO: d 轴电压输出限幅 (V)

#define PID_IQ_KP               0.0f        // TODO: q 轴电流环比例增益
#define PID_IQ_KI               0.0f        // TODO: q 轴电流环积分增益
#define PID_IQ_KD               0.0f        // q 轴电流环微分增益
#define PID_IQ_LIMIT            0.0f        // TODO: q 轴电压输出限幅 (V)
/*********************************************************速度环PID*********************************************************/
#define PID_SPEED_KP            0.0f        // TODO: 速度环比例增益
#define PID_SPEED_KI            0.0f        // TODO: 速度环积分增益
#define PID_SPEED_KD            0.0f        // 速度环微分增益
#define PID_SPEED_LIMIT         0.0f        // TODO: 速度环输出（q 轴电流给定）限幅 (A)
#define FOC_SPEED_DIV           10u         // 速度环分频比：速度环周期 = 电流环周期 × FOC_SPEED_DIV
/******************************************************梯形加减速(TAccDac)**************************************************/
#define TRA_TARGET_SPEED        0.0f        // 目标转速 (RPM)
#define TRA_ACC_SPEED           0.0f        // 加速度 (RPM/s)
/*********************************************************硬件引脚绑定******************************************************/
/* 电机驱动芯片使能引脚 */
#define MOTOR_A_PORT            GPIOC
#define MOTOR_A_PIN             GPIO_PIN_13
#define MOTOR_B_PORT            GPIOC
#define MOTOR_B_PIN             GPIO_PIN_14
#define MOTOR_C_PORT            GPIOC
#define MOTOR_C_PIN             GPIO_PIN_15
#define MOTOR_D_PORT            GPIOB
#define MOTOR_D_PIN             GPIO_PIN_0

/* 指示灯引脚 */
#define LED_A_PORT              GPIOA
#define LED_A_PIN               GPIO_PIN_0
#define LED_B_PORT              GPIOA
#define LED_B_PIN               GPIO_PIN_1
#define LED_C_PORT              GPIOA
#define LED_C_PIN               GPIO_PIN_2
/*************************************************************************************************************************/
/********************************************************变量重定义********************************************************/
/*************************************************************************************************************************/
#define u8                      uint8_t
#define u16                     uint16_t
#define u32                     uint32_t

#define s8                      int8_t

#define REGISTER_ERROR          -1          // 函数注册失败
#define REGISTER_SUCCESS        0           // 函数注册成功
/*************************************************************************************************************************/
/******************************************************宏定义错误检查******************************************************/
/*************************************************************************************************************************/
#ifndef MOTOR_CONTROLER                     // 没有定义电机控制算法
    #error "MOTOR_CONTROLER no defined..."
#endif // #ifndef FOC_SENEORLESS

#endif // #ifndef __USERDEF_H
