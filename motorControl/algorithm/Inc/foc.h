/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-26 16:44:41
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-09-26 16:58:40
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC\motorControl\algorithm\Inc\foc.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __FOC_H
#define __FOC_H

#include "math.h"
#include "userDef.h"

#define ONE_BY_SQRT3    0.577350269f
#define SQRT3_DIV2      0.86602540378f

typedef struct FOC FOC_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_limit;
    float output;
}PID_t;

typedef struct
{
    const char *name;
    void (*clarke)(FOC_t *p);
    void (*park)(FOC_t *p);
    void (*pid)(FOC_t *p);
    void (*antiPark)(FOC_t *p);
    void (*svpwm)(FOC_t *p);
}FOC_OPS_t;

struct FOC
{
    // 算法表
    const FOC_OPS_t *ops;
    // 运行时间：1/频率，该工程中通过宏定义CONTROLER_TIME赋值
    float Ts;
    // 电流、电压
    float Ia, Ib, Ic;
    float I_alpha, I_beta;
    float V_alpha, V_beta;
    float Id, Iq;
    float Vd, Vq;
    float Va, Vb, Vc;
    float V_bus;
    // 电气角度
    float theta;
    // 速度环PID参数
    PID_t pid_speed;    // 速度环
    PID_t pid_id;       // d轴电流环
    PID_t pid_iq;       // q轴电流环
    float Id_ref;       // d轴电流给定值
    float Iq_ref;       // q轴电流给定值
    float speed_ref;
    float speed_fb;
    u16 speed_div_cnt;  // 速度环计数器
    u16 speed_div;      // 速度环分频比
    // 占空比
    float duty_a, duty_b, duty_c;
};

/* 实例注册接口：应用层初始化时调用一次 foc_register()，中断里用 foc_get() 取用 */
int foc_register(FOC_t *p);

#endif // #ifndef __FOC_H
