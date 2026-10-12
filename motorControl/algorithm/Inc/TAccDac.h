/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-10 17:18:17
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-10 21:41:07
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\algorithm\Inc\TAccDac.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __TACCDAC_H
#define __TACCDAC_H

#include <string.h>
#include <math.h>
#include "userDef.h"

typedef struct TAccDac TAccDac_t;
typedef struct Angle Angle_t;

typedef struct
{
    const char *name;
    void (*TAccDac)(TAccDac_t *p);
}TAccDac_OPS_t;
typedef struct
{
    const char *name;
    void (*AngleGenerator)(Angle_t *p);
}Angle_OPS_t;

typedef enum
{
    MOTOR_ACC = 0U,
    MOTOR_DAC,
    MOTOR_UNIFORM,
}MOTOR_STATE;

struct TAccDac
{
    const TAccDac_OPS_t *ops;

    float Ts;
    float TargetSpeed;              // 目标速度，由外部指定
    float accSpeed;                 // 加速度
    float speedTargetIncrement;     // 目标速度增量
    float speedIncrement;           // 目标加速度增量
    float speedLastIncrement;       // 上一个速度增量
    float speedChangeIncrement;     // 变速阶段速度增量
    float speedDeltaIncrement;      // 变化的速度增量
    float speedOutDir;              // 速度方向
    float speedOut;                 // 输出速度

    MOTOR_STATE motor_state;        // 状态设置
};

struct Angle
{
    const Angle_OPS_t *ops;

    float Ts;
    float ElectricalAngleSpeedSet;  // 电角度速度
    float ElectricalAngleThetaOut;  // 电角度位置
};

int TAccDac_register(TAccDac_t *p, Angle_t *q);

#endif
