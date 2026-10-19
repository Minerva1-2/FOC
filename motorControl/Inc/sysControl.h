/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-19 10:06:58
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-19 14:24:51
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\Inc\sysControl.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYSCONTROL_H
#define __SYSCONTROL_H

#include "userDef.h"
#include "foc.h"
#include "observer.h"
#include "TAccDac.h"
#include "driver.h"
#include "sample.h"

typedef enum
{
    MOTOR_CALIB = 0,        // 电流校准
    OPEN_LOOP,              // 开环
    CLOSE_LOOP,             // 闭环
}GeneralMode_t;

typedef struct
{
    GeneralMode_t GeneralMode;
    GeneralMode_t LastGeneralMode;
    MOTOR_STATE   state;
    u8  Errflag;
    u16 ErrCnt;
    u16 ErrTimes;
    u16 CloseRunTime;
    u16 CheckCnt;
    float OpenCurr;
    float OpenCurrLast;
    float OpenCurrMax;
    float ThetaRef;
    float ThetaObs;
    float ThetaErr;
    float OpenSpeed;
    float CloseSpeed;
    float EleOpenSpeedAbs;
    float EleCloseSpeedAbs;
    float SpeedErr;
    float SpeedErrFlt;
    float LastTargetSpeed;
    float Curr_change_uint;
    float OpenToCloseSwitchSpeed;
    float CloseMinSpeed;
}StrongDragToObs_t;

typedef struct
{
    GeneralMode_t GeneralMode;
    FOC_t       foc;
    SMO_t       smo;
    PLL_t       pll;
    TAccDac_t   tra;
    Angle_t     angle;
    DRIVER_t    driver;
    SAMPLE_t    sample;
    StrongDragToObs_t StrongDragToObs;
}Motor_t;

// /* 全局唯一电机实例：所有模块的数据都在 sysControl.c 中统一初始化 */
// extern Motor_t motor;

static inline float Sat(float value, float min, float max){
    if(value >= max){
        return max;
    }
    if(value <= min){
        return min;
    }
    return value;
}

#endif // !__SYSCONTROL_H
