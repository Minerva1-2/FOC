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
    MOTOR_DEFAULT,          // 运行错误
    MOTOR_ALIGN,            // 开环强拖
    MOTOR_RUN,              // 闭环运行
}State_t;

typedef struct
{
    State_t   state;
    FOC_t     foc;
    SMO_t     smo;
    PLL_t     pll;
    TAccDac_t tra;
    Angle_t   angle;
    DRIVER_t  driver;
    SAMPLE_t  sample;
}Motor_t;

/* 全局唯一电机实例：所有模块的数据都在 sysControl.c 中统一初始化 */
extern Motor_t motor;
/* 一次性初始化：绑定各模块 ops 函数表并完成上电初始化，失败返回 REGISTER_ERROR */
int motorInit(void);
/* 一个 FOC 控制周期（在控制中断中调用）：依次驱动观测器、PLL、电流环、SVPWM */
void motorFocISR(void);

#endif // !__SYSCONTROL_H
