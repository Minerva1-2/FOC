#ifndef __DRIVER_H
#define __DRIVER_H

#include "userDef.h"
#include "gpio.h"
#include "tim.h"

typedef struct DRIVER DRIVER_t;

typedef struct
{
    const char *name;
    void (*motorEnable)(DRIVER_t *p);
    void (*motorDisable)(DRIVER_t *p);
    void (*LedOn)(DRIVER_t *p);
    void (*LedOff)(DRIVER_t *p);
    void (*SetPWM)(DRIVER_t *p);
    /* data */
}DRIVER_OPS_t;

struct DRIVER
{
    const DRIVER_OPS_t *ops;
    // 电机驱动芯片引脚
    GPIO_TypeDef *motor_a;
    GPIO_TypeDef *motor_b;
    GPIO_TypeDef *motor_c;
    GPIO_TypeDef *motor_d;
    u16 motor_pin_a;
    u16 motor_pin_b;
    u16 motor_pin_c;
    u16 motor_pin_d;
    // led驱动引脚
    GPIO_TypeDef *led_a;
    GPIO_TypeDef *led_b;
    GPIO_TypeDef *led_c;
    u16 led_pin_a;
    u16 led_pin_b;
    u16 led_pin_c;
    TIM_HandleTypeDef *htim;
    u16 ccr_a;
    u16 ccr_b;
    u16 ccr_c;
};

int driver_register(DRIVER_t *p);

#endif
