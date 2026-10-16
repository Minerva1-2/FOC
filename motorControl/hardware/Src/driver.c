/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-10 22:10:01
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-14 09:27:01
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\hardware\Src\driver.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "driver.h"

void motorEnable(DRIVER_t *p)
{
    HAL_GPIO_WritePin(p->motor_a, p->motor_pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p->motor_b, p->motor_pin_b, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p->motor_c, p->motor_pin_c, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p->motor_d, p->motor_pin_d, GPIO_PIN_SET);
}
void motorDisable(DRIVER_t *p)
{
    HAL_GPIO_WritePin(p->motor_a, p->motor_pin_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->motor_b, p->motor_pin_b, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->motor_c, p->motor_pin_c, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->motor_d, p->motor_pin_d, GPIO_PIN_RESET);
}
void LedOn(DRIVER_t *p)
{
    HAL_GPIO_WritePin(p->led_a, p->led_pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p->led_b, p->led_pin_b, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p->led_c, p->led_pin_c, GPIO_PIN_SET);
}
void LedOff(DRIVER_t *p)
{
    HAL_GPIO_WritePin(p->led_a, p->led_pin_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->led_b, p->led_pin_b, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->led_c, p->led_pin_c, GPIO_PIN_RESET);
}
static void SetPWM(DRIVER_t *p)
{
    __HAL_TIM_SET_COMPARE(p->htim, TIM_CHANNEL_1, p->ccr_a);
    __HAL_TIM_SET_COMPARE(p->htim, TIM_CHANNEL_2, p->ccr_b);
    __HAL_TIM_SET_COMPARE(p->htim, TIM_CHANNEL_3, p->ccr_c);
}
const DRIVER_OPS_t driver_ops = {
    .name = "driver_ops",
    .motorEnable = motorEnable,
    .motorDisable = motorDisable,
    .LedOn = LedOn,
    .LedOff = LedOff,
    .SetPWM = SetPWM,
};

int driver_register(DRIVER_t *p)
{
    if (p == NULL)
        return REGISTER_ERROR;

    p->ops = &driver_ops;

    return REGISTER_SUCCESS;
}