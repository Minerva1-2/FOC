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

const DRIVER_OPS_t driver_ops = {
    .name = "driver_ops",
    .motorEnable = motorEnable,
    .motorDisable = motorDisable,
    .LedOn = LedOn,
    .LedOff = LedOff,
};

int driver_register(DRIVER_t *p)
{
    if (p == NULL)
        return REGISTER_ERROR;

    p->driver_ops = &driver_ops;

    return REGISTER_SUCCESS;
}