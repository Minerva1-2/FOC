/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-24 21:15:59
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-09-24 22:03:48
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC\motorControl\algorithm\foc.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <string.h>
#include "foc.h"

static void Clarke(FOC_t *p)
{
    p->I_alpha = p->Ia;
    p->I_beta = (p->Ia + 2.0f * p->Ib) * ONE_BY_SQRT3;
}

static void Park(FOC_t *p)
{
    float sin_theta = sinf(p->theta);
    float cos_theta = cosf(p->theta);

    p->Id = p->I_alpha * cos_theta + p->I_beta * sin_theta;
    p->Iq = -p->I_alpha * sin_theta + p->I_beta * cos_theta;
}

static void Pid(FOC_t *p)
{

}
static void antiPark(FOC_t *p)
{

}
static void SVPWM(FOC_t *p)
{

}
const FOC_OPS_t foc_ops = {
    .name = "foc",
    .clarke = Clarke,
    .park = Park,
    .antiPark = antiPark,
    .svpwm = SVPWM,
};

const FOC_t foc = {
    .foc_ops = &foc_ops,
};

int foc_register(FOC_t *p)
{
    if ((p->foc_ops == NULL) || (p == NULL))
        return -1;

    return 0;
}