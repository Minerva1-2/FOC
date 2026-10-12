/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-10 17:18:17
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-12 22:11:19
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\algorithm\Src\observer.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "observer.h"

void SMO(SMO_t *p)
{
    // 1.电流预测(基于电机模型的前向欧拉离散化)
    // 公式：I_hat(k+1) = I_hat(k) + Ts * [-Rs / Ld * I_hat + (U - e_hatPLF) / Ld]
    p->i_alpha_hat += p->Ts * (-p->Rs / p->Ld * p->i_alpha_hat + (p->U_alpha - p->e_alpha_hat_plf) / p->Ld);
    p->i_beta_hat += p->Ts * (-p->Rs / p->Ld * p->i_beta_hat + (p->U_beta - p->e_beta_hat_plf) / p->Ld);
    // 2.滑膜切换函数(使用饱和函数，减小抖振)
    // 边界层设定为1.0f，可以根据电流范围误差调整
    if ((p->i_alpha_hat - p->I_alpha) > p->phi)
        p->e_alpha_hat = p->gain;
    else if ((p->i_alpha_hat - p->I_alpha) < -p->phi)
        p->e_alpha_hat = -p->gain;
    else
        p->e_alpha_hat = (p->i_alpha_hat - p->I_alpha) * p->gain;

    if ((p->i_beta_hat - p->I_beta) > p->phi)
        p->e_beta_hat = p->gain;
    else if ((p->i_beta_hat - p->I_beta) < -p->phi)
        p->e_beta_hat = -p->gain;
    else
        p->e_beta_hat = (p->i_beta_hat - p->I_beta) * p->gain;
    // 3.反电动势低通滤波(去除高频噪声)，e_hat_PLF_factor为滤波系数
    p->e_alpha_hat_plf = p->e_hat_PLF_factor * p->e_alpha_hat + p->e_alpha_hat_plf * (1.0f - p->e_hat_PLF_factor);
    p->e_beta_hat_plf = p->e_hat_PLF_factor * p->e_beta_hat + p->e_beta_hat_plf * (1.0f - p->e_hat_PLF_factor);
}
void PLL(PLL_t *p)
{
    // 计算角度误差(锁相环鉴相器)
    p->theta_err = p->Dir * (cosf(p->theta_hat) * p->A_in + sinf(p->theta_hat) * p->B_in);
    // PI控制器(环路滤波器)
    p->integral += p->Ki * p->theta_err;                // 积分项累加
    p->omega_hat = p->Kp * p->theta_err + p->integral;  // 观测角速度
    // 一阶低通滤波
    p->omega_hat_PLF = p->omega_hat * p->omega_hat_PLF_factor + (1.0f - p->omega_hat_PLF_factor) * p->omega_hat_PLF;
    // 计算观测角度(压控振荡器)
    p->theta_hat = p->omega_hat_PLF * p->Ts;
    if (p->theta_hat > TWO_PI)
        p->theta_hat -= TWO_PI;
    else if (p->theta_hat < 0)
        p->theta_hat += TWO_PI;
}

const SMO_OPS_t smo_ops = {
    .name = "smo_ops",
    .SMO = SMO,
};
const PLL_OPS_t pll_ops = {
    .name = "pll_ops",
    .PLL = PLL,
};

int smo_register(SMO_t *p, PLL_t *q)
{
    if (p == NULL || q == NULL)
        return REGISTER_ERROR;

    p->ops = &smo_ops;
    q->ops = &pll_ops;

    return REGISTER_SUCCESS;
}