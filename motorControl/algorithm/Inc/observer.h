#ifndef __OBSERVER_H
#define __OBSERVER_H

#include <string.h>
#include <math.h>
#include "userDef.h"

typedef struct SMO SMO_t;
typedef struct PLL PLL_t;

typedef struct
{
    const char *name;
    void (*SMO)(SMO_t *P);
}SMO_OPS_t;
typedef struct
{
    const char *name;
    void (*PLL)(PLL_t *P);
}PLL_OPS_t;

struct SMO
{
    const SMO_OPS_t *ops;
    float Ts;

    float Rs;               // 定子电阻
    float Ld;               // 定子电感
    float gain;             // 滑膜增益
    float phi;              // 边界层数

    float fc;               // 截止频率
    float wc;               // 截止角频率

    float i_alpha_hat;      // 电流预估值
    float i_beta_hat;

    float I_alpha;          // 实际电流值
    float I_beta;
    float U_alpha;          // 实际电压值
    float U_beta;

    float e_alpha_hat;      // 反电动势预估值
    float e_beta_hat;
    float e_alpha_hat_plf;  // 反电势预估滤波值
    float e_beta_hat_plf;

    float e_hat_PLF_factor;
};
struct PLL
{
    const PLL_OPS_t *ops;
    s8    Dir;              // 方向
    float Ts;
    float Kp;
    float Ki;
    float integral;         // 积分项

    float A_in, B_in;       // 输入

    float theta_err;
    float theta_hat;        // 预估电角度
    float omega_hat;        // 预估电角速度
    float omega_hat_PLF;    // 预估电角速度滤波值

    float omega_hat_PLF_factor;
};

int smo_register(SMO_t *p, PLL_t *q);

#endif
