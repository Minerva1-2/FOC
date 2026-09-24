#ifndef __FOC_H
#define __FOC_H

#include "math.h"
#include "userDef.h"

#define ONE_BY_SQRT3    0.577350269

typedef struct FOC FOC_t;

typedef struct
{
    const char *name;
    void (*clarke)(FOC_t *p);
    void (*park)(FOC_t *p);
    void (*antiPark)(FOC_t *p);
    void (*svpwm)(FOC_t *p);
}FOC_OPS_t;

struct FOC
{
    const FOC_OPS_t *foc_ops;           // 挂上的算法表

    float Ts;
    float Ia, Ib, Ic;
    float I_alpha, I_beta;
    float V_alpha, V_beta;
    float Id, Iq;
    float duty_a, duty_b, duty_c;
    float theta;
};

int foc_register(FOC_t *p);

#endif // #ifndef __FOC_H
