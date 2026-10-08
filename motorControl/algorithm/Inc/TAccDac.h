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
    void (*AngleGenerator)(Angle_t *p);
}TAccDac_OPS_t;

struct TAccDac
{
    const TAccDac_OPS_t *TAccDac_ops;
};

struct Angle
{

};

#endif
