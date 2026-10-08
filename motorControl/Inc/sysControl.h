#ifndef __SYSCONTROL_H
#define __SYSCONTROL_H

#include "userDef.h"
#include "foc.h"
#include "observer.h"
#include "driver.h"
#include "sample.h"

typedef struct
{
    FOC_t foc;
    SMO_t smo;
    PLL_t pll;
    DRIVER_t driver;
    SAMPLE_t sample;
}Motor_t;


#endif // !__SYSCONTROL_H