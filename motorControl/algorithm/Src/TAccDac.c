#include "TAccDac.h"

void TAccDac(TAccDac_t *p)
{

}
void AngleGenerator(Angle_t *p)
{

}

const TAccDac_OPS_t TAccDac_ops = {
    .name = "TAccDac_ops",
    .TAccDac = TAccDac,
    .AngleGenerator = AngleGenerator,
};

int TAccDac_register(TAccDac_t *p)
{
    if (p == NULL)
        return REGISTER_ERROR;

    p->TAccDac_ops = &TAccDac_ops;

    return REGISTER_SUCCESS;
}