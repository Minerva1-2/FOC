/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-10-10 17:18:17
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-10 18:45:07
 * @FilePath: \MDK-ARMd:\keil\project\FOC\motorControl\algorithm\Src\TAccDac.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "TAccDac.h"

void TAccDac(TAccDac_t *p)
{
    // 计算目标位置增量：目标速度 / 60.0f * Ts,将其转换为转/秒后乘以运行周期转换为每周期应该移动的步长
    p->speedTargetIncrement = p->TargetSpeed / 60.0f * p->Ts;
    // 计算速度增量：加速度 / 60.0f * Ts^2,将加速度 / 60.0f *Ts转换为转/秒，再*Ts得到每周期额外的增量
    p->speedIncrement = p->accSpeed / 60.0f * p->Ts * p->Ts;

    if (p->speedChangeIncrement > p->speedTargetIncrement)
    {
        // 当前步长大于目标步长，需要减小当前步长到目标步长，因此为减速过程
        p->speedChangeIncrement -= p->speedIncrement;
        if (p->speedChangeIncrement <= p->speedTargetIncrement)
            p->speedChangeIncrement = p->speedTargetIncrement;
    }
    else if (p->speedChangeIncrement < p->speedTargetIncrement)
    {
        // 当前步长小于目标步长，需要增加当前步长到目标步长，因此为加速过程
        p->speedChangeIncrement += p->speedIncrement;
        if (p->speedChangeIncrement >= p->speedTargetIncrement)
            p->speedChangeIncrement = p->speedTargetIncrement;
    }
    // 计算当前的位置变量并进行加减速判断
    p->speedDeltaIncrement = fabsf(p->speedChangeIncrement) - p->speedLastIncrement;
    p->speedLastIncrement = fabsf(p->speedChangeIncrement);
    // 状态判断
    p->motor_state = p->speedDeltaIncrement > 0.0f ? MOTOR_ACC :
                     (p->speedDeltaIncrement < 0.0f ? MOTOR_DAC : MOTOR_UNIFORM);
    // 锁相环方向计算
    p->speedOutDir = p->speedChangeIncrement > 0.0f ? -1 : 1;
    // 输出目标速度，由于输出给速度环使用，因此转换为电角速度，speedChangeIncrement单位是转数
    p->speedOut = p->speedChangeIncrement / p->Ts * 60.0f * MOTOR_PAIRS;
}
void AngleGenerator(Angle_t *p)
{

}

const TAccDac_OPS_t TAccDac_ops = {
    .name = "TAccDac_ops",
    .TAccDac = TAccDac,
    .AngleGenerator = AngleGenerator,
};

int TAccDac_register(TAccDac_t *p, Angle_t *q)
{
    if (p == NULL || q == NULL)
        return REGISTER_ERROR;

    p->TAccDac_ops = &TAccDac_ops;
    p->Ts = MOTOR_CONTROLER;
    q->Ts = MOTOR_CONTROLER;

    return REGISTER_SUCCESS;
}