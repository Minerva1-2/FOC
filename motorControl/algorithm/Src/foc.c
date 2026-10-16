/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-24 21:15:59
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-10-12 22:05:52
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

static float Pid_calc(PID_t *p, float target, float feedback, float Ts)
{
    float error = target - feedback;
    // 比例项
    float p_term = p->Kp * error;
    // 积分项
    p->integral += p->Ki * error * Ts;
    // 输出限幅
    if (p->integral > p->output_limit)
        p->integral = p->output_limit;
    if (p->integral < -p->output_limit)
        p->integral = -p->output_limit;

    float output = p_term + p->integral;
    if (output > p->output_limit)
        output = p->output_limit;
    if (output < -p->output_limit)
        output = -p->output_limit;

    p->prev_error = error;
    p->output = output;

    return output;
}

static void Pid(FOC_t *p)
{
    // 电流环, 20KHz
    p->Vd = Pid_calc(&p->pid_id, p->Id_ref, p->Id, p->Ts);
    p->Vq = Pid_calc(&p->pid_iq, p->Iq_ref, p->Iq, p->Ts);
    // 速度环, 电流环的speed_div时间执行一次
    if (p->speed_div > 0)
    {
        if (++p->speed_div_cnt >= p->speed_div)
        {
            p->speed_div_cnt = 0;
            float speed_Ts = (float)p->speed_div * p->Ts;
            p->Iq_ref = Pid_calc(&p->pid_speed, p->speed_ref, p->speed_fb, speed_Ts);
        }
    }
}

static void antiPark(FOC_t *p)
{
    float sin_theta = sinf(p->theta);
    float cos_theta = cosf(p->theta);

    p->V_alpha = p->Vd * cos_theta - p->Vq * sin_theta;
    p->V_beta = p->Vd * sin_theta + p->Vq * cos_theta;
}
/* 归一化占空比(0~1) -> TIMx CCR 计数值（可直接写入比较寄存器）
 *   CCR = ccr_min + duty × (ARR - 2×ccr_min)  [+0.5 四舍五入]
 * 1. !(d > 0.0f) 而非 (d < 0.0f)：NaN 与任何值比较都为假，此写法能兜住 NaN，
 *    避免把 NaN / 越界浮点强转成整型（C 未定义行为）
 * 2. 两端各让出 ccr_min：下端给自举电容充电和死区留时间，上端保持对称，
 *    使 duty=0.5 精确对应 ARR/2（零电压不引入共模偏置）
 * 3. +0.5f 是四舍五入，消除截断导致的 -0.5LSB 系统性偏置
 * 4. PWM 模式2 占空比为 (ARR-CCR)/ARR，取补后与实际导通率一致 */
static inline u16 Duty_To_Ccr(const FOC_t *p, float duty)
{
    float arr  = (float)p->pwm_cycle;
    float span = arr - 2.0f * (float)p->ccr_min;

    if (!(duty > 0.0f))
        duty = 0.0f;
    else if (duty > 1.0f)
        duty = 1.0f;

    u16 ccr = (u16)((float)p->ccr_min + duty * span + 0.5f);

    if (p->pwm_inverted != 0u)
        ccr = (u16)(p->pwm_cycle - ccr);

    return ccr;
}

static void SVPWM(FOC_t *p)
{
    // clarke逆变换
    float Va = p->V_alpha;
    float Vb = -0.5f * p->V_alpha + SQRT3_DIV2 * p->V_beta;
    float Vc = -0.5f * p->V_alpha - SQRT3_DIV2 * p->V_beta;
    // 求最大值与最小值
    float Vmax = Va;
    float Vmin = Va;
    if (Vb > Vmax)  Vmax = Vb;
    if (Vc > Vmax)  Vmax = Vc;
    if (Vb < Vmin)  Vmin = Vb;
    if (Vc < Vmin)  Vmin = Vc;
    // 零序分量注入
    float V_offset = -0.5f * (Vmax + Vmin);
    Va += V_offset;
    Vb += V_offset;
    Vc += V_offset;
    // 设置PWM占空比
    float inv_adc = 1.0f / p->V_bus;
    p->duty_a = 0.5f + Va * inv_adc;
    p->duty_b = 0.5f + Vb * inv_adc;
    p->duty_c = 0.5f + Vc * inv_adc;
    // 占空比限制
    p->duty_a = __constrain__(p->duty_a, 0.0f, 1.0f);
    p->duty_b = __constrain__(p->duty_b, 0.0f, 1.0f);
    p->duty_c = __constrain__(p->duty_c, 0.0f, 1.0f);
    // 输出到ccr中的数值
    p->ccr_a = Duty_To_Ccr(p, p->duty_a);
    p->ccr_b = Duty_To_Ccr(p, p->duty_b);
    p->ccr_c = Duty_To_Ccr(p, p->duty_c);
}
/*******************************************************************************************************************************/
/**********************************************************函数注册接口**********************************************************/
/*******************************************************************************************************************************/
const FOC_OPS_t foc_ops = {
    .name = "foc_ops",
    .clarke = Clarke,
    .park = Park,
    .pid = Pid,
    .antiPark = antiPark,
    .svpwm = SVPWM,
};

int foc_register(FOC_t *p)
{
    if (p == NULL)
        return REGISTER_ERROR;

    p->ops = &foc_ops;

    return REGISTER_SUCCESS;
}