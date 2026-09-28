/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-26 16:25:00
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-09-28 22:42:16
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC\motorControl\hardware\Src\adc.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "sample.h"
#include <math.h>

static void MotorCurrent(SAMPLE_t *p, ADC_HandleTypeDef *hadc)
{
    u16 raw_a, raw_c;
     // 获取注入通道adc原始数值
    raw_a = p->g_adc_buf[0];
    raw_c = p->g_adc_buf[1];
    // 启动前校准
    if (!p->cali_flag)
    {
        p->current_adc_a += raw_a;  
        p->current_adc_c += raw_c;
        // 采样周期计算
        if (++p->clai_time_cnt > SAMPLE_CALI_TIMES)
        {
            // 电流零位值
            p->offset_a = (u16)(p->current_adc_a / SAMPLE_CALI_TIMES);
            p->offset_c = (u16)(p->current_adc_c / SAMPLE_CALI_TIMES);
            // 标志位等清除
            p->current_adc_a = 0;
            p->current_adc_c = 0;
            p->clai_time_cnt = 0;
            p->cali_flag = true;
        }
    }
    // 校准结束，计算运行时三相电流值
    p->Ia = (float)(raw_a - p->offset_a) / SAMPLE_I_PER_LSB;
    p->Ic = (float)(raw_c - p->offset_c) / SAMPLE_I_PER_LSB;
    p->Ib = -(p->Ia + p->Ic);
}
static void MotorVbus(SAMPLE_t *p, ADC_HandleTypeDef *hadc)
{
    p->Vbus_adc = p->g_adc_buf[2];
    p->Vbus = (float)p->Vbus_adc / ADC_FULL_SCALE * ADC_VREF / VBUS_DIV_RATIO;
}
static void Washer(SAMPLE_t *p)
{
    p->washer_adc = p->g_adc_buf[3];
    p->washer_vlaue = p->washer_adc / ADC_FULL_SCALE * ADC_VREF;
}
static void MotorTemp(SAMPLE_t *p)
{
    float v, r_ntc;

    if (++p->temp_cnt < TEMP_SAMPLE_NUM)    return;
    
    p->temp_cnt = 0;
    p->temp_adc = p->g_adc_buf[4];
    v = (float)p->temp_adc / ADC_FULL_SCALE * ADC_VREF;

    r_ntc = VOLTAGE_DIVID_RESISTOR * v / (ADC_VREF - v);
    p->motor_temp = 1.0f / (1.0f / NTC_T25_K + logf(r_ntc / VOLTAGE_DIVID_RESISTOR) / NTC_B_VALUE) - K_TEMPERATURE;
}
const SAMPLE_OPS_t sample_ops = {
    .MotorCurrent = MotorCurrent,
    .MotorVbus = MotorVbus,
    .Washer = Washer,
    .MotorTemp = MotorTemp,
};

int sample_register(SAMPLE_t *p)
{
    if (p == NULL)
        return -1;

    p->ops = &sample_ops;

    return 0;
}