/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-26 16:25:11
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-09-27 10:49:52
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC\motorControl\hardware\Inc\sample.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SAMPLE_H
#define __SAMPLE_H

#include <stdbool.h>
#include "userDef.h"
#include "adc.h"

#define SHUNT_RESISTOR_OHM          0.005f                                          // 下桥采样电阻5mΩ
#define ADC_VREF                    3.3f                                            // 参考电压
#define ADC_FULL_SCALE              4096.0f                                         // adc满量程
#define I_ZERO_VOLT                 1.65f                                           // 基准电压
#define VBUS_DIV_RATIO              0.0449f                                         // 分压系数
#define K_TEMPERATURE               273.15f                                         // 开式温度
#define AMP_GAIN                    10u                                             // 运放增益
#define SAMPLE_CALI_TIMES           256u                                            // 零点校准次数
#define TEMP_SAMPLE_NUM             1000U                                           // 温度计算节流：约每 1000 次ADC完成回调算一次
#define I_ZERO_RAW                  (I_ZERO_VOLT / ADC_VREF * ADC_FULL_SCALE)       // 零位ADC数值
#define SAMPLE_I_PER_LSB            ((ADC_VREF / ADC_FULL_SCALE) / (SHUNT_RESISTOR_OHM * AMP_GAIN))     // 转换为电流时的系数

#define ADC_BUF_LEN                 2

typedef struct SAMPLE SAMPLE_t;

typedef struct
{
    void (*Current_Calibrate)(SAMPLE_t *p);
}SAMPLE_OPS_t;

struct SAMPLE
{
    /* 电机相电流采样 */
    const SAMPLE_OPS_t *ops;

    volatile u32 current_adc_a;     // a相adc原始值
    volatile u32 current_adc_c;     // c相adc原始值
    u16 offset_a, offset_c;         // a、c相adc零位值
    float Ia, Ib, Ic;               // 三相电流值
    u16 clai_time_cnt;              // 采样周期计数
    bool cali_flag;                 // 校准标志位
    /* 电机母线电压采样 */
    volatile u16 Vbus_adc;     // 母线adc原始值
    float Vbus;
    /* 电机温度采样 */
};

#endif // #ifndef __SAMPLE_H