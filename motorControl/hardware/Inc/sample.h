/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-09-26 16:25:11
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-09-28 22:27:26
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
#define VOLTAGE_DIVID_RESISTOR      10000.0f                                        // 分压电阻阻值
#define NTC_B_VALUE                 3434.0f                                         // NTC B 值(25/85)
#define NTC_T25_K                   (25.0f + K_TEMPERATURE)                         // 25℃ 对应的开氏温度
#define AMP_GAIN                    10u                                             // 运放增益
#define SAMPLE_CALI_TIMES           256u                                            // 零点校准次数
#define TEMP_SAMPLE_NUM             1000u                                           // 温度计算节流：约每 1000 次ADC完成回调算一次
#define I_ZERO_RAW                  (I_ZERO_VOLT / ADC_VREF * ADC_FULL_SCALE)       // 零位ADC数值
#define SAMPLE_I_PER_LSB            ((ADC_VREF / ADC_FULL_SCALE) / (SHUNT_RESISTOR_OHM * AMP_GAIN))     // 转换为电流时的系数

#define ADC_BUF_LEN                 5

typedef struct SAMPLE SAMPLE_t;

typedef struct
{
    void (*MotorCurrent)(SAMPLE_t *p, ADC_HandleTypeDef *hadc);
    void (*MotorVbus)(SAMPLE_t *p, ADC_HandleTypeDef *hadc);
    void (*Washer)(SAMPLE_t *p);
    void (*MotorTemp)(SAMPLE_t *p);
}SAMPLE_OPS_t;

struct SAMPLE
{
    /* 电机相电流采样 */
    const SAMPLE_OPS_t *ops;

    u16 g_adc_buf[ADC_BUF_LEN];                     // adc采样数组：0:1为相电流；2:4为DMA采样(2:母线电压;3:波轮电位器;4:温度)
    u32 current_adc_a;                              // a相adc原始值
    u32 current_adc_c;                              // c相adc原始值
    float Ia, Ib, Ic;                               // 三相电流值
    u16 offset_a, offset_c;                         // a、c相adc零位值
    u16 clai_time_cnt;                              // 采样周期计数
    bool cali_flag;                                 // 校准标志位
    /* 电机母线电压采样 */
    u16 Vbus_adc;                                   // 母线adc原始值
    float Vbus;
    /* 波轮电位器采样 */
    u16 washer_adc;                                 // 电位器adc原始值
    u16 washer_vlaue;                               // 计算得到的数值
    /* 电机温度采样 */
    u16 temp_adc;                                   // adc原始值
    float current_resistance;                       // 当前阻值
    float motor_temp;                               // 电机温度
    u16 temp_cnt;
};

int sample_register(SAMPLE_t *p);

#endif // #ifndef __SAMPLE_H