/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
/* printf 重定向需要 stdio.h 中的 FILE 类型与 fputc 原型 */
#include <stdio.h>

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*************************************************************************************************************************/
/**************************************************printf 重定向到 USART1*************************************************/
/*************************************************************************************************************************/
/* 说明：本工程未勾选 Keil 的 Use MicroLIB（MDK-ARM/FOC.uvprojx 中 useUlib=0），此时直接调用 printf，
 *       ARM C 库会走半主机(semihosting)，脱机运行时停在 BKPT 上。这里用 __use_no_semihosting 关闭半主机，
 *       并提供库要求的桩函数（_sys_exit / _ttywrch / _sys_open），再通过 fputc 把字符写进 USART1 数据寄存器。
 * 注意：fputc 是阻塞轮询（115200 下一个字符约 87us），不要在 FOC 电流环/ADC 注入中断里调用 printf。 */
#if defined(__MICROLIB)
/* MicroLIB 有自己精简的 retarget 路径，不需要半主机桩函数 */
#else
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting\n\t");
#else
#pragma import(__use_no_semihosting)
#endif

/* 半主机桩函数：供链接器解析，使 arm C 库不引入 semihosting */
void _sys_exit(int x)
{
    (void)x;
    for (;;)
    {
    }
}

void _ttywrch(int ch)
{
    (void)ch;
}

/* arm C 库在链接 printf 时会引用 _sys_open（半主机文件打开接口），必须给它一个定义，
 * 否则链接报 L6915E: __use_no_semihosting was requested, but _sys_open was referenced。
 * 这里返回 -1（无文件系统）即可；链接时该段因未被实际调用会被 --remove 掉，不占 Flash */
int _sys_open(const char *name, int openmode)
{
    (void)name;
    (void)openmode;
    return -1;
}

FILE __stdout;
#endif /* __MICROLIB */

/* printf -> fputc -> USART1->TDR
 * 1. 用寄存器轮询而非 HAL_UART_Transmit：后者依赖 HAL tick 和超时机制，在中断上下文里不安全
 * 2. guard 是保险：USART1 未使能（TXE 意外不置位）时不会在 fputc 里无限等待
 * 3. 只等 TXE 不等 TC：末尾字符仍在移位寄存器中发送，复位前要保证发完可再等一次 TC
 * 4. printf 不可重入：中断与主循环同时打印会让字符交错，调试输出请集中在一个上下文里 */
int fputc(int ch, FILE *f)
{
    uint32_t guard = 100000u;

    (void)f;
    while (((USART1->ISR & USART_ISR_TXE_TXFNF) == 0u) && (--guard != 0u))
    {
    }
    if (guard != 0u)
        USART1->TDR = (uint8_t)ch;

    return ch;
}

/* USER CODE END 1 */

