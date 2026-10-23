/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "userDef.h"
#include "sysControl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- 调试数据流（VOFA+ JustFloat 协议）----
 * 每帧 = DBG_CH_NUM 个 float（小端）+ 4 字节帧尾 00 00 80 7F。
 * 帧长固定，不会像 printf("%.1f") 那样随数值位数变化；发送过程非阻塞 */
#define DBG_CH_NUM      15u
#define DBG_FRAME_LEN   (DBG_CH_NUM * 4u + 4u)

static uint8_t  dbg_frame[DBG_FRAME_LEN];
static uint16_t dbg_idx = 0u;
static uint8_t  dbg_busy = 0u;
static uint32_t dbg_last_tick = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void DebugStream_Poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    motorInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* VOFA+ JustFloat 非阻塞推流：每 10ms 组一帧，主循环每次只推 1 个字节 */
    DebugStream_Poll();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * 组一帧调试数据：DBG_CH_NUM 个 float + JustFloat 帧尾（00 00 80 7F）
 * 通道顺序要与 VOFA+ 里配置的一致，改动这里记得同步改上位机
 */
static void DebugStream_Load(void)
{
    float ch[DBG_CH_NUM];

    /* 三相电流要在同一控制周期内取值，否则中断插在中间会读到不同拍的混合值 */
    float iq = motor.foc.Iq_LPF;
    float spd_out = motor.tra.speedOut;

    /* ---- 通道 0~3：梯形速度曲线专用，统一换算成"机械 RPM"，量级一致可直接对比 ----
     *   I0 = 目标速度（电位器给出的阶跃）
     *   I1 = 曲线输出（梯形斜坡）
     *   I2 = 实际反馈（PLL 观测转速）
     *   I3 = 运动状态 0=加速 1=匀速 2=减速（左轴会被压扁，可单独放大看）
     */
    ch[0] = motor.tra.TargetSpeed;                                    /* 机械 RPM */
    ch[1] = spd_out / MOTOR_PAIRS;                                    /* 电RPM → 机械RPM */
    ch[2] = motor.pll.omega_hat_PLF / TWO_PI * 60.0f / MOTOR_PAIRS;   /* rad/s → 机械RPM */
    ch[3] = (float)motor.tra.motor_state;

    /* ---- 通道 4~14：其他调试量 ---- */
    ch[4]  = motor.sample.Vbus;
    ch[5]  = iq;                                                      /* q 轴电流（A）*/
    ch[6]  = motor.sample.washer_vlaue;                               /* 电位器给定（机械 RPM）*/
    ch[7]  = (float)motor.StrongDragToObs.GeneralMode;                /* 1=开环 2=闭环 */
    ch[8]  = (float)motor.StrongDragToObs.CheckCnt;
    ch[9]  = motor.StrongDragToObs.ThetaErr;
    ch[10] = motor.pll.omega_hat_PLF;                                 /* 电角速度 rad/s */
    ch[11] = motor.sample.motor_temp;
    ch[12] = motor.foc.duty_a;
    ch[13] = motor.tra.speedChangeIncrement * 1.0e6f;                 /* 位置增量×1e6，看斜坡细分用 */
    /* I14：中断里逐拍累加的电角度（rad）。
     * 不能在主循环里用 10ms 的采样点做差分展开：812RPM 时每 10ms 转 5.97 rad > π，
     * 超过奈奎斯特极限，展开会失败（表现为"角度几乎不变"）。
     * 中断里每 50us 只转 0.03 rad，可以正确累加，斜率就是电角速度。 */
    ch[14] = motor.theta_acc;

    memcpy(dbg_frame, ch, sizeof(ch));
    dbg_frame[DBG_CH_NUM * 4u + 0u] = 0x00u;
    dbg_frame[DBG_CH_NUM * 4u + 1u] = 0x00u;
    dbg_frame[DBG_CH_NUM * 4u + 2u] = 0x80u;
    dbg_frame[DBG_CH_NUM * 4u + 3u] = 0x7Fu;

    dbg_idx  = 0u;
    dbg_busy = 1u;
}

/**
 * 非阻塞调试推流
 * 1. 每 10ms 组一帧（固定 44 字节），帧长与数值大小无关，转速再高也不会变长
 * 2. 之后每个主循环只在 TXE 置位时写一个字节，单次开销几十 ns，主循环不再被串口阻塞
 * 3. 44 字节在 115200 下约 3.8ms 发完，留 6ms 余量
 */
static void DebugStream_Poll(void)
{
    if (dbg_busy == 0u)
    {
        if ((HAL_GetTick() - dbg_last_tick) >= 10u)
        {
            dbg_last_tick = HAL_GetTick();
            DebugStream_Load();
        }
        return;
    }
    if ((USART1->ISR & USART_ISR_TXE_TXFNF) != 0u)
    {
        USART1->TDR = dbg_frame[dbg_idx++];
        if (dbg_idx >= DBG_FRAME_LEN)
            dbg_busy = 0u;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
