/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "gpio.h"
#include "l298n.h"
#include "timers.h"
#include "interrupts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//GPIOs
#define IR_GPIO GPIOC
#define BUZZER_GPIO GPIOB
#define DRIVER_GPIO GPIOC
#define POT_GPIO GPIOA

// Timers
#define BUZZER_TIMER TIM4
#define DRIVER_TIMER TIM3
#define TOC_TIMER TIM2
#define NVIC_TOC_TIMER NVIC_TIM2

// Ports
#define BUZZER 8
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define IR_RIGHT 1
#define IR_LEFT 2
#define POT_PORT 5
#define USER_BUTTON 0

// Timer channels
#define BUZZER_CHANNEL 3
#define IN1_CHANNEL 1
#define IN2_CHANNEL 2
#define IN3_CHANNEL 3
#define IN4_CHANNEL 4

// ADC channels
#define POT_CHANNEL 5

#define BOTH_READS ((GPIOC->IDR >> IR_RIGHT) & (GPIOC->IDR >> IR_LEFT) & 1)
#define LEFT_READ ((GPIOC->IDR >> IR_LEFT) & 1)
#define RIGHT_READ ((GPIOC->IDR >> IR_RIGHT) & 1)

#define OFF (0)
#define ON (~0)
#define BLINK (1 << 9)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
static uint8_t continue_from_pulse = 0;
static uint8_t disable_buzzer = 0;
static uint8_t end_of_line = 0;

static Motor motor1 = {
    .in1 = &DRIVER_TIMER->CCR1, .in1_offset = 0,
    .in2 = &DRIVER_TIMER->CCR2, .in2_offset = 0,
};

static Motor motor2 = {
    .in1 = &DRIVER_TIMER->CCR3, .in1_offset = 0,
    .in2 = &DRIVER_TIMER->CCR4, .in2_offset = 0,
};

static Motor_State off = {
    .value_1 = 0, .value_2 = 0,
};

static Motor_State motor_pwm = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EXTI0_IRQHandler(void)
{
   __disable_irq();
    continue_from_pulse = 1;
   __enable_irq();
    EXTI->PR |= 1;
    NVIC->ICER[0] |= 1; // Desactivar esta interrupcion una vez ocurrida.
}

/* void EXTI1_IRQHandler(void) */
/* { */
/*     uint8_t value = (GPIOC->IDR >> IR_RIGHT) & 1; */
/*     __disable_irq(); */
/*     if (BOTH_READS) */
/*     { */
/*         timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, ON); */
/*         driver_set_motor(&motor1, &off); */
/*         driver_set_motor(&motor2, &off); */
/*         end_of_line = 1; */
/*     } */
/*     else */
/*     { */
/*         timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, (~value & 1) << 9); */
/*         driver_set_motor(&motor1, &motor_pwm); */
/*         driver_set_motor(&motor2, &off); */
/*     } */
/*     __enable_irq(); */
/*     EXTI->PR |= 2; */
/* } */

/* void EXTI2_IRQHandler(void) */
/* { */
/*     uint8_t value = (GPIOC->IDR >> IR_RIGHT) & 1; */
/*     __disable_irq(); */
/*     if (BOTH_READS) */
/*     { */
/*         timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, ON); */
/*         driver_set_motor(&motor1, &off); */
/*         driver_set_motor(&motor2, &off); */
/*         end_of_line = 1; */
/*     } */
/*     else */
/*     { */
/*         timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, (~value & 1) << 9); */
/*         driver_set_motor(&motor1, &off); */
/*         driver_set_motor(&motor2, &motor_pwm); */
/*     } */
/*     __enable_irq(); */
/*     EXTI->PR |= 4; */
/* } */

void TIM2_IRQHandler(void)
{
    if (!(TIM2->SR & (1 << CC1IF))) return;
    __disable_irq();
    continue_from_pulse = 1;
    TIM2->SR &= 0;
    if (disable_buzzer) timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, OFF);
    __enable_irq();
}
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
  MX_ADC_Init();
  MX_LCD_Init();
  MX_TS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  gpio_set_pin_mode(POT_GPIO, POT_PORT, ANALOG);
  gpio_set_pin_mode(GPIOA, USER_BUTTON, INPUT);
  gpio_set_pin_mode(IR_GPIO, IR_RIGHT, INPUT);
  gpio_set_pin_mode(IR_GPIO, IR_LEFT, INPUT);
  gpio_set_pin_mode(DRIVER_GPIO, IN1, AF);
  gpio_set_pin_mode(DRIVER_GPIO, IN2, AF);
  gpio_set_pin_mode(DRIVER_GPIO, IN3, AF);
  gpio_set_pin_mode(DRIVER_GPIO, IN4, AF);
  gpio_set_pin_mode(BUZZER_GPIO, BUZZER, AF);
  gpio_set_pin_alternate_function(DRIVER_GPIO, IN1, AF2);
  gpio_set_pin_alternate_function(DRIVER_GPIO, IN2, AF2);
  gpio_set_pin_alternate_function(DRIVER_GPIO, IN3, AF2);
  gpio_set_pin_alternate_function(DRIVER_GPIO, IN4, AF2);
  gpio_set_pin_alternate_function(GPIOB, BUZZER, AF2);


  timer_configure_clock_signal(BUZZER_TIMER, 16e3, 1e3);    // T = 0.5 s
  timer_configure_clock_signal(DRIVER_TIMER, 128, 1e3);   // f = 250 Hz
  timer_configure_clock_signal(TOC_TIMER, 2e3, 32000);        // T = 2 s

  configure_interrupt(EXTI0, FALLING_EDGE, A);

  timer_set_pwm(BUZZER_TIMER, BUZZER_CHANNEL, NORMAL, OFF);
  timer_set_pwm(DRIVER_TIMER, IN1_CHANNEL, NORMAL, OFF);
  timer_set_pwm(DRIVER_TIMER, IN2_CHANNEL, NORMAL, OFF);
  timer_set_pwm(DRIVER_TIMER, IN3_CHANNEL, NORMAL, OFF);
  timer_set_pwm(DRIVER_TIMER, IN4_CHANNEL, NORMAL, OFF);
  timer_set_toc(TOC_TIMER, CH1, 31999, WITH_IRQ);

  adc_init(SCAN_OFF, BIT_12, POT_CHANNEL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (!continue_from_pulse)
      ;
  continue_from_pulse = 0;
  TOC_TIMER->SR &= 0;
  TOC_TIMER->CR1 |= 1;
  unmask_interrupt(NVIC_TOC_TIMER);
  while (!continue_from_pulse)
      ;
  TOC_TIMER->CR1 &= ~1;
  mask_interrupt(NVIC_TOC_TIMER);
  
  TIM4->SR &= 0;
  TIM4->CR1 |= 1;
  ADC_START;
  ADC_WAIT_UNTIL_READY;
  while (1)
  {
      if (!end_of_line)
      {

          if (BOTH_READS)
          {
              timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, ON);
              driver_set_motor(&motor1, &off);
              driver_set_motor(&motor2, &off);
              end_of_line = 1;
          }
          else
          {
              ADC1->CR2 |= (1 << SWSTART);
              uint16_t value = ADC1->DR;
              /*
               * value = 0 -> CCRy = 50%
               * value = max -> CCRy = 100%
               */
#define U32_HALF ((1U << 16) - 1)
#define U12_MAX ((1U << 12) - 1)
              uint32_t result = (uint32_t) (value * (U32_HALF / U12_MAX) + U32_HALF);
              motor_pwm = (Motor_State) { .value_1 = result, .value_2 = 0 };
              if (!LEFT_READ) driver_set_motor(&motor1, &motor_pwm);
              else driver_set_motor(&motor1, &off);
              
              if (!RIGHT_READ) driver_set_motor(&motor2, &motor_pwm);
              else driver_set_motor(&motor2, &off);
              
              if (!BOTH_READS) timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, BLINK);
              else timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, OFF);
          }
          /* USER CODE END WHILE */
          /* USER CODE BEGIN 3 */
      }
      else
      {
          // Reactivar TOC
          disable_buzzer = 1;
          TOC_TIMER->CR1 |= 1;
          // Pitido continuo durante 2s
          timer_set_pwm_dc(BUZZER_TIMER, BUZZER_CHANNEL, ON);
          continue_from_pulse = 0;
          while (!continue_from_pulse)
              ;
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LCD;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_4;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
