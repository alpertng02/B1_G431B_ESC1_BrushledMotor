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
#include "comp.h"
#include "dac.h"
#include "fdcan.h"
#include "gpio.h"
#include "opamp.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_uart.h"
#include "system_stm32g4xx.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSCLK_FREQ (170e6)
#define PWM_INPUT_MAX_PERIOD_MS (5)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile static int32_t current_raw = 0;
volatile static float current_mA = 0;
volatile static float target_speed = 0; // Changes from -100.0 to +100.0

volatile static uint32_t pwm_input_frequency = 0;
volatile static uint32_t pwm_input_cycletime = 0;
volatile static float pwm_input_duty = 0.0;

volatile static bool pot_mode_enabled = false;
static GPIO_PinState last_devboard_button_state = GPIO_PIN_SET;

volatile static uint64_t runtime_ms = 0;
volatile static uint64_t last_pwm_input_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**

* @brief Drives the brushed DC motor on OUT1 and OUT2

* @param speed_percent: -100 to 100 (Negative for reverse, Positive for forward,
0 to coast)

*/

void set_motor_speed(int8_t speed_percent) {

  // Clamp the input to a safe max of 95% for the bootstrap capacitors

  if (speed_percent > 95)
    speed_percent = 95;

  if (speed_percent < -95)
    speed_percent = -95;

  // Get the current Auto-Reload Register (ARR) value
  const uint32_t arr_val = __HAL_TIM_GET_AUTORELOAD(&htim1);
  const uint32_t ccr_val = (abs(speed_percent) * arr_val) / 100;

  if (speed_percent > 0) {

    // Forward: Drive OUT1, Coast OUT2
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_val);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

  }

  else if (speed_percent < 0) {
    // Reverse: Coast OUT1, Drive OUT2
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_val);
  }

  else {
    // Coast: Both low
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  }
}

void ADC_Select_Channel(ADC_HandleTypeDef *hadc, uint32_t channel) {
  // 1. Force the ADC to stop any ongoing background processes
  HAL_ADC_Stop(hadc);

  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_DAC3_Init();
  MX_FDCAN1_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // 1. Power up the Operational Amplifiers
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);

  // 2. Calibrate the ADCs (CRITICAL for STM32G4!)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // 3. Start Phase U (OUT1) PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  // 4. Start Phase V (OUT2) PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  // Start PWM Input
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // Signal Input Channel (Main)
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);    // Secondary Channel

  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    GPIO_PinState devboard_button_pressed =
        HAL_GPIO_ReadPin(DEVBOARD_BUTTON_GPIO_Port, DEVBOARD_BUTTON_Pin);
    if (devboard_button_pressed == GPIO_PIN_RESET &&
        last_devboard_button_state != GPIO_PIN_RESET) {
      pot_mode_enabled = !pot_mode_enabled;
    }
    last_devboard_button_state = devboard_button_pressed;
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    uint32_t cl = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t ch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (cl > 0) {
      pwm_input_frequency = (float)SYSCLK_FREQ / (cl + 1);
      pwm_input_duty = 100.0f * (float)ch / cl;

      last_pwm_input_ms = runtime_ms;
    }
  }
}
/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == TIM6) {
    runtime_ms++;

    if (pot_mode_enabled) {
      // ---------------------------------------------------------
      // 1. READ POTENTIOMETER & SET SPEED
      // ---------------------------------------------------------
      // Explicitly select Channel 11 (Potentiometer) on ADC1
      ADC_Select_Channel(&hadc1, ADC_CHANNEL_11);
      HAL_ADC_Start(&hadc1);

      if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);

        target_speed = ((adc_val * 200) / 4095) - 100;
        if (target_speed > -5 && target_speed < 5) {
          target_speed = 0;
        }
      }
    } else {

      if (runtime_ms - last_pwm_input_ms > PWM_INPUT_MAX_PERIOD_MS) {
        pwm_input_duty = 0.0f;
        pwm_input_frequency = 0.0f;
      }

      target_speed = pwm_input_duty;
    }
    set_motor_speed(target_speed);

    // ---------------------------------------------------------
    // 2. READ THE CORRECT CURRENT SHUNT BASED ON DIRECTION
    // ---------------------------------------------------------
    if (target_speed > 0) {
      // FORWARD: Current goes to ground via Phase V (ADC2 / OPAMP2)
      HAL_ADC_Start(&hadc2);
      if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK) {
        uint32_t adc2_raw = HAL_ADC_GetValue(&hadc2);

        current_raw = adc2_raw - 2540; // Use your Phase V offset
        // if (current_raw < 0) current_raw = 0; // Prevent noise from showing
        // negative

        current_mA = (float)(current_raw) * 16.786f;
      }
    } else if (target_speed < 0) {
      // REVERSE: Current goes to ground via Phase U (ADC1 / OPAMP1)
      // Switch ADC1 to read OPAMP1 (Channel 3)
      ADC_Select_Channel(&hadc1, ADC_CHANNEL_3);
      HAL_ADC_Start(&hadc1);

      if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
        uint32_t adc1_raw = HAL_ADC_GetValue(&hadc1);

        current_raw = adc1_raw - 2540; // NOTE: Phase U might have a slightly
                                       // different offset than Phase V!
                                       // if (current_raw < 0) current_raw = 0;

        current_mA = (float)(current_raw) * 16.786f;
      }
    } else {
      // COAST/STOPPED
      current_raw = 0;
      current_mA = 0.0f;
    }
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
