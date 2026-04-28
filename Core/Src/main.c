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
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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
#define CURRENT_LOWPASS_ALPHA (0.1f)
#define OVERCURRENT_PROTECTION_THRESHOLD_AMPS (20.0f)
#define DAC_AMP_TO_BITS_RATIO (59.5f)
#define OVERCURRENT_PROTECTION_TIMEOUT_MS (2000)
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

volatile bool hardware_fault_triggered = false;
volatile static bool pot_mode_enabled = false;
static GPIO_PinState last_devboard_button_state = GPIO_PIN_SET;

volatile static uint32_t phase_u_raw = 0;
volatile static uint32_t phase_v_raw = 0;

volatile float current_u_filtered = 0.0f;
volatile float current_v_filtered = 0.0f;

// [0] Phase U Current, [1] Bus Voltage, [2] NTC Temperature, [3] Potentiometer
volatile uint32_t adc1_buffer[4];

volatile static int32_t offset_u = 2540;
volatile static int32_t offset_v = 2540;

volatile static uint64_t runtime_ms = 0;
volatile static uint64_t last_pwm_input_ms = 0;
volatile static uint64_t fault_timestamp_ms = 0;
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

void set_motor_speed(float speed_percent) {

  // Clamp the input to a safe max of 95% for the bootstrap capacitors

  if (speed_percent > 98.0f)
    speed_percent = 98.0f;

  if (speed_percent < -98.0f)
    speed_percent = -98.0f;

  // Get the current Auto-Reload Register (ARR) value
  const uint32_t arr_val = __HAL_TIM_GET_AUTORELOAD(&htim1);
  const uint32_t ccr_val =
      (uint32_t)((fabsf(speed_percent) * arr_val) / 100.0f);

  if (speed_percent > 2.0f) {

    // Forward: Drive OUT1, Coast OUT2
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_val);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

  }

  else if (speed_percent < -2.0f) {
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
  MX_DMA_Init();
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

  // Start ADCs in Hardware-Triggered Interrupt Mode
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, 4);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&phase_v_raw, 1);

  // 3. Start Phase U (OUT1) PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  // 4. Start Phase V (OUT2) PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_Delay(10);

  uint32_t sum_u = 0;
  uint32_t sum_v = 0;
  const int NUM_CAL_SAMPLES = 100;

  for (int i = 0; i < NUM_CAL_SAMPLES; i++) {
    // The DMA updates these variables automatically at 20kHz
    sum_u += adc1_buffer[0];
    sum_v += phase_v_raw;
    HAL_Delay(1); // Wait 1ms between reads (collecting data over 100ms)
  }

  // Calculate the average offset
  offset_u = sum_u / NUM_CAL_SAMPLES;
  offset_v = sum_v / NUM_CAL_SAMPLES;

  // ==========================================================
  // --- HARDWARE OVERCURRENT PROTECTION SETUP ---
  // ==========================================================
  // 1 Amp = ~59.5 raw units. For 20A trip limit: 20 * 59.5 = 1190
  const uint32_t TRIP_DELTA =
      (uint32_t)(OVERCURRENT_PROTECTION_THRESHOLD_AMPS * DAC_AMP_TO_BITS_RATIO);

  uint32_t trip_limit_u = offset_u + TRIP_DELTA;
  uint32_t trip_limit_v = offset_v + TRIP_DELTA;

  // Clamp limits to maximum 12-bit value (4095) just in case
  if (trip_limit_u > 4095)
    trip_limit_u = 4095;
  if (trip_limit_v > 4095)
    trip_limit_v = 4095;

  // 1. Set the DAC thresholds
  // DAC CH1 feeds COMP1 (Phase U), DAC CH2 feeds COMP2 (Phase V)
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, trip_limit_u);
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, trip_limit_v);

  // 2. Start the DAC
  HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac3, DAC_CHANNEL_2);

  // 3. Start the Comparators to watch the OPAMPs
  HAL_COMP_Start(&hcomp1);
  HAL_COMP_Start(&hcomp2);
  // ==========================================================

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

// This automatically fires if COMP1 or COMP2 trips TIM1
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    hardware_fault_triggered = true;
    target_speed = 0.0f;
    fault_timestamp_ms = runtime_ms;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

  if (hadc->Instance == ADC1) {
    // 1. Calculate real mA for Phase U
    int32_t raw_u = (int32_t)adc1_buffer[0] - offset_u;
    float inst_mA_u = (float)raw_u * 16.786f;

    // 2. Apply 20 kHz Software IIR Filter
    current_u_filtered = (CURRENT_LOWPASS_ALPHA * inst_mA_u) +
                         ((1.0f - CURRENT_LOWPASS_ALPHA) * current_u_filtered);
  } else if (hadc->Instance == ADC2) {
    // 1. Calculate real mA for Phase V
    int32_t raw_v = (int32_t)phase_v_raw - offset_v;
    float inst_mA_v = (float)raw_v * 16.786f;

    // 2. Apply 20 kHz Software IIR Filter
    current_v_filtered = (CURRENT_LOWPASS_ALPHA * inst_mA_v) +
                         ((1.0f - CURRENT_LOWPASS_ALPHA) * current_v_filtered);
  }
}

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

    if (hardware_fault_triggered) {
      target_speed = 0.0f;

      // Wait for the timeout
      if ((runtime_ms - fault_timestamp_ms) >= OVERCURRENT_PROTECTION_TIMEOUT_MS) {

        float pending_throttle = 0.0f;
        if (pot_mode_enabled) {
          uint32_t adc_val = adc1_buffer[3];
          pending_throttle = ((adc_val * 200.0f) / 4095.0f) - 100.0f;
        } else {
          pending_throttle = pwm_input_duty;
        }

        // Only re-arm if the throttle is in the neutral deadzone
        if (pending_throttle > -5.0f && pending_throttle < 5.0f) {
          hardware_fault_triggered = false;
          __HAL_TIM_MOE_ENABLE(&htim1); // Re-enable the pwm output
        }
      }

    } else if (pot_mode_enabled) {
      // DMA constantly updates adc1_buffer
      uint32_t adc_val = adc1_buffer[3];
      target_speed = ((adc_val * 200.0f) / 4095.0f) - 100.0f;
      if (target_speed > -5 && target_speed < 5)
        target_speed = 0;
    } else {
      if (runtime_ms - last_pwm_input_ms > PWM_INPUT_MAX_PERIOD_MS) {
        pwm_input_duty = 0.0f;
        pwm_input_frequency = 0;
      }
      target_speed = pwm_input_duty; // Assuming mapping is fixed!
    }

    set_motor_speed(target_speed);

    // Get current based on direction
    if (target_speed > 0) {
      // Forward: Phase V
      current_mA = current_v_filtered;
    } else if (target_speed < 0) {
      // Reverse: Phase U
      current_mA = current_u_filtered;
    } else {
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
