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
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_tim.h"
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

typedef enum {
  ESC_STATE_BOOTING,
  ESC_STATE_IDLE,    // Waiting for valid zero-throttle command
  ESC_STATE_RUNNING, // Actively driving the motor
  ESC_STATE_FAULT    // Safety lockout
} ESC_State;

typedef enum {
  ESC_INPUT_MODE_PWM,
  ESC_INPUT_MODE_UART,
  ESC_INPUT_MODE_CAN,
  ESC_INPUT_MODE_POT
} ESC_InputMode;

typedef struct {
  float phase_u_current;
  float phase_v_current;

  float bemf_u_voltage;
  float bemf_v_voltage;

  float bus_voltage;
  float board_temp_c;
} MotorSensors_t;

typedef struct {
  float target_speed;
  float current_speed;

  // Independent raw inputs
  float raw_pwm_input;
  float raw_uart_input;
  float raw_can_input;

  // Independent timeout trackers
  uint32_t last_pwm_cmd_ms;
  uint32_t last_uart_cmd_ms;
  uint32_t last_can_cmd_ms;

  // State tracking
  bool pot_override_active;
  ESC_InputMode active_mode; // Kept so telemetry knows who is driving!
} MotorControl_t;

typedef struct {
  bool overtemperature_protection_on;
  bool voltage_protection_on;

  uint8_t battery_cell_count;
  uint8_t current_threshold_amps;
  uint8_t temperature_threshold_c;
} ESC_Protection_t;

typedef struct {
  bool overcurrent;
  bool overtemp;
  bool overvoltage;
  bool fault_latch;

  uint32_t oc_timestamp_ms;
  uint32_t ot_timestamp_ms;
  uint32_t ov_timestamp_ms;
} SystemFaults_t;

// The Master Context
typedef struct {
  MotorSensors_t sensors;
  MotorControl_t control;
  ESC_Protection_t protection;
  SystemFaults_t faults;
  ESC_State state;
} ESC_Context_t;

#pragma pack(push, 1)
typedef struct {
  float target_speed; // 4 Bytes (-100.0 to 100.0)

  uint16_t config_change : 1;
  uint16_t enable_running_mode : 1;
  uint16_t enable_encoder : 1;
  uint16_t enable_overtemperature_protection : 1;
  uint16_t enable_voltage_protection : 1;
  uint16_t overcurrent_threshold_amps : 6;
  uint16_t battery_cell_count : 4;
  //
} ControlPacket; // Total = 12 Bytes
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  uint16_t header; // 2 Bytes
  ControlPacket packet;
  uint16_t footer; // 2 Bytes
} ControlPacketUART;
#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSCLK_FREQ (170e6)

#define PWM_INPUT_MAX_PERIOD_MS (50)
#define PWM_INPUT_MIN_US (1000.0f)
#define PWM_INPUT_MAX_US (2000.0f)
#define PWM_INPUT_CENTER_US (1500.0f)
#define PWM_INPUT_DEADZONE_US (20.0f)

#define UART_HEADER_BYTES (0x726F)
#define UART_FOOTER_BYTES (0x7365)

#define CURRENT_LOWPASS_ALPHA (0.1f)

#define BUS_VOLTAGE_DIVIDER_RATIO (10.388f)
#define BEMF_VOLTAGE_DIVIDER_RATIO (8.02f)
#define DAC_AMP_TO_BITS_RATIO (59.5f)

#define CANBUS_TERMINATION_RESISTOR_ACTIVE (0)
#define CANBUS_TRANSCEIVER_ACTIVE (1)

#define DEFAULT_BATTERY_CELL_COUNT (6)
#define CELL_VOLTAGE_LOW (3.0f)
#define CELL_VOLTAGE_HIGH (4.2f)

#define OVERVOLTAGE_RATE_ALLOWED (1.2f)
#define UNDERVOLTAGE_RATE_ALLOWED (1.0f)

#define OVERTEMPERATURE_PROTECTION_THRESHOLD_CELCIUS (80.0f)
#define OVERCURRENT_PROTECTION_THRESHOLD_AMPS (20.0f)

#define OVERCURRENT_PROTECTION_TIMEOUT_MS (2000)
#define OVERTEMPERATURE_PROTECTION_TIMEOUT_MS (5000)
#define VOLTAGE_PROTECTION_TIMEOUT_MS (2000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MAX3(a, b, c) MAX(MAX(a, b), c)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// --- THE MASTER SYSTEM VARIABLE ---
ESC_Context_t esc_system = {0};

// Hardware tracking and filtering
volatile float current_u_filtered = 0.0f;
volatile float current_v_filtered = 0.0f;
volatile static int32_t offset_u = 2540;
volatile static int32_t offset_v = 2540;

volatile static float bemf_u_V = 0.0f;
volatile static float bemf_v_V = 0.0f;
volatile static float bemf_w_V = 0.0f;

volatile uint32_t adc1_buffer[4];
volatile uint32_t adc2_buffer[4];

// Communication Buffers
static uint8_t uart_rx_buffer[sizeof(ControlPacketUART)];

FDCAN_RxHeaderTypeDef CanRxHeader;
static uint8_t can_rx_data[64];

static GPIO_PinState last_devboard_button_state = GPIO_PIN_SET;
volatile static uint64_t current_time_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_motor_speed(float speed_percent);

void setup_serial_communication(uint32_t can_id, uint8_t *uart_rx_buf,
                                uint16_t packet_size);
void set_overcurrent_protection_threshold(float threshold_A,
                                          int32_t shunt_offset_u,
                                          int32_t shunt_offset_v);
void calibrate_shunt_adc_offsets(int32_t *shunt_offset_u,
                                 int32_t *shunt_offset_v);
float get_ntc_temperature_c(uint32_t adc_val);
float get_bus_voltage(uint32_t adc_val);
float get_shunt_resistor_current(uint32_t adc_val, uint32_t offset);
float get_potentiometer_target_speed(uint32_t adc_val);
float get_rc_pwm_target_speed(uint32_t ch_ticks);

void MotorSensors_Update(ESC_Context_t *ctx, uint32_t current_time_ms);
void FaultMonitor_Update(ESC_Context_t *ctx, uint32_t current_time_ms);
void MotorControl_Update(ESC_Context_t *ctx, uint32_t current_time_ms);
void MotorState_Update(ESC_Context_t *ctx);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  esc_system.state = ESC_STATE_BOOTING;
  esc_system.control.active_mode = ESC_INPUT_MODE_PWM;

  esc_system.protection.overtemperature_protection_on = true;
  esc_system.protection.voltage_protection_on = false;
  esc_system.protection.battery_cell_count = DEFAULT_BATTERY_CELL_COUNT;
  esc_system.protection.current_threshold_amps =
      OVERCURRENT_PROTECTION_THRESHOLD_AMPS;
  esc_system.protection.temperature_threshold_c =
      OVERTEMPERATURE_PROTECTION_THRESHOLD_CELCIUS;

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

  // Enable or disable the
  HAL_GPIO_WritePin(CAN_TRANSCEIVER_SHUTDOWN_GPIO_Port,
                    CAN_TRANSCEIVER_SHUTDOWN_Pin, !CANBUS_TRANSCEIVER_ACTIVE);

  // Enable or disable the CAN termination resistor according to config
  HAL_GPIO_WritePin(CAN_TERMINATION_GPIO_Port, CAN_TERMINATION_Pin,
                    CANBUS_TERMINATION_RESISTOR_ACTIVE);

  // Power up the Operational Amplifiers
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);

  // Calibrate the ADCs (CRITICAL for STM32G4!)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // Start ADCs in Hardware-Triggered Interrupt Mode
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, 4);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buffer, 4);

  // Start Phase U (OUT1) PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  // Start Phase V (OUT2) PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_Delay(10);

  int32_t shunt_offset_u = 0;
  int32_t shunt_offset_v = 0;
  calibrate_shunt_adc_offsets(&shunt_offset_u, &shunt_offset_v);
  offset_u = shunt_offset_u;
  offset_v = shunt_offset_v;

  set_overcurrent_protection_threshold(OVERCURRENT_PROTECTION_THRESHOLD_AMPS,
                                       offset_u, offset_v);

  // Start the DAC
  HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac3, DAC_CHANNEL_2);

  // Start the Comparators to watch the OPAMPs
  HAL_COMP_Start(&hcomp1);
  HAL_COMP_Start(&hcomp2);
  // ==========================================================

  // Start PWM Input
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // Signal Input Channel (Main)
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);    // Secondary Channel

  setup_serial_communication(FDCAN_STANDARD_ID, uart_rx_buffer,
                             sizeof(uart_rx_buffer));

  esc_system.state = ESC_STATE_IDLE;
  // Start motor control timer
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    GPIO_PinState devboard_button_pressed =
        HAL_GPIO_ReadPin(DEVBOARD_BUTTON_GPIO_Port, DEVBOARD_BUTTON_Pin);
    if (devboard_button_pressed == GPIO_PIN_RESET &&
        last_devboard_button_state != GPIO_PIN_RESET) {
      esc_system.control.pot_override_active =
          !esc_system.control.pot_override_active;
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
    esc_system.faults.overcurrent = true;
    esc_system.faults.oc_timestamp_ms = current_time_ms;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    float inst_mA_u = get_shunt_resistor_current(adc1_buffer[0], offset_u);
    current_u_filtered = (CURRENT_LOWPASS_ALPHA * inst_mA_u) +
                         ((1.0f - CURRENT_LOWPASS_ALPHA) * current_u_filtered);
  } else if (hadc->Instance == ADC2) {
    float inst_mA_v = get_shunt_resistor_current(adc2_buffer[0], offset_v);
    current_v_filtered = (CURRENT_LOWPASS_ALPHA * inst_mA_v) +
                         ((1.0f - CURRENT_LOWPASS_ALPHA) * current_v_filtered);
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    uint32_t cl = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t ch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (cl > 0) {
      // Feed the raw value into the struct and reset the watchdog timer!
      esc_system.control.raw_pwm_input = get_rc_pwm_target_speed(ch);
      esc_system.control.last_pwm_cmd_ms = current_time_ms;
    }
  }
}

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    current_time_ms++;

    MotorSensors_Update(&esc_system, current_time_ms);
    FaultMonitor_Update(&esc_system, current_time_ms);
    MotorControl_Update(&esc_system, current_time_ms);
    MotorState_Update(&esc_system);
  }
}

// ============================================================================
// --- MODULAR ARCHITECTURE FUNCTIONS ---
// ============================================================================

void MotorSensors_Update(ESC_Context_t *esc, uint32_t current_time_ms) {

  esc->sensors.phase_u_current = current_u_filtered;
  esc->sensors.phase_v_current = current_v_filtered;

  esc->sensors.bus_voltage = get_bus_voltage(adc1_buffer[1]);
  esc->sensors.board_temp_c = get_ntc_temperature_c(adc1_buffer[2]);

  esc->sensors.bemf_u_voltage =
      (float)adc2_buffer[1] * (BEMF_VOLTAGE_DIVIDER_RATIO * 3.3f / 4095.0f);
  esc->sensors.bemf_v_voltage =
      (float)adc2_buffer[2] * (BEMF_VOLTAGE_DIVIDER_RATIO * 3.3f / 4095.0f);
}

void FaultMonitor_Update(ESC_Context_t *esc, uint32_t current_time_ms) {
  // Check Voltage
  if (esc->protection.voltage_protection_on) {
    const float v_min = esc->protection.battery_cell_count * CELL_VOLTAGE_LOW *
                        UNDERVOLTAGE_RATE_ALLOWED;
    const float v_max = esc->protection.battery_cell_count * CELL_VOLTAGE_HIGH *
                        OVERVOLTAGE_RATE_ALLOWED;
    if (esc->sensors.bus_voltage < v_min || esc->sensors.bus_voltage > v_max) {
      esc->faults.overvoltage = true;
      esc->faults.ov_timestamp_ms = current_time_ms;
    } else if (current_time_ms - esc->faults.ov_timestamp_ms >
               VOLTAGE_PROTECTION_TIMEOUT_MS) {
      esc->faults.overvoltage = false;
    }
  }

  // Check Temperature
  if (esc->protection.overtemperature_protection_on) {
    if (esc->sensors.board_temp_c > esc->protection.temperature_threshold_c) {
      esc->faults.overtemp = true;
      esc->faults.ot_timestamp_ms = current_time_ms;
    } else if (current_time_ms - esc->faults.ot_timestamp_ms >
               OVERTEMPERATURE_PROTECTION_TIMEOUT_MS) {
      esc->faults.overtemp = false;
    }
  }

  // Check Current Timeout (Flag is set to true by hardware BreakCallback)
  if (esc->faults.overcurrent &&
      (current_time_ms - esc->faults.oc_timestamp_ms >
       OVERCURRENT_PROTECTION_TIMEOUT_MS)) {
    esc->faults.overcurrent = false;
  }

  // Master Latch
  esc->faults.fault_latch = esc->faults.overvoltage || esc->faults.overtemp ||
                            esc->faults.overcurrent;
  HAL_GPIO_WritePin(DEVBOARD_LED_GPIO_Port, DEVBOARD_LED_Pin,
                    esc->faults.fault_latch);
}

void MotorControl_Update(ESC_Context_t *esc, uint32_t current_time_ms) {
  // --- 1. POT OVERRIDE (Absolute Highest Priority) ---
  if (esc->control.pot_override_active) {
    esc->control.active_mode = ESC_INPUT_MODE_POT;
    esc->control.target_speed = get_potentiometer_target_speed(adc1_buffer[3]);
  }
  // --- 2. AUTO-DETECT ROUTER ---
  else {
    // Evaluate which connections are "alive" (received a packet recently)
    bool can_active = (current_time_ms - esc->control.last_can_cmd_ms) <= 500;
    bool uart_active = (current_time_ms - esc->control.last_uart_cmd_ms) <= 500;
    bool pwm_active = (current_time_ms - esc->control.last_pwm_cmd_ms) <=
                      PWM_INPUT_MAX_PERIOD_MS;

    // Cascade down the priority list: CAN -> UART -> PWM
    if (can_active) {
      esc->control.active_mode = ESC_INPUT_MODE_CAN;
      esc->control.target_speed = esc->control.raw_can_input;
    } else if (uart_active) {
      esc->control.active_mode = ESC_INPUT_MODE_UART;
      esc->control.target_speed = esc->control.raw_uart_input;
    } else if (pwm_active) {
      esc->control.active_mode = ESC_INPUT_MODE_PWM;
      esc->control.target_speed = esc->control.raw_pwm_input;
    } else {
      esc->control.target_speed = 0.0f; // Failsafe: No valid inputs
    }
  }
}

void MotorState_Update(ESC_Context_t *esc) {
  switch (esc->state) {
  case ESC_STATE_BOOTING:
    // Do nothing, wait for initialization to finish
    break;

  case ESC_STATE_IDLE:
    set_motor_speed(0.0f);
    if (esc->faults.fault_latch) {
      esc->state = ESC_STATE_FAULT;
    }
    // Re-arm only if throttle is in neutral deadzone (-5% to 5%)
    else if (esc->control.target_speed > -5.0f &&
             esc->control.target_speed < 5.0f) {
      __HAL_TIM_MOE_ENABLE(&htim1);
      esc->state = ESC_STATE_RUNNING;
    }
    break;

  case ESC_STATE_RUNNING:
    if (esc->faults.fault_latch) {
      esc->state = ESC_STATE_FAULT;
    } else {
      set_motor_speed(esc->control.target_speed);
    }
    break;

  case ESC_STATE_FAULT:
    __HAL_TIM_MOE_DISABLE(&htim1); // Immediately kill MOSFETs
    set_motor_speed(0.0f);         // Force target math to zero

    // Once all sensors read safe, drop to IDLE to wait for neutral stick
    if (!esc->faults.fault_latch) {
      esc->state = ESC_STATE_IDLE;
    }
    break;
  }
}

void setup_serial_communication(uint32_t can_id, uint8_t *uart_rx_buf,
                                uint16_t packet_size) {
  // ==========================================================
  // --- FDCAN INITIALIZATION ---
  // ==========================================================
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = can_id;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000; // Mask of 0 accepts all standard IDs

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // ==========================================================
  // --- UART DMA INITIALIZATION ---
  // ==========================================================
  // We use ReceiveToIdle instead of standard Receive.
  // This prevents the buffer from permanently misaligning if a byte is dropped!
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buf, packet_size);
}

/**
* @brief Drives the brushed DC motor on OUT1 and OUT2

* @param speed_percent: -100 to 100 (Negative for reverse, Positive for
forward, 0 to coast)
*/
void set_motor_speed(float speed_percent) {

  // Clamp the input to a safe max for the bootstrap capacitors
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

void calibrate_shunt_adc_offsets(int32_t *shunt_offset_u,
                                 int32_t *shunt_offset_v) {
  uint32_t sum_u = 0;
  uint32_t sum_v = 0;
  const int NUM_CAL_SAMPLES = 100;

  for (int i = 0; i < NUM_CAL_SAMPLES; i++) {
    // The DMA updates these variables automatically at 20kHz
    sum_u += adc1_buffer[0];
    sum_v += adc2_buffer[0];
    HAL_Delay(1); // Wait 1ms between reads (collecting data over 100ms)
  }

  // Calculate the average offset
  if (shunt_offset_u)
    *shunt_offset_u = sum_u / NUM_CAL_SAMPLES;
  if (shunt_offset_v)
    *shunt_offset_v = sum_v / NUM_CAL_SAMPLES;
}

void set_overcurrent_protection_threshold(float threshold_A,
                                          int32_t shunt_offset_u,
                                          int32_t shunt_offset_v) {
  // 1 Amp = ~59.5 raw units. For 20A trip limit: 20 * 59.5 = 1190
  const uint32_t TRIP_DELTA = (uint32_t)(threshold_A * DAC_AMP_TO_BITS_RATIO);

  uint32_t trip_limit_u = shunt_offset_u + TRIP_DELTA;
  uint32_t trip_limit_v = shunt_offset_v + TRIP_DELTA;

  // Clamp limits to maximum 12-bit value (4095) just in case
  if (trip_limit_u > 4095)
    trip_limit_u = 4095;
  if (trip_limit_v > 4095)
    trip_limit_v = 4095;

  // 1. Set the DAC thresholds
  // DAC CH1 feeds COMP1 (Phase U), DAC CH2 feeds COMP2 (Phase V)
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, trip_limit_u);
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, trip_limit_v);
}

float get_ntc_temperature_c(uint32_t adc_val) {
  // Prevent division by zero if ADC reads 0 during boot/fault
  if (adc_val == 0)
    return 25.0f;

  float ntc_voltage = (float)adc_val * (3.3f / 4095.0f);

  // CORRECTED: On the B-G431B-ESC1, the NTC is connected to 3.3V (High side)
  // and the fixed 4.7k resistor is connected to Ground (Low side).
  float ntc_resistance = 4700.0f * (3.3f - ntc_voltage) / ntc_voltage;

  // Beta Equation for Temperature (Beta = 3380 for NCP15XH103F03RC)
  // T = 1 / ( (1/T0) + (1/B)*ln(R/R0) )
  float temp_kelvin =
      1.0f /
      ((1.0f / 298.15f) + (1.0f / 3380.0f) * logf(ntc_resistance / 10000.0f));

  return temp_kelvin - 273.15f;
}

float get_bus_voltage(uint32_t adc_val) {
  float bus_voltage =
      (float)adc_val * (BUS_VOLTAGE_DIVIDER_RATIO * 3.3f / 4095);
  return bus_voltage;
}

float get_shunt_resistor_current(uint32_t adc_val, uint32_t offset) {
  int32_t raw_val = (int32_t)adc_val - offset;
  float inst_mA = (float)raw_val * 16.786f;

  return inst_mA;
}

float get_potentiometer_target_speed(uint32_t adc_val) {
  float target = ((adc_val * 200.0f) / 4095.0f) - 100.0f;
  if (target > -5 && target < 5)
    target = 0;
  return target;
}

float get_rc_pwm_target_speed(uint32_t ch_ticks) {
  float pulse_width_us = (float)ch_ticks / (SYSCLK_FREQ / 1000000.0f);
  float target = 0.0f;

  if (pulse_width_us > PWM_INPUT_CENTER_US + PWM_INPUT_DEADZONE_US) {
    target = ((pulse_width_us - PWM_INPUT_CENTER_US) /
              (PWM_INPUT_MAX_US - PWM_INPUT_CENTER_US)) *
             100.0f;
  } else if (pulse_width_us < PWM_INPUT_CENTER_US - PWM_INPUT_DEADZONE_US) {
    target = ((PWM_INPUT_CENTER_US - pulse_width_us) /
              (PWM_INPUT_CENTER_US - PWM_INPUT_MIN_US)) *
             -100.0f;
  }

  // Clamp limits
  if (target > 100.0f)
    target = 100.0f;
  if (target < -100.0f)
    target = -100.0f;

  return target;
}

float handle_control_packet(ESC_Context_t *esc, const ControlPacket *packet) {

  if (packet->config_change) {
    esc->protection.battery_cell_count = packet->battery_cell_count;
    esc->protection.overtemperature_protection_on =
        packet->enable_overtemperature_protection;
    esc->protection.voltage_protection_on = packet->enable_voltage_protection;
    esc->protection.current_threshold_amps = packet->overcurrent_threshold_amps;

    set_overcurrent_protection_threshold(esc->protection.current_threshold_amps,
                                         offset_u, offset_v);
  }

  if (packet->enable_running_mode) {
    return packet->target_speed;
  } else {
    return 0.0f;
  }
}

// --- UART RECEIVE EVENT ---
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART2) {
    // Check against the UART wrapper
    if (Size == sizeof(ControlPacketUART)) {
      ControlPacketUART *uart_msg = (ControlPacketUART *)uart_rx_buffer;

      // Verify data integrity using your new macros
      if (uart_msg->header == UART_HEADER_BYTES &&
          uart_msg->footer == UART_FOOTER_BYTES) {

        esc_system.control.raw_uart_input =
            handle_control_packet(&esc_system, &(uart_msg->packet));

        // Reset the UART watchdog!
        esc_system.control.last_uart_cmd_ms = current_time_ms;
      }
    }
    // Instantly restart the DMA to listen for the next packet
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buffer,
                                 sizeof(ControlPacketUART));
  }
}
// --- FDCAN RECEIVE EVENT ---
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

    // Pull the frame out of the hardware FIFO
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CanRxHeader,
                               can_rx_data) == HAL_OK) {

      // The FDCAN DataLength is an enum (e.g., FDCAN_DLC_BYTES_12 is actually
      // 0x00090000) As long as the sender packed at least 12 bytes, we can
      // parse it.
      ControlPacket *packet = (ControlPacket *)can_rx_data;

      esc_system.control.raw_can_input =
          handle_control_packet(&esc_system, packet);

      // Reset the CAN watchdog!
      esc_system.control.last_can_cmd_ms = current_time_ms;
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
