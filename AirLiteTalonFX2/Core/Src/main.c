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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "arm_math.h"

#include "imu.h"
#include "mouse.h"
#include "motor.h"
#include "encoder.h"
#include "quaternion.h"
#include "ekf.h"
#include "stat_tracker.h"
#include "dsens.h"

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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void CalibrateSensors(float32_t *gyroOffset, float32_t *R_gyro, float32_t *R_v, float32_t *R_w);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float32_t x_theta = 0;
float32_t y_theta = 0;
float32_t z_theta = 0;

float32_t R_gyro_yaw = 0.05f;
float32_t R_mouse_v = 0.2f;
float32_t R_mouse_w = 0.3f;

float32_t mouse_x_velo = 0;
float32_t mouse_omega = 0;
float32_t mouse_omega_deg = 0;
float32_t mouse_theta_est = 0;

float32_t rotation_quat[4] = {1, 0, 0, 0};

float32_t flDist = 0;
float32_t lDist = 0;
float32_t rDist = 0;
float32_t frDist = 0;

uint32_t last_button1_time = 0;
uint32_t last_button2_time = 0;
uint32_t last_button3_time = 0;
uint32_t debounce_delay = 50;

EKF stateFilter = {0};

Motor rightMotor = {0};
Motor leftMotor = {0};

Encoder rightEncoder = {0};
Encoder leftEncoder = {0};

DSens flSens = {0};
DSens lSens = {0};
DSens rSens = {0};
DSens frSens = {0};

volatile uint32_t us_multiplier;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	uint32_t currentCount = __HAL_TIM_GET_COUNTER(htim);
	Encoder_Callback(&leftEncoder, htim, currentCount);
	Encoder_Callback(&rightEncoder, htim, currentCount);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t current_tick = HAL_GetTick();
	if(GPIO_Pin == BUTTON_1_Pin && (current_tick - last_button1_time > debounce_delay)) {
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		last_button1_time = current_tick;
	} else if(GPIO_Pin == BUTTON_2_Pin && (current_tick - last_button2_time > debounce_delay)) {
		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		last_button2_time = current_tick;
	} else if(GPIO_Pin == BUTTON_3_Pin && (current_tick - last_button3_time > debounce_delay)) {
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		last_button3_time = current_tick;
	}
}

// Call this function inside main() BEFORE using the mouse
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Use HAL_RCC_GetHCLKFreq() here to get the exact system logic freq
    uint32_t freq = HAL_RCC_GetHCLKFreq();

    // Calculate how many ticks are in 1 microsecond
    us_multiplier = freq / 1000000;
}

void delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t ticksNeeded = us * us_multiplier;

    while ((DWT->CYCCNT - startTick) < ticksNeeded);
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

  // Enable CPU timer
  DWT_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  bool mouse_connected = Mouse_Init(&hspi3);
  bool imu_connected = IMU_Init(&hspi1);

  float32_t gyroOffset[3];
  IMU_CalibrateGyro(&hspi1, gyroOffset);

  float32_t accelReadings[3];
  IMU_ReadAccel(&hspi1, accelReadings);
  IMU_GenGravQuat(rotation_quat, accelReadings);

  EKF_Init(&stateFilter, 0.8f);

  CalibrateSensors(gyroOffset, &R_gyro_yaw, &R_mouse_v, &R_mouse_w);

  stateFilter.process_noise_data[EKF_CalculateIndex(5, 0, 0)] = 1e-4f; // X position variance
  stateFilter.process_noise_data[EKF_CalculateIndex(5, 1, 1)] = 1e-4f; // Y position variance
  stateFilter.process_noise_data[EKF_CalculateIndex(5, 2, 2)] = 1e-4f; // Yaw variance
  stateFilter.process_noise_data[EKF_CalculateIndex(5, 3, 3)] = 1e-2f; // Linear Velocity variance
  stateFilter.process_noise_data[EKF_CalculateIndex(5, 4, 4)] = 1e-2f; // Angular Velocity variance

  stateFilter.measurement_noise_data[EKF_CalculateIndex(5, 0, 0)] = 0.1f;  // Left Encoder jitter
  stateFilter.measurement_noise_data[EKF_CalculateIndex(5, 1, 1)] = 0.1f;  // Right Encoder jitter
  stateFilter.measurement_noise_data[EKF_CalculateIndex(5, 2, 2)] = R_gyro_yaw;
  stateFilter.measurement_noise_data[EKF_CalculateIndex(5, 3, 3)] = R_mouse_v;
  stateFilter.measurement_noise_data[EKF_CalculateIndex(5, 4, 4)] = R_mouse_w;

  bool left_motor_initialized = Motor_Init(&rightMotor, M1_FWD_GPIO_Port, M1_BACK_GPIO_Port, M1_FWD_Pin, M1_BACK_Pin, &TIM5->CCR3, &htim5, TIM_CHANNEL_3, true);
  bool right_motor_initialized = Motor_Init(&leftMotor, M2_FWD_GPIO_Port, M2_BACK_GPIO_Port, M2_FWD_Pin, M2_BACK_Pin, &TIM5->CCR4, &htim5, TIM_CHANNEL_4, false);

  bool left_encoder_initialized = Encoder_Init(&rightEncoder, &htim2, true, 1.0f / 5968.31f);
  bool right_encoder_initialized = Encoder_Init(&leftEncoder, &htim4, false, 1.0f / 5968.31f);

  DSens_Init(&flSens, EMIT_1_GPIO_Port, EMIT_1_Pin, &hadc1, ADC_CHANNEL_9, 309608344.0f, -2.02718f);
  DSens_Init(&lSens, EMIT_2_GPIO_Port, EMIT_2_Pin, &hadc1, ADC_CHANNEL_8, 30089.7773f, -0.835766f);
  DSens_Init(&rSens, EMIT_3_GPIO_Port, EMIT_3_Pin, &hadc1, ADC_CHANNEL_15, 587887.835f, -1.21735f);
  DSens_Init(&frSens, EMIT_4_GPIO_Port, EMIT_4_Pin, &hadc1, ADC_CHANNEL_14, 1373374.48f, -1.32993f);

  if (!(mouse_connected && imu_connected && left_motor_initialized && right_motor_initialized && left_encoder_initialized && right_encoder_initialized)) {
	  // Something didn't initialize properly
	  for (int i = 0; i < 50; i++) {
		  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		  HAL_Delay(100);
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tickTime = DWT->CYCCNT;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  MouseData mouseData = {0};
	  Mouse_MotionRead(&hspi3, &mouseData);

	  float32_t gyroReadings[3];
	  float32_t accelReadings[3];

	  IMU_ReadGyroRadPerSec(&hspi1, gyroReadings);
	  IMU_ReadAccel(&hspi1, accelReadings);

	  uint32_t tickTimeNew = DWT->CYCCNT;
	  uint32_t delta = tickTimeNew - tickTime;
	  float32_t dt = (float32_t)delta / (float32_t)HAL_RCC_GetHCLKFreq();
	  tickTime = tickTimeNew;

	  // Gyro processing
	  IMU_ApplyGyroOffset(gyroReadings, gyroOffset);

	  IMU_GyroAccelMadgwickFilter(0.1, rotation_quat, gyroReadings, accelReadings, dt);

	  float32_t rotationEuler[3];
	  Quaternion_To_Euler_Deg(rotation_quat, rotationEuler);

	  x_theta = rotationEuler[0];
	  y_theta = rotationEuler[1];
	  z_theta = rotationEuler[2];


	  // Mouse processing
	  Mouse_GetVelocity(&mouseData, dt, &mouse_x_velo, &mouse_omega);
	  mouse_omega_deg = mouse_omega * 180 / PI;
	  mouse_theta_est += mouse_omega_deg * dt;

	  // Encoder processing
	  Encoder_Tick(&leftEncoder, dt);
	  Encoder_Tick(&rightEncoder, dt);

	  // Distance sensor processing
	  DSens_Update(&flSens);
	  DSens_Update(&lSens);
	  DSens_Update(&rSens);
	  DSens_Update(&frSens);

	  flDist = DSens_GetDist(&flSens);
	  lDist = DSens_GetDist(&lSens);
	  rDist = DSens_GetDist(&rSens);
	  frDist = DSens_GetDist(&frSens);

	  EKF_AddMeasurementAndUpdate(&stateFilter, leftEncoder.rawVeloMeters, rightEncoder.rawVeloMeters, z_theta, mouse_x_velo, mouse_omega, dt);

	  const float32_t kp = 1;
	  const float32_t error = 1 - EKF_GetPosX(&stateFilter);
	  Motor_Set(&leftMotor, kp * error);
	  Motor_Set(&rightMotor, kp * error);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1024;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin|M1_FWD_Pin|M1_BACK_Pin|M2_FWD_Pin
                          |M2_BACK_Pin|MOUSE_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EMIT_4_Pin|EMIT_3_Pin|EMIT_2_Pin|EMIT_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_RED_Pin|LED_BLUE_Pin|LED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : IMU_CS_Pin M1_FWD_Pin M1_BACK_Pin M2_FWD_Pin
                           M2_BACK_Pin MOUSE_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin|M1_FWD_Pin|M1_BACK_Pin|M2_FWD_Pin
                          |M2_BACK_Pin|MOUSE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EMIT_4_Pin EMIT_3_Pin EMIT_2_Pin EMIT_1_Pin */
  GPIO_InitStruct.Pin = EMIT_4_Pin|EMIT_3_Pin|EMIT_2_Pin|EMIT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CalibrateSensors(float32_t *gyroOffset, float32_t *R_gyro, float32_t *R_v, float32_t *R_w) {
  StatTracker gyroStats = {0};
  StatTracker mouseVStats = {0};
  StatTracker mouseWStats = {0};

  uint32_t tickTime = DWT->CYCCNT;

  for (int i = 0; i < 1000; i++) {
	  float32_t g[3];
	  IMU_ReadGyroRadPerSec(&hspi1, g);
	  IMU_ApplyGyroOffset(g, gyroOffset);
	  StatTracker_Update(&gyroStats, g[2]);

	  MouseData mData = {0};
	  Mouse_MotionRead(&hspi3, &mData);

	  uint32_t tickTimeNew = DWT->CYCCNT;
	  uint32_t delta = tickTimeNew - tickTime;
	  float32_t dt = (float32_t)delta / (float32_t)HAL_RCC_GetHCLKFreq();
	  tickTime = tickTimeNew;

	  float32_t v, w;
	  Mouse_GetVelocity(&mData, dt, &v, &w);

	  StatTracker_Update(&mouseVStats, v);
	  StatTracker_Update(&mouseWStats, w);

	  HAL_Delay(2);
  }

  *R_gyro = StatTracker_GetVariance(&gyroStats);
  *R_v = StatTracker_GetVariance(&mouseVStats);
  *R_w = StatTracker_GetVariance(&mouseWStats);

  // Clamp to avoid zero variance
  if (*R_gyro < 1e-6f) *R_gyro = 1e-6f;
  if (*R_v < 1e-6f) *R_v = 1e-6f;
  if (*R_w < 1e-6f) *R_w = 1e-6f;
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

