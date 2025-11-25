/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "math.h"
#include "encoders.h"
#include "motor.h"
#include "vsens.h"
#include "dsens.h"
#include "motionprofile.h"
#include "PIDFF.h"
#include "differentialdrive.h"
#include "FloodFill.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
Encoder leftEncoder(&htim3, false, 0.151843644924);
Encoder rightEncoder(&htim4, false, 0.151843644924);
Motor leftMotor(M1_FWD_GPIO_Port, M1_BCK_GPIO_Port, M1_FWD_Pin, M1_BCK_Pin, &TIM2->CCR4, &htim2, TIM_CHANNEL_3, false);
Motor rightMotor(M2_FWD_GPIO_Port, M2_BCK_GPIO_Port, M2_FWD_Pin, M2_BCK_Pin, &TIM2->CCR3, &htim2, TIM_CHANNEL_4, true);
VSens vsens(&hadc1, ADC_CHANNEL_1);

DSens flSens(EMIT_1_GPIO_Port, EMIT_1_Pin, &hadc1, ADC_CHANNEL_9, FL_FACTOR, FL_POW);
DSens lSens(EMIT_2_GPIO_Port, EMIT_2_Pin, &hadc1, ADC_CHANNEL_8, L_FACTOR, L_POW);
DSens rSens(EMIT_3_GPIO_Port, EMIT_3_Pin, &hadc1, ADC_CHANNEL_5, R_FACTOR, R_POW);
DSens frSens(EMIT_4_GPIO_Port, EMIT_4_Pin, &hadc1, ADC_CHANNEL_4, FR_FACTOR, FR_POW);

PIDFF centeringPID(0.005, 0, 0, 0);

PIDFF leftPIDFF(0.003, 0, LEFT_KV, LEFT_KS);
PIDFF rightPIDFF(0.003, 0, RIGHT_KV, RIGHT_KS);

DifferentialDrive diffy(50);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	uint32_t currentCount = __HAL_TIM_GET_COUNTER(htim);
	leftEncoder.callback(htim, currentCount);
	rightEncoder.callback(htim, currentCount);
}

uint32_t lastTick = 0;
uint32_t curTick = 0;
uint32_t rawStartTick = 0;
uint32_t rawCurTick = 0;
uint32_t dt = 0;

float lastTheta = 0;
float theta = 0;

//float spedL[1000];
//float spedR[1000];
uint64_t ticks = 0;


MotionProfileState nextState = {0};

typedef enum {
	STOP,
	DFORWARD,
	DLEFT,
	DRIGHT,
	DSFORWARD,
	DUTURN
} Action;

typedef struct {
	float velo;
	float thetaVelo;
	bool complete;
	bool useWallCorrection;
} DiffyState;

typedef struct {
    int x = 0;
    int y = 0;
} Position;

bool forwardOneBoxInit = true;
float forwardOneBoxLPos = 0;
float forwardOneBoxRPos = 0;

MapNode race_map[MAP_SIZE][MAP_SIZE];

Position last_position = Position();
Position position =  Position();
Orientation orientation = NORTH;

DiffyState forwardOneBox(uint32_t dt) {
	if (forwardOneBoxInit) {
		forwardOneBoxLPos = leftEncoder.getPositionMeters();
		forwardOneBoxRPos = rightEncoder.getPositionMeters();

		last_position = position;

		switch (orientation) {
			case NORTH:
				position.y++;
				break;
			case EAST:
				position.x++;
				break;
			case SOUTH:
				position.y--;
				break;
			case WEST:
				position.x--;
				break;
		}

	}
	forwardOneBoxInit = false;

	DiffyState state = {0};
	state.useWallCorrection = true;
	float avgPosMeters = ((leftEncoder.getPositionMeters() - forwardOneBoxLPos) + (rightEncoder.getPositionMeters() - forwardOneBoxRPos)) / 2;
	MotionProfileState currentState = {avgPosMeters, (leftEncoder.getVeloMeters() + rightEncoder.getVeloMeters()) / 2, };
	MotionProfileState goalState = {180, 0};
	MotionProfileState nextState =  calculate_next_state(dt * 0.001, currentState, goalState, {200, 700});

	state.velo = nextState.velocity;

//	if (nextState.velocity <= 10) { /* abs(nextState.position - avgPosMeters) < 10 &&  */
//		state.complete = true;
//	}

	state.complete = is_finished(currentState, goalState, 30);

	return state;
}

Orientation rotate_orientation_left(Orientation orientation) {
    if (orientation == NORTH) {
        orientation = WEST;
        return orientation;
    }

    orientation = static_cast<Orientation>(orientation - 1);

    return orientation;
}

Orientation rotate_orientation_right(Orientation orientation) {
    if (orientation == WEST) {
        orientation = NORTH;
        return orientation;
    }

    orientation = static_cast<Orientation>(orientation + 1);

    return orientation;
}


bool rotateInit = true;
DiffyState rotate(uint32_t dt, bool left) {
	if (rotateInit) {
		// On init rotate the rotation state
		if (left) {
			orientation = rotate_orientation_left(orientation);
		} else {
			orientation = rotate_orientation_right(orientation);
		}
	}


	DiffyState state = {0};
	state.useWallCorrection = false;
	MotionProfileState currentState;
	  if (!rotateInit) {
		  currentState = {nextState.position, nextState.velocity};
	  } else {
		  currentState = {0, 0};
	  }
		rotateInit = false;
		MotionProfileState goalState = {(left ? -1 : 1) * 3.141592/2, 0};
	  nextState = calculate_next_state(dt * 0.001, currentState, goalState, {3.141592, 3.141592*4});

	state.thetaVelo = nextState.velocity;

	/*if (nextState.velocity <= 0.3141592) {
		state.complete = true;
	}*/

	state.complete = is_finished(currentState, goalState, 3.141592 / 8);


	return state;
}


bool rotate2Init = true;
DiffyState rotate2(uint32_t dt) {

	DiffyState state = {0};
	state.useWallCorrection = false;
	MotionProfileState currentState;
	  if (!rotateInit) {
		  currentState = {nextState.position, nextState.velocity};
	  } else {
		  currentState = {0, 0};
	  }
		rotateInit = false;
		MotionProfileState goalState = {3.141592, 0};
	  nextState = calculate_next_state(dt * 0.001, currentState, goalState, {3.141592, 3.141592*4});

	state.thetaVelo = nextState.velocity;

	/*if (nextState.velocity <= 0.3141592) {
		state.complete = true;
	}*/

	state.complete = is_finished(currentState, goalState, 3.141592 / 8);


	return state;
}

Action actionState = DSFORWARD;
Action enqueuedAction = DUTURN;

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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);

  //altfx_init();
  HAL_Delay(2000);

  // Initial floodfill logic
  generate_map(race_map);
  propogate_race_distances(race_map);


  rawStartTick = HAL_GetTick();

  leftEncoder.init();
  rightEncoder.init();


  leftMotor.init();
  rightMotor.init();

  leftMotor.brakeMode = false;
  rightMotor.brakeMode = false;

  leftMotor.set(0);
  rightMotor.set(0);

  vsens.init();
  while (!vsens.poll());
  vsens.getVolts();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //MotionProfile profile = generate_profile(200, 700, 180, 0, 0);

  bool init = true;
  lastTick = 0;
  while (1) {
	  if (init) {
		  lastTick = HAL_GetTick() - rawStartTick;
	  }

	  flSens.updateSens();
	  flSens.getDist();
	  lSens.updateSens();
	  lSens.getDist();
	  rSens.updateSens();
	  rSens.getDist();
	  frSens.updateSens();
	  frSens.getDist();
	  vsens.poll();
	  vsens.getVolts();
	  vsens.getCompMult();


	  leftEncoder.tick();
	  rightEncoder.tick();
	  rawCurTick = HAL_GetTick();
	  curTick = rawCurTick - rawStartTick;
	  uint32_t dt = curTick - lastTick;

	  float leftDelta = leftEncoder.getPositionMeters() - leftEncoder.getLastPositionMeters();
	  float rightDelta = rightEncoder.getPositionMeters() - rightEncoder.getLastPositionMeters();

	  float dTheta = (rightDelta - leftDelta) / diffy._rad;
	  theta += dTheta;


//	  HAL_GPIO_TogglePin(POWER_LED_GPIO_Port, POWER_LED_Pin);

	  //float v = profile_velo_wrt_t(rawCurTick * 0.001, startTime * 0.001, profile);


	 /* MotionProfileState currentState;
	  if (!init) {
		  currentState = {nextState.position, nextState.velocity};
	  } else {
		  currentState = {0, 0};
	  }
	  nextState = calculate_next_state(dt * 0.001, currentState, {3.141592 * 2, 0}, {3.141592, 3.141592*4});*/
//	  DiffyState targetState;
//	  if (actionState == DSFORWARD) {
//		  targetState = {500, 0, false, true};
//		  if (flSens.getDist() <= FL_THRESH && frSens.getDist() <= FR_THRESH) {
//				targetState.complete = true;
//		  }
//		  enqueuedAction = DUTURN;
//	  }
//	  else if (actionState == DUTURN) {
//		  targetState = rotate2(dt);
//		  enqueuedAction = DSFORWARD;
//	  }
//	  else if (actionState == DFORWARD) {
//		  targetState = forwardOneBox(dt);
//		  enqueuedAction = STOP;
//	  } else if (actionState == DLEFT) {
//		  targetState = rotate(dt, true);
//		  enqueuedAction = DFORWARD;
//	  } else if (actionState == DRIGHT) {
//		  targetState = rotate(dt, false);
//		  enqueuedAction = DFORWARD;
//	  } else if (actionState == STOP) {
//		  //targetState = {0, 0, true, true};
//		  if (lSens.getDist() <= LEFT_THRESH) {
//	            Orientation rotated_orientation = rotate_orientation_left(orientation);
//	            sever_path(&race_map[position.x][position.y], rotated_orientation);
//		  }
//
//		  if (rSens.getDist() <= RIGHT_THRESH) {
//	            Orientation rotated_orientation = rotate_orientation_right(orientation);
//	            sever_path(&race_map[position.x][position.y], rotated_orientation);
//		  }
//
//
//		  if (flSens.getDist() <= FL_THRESH && frSens.getDist() <= FR_THRESH) {
//	            sever_path(&race_map[position.x][position.y], orientation);
//		  }
//
//
//	        map_node(&race_map[position.x][position.y]);
//
//	        /*if (race_map[position.x][position.y].distance == 0) {
//				state = REHOMING;
//				break;
//			}*/
//
//	        Orientation target_dir = get_most_optimal_node_connection(&race_map[position.x][position.y], true, false).connection_orientation;
//		   int delta_theta = orientation - target_dir; // + means turn left | - means turn right
//
//		   if (abs(delta_theta) > 2) delta_theta = signbit(delta_theta) ? 1 : -1;
//
//		   if (delta_theta == 1) {
//			   actionState = DLEFT;
//		   } else if (delta_theta == -1) {
//			   actionState = DRIGHT;
//		   } else {
//			   actionState = DFORWARD;
//		   }
//
//	  }
//
//	  if (targetState.complete) {
//		  actionState = enqueuedAction;
//		  enqueuedAction = STOP;
//		  rotateInit = true;
//		  forwardOneBoxInit = true;
//	  }
//	  DiffyState targetState = {300, 0, false};



	  //DifferentialDrive::WheelSpeeds speeds = diffy.drive(newState.velocity, centeringPID.calculate(0, rSens.getDist() - lSens.getDist()));
	  /*DifferentialDrive::WheelSpeeds speeds = diffy.drive(targetState.velo, targetState.thetaVelo + ((targetState.useWallCorrection ? 1 : 0) * (centeringPID.calculate(0, rSens.getDist() - lSens.getDist())))); // 3.141592 * 2

	  leftMotor.set(leftPIDFF.calculate(speeds.lv, leftEncoder.getVeloMeters()) * vsens.getCompMult());
	  rightMotor.set(rightPIDFF.calculate(speeds.rv, rightEncoder.getVeloMeters()) * vsens.getCompMult());
*/
//	  leftMotor.set(leftPIDFF.calculate(0, leftEncoder.getVeloMeters()) * vsens.getCompMult());
//	  rightMotor.set(rightPIDFF.calculate(0, rightEncoder.getVeloMeters()) * vsens.getCompMult());
	  leftMotor.set(0);
	  rightMotor.set(0);
	  lastTick = curTick;
	  lastTheta = theta;
	  init = false;

  /*leftMotor.set(0);
  rightMotor.set(0); //\frac{1}{50\cdot12}\cdot2\pi\cdot\frac{29}{2}

  while (1)
  {
	  leftEncoder.tick();
	  rightEncoder.tick();
	  rawCurTick = HAL_GetTick();
	  curTick = rawCurTick - rawStartTick;

	  DifferentialDrive::WheelSpeeds speeds = diffy.drive(0, 2 * 3.14159264);

	  leftMotor.set(leftPIDFF.calculate(speeds.lv, leftEncoder.getVeloMeters()) * vsens.getCompMult());
	  rightMotor.set(rightPIDFF.calculate(speeds.rv, rightEncoder.getVeloMeters()) * vsens.getCompMult());*/
	  /*leftMotor.set(0.1);
	  rightMotor.set(0.1);

	  if (curTick > 2000 && curTick <= 4000) {
		  leftMotor.set(0.2);
		  rightMotor.set(0.2);
	  } else if (curTick > 4000 && curTick <= 6000) {
		  leftMotor.set(0.3);
		  rightMotor.set(0.3);
	  } else if (curTick > 6000 && curTick <= 8000) {
		  leftMotor.set(0.4);
		  rightMotor.set(0.4);
	  } else if (curTick > 8000 && curTick <= 10000) {
		  leftMotor.set(0.5);
		  rightMotor.set(0.5);
	  } else if (curTick > 10000 && curTick <= 12000) {
		  leftMotor.set(0.6);
		  rightMotor.set(0.6);
	  } else if (curTick > 12000 && curTick <= 14000) {
		  leftMotor.set(0.7);
		  rightMotor.set(0.7);
	  } else if (curTick > 14000) {
		  leftMotor.set(0);
		  rightMotor.set(0);
		  leftEncoder.getPositionMeters();
	  }*/

//	  spedL[ticks] = leftEncoder.getVeloMeters();
//	  spedR[ticks] = rightEncoder.getVeloMeters();
	  ticks++;

	  dt = curTick - lastTick;

	//altfx_loop();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
	Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1024;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;(leftPIDFF.calculate(0, leftEncoder.getVeloMeters()) * vsens.getCompMult());
  //	  rightMotor.se
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EMIT_3_Pin|EMIT_2_Pin|EMIT_1_Pin|M2_FWD_Pin
                          |M1_BCK_Pin|M2_BCK_Pin|EMIT_4_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M1_FWD_Pin|LED_RED_Pin|LED_BLUE_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : POWER_LED_Pin */
  GPIO_InitStruct.Pin = POWER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EMIT_3_Pin EMIT_2_Pin EMIT_1_Pin M2_FWD_Pin
                           M1_BCK_Pin M2_BCK_Pin EMIT_4_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = EMIT_3_Pin|EMIT_2_Pin|EMIT_1_Pin|M2_FWD_Pin
                          |M1_BCK_Pin|M2_BCK_Pin|EMIT_4_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_FWD_Pin */
  GPIO_InitStruct.Pin = M1_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_FWD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
