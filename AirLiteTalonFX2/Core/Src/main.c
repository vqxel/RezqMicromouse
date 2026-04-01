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
#include "math.h"

#include "pmw3389_srom.h"

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

double x_theta = 0;
double y_theta = 0;
double z_theta = 0;

double x_accel = 0;
double y_accel = 0;
double z_accel = 0;

double x_velo = 0;
double y_velo = 0;
double z_velo = 0;

double gyroDeltaQuat[4];
double rotation_quat[4] = {1, 0, 0, 0};

volatile uint32_t us_multiplier;

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

void Mouse_ReadRegister(uint8_t addressByte, uint8_t *buffer, size_t len) {
    uint8_t addr = addressByte & 0x7F;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, &addr, 1, 100);
    delay_us(160);
    HAL_SPI_Receive(&hspi3, buffer, len, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    delay_us(20);
}

void Mouse_WriteRegister(uint8_t addressByte, uint8_t value) {
	uint8_t txData[2] = {addressByte | 0x80, value};

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, txData, 2, 100);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    delay_us(180);
}

/**
 * @brief  Initializes the PMW3389 Sensor and uploads firmware.
 * @return 1 if successful, 0 if SROM verification failed.
 */
bool Mouse_Init(void) {
    uint8_t data = 0;

    // --- PHASE 1: HARDWARE CHECK & RESET ---

    // 1. Perform Soft Reset (Write 0x5A to 0x3A)
    // Even if it "works" without this, this ensures a clean state every boot.
    Mouse_WriteRegister(0x3A, 0x5A);
    HAL_Delay(50); // Wait for chip reboot [cite: 521]

    // 2. Verify Product ID (0x00) -> Expect 0x47
    Mouse_ReadRegister(0x00, &data, 1);
    if (data != 0x47) {
        return false; // Error: SPI Connection Failed
    }

    // --- PHASE 2: SROM UPLOAD SEQUENCE ---
    // This specific sequence "unlocks" the DSP for firmware upload.

    // 3. Disable Rest Mode (Config2 0x10 -> 0x00)
    Mouse_WriteRegister(0x10, 0x00);

    Mouse_WriteRegister(0x13, 0x1D);
    HAL_Delay(10);
    Mouse_WriteRegister(0x13, 0x18);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

	// B. Send Burst Load Address (0x62 | 0x80 for Write)
	uint8_t burstAddr = 0x62 | 0x80;
	HAL_SPI_Transmit(&hspi3, &burstAddr, 1, 100);

	delay_us(15);

	// D. Send Firmware Loop
	for (int i = 0; i < FIRMWARE_LENGTH; i++) {
		// Read byte from your header array
		uint8_t byte = firmware_data[i];

		// Transmit 1 Byte
		HAL_SPI_Transmit(&hspi3, &byte, 1, 100);

		// CRITICAL: 15us delay between bytes is required for Burst Mode
		delay_us(15);
	}

	// E. End Transaction
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

	// F. Allow SROM to initialize
	delay_us(200);


	// --- PHASE 3: VERIFICATION ---

	// 8. Check SROM ID (0x2A)
	// If successful, this will NOT be 0x00.
	Mouse_ReadRegister(0x2A, &data, 1);

    if (data == 0x00) {
        return false; // Error: SROM Download Failed
    }

    // --- PHASE 4: FINAL CONFIGURATION ---

    // 9. Force Run Mode (Config2) again to prevent auto-sleep
    Mouse_WriteRegister(0x10, 0x00);

    // (Optional) Enable Motion Burst in 0x50 if needed later
    // Mouse_WriteRegister(0x50, ...);

    return true; // Success
}

void IMU_WriteRegister(uint8_t addressByte, uint8_t value) {
	uint8_t txData[2] = {addressByte & 0x7F, value};

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, txData, 2, 100);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void IMU_ReadRegister(uint8_t addressByte, uint8_t *buffer, size_t len) {
	len++;

	uint8_t txData[len];
	uint8_t rxData[len];

	memset(txData, 0, len);
	txData[0] = addressByte | 0x80;

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, txData, rxData, len, 100);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

	memcpy(buffer, &rxData[1], len--);
}

bool IMU_WriteAndValidateRegister(uint8_t addressByte, uint8_t value) {
	uint8_t buffer[1] = {0x00};
	IMU_WriteRegister(addressByte, value);
	IMU_ReadRegister(addressByte, buffer, 1);

	return buffer[0] == value;
}

bool IMU_TryWriteRegister(uint8_t addressByte, uint8_t value, uint8_t maxAttempts) {
	for (int attempts = 0; attempts < maxAttempts; attempts++) {
		bool success = IMU_WriteAndValidateRegister(addressByte, value);
		if (success) {
			return true;
		}
	}

	return false;
}

void IMU_ReadGyroDegPerSec(double *gyroReadings) {
	uint8_t gyroBuffer[6];
	IMU_ReadRegister(0x06, gyroBuffer, 6);

	// Change from uint16_t to int16_t
	int16_t x_raw = (int16_t)(((uint16_t)gyroBuffer[1] << 8) | gyroBuffer[0]);
	int16_t y_raw = (int16_t)(((uint16_t)gyroBuffer[3] << 8) | gyroBuffer[2]);
	int16_t z_raw = (int16_t)(((uint16_t)gyroBuffer[5] << 8) | gyroBuffer[4]);

	gyroReadings[0] = (double)x_raw / -32.8; // Pitch (pitch down/ccw from left is +) NOTE: Gyro is mounted backwards
	gyroReadings[1] = (double)y_raw / -32.8; // Roll (roll ccw from back is +) NOTE: Gyro is mounted backwards
	gyroReadings[2] = (double)z_raw / 32.8; // Yaw (ccw + from top)
}

void IMU_ReadGyroRadPerSec(double *gyroReadings) {
	IMU_ReadGyroDegPerSec(gyroReadings);

	gyroReadings[0] = gyroReadings[0] / 180 * M_PI;
	gyroReadings[1] = gyroReadings[1] / 180 * M_PI;
	gyroReadings[2] = gyroReadings[2] / 180 * M_PI;
}

void IMU_GenInstQuat(double *quat, double *gyroReadings, double dt) {
	double vecMagSq = gyroReadings[0]*gyroReadings[0] + gyroReadings[1]*gyroReadings[1] + gyroReadings[2]*gyroReadings[2];

	if (vecMagSq < 1e-10) {
	  // No rotation: Identity quaternion
	  quat[0] = 1.0; quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;
	} else {
	  double vecMag = sqrt(vecMagSq);
	  double quatTheta = dt * vecMag;
	  // TODO: Consider small angle approx for sin and cos
	  double sinD2 = sin(quatTheta / 2.0);

	  quat[0] = cos(quatTheta / 2.0);
	  // Note: (gyroReadings[i] / vecMag) * sinD2
	  double scale = sinD2 / vecMag;
	  quat[1] = gyroReadings[0] * scale;
	  quat[2] = gyroReadings[1] * scale;
	  quat[3] = gyroReadings[2] * scale;
	}
}

void IMU_CalibrateGyro(double *gyroOffset) {
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

	const uint32_t samples = 1000;
	double x_sum = 0, y_sum = 0, z_sum = 0;

	double gyroReadings[3];
	for(int i = 0; i < samples; i++) {
		IMU_ReadGyroRadPerSec(gyroReadings);
		x_sum += gyroReadings[0];
		y_sum += gyroReadings[1];
		z_sum += gyroReadings[2];
		HAL_Delay(1);
	}

	gyroOffset[0] = x_sum / (double) samples;
	gyroOffset[1] = y_sum / (double) samples;
	gyroOffset[2] = z_sum / (double) samples;

	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}

void IMU_ReadAccel(double *accelReadings) {
	uint8_t accelBuffer[6];
	IMU_ReadRegister(0x00, accelBuffer, 6);

	// Change from uint16_t to int16_t
	int16_t x_raw = (int16_t)(((uint16_t)accelBuffer[1] << 8) | accelBuffer[0]);
	int16_t y_raw = (int16_t)(((uint16_t)accelBuffer[3] << 8) | accelBuffer[2]);
	int16_t z_raw = (int16_t)(((uint16_t)accelBuffer[5] << 8) | accelBuffer[4]);

	accelReadings[0] = (double)x_raw / -8192.0 * 9.80665; // + x is right NOTE: Inverted from physical mounting
	accelReadings[1] = (double)y_raw / -8192.0 * 9.80665; // + y is forwards NOTE: Inverted from physical mounting
	accelReadings[2] = (double)z_raw / 8192.0 * 9.80665; // + z is up
}

void IMU_GenGravQuat(double *quat, double *accelReadings) {
	double vecMagSq = accelReadings[0]*accelReadings[0] + accelReadings[1]*accelReadings[1] + accelReadings[2]*accelReadings[2];

	if (vecMagSq < 1e-10) {
	  // No rotation: Identity quaternion
	  quat[0] = 1.0; quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;
	} else {
	  double vecMag = sqrt(vecMagSq);

	  double quatTheta = acos(accelReadings[2] / vecMag);
	  double rotationAxis[3] = {accelReadings[1], -accelReadings[0], 0};
	  double rotationAxisMag = sqrt(rotationAxis[0]*rotationAxis[0]+rotationAxis[1]*rotationAxis[1]+rotationAxis[2]*rotationAxis[2]);

	  if (rotationAxisMag < 1e-10) {
		  // Already vertical, no rotation needed
		  quat[0] = 1.0; quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;
	  } else {
		  rotationAxis[0] /= rotationAxisMag;
		  rotationAxis[1] /= rotationAxisMag;
		  rotationAxis[2] /= rotationAxisMag;

		  // TODO: Consider small angle approx for sin and cos
		  double sinD2 = sin(quatTheta / 2.0);
		  quat[0] = cos(quatTheta / 2.0);
		  quat[1] = rotationAxis[0] * sinD2;
		  quat[2] = rotationAxis[1] * sinD2;
		  quat[3] = 0; //rotationAxis[2] * sinD2; -> 0
	  }
	}
}

void IMU_CalibrateAccel(double *accelOffset) {
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

	const uint32_t samples = 1000;
	double x_sum = 0, y_sum = 0, z_sum = 0;

	double accelReadings[3];
	for(int i = 0; i < samples; i++) {
		IMU_ReadAccel(accelReadings);
		x_sum += accelReadings[0];
		y_sum += accelReadings[1];
		z_sum += accelReadings[2];
		HAL_Delay(1);
	}

	accelOffset[0] = x_sum / (double) samples;
	accelOffset[1] = y_sum / (double) samples;
	accelOffset[2] = z_sum / (double) samples;

	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}

bool IMU_Init() {
	bool imu_connected = false;
	uint8_t whoAmIBuf[1] = {0x00};
	for (int attempts = 0; attempts < 5; attempts++) {
	  IMU_ReadRegister(0x72, whoAmIBuf, 1);
	  if (whoAmIBuf[0] == 0xe5) {
		  imu_connected = true;
		  break;
	  }
	}

	bool imu_initialized = false;
	if (imu_connected) {
	  bool imu_init_failed = false;

	  // Set to low noise mode
	  imu_init_failed |= !IMU_TryWriteRegister(0x10, 0x0F, 5);

	  // Set to +- 4g accel res
	  imu_init_failed |= !IMU_TryWriteRegister(0x1B, 0b0110110, 5);

	  // Set to 1000 dps gyro res
	  imu_init_failed |= !IMU_TryWriteRegister(0x1C, 0b00100110, 5);

	  imu_initialized = !imu_init_failed;
	}

	return imu_initialized;
}

void Quaternion_Multiply(double *out, const double *q, const double *p) {
	// q * p NOT p * q
    double w = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
    double x = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
    double y = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1];
    double z = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0];

    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

void Quaternion_Normalize(double *q) {
    double mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (mag > 1e-10) {
        q[0] /= mag; q[1] /= mag; q[2] /= mag; q[3] /= mag;
    }
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

  // TODO: On the kirk it somehow works without me following the required startup reset procedure lol xd !
  /*uint8_t buffer[1] = {0x67};
  Mouse_ReadRegister(0x10, buffer, 1);
  Mouse_WriteRegister(0x10, 0x00);
  Mouse_ReadRegister(0x10, buffer, 1);*/
  //Mouse_Init();

  // RESET SPI (high low high)

  uint8_t buffer[1] = {0x67};

	/*Mouse_WriteRegister(0x3A, 0x5A);
    HAL_Delay(50); // Wait for chip reboot [cite: 521]

    // 2. Verify Product ID (0x00) -> Expect 0x47
    Mouse_ReadRegister(0x00, buffer, 1);
    if (buffer[0] != 0x47) {
        return 0; // Error: SPI Connection Failed
    }*/

  bool mouse_connected = Mouse_Init();


  bool imu_connected = IMU_Init();

  double gyroOffset[3];
  IMU_CalibrateGyro(gyroOffset);

  //double accelOffset[3];
  //IMU_CalibrateAccel(accelOffset);

  double accelReadings[3];
  IMU_ReadAccel(accelReadings);
  IMU_GenGravQuat(rotation_quat, accelReadings);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tickTime = DWT->CYCCNT;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  double gyroReadings[3];
	  IMU_ReadGyroRadPerSec(gyroReadings);

	  uint32_t tickTimeNew = DWT->CYCCNT;
	  uint32_t delta = tickTimeNew - tickTime;
	  double dt = (double)delta / (double)HAL_RCC_GetHCLKFreq();
	  tickTime = tickTimeNew;

	  gyroReadings[0] -= gyroOffset[0];
	  gyroReadings[1] -= gyroOffset[1];
	  gyroReadings[2] -= gyroOffset[2];

	  x_theta += gyroReadings[0] * dt;
	  y_theta += gyroReadings[1] * dt;
	  z_theta += gyroReadings[2] * dt;


	  IMU_GenInstQuat(gyroDeltaQuat, gyroReadings, dt);

	  double gyroMadQuat[4] = {0, gyroReadings[0]/2, gyroReadings[1]/2, gyroReadings[2]/2, gyroReadings[3]/2};
	  double rotationQuatRawPrime[4];
	  Quaternion_Multiply(rotationQuatRawPrime, rotation_quat, gyroMadQuat);
	  Quaternion_Normalize(rotationQuatRawPrime);

	  Quaternion_Multiply(rotation_quat, rotation_quat, gyroDeltaQuat);
	  Quaternion_Normalize(rotation_quat);

	  double accelReadings[3];
	  IMU_ReadAccel(accelReadings);
	  x_accel = accelReadings[0];
	  y_accel = accelReadings[1];
	  z_accel = accelReadings[2];

	  double quatGrav[4];
	  IMU_GenGravQuat(quatGrav, accelReadings);

	  double beta = 0.1; // Tuning parameter: higher = trust accel more, lower = trust gyro more

	  	double ax = accelReadings[0];
	  	double ay = accelReadings[1];
	  	double az = accelReadings[2];

	  	// Only apply correction if the accelerometer is valid (not in 0g freefall)
	  	if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

			// 1. Normalize accelerometer reading
			double recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Grab the current gyro-predicted quaternion
			double q0 = rotation_quat[0];
			double q1 = rotation_quat[1];
			double q2 = rotation_quat[2];
			double q3 = rotation_quat[3];

			// Pre-compute variables to save CPU cycles
			double _2q0 = 2.0 * q0; double _2q1 = 2.0 * q1;
			double _2q2 = 2.0 * q2; double _2q3 = 2.0 * q3;
			double _4q0 = 4.0 * q0; double _4q1 = 4.0 * q1; double _4q2 = 4.0 * q2;
			double _8q1 = 8.0 * q1; double _8q2 = 8.0 * q2;
			double q0q0 = q0 * q0; double q1q1 = q1 * q1;
			double q2q2 = q2 * q2; double q3q3 = q3 * q3;

			// 2. Calculate the Gradient (Nabla f)
			double s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			double s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			double s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			double s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

			// 3. Normalize the Gradient
			recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// 4. Subtract the scaled error gradient from our current quaternion
			rotation_quat[0] -= beta * s0 * dt;
			rotation_quat[1] -= beta * s1 * dt;
			rotation_quat[2] -= beta * s2 * dt;
			rotation_quat[3] -= beta * s3 * dt;

			// 5. Re-normalize to snap back to the 4D unit sphere
			Quaternion_Normalize(rotation_quat);
		}

	  //Quaternion_Multiply(rotation_quat, rotation_quat, quatGrav);


	double sinr_cosp = 2.0 * (rotation_quat[0] * rotation_quat[1] + rotation_quat[2] * rotation_quat[3]);
	double cosr_cosp = 1.0 - 2.0 * (rotation_quat[1] * rotation_quat[1] + rotation_quat[2] * rotation_quat[2]);
	double xAxisRad = atan2(sinr_cosp, cosr_cosp);

	// Calculate Rotation around Y-axis (Which is ROLL on your hardware)
	double sinp = 2.0 * (rotation_quat[0] * rotation_quat[2] - rotation_quat[3] * rotation_quat[1]);
	double yAxisRad;
	if (fabs(sinp) >= 1.0) {
		yAxisRad = copysign(M_PI / 2.0, sinp); // Lock to 90 degrees if out of bounds
	} else {
		yAxisRad = asin(sinp);
	}

	// Convert to degrees and assign correctly based on your mounting
	double pitchDeg = xAxisRad * (180.0 / M_PI);
	double rollDeg  = yAxisRad * (180.0 / M_PI);

	// Assign to your velocity/tilt tracking variables
	// (You'll need to map pitch and roll to whichever physical axis x_velo represents)
	x_velo = rollDeg;
	y_velo = pitchDeg;
	  //y_velo += x_theta * dt;
	  //z_velo += y_theta * dt;
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
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
