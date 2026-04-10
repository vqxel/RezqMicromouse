/*
 * imu.c
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */
#include "imu.h"

void IMU_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value) {
	uint8_t txData[2] = {addressByte & 0x7F, value};

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, txData, 2, 100);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void IMU_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t *buffer, size_t len) {
	len++;

	uint8_t txData[len];
	uint8_t rxData[len];

	memset(txData, 0, len);
	txData[0] = addressByte | 0x80;

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, txData, rxData, len, 100);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

	memcpy(buffer, &rxData[1], len--);
}

bool IMU_WriteAndValidateRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value) {
	uint8_t buffer[1] = {0x00};
	IMU_WriteRegister(hspi, addressByte, value);
	IMU_ReadRegister(hspi, addressByte, buffer, 1);

	return buffer[0] == value;
}

bool IMU_TryWriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value, uint8_t maxAttempts) {
	for (int attempts = 0; attempts < maxAttempts; attempts++) {
		bool success = IMU_WriteAndValidateRegister(hspi, addressByte, value);
		if (success) {
			return true;
		}
	}

	return false;
}

void IMU_ReadGyroDegPerSec(SPI_HandleTypeDef *hspi, float32_t *gyroReadings) {
	uint8_t gyroBuffer[6];
	IMU_ReadRegister(hspi, 0x06, gyroBuffer, 6);

	// Change from uint16_t to int16_t
	int16_t x_raw = (int16_t)(((uint16_t)gyroBuffer[1] << 8) | gyroBuffer[0]);
	int16_t y_raw = (int16_t)(((uint16_t)gyroBuffer[3] << 8) | gyroBuffer[2]);
	int16_t z_raw = (int16_t)(((uint16_t)gyroBuffer[5] << 8) | gyroBuffer[4]);

	gyroReadings[0] = (float32_t)x_raw / -32.8; // Pitch (pitch down/ccw from left is +) NOTE: Gyro is mounted backwards
	gyroReadings[1] = (float32_t)y_raw / -32.8; // Roll (roll ccw from back is +) NOTE: Gyro is mounted backwards
	gyroReadings[2] = (float32_t)z_raw / 32.8; // Yaw (ccw + from top)
}

void IMU_ReadGyroRadPerSec(SPI_HandleTypeDef *hspi, float32_t *gyroReadings) {
	IMU_ReadGyroDegPerSec(hspi, gyroReadings);

	gyroReadings[0] = gyroReadings[0] / 180 * M_PI;
	gyroReadings[1] = gyroReadings[1] / 180 * M_PI;
	gyroReadings[2] = gyroReadings[2] / 180 * M_PI;
}

void IMU_ApplyGyroOffset(float32_t *gyroReadings, float32_t *gyroOffset) {
	  gyroReadings[0] -= gyroOffset[0];
	  gyroReadings[1] -= gyroOffset[1];
	  gyroReadings[2] -= gyroOffset[2];
}

void IMU_GenInstQuat(float32_t *quat, float32_t *gyroReadings, float32_t dt) {
	float32_t vecMagSq = gyroReadings[0]*gyroReadings[0] + gyroReadings[1]*gyroReadings[1] + gyroReadings[2]*gyroReadings[2];

	if (vecMagSq < 1e-10) {
	  // No rotation: Identity quaternion
	  quat[0] = 1.0; quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;
	} else {
	  float32_t vecMag = sqrtf(vecMagSq);
	  float32_t quatTheta = dt * vecMag;
	  // TODO: Consider small angle approx for sin and cos
	  float32_t sinD2 = arm_sin_f32(quatTheta / 2.0);

	  quat[0] = arm_cos_f32(quatTheta / 2.0);
	  // Note: (gyroReadings[i] / vecMag) * sinD2
	  float32_t scale = sinD2 / vecMag;
	  quat[1] = gyroReadings[0] * scale;
	  quat[2] = gyroReadings[1] * scale;
	  quat[3] = gyroReadings[2] * scale;
	}
}

void IMU_CalibrateGyro(SPI_HandleTypeDef *hspi, float32_t *gyroOffset) {
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

	const uint32_t samples = 1000;
	float32_t x_sum = 0, y_sum = 0, z_sum = 0;

	float32_t gyroReadings[3];
	for(int i = 0; i < samples; i++) {
		IMU_ReadGyroRadPerSec(hspi, gyroReadings);
		x_sum += gyroReadings[0];
		y_sum += gyroReadings[1];
		z_sum += gyroReadings[2];
		HAL_Delay(1);
	}

	gyroOffset[0] = x_sum / (float32_t) samples;
	gyroOffset[1] = y_sum / (float32_t) samples;
	gyroOffset[2] = z_sum / (float32_t) samples;

	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}

void IMU_ReadAccel(SPI_HandleTypeDef *hspi, float32_t *accelReadings) {
	uint8_t accelBuffer[6];
	IMU_ReadRegister(hspi, 0x00, accelBuffer, 6);

	// Change from uint16_t to int16_t
	int16_t x_raw = (int16_t)(((uint16_t)accelBuffer[1] << 8) | accelBuffer[0]);
	int16_t y_raw = (int16_t)(((uint16_t)accelBuffer[3] << 8) | accelBuffer[2]);
	int16_t z_raw = (int16_t)(((uint16_t)accelBuffer[5] << 8) | accelBuffer[4]);

	accelReadings[0] = (float32_t)x_raw / -8192.0 * 9.80665; // + x is right NOTE: Inverted from physical mounting
	accelReadings[1] = (float32_t)y_raw / -8192.0 * 9.80665; // + y is forwards NOTE: Inverted from physical mounting
	accelReadings[2] = (float32_t)z_raw / 8192.0 * 9.80665; // + z is up
}

void IMU_GenGravQuat(float32_t *quat, float32_t *accelReadings) {
	float32_t vecMagSq = accelReadings[0]*accelReadings[0] + accelReadings[1]*accelReadings[1] + accelReadings[2]*accelReadings[2];

	if (vecMagSq < 1e-10) {
	  // No rotation: Identity quaternion
	  quat[0] = 1.0; quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;
	} else {
	  float32_t vecMag = sqrtf(vecMagSq);

	  float32_t quatTheta = arm_cos_f32(accelReadings[2] / vecMag);
	  float32_t rotationAxis[3] = {accelReadings[1], -accelReadings[0], 0};
	  float32_t rotationAxisMag = sqrtf(rotationAxis[0]*rotationAxis[0]+rotationAxis[1]*rotationAxis[1]+rotationAxis[2]*rotationAxis[2]);

	  if (rotationAxisMag < 1e-10) {
		  // Already vertical, no rotation needed
		  quat[0] = 1.0; quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;
	  } else {
		  rotationAxis[0] /= rotationAxisMag;
		  rotationAxis[1] /= rotationAxisMag;
		  rotationAxis[2] /= rotationAxisMag;

		  // TODO: Consider small angle approx for sin and cos
		  float32_t sinD2 = arm_sin_f32(quatTheta / 2.0);
		  quat[0] = arm_cos_f32(quatTheta / 2.0);
		  quat[1] = rotationAxis[0] * sinD2;
		  quat[2] = rotationAxis[1] * sinD2;
		  quat[3] = 0; //rotationAxis[2] * sinD2; -> 0
	  }
	}
}

void IMU_GyroAccelMadgwickFilter(float32_t beta, float32_t *rotationQuat, float32_t *gyroReadings, float32_t *accelReadings, float32_t dt) {
	// Beta Tuning parameter: higher = trust accel more, lower = trust gyro more

	float32_t gyroDeltaQuat[4];
	IMU_GenInstQuat(gyroDeltaQuat, gyroReadings, dt);

	Quaternion_Multiply(rotationQuat, rotationQuat, gyroDeltaQuat);
	Quaternion_Normalize(rotationQuat);

	float32_t ax = accelReadings[0];
	float32_t ay = accelReadings[1];
	float32_t az = accelReadings[2];

	// Only apply correction if the accelerometer is valid (not in 0g freefall)
	if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

		// 1. Normalize accelerometer reading
		float32_t recipNorm = 1.0 / sqrtf(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Grab the current gyro-predicted quaternion
		float32_t q0 = rotationQuat[0];
		float32_t q1 = rotationQuat[1];
		float32_t q2 = rotationQuat[2];
		float32_t q3 = rotationQuat[3];

		// Pre-compute variables to save CPU cycles
		float32_t _2q0 = 2.0 * q0; float32_t _2q1 = 2.0 * q1;
		float32_t _2q2 = 2.0 * q2; float32_t _2q3 = 2.0 * q3;
		float32_t _4q0 = 4.0 * q0; float32_t _4q1 = 4.0 * q1; float32_t _4q2 = 4.0 * q2;
		float32_t _8q1 = 8.0 * q1; float32_t _8q2 = 8.0 * q2;
		float32_t q0q0 = q0 * q0; float32_t q1q1 = q1 * q1;
		float32_t q2q2 = q2 * q2; float32_t q3q3 = q3 * q3;

		// 2. Calculate the Gradient (Nabla f)
		float32_t s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		float32_t s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		float32_t s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		float32_t s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

		// 3. Normalize the Gradient
		recipNorm = 1.0 / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// 4. Subtract the scaled error gradient from our current quaternion
		rotationQuat[0] -= beta * s0 * dt;
		rotationQuat[1] -= beta * s1 * dt;
		rotationQuat[2] -= beta * s2 * dt;
		rotationQuat[3] -= beta * s3 * dt;

		// 5. Re-normalize to snap back to the 4D unit sphere
		Quaternion_Normalize(rotationQuat);
	}
}

bool IMU_Init(SPI_HandleTypeDef *hspi) {
	bool imu_connected = false;
	uint8_t whoAmIBuf[1] = {0x00};
	for (int attempts = 0; attempts < 5; attempts++) {
	  IMU_ReadRegister(hspi, 0x72, whoAmIBuf, 1);
	  if (whoAmIBuf[0] == 0xe5) {
		  imu_connected = true;
		  break;
	  }
	}

	bool imu_initialized = false;
	if (imu_connected) {
	  bool imu_init_failed = false;

	  // Set to low noise mode
	  imu_init_failed |= !IMU_TryWriteRegister(hspi, 0x10, 0x0F, 5);

	  // Set to +- 4g accel res
	  imu_init_failed |= !IMU_TryWriteRegister(hspi, 0x1B, 0b0110110, 5);

	  // Set to 1000 dps gyro res
	  imu_init_failed |= !IMU_TryWriteRegister(hspi, 0x1C, 0b00100110, 5);

	  imu_initialized = !imu_init_failed;
	}

	return imu_initialized;
}
