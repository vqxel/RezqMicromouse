/*
 * imu.h
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "math.h"
#include "arm_math.h"
#include "main.h"
#include "quaternion.h"

void IMU_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value);

void IMU_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t *buffer, size_t len);

bool IMU_WriteAndValidateRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value);

bool IMU_TryWriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value, uint8_t maxAttempts);

void IMU_ReadGyroDegPerSec(SPI_HandleTypeDef *hspi, float32_t *gyroReadings);

void IMU_ReadGyroRadPerSec(SPI_HandleTypeDef *hspi, float32_t *gyroReadings);

void IMU_ApplyGyroOffset(float32_t *gyroReadings, float32_t *gyroOffset);

void IMU_GenInstQuat(float32_t *quat, float32_t *gyroReadings, float32_t dt);

void IMU_CalibrateGyro(SPI_HandleTypeDef *hspi, float32_t *gyroOffset);

void IMU_ReadAccel(SPI_HandleTypeDef *hspi, float32_t *accelReadings);

void IMU_GenGravQuat(float32_t *quat, float32_t *accelReadings);

void IMU_GyroAccelMadgwickFilter(float32_t beta, float32_t *rotationQuat, float32_t *gyroReadings, float32_t *accelReadings, float32_t dt);

bool IMU_Init(SPI_HandleTypeDef *hspi);

#endif /* INC_IMU_H_ */
