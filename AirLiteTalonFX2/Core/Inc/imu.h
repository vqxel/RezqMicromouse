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
#include "main.h"
#include "quaternion.h"

void IMU_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value);

void IMU_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t *buffer, size_t len);

bool IMU_WriteAndValidateRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value);

bool IMU_TryWriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value, uint8_t maxAttempts);

void IMU_ReadGyroDegPerSec(SPI_HandleTypeDef *hspi, double *gyroReadings);

void IMU_ReadGyroRadPerSec(SPI_HandleTypeDef *hspi, double *gyroReadings);

void IMU_ApplyGyroOffset(double *gyroReadings, double *gyroOffset);

void IMU_GenInstQuat(double *quat, double *gyroReadings, double dt);

void IMU_CalibrateGyro(SPI_HandleTypeDef *hspi, double *gyroOffset);

void IMU_ReadAccel(SPI_HandleTypeDef *hspi, double *accelReadings);

void IMU_GenGravQuat(double *quat, double *accelReadings);

void IMU_GyroAccelMadgwickFilter(double beta, double *rotationQuat, double *gyroReadings, double *accelReadings, double dt);

bool IMU_Init(SPI_HandleTypeDef *hspi);

#endif /* INC_IMU_H_ */
