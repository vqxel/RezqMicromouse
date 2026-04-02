/*
 * imu.h
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "imu.h"

void IMU_WriteRegister(uint8_t addressByte, uint8_t value);

void IMU_ReadRegister(uint8_t addressByte, uint8_t *buffer, size_t len);

bool IMU_WriteAndValidateRegister(uint8_t addressByte, uint8_t value);

bool IMU_TryWriteRegister(uint8_t addressByte, uint8_t value, uint8_t maxAttempts);

void IMU_ReadGyroDegPerSec(double *gyroReadings);

void IMU_ReadGyroRadPerSec(double *gyroReadings);

void IMU_ApplyGyroOffset(double *gyroReadings, double *gyroOffset);

void IMU_GenInstQuat(double *quat, double *gyroReadings, double dt);

void IMU_CalibrateGyro(double *gyroOffset);

void IMU_ReadAccel(double *accelReadings);

void IMU_GenGravQuat(double *quat, double *accelReadings);

void IMU_GyroAccelMadgwickFilter(double beta, double *rotationQuat, double *gyroReadings, double *accelReadings, double dt);

bool IMU_Init();

#endif /* INC_IMU_H_ */
