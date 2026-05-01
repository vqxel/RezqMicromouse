/*
 * mouse.h
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */

#ifndef INC_MOUSE_H_
#define INC_MOUSE_H_

#include "pmw3389_srom.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
#include "main.h"
#include "arm_math.h"

typedef struct {
	uint8_t motion;
	uint8_t observation;
	uint8_t deltaXL;
	uint8_t deltaXH;
	uint8_t deltaYL;
	uint8_t deltaYH;
	uint8_t SQUAL;
	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;
	uint8_t shutterUpper;
	uint8_t shutterLower;
} MouseData;

#define MOUSE_CPI 5000.0f
#define MOUSE_OFFSET_INCHES 0.74212598f

void Mouse_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t *buffer);

void Mouse_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value);

bool Mouse_Init(SPI_HandleTypeDef *hspi);

void Mouse_MotionRead(SPI_HandleTypeDef *hspi, MouseData *mouseData);

void Mouse_GetVelocity(MouseData *data, float32_t dt, float32_t *v, float32_t *w);

#endif /* INC_MOUSE_H_ */
