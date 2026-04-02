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

void Mouse_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t *buffer, size_t len);

void Mouse_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addressByte, uint8_t value);

bool Mouse_Init(SPI_HandleTypeDef *hspi);

#endif /* INC_MOUSE_H_ */
