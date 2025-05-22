/*
 * dsens.h
 *
 *  Created on: May 22, 2025
 *      Author: rezq
 */

#pragma once

#include "main.h"
#include "constants.h"
#include "stm32f1xx_hal.h"

class DSens {
private:
	GPIO_TypeDef *emitterPort;
	uint16_t emitterPin;
	GPIO_TypeDef *recPort;
	uint16_t recPin;

};
