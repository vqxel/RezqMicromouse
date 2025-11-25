/*
 * drive.h
 *
 *  Created on: Jan 30, 2025
 *      Author: mine215
 */

#pragma once

#include "constants.h"
#include "stm32f1xx_hal.h"
#include "main.h"

class Encoder {
private:
	TIM_HandleTypeDef *htim;

	bool inverted;

	uint16_t rawPos;
	int64_t pos;
public:
	Encoder(TIM_HandleTypeDef *htim, bool inverted);

	void init();
	void callback(TIM_HandleTypeDef *htimCall, uint32_t cur);

	int64_t getPosition();
};
