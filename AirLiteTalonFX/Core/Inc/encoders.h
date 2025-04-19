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
	GPIO_TypeDef *pinAPort;
	GPIO_TypeDef *pinBPort;

	uint16_t pinA;
	uint16_t pinB;

	GPIO_PinState pinAState;
	GPIO_PinState pinBState;

	int32_t position;

	int32_t overflows;

public:
	Encoder(GPIO_TypeDef *pinAPort, GPIO_TypeDef *pinBPort, uint16_t pinA, uint16_t pinB);

	void init();
	void gpioCallback(uint16_t GPIO_Pin);

	int32_t getPosition();
};
