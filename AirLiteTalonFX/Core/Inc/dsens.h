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
#include "adcchannel.h"
#include "math.h"

class DSens: public ADCChannel {
private:
	GPIO_TypeDef *_emitterPort;
	uint16_t _emitterPin;

	ADC_HandleTypeDef *_adc;

	float _dist;

	float _convSlope;
	float _convInter;

public:
	DSens(GPIO_TypeDef *emitterPort, uint16_t emitterPin, ADC_HandleTypeDef *adc, uint32_t channel, float convSlope, float convInter);

	bool updateSens();

	float getDist();
};
