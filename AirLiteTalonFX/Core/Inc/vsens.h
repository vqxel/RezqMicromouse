/*
 * vsens.h
 *
 *  Created on: May 22, 2025
 *      Author: rezq
 */

#pragma once

#include "constants.h"
#include "stm32f1xx_hal.h"
#include "main.h"

class VSens {
private:
	ADC_HandleTypeDef *adc;

	float _adcValue;
	float _volts;
	float _compMult;
public:
	VSens(ADC_HandleTypeDef *adc);

	void init();
	bool poll();
	float getRaw();
	float getVolts();

	float getCompMult();
};
