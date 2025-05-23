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
#include "adcchannel.h"

class VSens: public ADCChannel {
private:
	ADC_HandleTypeDef *adc;

	float _volts;
	float _compMult;
public:
	VSens(ADC_HandleTypeDef *adc, uint32_t channel);

	float getVolts();

	float getCompMult();
};
