/*
 * adc.h
 *
 *  Created on: May 22, 2025
 *      Author: rezq
 */

#pragma once

#include "stm32f1xx_hal.h"
#include "constants.h"
#include "main.h"

class ADCChannel {
private:
	ADC_HandleTypeDef *_adc;
	ADC_ChannelConfTypeDef _config;
	uint32_t _channel;

	float _adcValue;
public:
	ADCChannel(ADC_HandleTypeDef *adc, uint32_t channel);

	void init();
	bool poll();
	float getRaw();
	bool enable();
};
