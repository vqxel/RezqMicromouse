/*
 * adcchannel.cpp
 *
 *  Created on: May 22, 2025
 *      Author: rezq
 */

#include <adcchannel.h>

ADCChannel::ADCChannel(ADC_HandleTypeDef *adc, uint32_t channel): _adc(adc), _channel(channel) {
	_config = {0};
	_config.Channel = channel;
	_config.Rank = ADC_REGULAR_RANK_1;
	_config.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
}

bool ADCChannel::enable() {
	if (HAL_ADC_ConfigChannel(_adc, &_config) != HAL_OK) {
		Error_Handler();
		return false;
	}
	return true;
}

void ADCChannel::init() {
	enable();
	HAL_ADC_Stop(_adc);
	HAL_Delay(10);
	HAL_ADCEx_Calibration_Start(_adc);
	HAL_Delay(10);
	HAL_ADC_Stop(_adc);
}

bool ADCChannel::poll() {
	enable();
	HAL_ADC_Start(_adc);

	if (HAL_ADC_PollForConversion(_adc, ADC_TIMEOUT_MS) == HAL_OK) {
	  _adcValue = HAL_ADC_GetValue(_adc);
	  HAL_ADC_Stop(_adc);
	  return true;
	}
	return false;
}

float ADCChannel::getRaw() {
	return _adcValue;
}
