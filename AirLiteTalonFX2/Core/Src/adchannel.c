/*
 * adchannel.c
 *
 *  Created on: Apr 30, 2026
 *      Author: rezq
 */

#include "adcchannel.h"

void ADCChannel_Init(ADCChannel *adcChannel, ADC_HandleTypeDef *adc, uint32_t channel) {
	adcChannel->adc = adc;
	adcChannel->channel = channel;
	adcChannel->config.Channel = channel;
	adcChannel->config.Rank = 1;
	adcChannel->config.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	adcChannel->config.Offset = 0;
}

bool ADCChannel_Config(ADCChannel *adcChannel) {
	if (HAL_ADC_ConfigChannel(adcChannel->adc, &adcChannel->config) != HAL_OK) {
		return false;
	}
	return true;
}

bool ADCChannel_Poll(ADCChannel *adcChannel) {
	if (!ADCChannel_Config(adcChannel)) return false;

	HAL_ADC_Start(adcChannel->adc);

	if (HAL_ADC_PollForConversion(adcChannel->adc, 100) == HAL_OK) {
	  adcChannel->adcValue = HAL_ADC_GetValue(adcChannel->adc);
	  HAL_ADC_Stop(adcChannel->adc);
	  return true;
	}
	return false;
}
