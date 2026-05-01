/*
 * adcchannel.h
 *
 *  Created on: Apr 30, 2026
 *      Author: rezq
 */

#ifndef INC_ADCCHANNEL_H_
#define INC_ADCCHANNEL_H_

#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "main.h"
#include "stdbool.h"

typedef struct {
	ADC_HandleTypeDef *adc;
	uint32_t channel;
	ADC_ChannelConfTypeDef config;

	float32_t adcValue;
} ADCChannel;


void ADCChannel_Init(ADCChannel *adcChannel, ADC_HandleTypeDef *adc, uint32_t channel);
bool ADCChannel_Poll(ADCChannel *adcChannel);
bool ADCChannel_Config(ADCChannel *adcChannel);


#endif /* INC_ADCCHANNEL_H_ */
