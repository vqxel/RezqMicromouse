/*
 * dsens.c
 *
 *  Created on: Apr 30, 2026
 *      Author: rezq
 */

#include "dsens.h"
#include "math.h"

void DSens_Init(DSens *dsens, GPIO_TypeDef *emitterPort, uint16_t emitterPin, ADC_HandleTypeDef *adc, uint32_t channel, float32_t convSlope, float32_t convInter) {
	ADCChannel_Init(&dsens->adcChannel, adc, channel);
	dsens->emitterPort = emitterPort;
	dsens->emitterPin = emitterPin;
	dsens->convSlope = convSlope;
	dsens->convInter = convInter;
}

bool DSens_Update(DSens *dsens) {
	HAL_GPIO_WritePin(dsens->emitterPort, dsens->emitterPin, GPIO_PIN_SET);
	// We might need a small delay here for the LED to turn on and the phototransistor to respond
	// The original had 5ms, which is quite long for a micromouse.
	HAL_Delay(1); // Reduced to 1ms for now, can be adjusted or moved to delay_us

	bool pRet = ADCChannel_Poll(&dsens->adcChannel);

	HAL_GPIO_WritePin(dsens->emitterPort, dsens->emitterPin, GPIO_PIN_RESET);

	return pRet;
}

float32_t DSens_GetDist(DSens *dsens) {
	// dist = convSlope * pow(raw, convInter)
	dsens->dist = dsens->convSlope * powf(dsens->adcChannel.adcValue, dsens->convInter);
	return dsens->dist;
}
