/*
 * dsens.h
 *
 *  Created on: Apr 30, 2026
 *      Author: rezq
 */

#ifndef INC_DSENS_H_
#define INC_DSENS_H_

#include "main.h"
#include "adcchannel.h"
#include "arm_math.h"
#include "stdbool.h"

typedef struct {
	ADCChannel adcChannel;
	GPIO_TypeDef *emitterPort;
	uint16_t emitterPin;

	float32_t dist;

	float32_t convSlope;
	float32_t convInter;
} DSens;

void DSens_Init(DSens *dsens, GPIO_TypeDef *emitterPort, uint16_t emitterPin, ADC_HandleTypeDef *adc, uint32_t channel, float32_t convSlope, float32_t convInter);
bool DSens_Update(DSens *dsens);
float32_t DSens_GetDist(DSens *dsens);

#endif /* INC_DSENS_H_ */
