/*
 * encoder.h
 *
 *  Created on: Apr 28, 2026
 *      Author: rezq
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "arm_math.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	bool inverted;
	float32_t conversionFactor;

	uint16_t rawPos;

	int64_t lastPos;
	int64_t pos;

	float32_t rawVeloMeters;
} Encoder;

bool Encoder_Init(Encoder *encoder, TIM_HandleTypeDef *htim, bool inverted, float conversionFactor);

void Encoder_Callback(Encoder *encoder, TIM_HandleTypeDef *htimCall, uint32_t reading);

void Encoder_Tick(Encoder *encoder, float32_t dt);

#endif /* INC_ENCODER_H_ */
