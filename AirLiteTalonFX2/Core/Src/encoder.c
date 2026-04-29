/*
 * encoder.c
 *
 *  Created on: Apr 28, 2026
 *      Author: rezq
 */


#include "encoder.h"

bool Encoder_Init(Encoder *encoder, TIM_HandleTypeDef *htim, bool inverted, float conversionFactor) {
	return HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL) == HAL_OK;
}

void Encoder_Callback(Encoder *encoder, TIM_HandleTypeDef *htimCall, uint32_t reading) {
	if (htimCall->Instance != encoder->htim->Instance) return;

	uint16_t readingCompressed = (uint16_t) reading;

	int16_t delta = readingCompressed - encoder->rawPos;

	encoder->pos += (encoder->inverted ? -1 : 1) * delta;
	encoder->rawPos = readingCompressed;
}

void Encoder_Tick(Encoder *encoder, float32_t dt) {
	// TODO: Consider adding filtering at this level altho kalman filter should account for it by default
	encoder->rawVeloMeters = encoder->conversionFactor * (encoder->pos - encoder->lastPos) / dt;

	encoder->lastPos = encoder->pos;
}
