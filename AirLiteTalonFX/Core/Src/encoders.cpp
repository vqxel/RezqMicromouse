/*
 * drive.c
 *
 *  Created on: Feb 5, 2025
 *      Author: mine215
 */

#include "encoders.h"

Encoder::Encoder(TIM_HandleTypeDef *htim, bool inverted, float conversionFactor): htim(htim), inverted(inverted), rawPos(0), pos(0), _conversionFactor(conversionFactor), lastPosMeters(0), lastTick(0), veloMeters(0) {
}

void Encoder::init() {
  HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL);
}

void Encoder::callback(TIM_HandleTypeDef *htimCall, uint32_t cur) {
  if (htimCall->Instance != htim->Instance) return;

  uint16_t current_raw = static_cast<uint16_t>(cur);

	int16_t delta = static_cast<int16_t>(current_raw - rawPos);

	pos += (inverted ? 1 : -1) * delta;
	rawPos = current_raw;
}

int64_t Encoder::getPosition() {
	return pos;
}

float Encoder::getLastPositionMeters() {
	return lastPosMeters;
}

float Encoder::getPositionMeters() {
	posMeters = pos * _conversionFactor;
	return posMeters;
}

float Encoder::getVeloMeters() {
	return veloMeters;
}

void Encoder::tick() {
	getPositionMeters();

	uint32_t tick = HAL_GetTick();

	float tempVeloMeters = (posMeters - lastPosMeters) / (tick - lastTick) * 1000;

	pmHistory.push(tempVeloMeters);

	veloMeters = (pmHistory.sum() / pmHistory.size());

	lastTick = tick;
	lastPosMeters = posMeters;
	/*pmHistory.push(posMeters);
	tickHistory.push(HAL_GetTick());
	veloMeters = pmHistory.delta() / tickHistory.delta() * 1000;*/
}
