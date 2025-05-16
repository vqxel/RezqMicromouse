/*
 * drive.c
 *
 *  Created on: Feb 5, 2025
 *      Author: mine215
 */

#include "encoders.h"
#include <limits>

Encoder::Encoder(TIM_HandleTypeDef *htim, bool inverted): htim(htim), inverted(inverted), rawPos(0), pos(0) {
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
