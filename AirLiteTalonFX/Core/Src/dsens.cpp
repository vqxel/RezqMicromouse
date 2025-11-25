/*
 * DSens.cpp
 *
 *  Created on: May 22, 2025
 *      Author: rezq
 */

#include <dsens.h>

DSens::DSens(GPIO_TypeDef *emitterPort, uint16_t emitterPin, ADC_HandleTypeDef *adc, uint32_t channel, float convSlope, float convInter): ADCChannel(adc, channel), _emitterPort(emitterPort), _emitterPin(emitterPin), _adc(adc), _convSlope(convSlope), _convInter(convInter) {

}

bool DSens::updateSens() {
	HAL_GPIO_WritePin(_emitterPort, _emitterPin, GPIO_PIN_SET);
	HAL_Delay(5); // TODO: major tick rate limiter lol

	bool pRet = poll();

	HAL_GPIO_WritePin(_emitterPort, _emitterPin, GPIO_PIN_RESET);

	if (!pRet) return false;
	return true;
}


float DSens::getDist() {
	_dist = _convSlope * pow(getRaw(), _convInter);
	return _dist;
}
