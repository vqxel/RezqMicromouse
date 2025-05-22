/*
 * vsens.cpp
 *
 *  Created on: May 22, 2025
 *      Author: rezq
 */

#include <vsens.h>

void VSens::VSens(ADC_HandleTypeDef *adc): adc(adc) {

}

void VSens::init() {
	HAL_ADCEx_Calibration_Start(adc);
}

bool VSens::poll() {
	HAL_ADC_Start(adc);

	if (HAL_ADC_PollForConversion(adc, ADC_TIMEOUT_MS) == HAL_OK) {
	  _adcValue = HAL_ADC_GetValue(adc);
	  HAL_ADC_Stop(adc);
	  return true;
	}
	return false;
}

float VSens::getRaw() {
	return _adcValue;
}

float VSens::getVolts() {
	_volts = VOLT_NORM_SLOPE * _adcValue + VOLT_NORM_INT;
	return _volts;
}

float VSens::getCompMult() {
	_compMult = this.getVolts() / NOM_VOLTAGE;
	return _compMult;
}
