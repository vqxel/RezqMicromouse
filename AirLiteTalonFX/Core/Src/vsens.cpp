/*
 * vsens.cpp
 *
 *  Created on: May 22, 2025
 *      Author: rezq
 */

#include <vsens.h>

VSens::VSens(ADC_HandleTypeDef *adc, uint32_t channel): ADCChannel(adc, channel), adc(adc) {

}

float VSens::getVolts() {
	//float vdda_voltage = getRaw() / 4096 * 3.3 * 3;
	_volts = ADC_VOLTS(getRaw());
	return _volts;
}

float VSens::getCompMult() {
	_compMult = NOM_VOLTAGE / getVolts();
	return _compMult;
}
