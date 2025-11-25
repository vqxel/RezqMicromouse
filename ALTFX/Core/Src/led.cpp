/*
 * drive.c
 *
 *  Created on: Feb 5, 2025
 *      Author: mine215
 */

#include "led.h"

LED::LED(GPIO_TypeDef *port, uint16_t pin): port(port), pin(pin) {
	// fancy c++
}

void LED::setState(LEDState state) {
	HAL_GPIO_WritePin(port, pin, state == LEDState::ON ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
