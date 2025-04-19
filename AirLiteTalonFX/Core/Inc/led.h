/*
 * led.h
 *
 *  Created on: Mar 5, 2025
 *      Author: mine215
 */

#pragma once

#include "constants.h"
#include "stm32f1xx_hal.h"
#include "main.h"

typedef enum {
	ON,
	OFF
} LEDState;

class LED {
private:
	GPIO_TypeDef *port;
	uint16_t pin;
public:
	LED(GPIO_TypeDef *port, uint16_t pin);

	void setState(LEDState state);
};
