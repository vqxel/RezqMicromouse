/*
 * drive.c
 *
 *  Created on: Feb 5, 2025
 *      Author: mine215
 */

#include "encoders.h"
#include <limits>

Encoder::Encoder(GPIO_TypeDef *pinAPort, GPIO_TypeDef *pinBPort, uint16_t pinA, uint16_t pinB): pinAPort(pinAPort), pinBPort(pinBPort), pinA(pinA), pinB(pinB), position(0), overflows(0) {
	pinAState = GPIO_PIN_RESET;
	pinBState = GPIO_PIN_RESET;
}

void Encoder::init() {
	  pinAState = HAL_GPIO_ReadPin(pinAPort, pinA);
	  pinBState = HAL_GPIO_ReadPin(pinBPort, pinB);
}

void Encoder::gpioCallback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == pinA || GPIO_Pin == pinB) {
		// Handle over/underflow
		if (position == ((1 << 31) - 1)) {
			overflows++;
		} else if (position == -((1 << 31) - 1)) {
			overflows--;
		}
	}
	// https://www.allaboutcircuits.com/uploads/articles/rotary-encoder-waveform-v2.jpg

	// TODO: Optimize this, I think this can be done in a SIGNIFICANTLY more efficient manner lmao

	// CCW - CW +
	if (GPIO_Pin == pinA) {
		// A rising/falling
		if (pinAState == GPIO_PIN_SET) {
			pinAState = GPIO_PIN_RESET;

			if (pinBState == GPIO_PIN_RESET) {
				// If A goes low and B is low CCW
				position--;
			} else if (pinBState == GPIO_PIN_SET) {
				// If A goes low and B is high CW
				position++;
			}
		} else {
			pinAState = GPIO_PIN_SET;

			if (pinBState == GPIO_PIN_RESET) {
				// If A goes high and B is low CW
				position++;
			} else if (pinBState == GPIO_PIN_SET) {
				// If A goes high and B is high CCW
				position--;
			}
		}
	} else if (GPIO_Pin == M1_ENC_B_Pin) {
		// B rising/falling
		if (pinBState == GPIO_PIN_SET) {
			pinBState = GPIO_PIN_RESET;

			if (pinAState == GPIO_PIN_RESET) {
				// If B goes low and A is low CW
				position++;
			} else if (pinAState == GPIO_PIN_SET) {
				// If B goes low and A is high CCW
				position--;
			}
		} else {
			pinBState = GPIO_PIN_SET;

			if (pinAState == GPIO_PIN_RESET) {
				// If B goes high and A is low CCW
				position--;
			} else if (pinAState == GPIO_PIN_SET) {
				// If B goes high and A is high CW
				position++;
			}
		}
	}
}



int32_t Encoder::getPosition() {
	return position;
}
