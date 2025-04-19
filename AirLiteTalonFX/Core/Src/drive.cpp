/*
 * drive.c
 *
 *  Created on: Feb 5, 2025
 *      Author: mine215
 */

#include "drive.h"

Motor::Motor(GPIO_TypeDef *forwardPort, GPIO_TypeDef *backPort, uint16_t forwardPin, uint16_t backPin, volatile uint32_t *pwmCcr): forwardPort(forwardPort), backPort(backPort), forwardPin(forwardPin), backPin(backPin), pwmCcr(pwmCcr) {
	// idk what this init method is called but i'm doing it using c++ stuff!11!1
}

void Motor::setDriveDirection(DriveDirection direction) {
	GPIO_PinState forwardPinState;
	GPIO_PinState backwardPinState;

	switch (direction) {
	case FORWARD:
		forwardPinState = GPIO_PIN_SET;
		backwardPinState = GPIO_PIN_RESET;
		break;
	case BACKWARD:
		forwardPinState = GPIO_PIN_RESET;
		backwardPinState = GPIO_PIN_SET;
		break;
	case OFF:
		forwardPinState = GPIO_PIN_RESET;
		backwardPinState = GPIO_PIN_RESET;
		break;
	}

	HAL_GPIO_WritePin(forwardPort, forwardPin, forwardPinState);
	HAL_GPIO_WritePin(backPort, backPin, backwardPinState);
}

void Motor::setDriveDutyCycle(float dutyCycle) {
	// Clamp duty cycle to legal values
	if (dutyCycle > 1) {
		dutyCycle = 1;
	} else if (dutyCycle < 0) {
		dutyCycle = 0;
	}

	int ccr = PWM_ARR * dutyCycle;

	*pwmCcr = ccr;
}

/*void setDriveDutyCycle(DriveSide driveSide, float dutyCycle) {
	// Clamp duty cycle to legal values
	if (dutyCycle > 1) {
		dutyCycle = 1;
	} else if (dutyCycle < 0) {
		dutyCycle = 0;
	}

	int ccr = PWM_ARR * dutyCycle;

	switch (driveSide) {
	case LEFT:
		 TIM2->CCR1 = ccr;
	case RIGHT:
		 TIM1->CCR3 = ccr;
	}
}*/
