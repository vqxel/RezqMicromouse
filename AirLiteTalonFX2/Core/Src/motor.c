/*
 * motor.c
 *
 *  Created on: Apr 28, 2026
 *      Author: rezq
 */

#include "motor.h"

bool Motor_Init(Motor *motor, GPIO_TypeDef *forwardPort, GPIO_TypeDef *backwardPort, uint16_t forwardPin, uint16_t backwardPin, volatile uint32_t *pwmCcr, TIM_HandleTypeDef *tim, uint32_t timChannel, bool inverted) {
	motor->forwardPort = forwardPort;
	motor->backwardPort = backwardPort;
	motor->forwardPin = forwardPin;
	motor->backwardPin = backwardPin;
	motor->pwmCcr = pwmCcr;
	motor->tim = tim;
	motor->timChannel = timChannel;
	motor->inverted = inverted;

	motor->deadband = 0.01;

	Motor_SetDirection(motor, OFF);

	return HAL_TIM_PWM_Start(motor->tim, motor->timChannel) == HAL_OK;
}

void Motor_SetDirection(Motor *motor, DriveDirection direction) {
	GPIO_PinState forwardPinState;
	GPIO_PinState backwardPinState;

	switch (direction) {
	case FORWARD:
		forwardPinState = motor->inverted ? GPIO_PIN_RESET : GPIO_PIN_SET;
		backwardPinState = motor->inverted ? GPIO_PIN_SET : GPIO_PIN_RESET;
		break;
	case BACKWARD:
		forwardPinState = motor->inverted ? GPIO_PIN_SET : GPIO_PIN_RESET;
		backwardPinState = motor->inverted ? GPIO_PIN_RESET : GPIO_PIN_SET;
		break;
	case OFF:
		forwardPinState = GPIO_PIN_RESET;
		backwardPinState = GPIO_PIN_RESET;
		break;
	}

	HAL_GPIO_WritePin(motor->forwardPort, motor->forwardPin, forwardPinState);
	HAL_GPIO_WritePin(motor->backwardPort, motor->backwardPin, backwardPinState);

	motor->lastDriveDirection = direction;
}

void Motor_SetDutyCycle(Motor *motor, float32_t dutyCycle) {
	// Clamp duty cycle to legal values TODO: Why was it originally capped at 0.72...
	if (dutyCycle > 1) {
		dutyCycle = 1;
	} else if (dutyCycle < 0) {
		dutyCycle = 0;
	}

	int ccr = 1024 * dutyCycle;

	*(motor->pwmCcr) = ccr;
}

void Motor_Set(Motor *motor, float32_t dutyCycle) {
	float32_t absDutyCycle = fabsf(dutyCycle);

	DriveDirection extractedDirection;
	if (absDutyCycle < motor->deadband) {
		extractedDirection = OFF;
		dutyCycle = 0;
	} else if (dutyCycle > motor->deadband) {
		extractedDirection = FORWARD;
	} else {
		extractedDirection = BACKWARD;
	}

	if (extractedDirection != motor->lastDriveDirection) {
		Motor_SetDirection(motor, extractedDirection);
	}

	if (absDutyCycle > 1.0f) {
		dutyCycle = 1.0f;
	}

	Motor_SetDutyCycle(motor, absDutyCycle);
}
