/*
 * motor.h
 *
 *  Created on: Apr 28, 2026
 *      Author: rezq
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "arm_math.h"

typedef enum {
	FORWARD,
	BACKWARD,
	OFF
} DriveDirection;

typedef struct {
	GPIO_TypeDef *forwardPort;
	GPIO_TypeDef *backwardPort;
	uint16_t forwardPin;
	uint16_t backwardPin;
	volatile uint32_t *pwmCcr;
	TIM_HandleTypeDef *tim;
	uint32_t timChannel;
	bool inverted;

	float32_t deadband;
	DriveDirection lastDriveDirection;
} Motor;

bool Motor_Init(Motor *motor, GPIO_TypeDef *forwardPort, GPIO_TypeDef *backwardPort, uint16_t forwardPin, uint16_t backwardPin, volatile uint32_t *pwmCcr, TIM_HandleTypeDef *tim, uint32_t timChannel, bool inverted);

void Motor_SetDirection(Motor *motor, DriveDirection direction); // TODO: I could make this return a bool but GPIO pin sets don't really fail... lol

void Motor_SetDutyCycle(Motor *motor, float32_t dutyCycle);

void Motor_Set(Motor *motor, float32_t dutyCycle);

#endif /* INC_MOTOR_H_ */
