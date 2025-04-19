/*
 * drive.h
 *
 *  Created on: Jan 30, 2025
 *      Author: mine215
 */

#pragma once

#include "constants.h"
#include "stm32f1xx_hal.h"
#include "main.h"

typedef enum {
	FORWARD,
	BACKWARD,
	OFF
} DriveDirection;

typedef enum {
	LEFT,
	RIGHT
} DriveSide;

class Motor {
private:
	GPIO_TypeDef *forwardPort;
	GPIO_TypeDef *backPort;

	uint16_t forwardPin;
	uint16_t backPin;

	volatile uint32_t *pwmCcr;

public:
	Motor(GPIO_TypeDef *forwardPort, GPIO_TypeDef *backPort, uint16_t forwardPin, uint16_t backPin, volatile uint32_t *pwmCcr);

	void setDriveDirection(DriveDirection direction);

	void setDriveDutyCycle(float dutyCycle);
};

void setDriveDirection(DriveSide driveSide, DriveDirection direction);

void setDriveDutyCycle(DriveSide driveSide, float dutyCycle);
