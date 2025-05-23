#pragma once

#include "stm32f1xx_hal.h"

typedef enum {
	PID_RUNNING,
	PID_STOPPED
} PID_LOOP_STATE;

class PIDFF {
private:
    float kP;
    float kD;
    float kV;
    float kS;

    float lastError;

    uint32_t _lastTime;
    uint32_t _currentTime;

    PID_LOOP_STATE _state;
public:
    PIDFF(float kP, float kD, float kV, float kS);

    float calculate(float target, float current);

    void stop();
};
