#pragma once

class PIDFF {
private:
    float kP;
    float kD;
    float kV;
    float kS;

    float deltaTime;

    float lastError;
public:
    PIDFF(float kP, float kD, float kV, float kS, float deltaTime);

    float calculate(float target, float current);

    void reset();
};
