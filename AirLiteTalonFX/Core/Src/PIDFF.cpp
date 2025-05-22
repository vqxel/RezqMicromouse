#include <PIDFF.h>

PIDFF::PIDFF(float kP, float kD, float kV, float kS, float deltaTime): kP(kP), kD(kD), kV(kV), kS(kS), deltaTime(deltaTime) {
}

float PIDFF::calculate(float target, float current) {
  float error = target - current;

  float p = kP * error;
  float d = kD * ((error - lastError) / deltaTime);
  float v = kV * target;
  float s = kS;

  lastError = error;

  return p + d + v + s;
}

void PIDFF::reset() {
	lastError = 0;
}
