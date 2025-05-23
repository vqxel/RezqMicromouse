#include <PIDFF.h>

PIDFF::PIDFF(float kP, float kD, float kV, float kS): kP(kP), kD(kD), kV(kV), kS(kS), _lastTime(0), _currentTime(0), _state(PID_STOPPED) {
}

float PIDFF::calculate(float target, float current) {
  float error = target - current;

  if (_state == PID_STOPPED) {
	  lastError = error;
	  _state = PID_RUNNING;
  }

  _currentTime = HAL_GetTick();
  float deltaTime = _currentTime - _lastTime;

  float p = kP * error;
  // even if DT gets huge cuz lt is 0, that'll only happen on first run and in that case e - le == 0
  float d = kD * ((error - lastError) / deltaTime);
  float v = kV * target;
  float s = kS;

  lastError = error;

  _lastTime = _currentTime;

  return p + d + v + s;
}

void PIDFF::stop() {
	_state = PID_STOPPED;
}
