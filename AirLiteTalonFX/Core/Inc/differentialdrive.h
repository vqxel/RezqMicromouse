#pragma once

class DifferentialDrive {
public:
	float _rad;

	DifferentialDrive(float rad);

	typedef struct {
		float lv;
		float rv;
	} WheelSpeeds;

	WheelSpeeds drive(float velo, float angularVelo);
};
