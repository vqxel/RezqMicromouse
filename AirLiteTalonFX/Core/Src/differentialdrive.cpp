#include "differentialdrive.h"

DifferentialDrive::DifferentialDrive(float rad): _rad(rad) {

}

DifferentialDrive::WheelSpeeds DifferentialDrive::drive(float velocity, float angularVelocity) {
	return {velocity - angularVelocity * _rad, velocity + angularVelocity * _rad};
}
