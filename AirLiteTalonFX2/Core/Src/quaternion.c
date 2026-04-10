#include "quaternion.h"

void Quaternion_Multiply(float32_t *out, const float32_t *q, const float32_t *p) {
	// q * p NOT p * q
    float32_t w = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
    float32_t x = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
    float32_t y = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1];
    float32_t z = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0];

    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

void Quaternion_Normalize(float32_t *q) {
    float32_t mag = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (mag > 1e-10) {
        q[0] /= mag; q[1] /= mag; q[2] /= mag; q[3] /= mag;
    }
}

void Quaternion_To_Euler(const float32_t quat[4], float32_t euler[3]) {
    // Extract for readability (assuming quat[0] is w)
    float32_t w = quat[0];
    float32_t x = quat[1];
    float32_t y = quat[2];
    float32_t z = quat[3];

    // 1. X-axis Rotation (Roll)
    float32_t sinr_cosp = 2.0 * (w * x + y * z);
    float32_t cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[0] = atan2f(sinr_cosp, cosr_cosp);

    // 2. Y-axis Rotation (Pitch)
    float32_t sinp = 2.0 * (w * y - z * x);
    // Protect against floating point errors pushing sinp slightly outside [-1, 1]
    if (fabsf(sinp) >= 1.0) {
        euler[1] = copysignf(M_PI / 2.0, sinp); // Lock to +/- 90 degrees
    } else {
        euler[1] = asinf(sinp);
    }

    // 3. Z-axis Rotation (Yaw)
    float32_t siny_cosp = 2.0 * (w * z + x * y);
    float32_t cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[2] = atan2f(siny_cosp, cosy_cosp);
}

void Quaternion_To_Euler_Deg(const float32_t quat[4], float32_t euler[3]) {
	Quaternion_To_Euler(quat, euler);

	euler[0] = euler[0] * (180.0 / M_PI);
	euler[1] = euler[1] * (180.0 / M_PI);
	euler[2] = euler[2] * (180.0 / M_PI);
}
