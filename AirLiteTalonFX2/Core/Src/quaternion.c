#include "quaternion.h"

void Quaternion_Multiply(double *out, const double *q, const double *p) {
	// q * p NOT p * q
    double w = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
    double x = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
    double y = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1];
    double z = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0];

    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

void Quaternion_Normalize(double *q) {
    double mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (mag > 1e-10) {
        q[0] /= mag; q[1] /= mag; q[2] /= mag; q[3] /= mag;
    }
}

void Quaternion_To_Euler(const double quat[4], double euler[3]) {
    // Extract for readability (assuming quat[0] is w)
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];

    // 1. X-axis Rotation (Roll)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);

    // 2. Y-axis Rotation (Pitch)
    double sinp = 2.0 * (w * y - z * x);
    // Protect against floating point errors pushing sinp slightly outside [-1, 1]
    if (fabs(sinp) >= 1.0) {
        euler[1] = copysign(M_PI / 2.0, sinp); // Lock to +/- 90 degrees
    } else {
        euler[1] = asin(sinp);
    }

    // 3. Z-axis Rotation (Yaw)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[2] = atan2(siny_cosp, cosy_cosp);
}
