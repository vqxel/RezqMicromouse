/*
 * quaternions.h
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "math.h"
#include "arm_math.h"

void Quaternion_Multiply(float32_t *out, const float32_t *q, const float32_t *p);

void Quaternion_Normalize(float32_t *q);

void Quaternion_To_Euler(const float32_t quat[4], float32_t euler[3]);

void Quaternion_To_Euler_Deg(const float32_t quat[4], float32_t euler[3]);

#endif /* INC_QUATERNION_H_ */
