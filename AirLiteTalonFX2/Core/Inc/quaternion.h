/*
 * quaternions.h
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "math.h"

void Quaternion_Multiply(double *out, const double *q, const double *p);

void Quaternion_Normalize(double *q);

void Quaternion_To_Euler(const double quat[4], double euler[3]);

#endif /* INC_QUATERNION_H_ */
