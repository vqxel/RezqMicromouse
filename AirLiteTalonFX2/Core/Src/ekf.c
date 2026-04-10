/*
 * imu.c
 *
 *  Created on: Apr 9, 2026
 *      Author: rezq
 */
#include "ekf.h"

void EKF_Test() {
	float32_t A_data[6] = {
	    1.0f, 2.0f, 3.0f,  // Row 0
	    4.0f, 5.0f, 6.0f   // Row 1
	};

	// A 3x2 matrix requires 6 floats
	float32_t B_data[6] = {
	    1.0f, 2.0f,
	    3.0f, 4.0f,
	    5.0f, 6.0f
	};

	// We need a place to store the 2x2 result (4 floats)
	float32_t C_data[4] = {0.0f};

	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 C;

	// arm_mat_init_f32(instance, rows, cols, data_pointer)
	arm_mat_init_f32(&A, 2, 3, A_data);
	arm_mat_init_f32(&B, 3, 2, B_data);
	arm_mat_init_f32(&C, 2, 2, C_data);

	arm_status status;

	// C = A * B
	status = arm_mat_mult_f32(&A, &B, &C);

	if (status == ARM_MATH_SUCCESS) {
	    // C_data now contains the valid result
	} else if (status == ARM_MATH_SIZE_MISMATCH) {
	    // You tried to multiply a 2x3 by a 4x2, and the library caught it.
	}
}
