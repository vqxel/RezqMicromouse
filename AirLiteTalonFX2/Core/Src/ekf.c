/*
 * imu.c
 *
 *  Created on: Apr 9, 2026
 *      Author: rezq
 */
#include "ekf.h"

uint16_t EKF_CalculateIndex(uint16_t cols, uint16_t row, uint16_t col) {
	return (row * cols) + col;
}

void EKF_Init(EKF *ekf, float32_t wheelbase) {
    // 1. Clear all memory to 0.0f to prevent garbage data from causing NaN math faults.
    memset(ekf, 0.0f, sizeof(EKF));

    // =========================================================================
    // 2. Populate Static Matrices
    // =========================================================================

    // --- Setup State to Measurement Matrix (H) ---
    //  l_velo = v - w * d * 0.5
    //  r_velo = v + w * d * 0.5
    //  theta = theta
    //  mouse_v = v
    //  mouse_w = w
    ekf->state_to_meas_data[EKF_CalculateIndex(5, 0, 3)] = 1.0f;
    ekf->state_to_meas_data[EKF_CalculateIndex(5, 0, 4)] = -0.5f * wheelbase;

    ekf->state_to_meas_data[EKF_CalculateIndex(5, 1, 3)] = 1.0f;
    ekf->state_to_meas_data[EKF_CalculateIndex(5, 1, 4)] = 0.5f * wheelbase;

    ekf->state_to_meas_data[EKF_CalculateIndex(5, 2, 2)] = 1.0f;
    ekf->state_to_meas_data[EKF_CalculateIndex(5, 3, 3)] = 1.0f;
    ekf->state_to_meas_data[EKF_CalculateIndex(5, 4, 4)] = 1.0f;

    // --- Setup Identity Matrix (I) ---
    for (uint8_t i = 0; i < 5; i++) {
        ekf->identity_data[EKF_CalculateIndex(5, i, i)] = 1.0f;
    }

    // --- Setup Initial Covariance (P_init) ---
    // We are clamped in the start cell, so we are extremely confident (low variance)
    for (uint8_t i = 0; i < 5; i++) {
        ekf->cov_data[EKF_CalculateIndex(5, i, i)] = 1e-6f;
    }

    // --- Setup Process Noise (Q) ---
    // Tune these to dictate how much you trust the kinematic physics model
    ekf->process_noise_data[EKF_CalculateIndex(5, 0, 0)] = 1e-4f; // X
    ekf->process_noise_data[EKF_CalculateIndex(5, 1, 1)] = 1e-4f; // Y
    ekf->process_noise_data[EKF_CalculateIndex(5, 2, 2)] = 1e-4f; // Theta
    ekf->process_noise_data[EKF_CalculateIndex(5, 3, 3)] = 1e-2f; // Velocity (higher noise due to slip)
    ekf->process_noise_data[EKF_CalculateIndex(5, 4, 4)] = 1e-2f; // Angular Velo (higher noise)

    // --- Setup Measurement Noise (R) ---
    // Tune these based on actual static/dynamic hardware variance tests
    ekf->measurement_noise_data[EKF_CalculateIndex(5, 0, 0)] = 0.1f;  // L_Encoder
    ekf->measurement_noise_data[EKF_CalculateIndex(5, 1, 1)] = 0.1f;  // R_Encoder
    ekf->measurement_noise_data[EKF_CalculateIndex(5, 2, 2)] = 0.05f; // Gyro Theta (highly trusted)
    ekf->measurement_noise_data[EKF_CalculateIndex(5, 3, 3)] = 0.2f;  // Mouse Fwd Velo
    ekf->measurement_noise_data[EKF_CalculateIndex(5, 4, 4)] = 0.3f;  // Mouse Ang Velo (Needs manual scaling based on d_x squared)

    // =========================================================================
    // 3. Bind Data Arrays to CMSIS-DSP Matrix Instances
    // =========================================================================

    // --- 5x1 Vectors ---
    arm_mat_init_f32(&(ekf->state),       5, 1, ekf->state_data);
    arm_mat_init_f32(&(ekf->pred_state),  5, 1, ekf->pred_state_data);
    arm_mat_init_f32(&(ekf->measurement), 5, 1, ekf->measurement_data);
    arm_mat_init_f32(&(ekf->innovation),  5, 1, ekf->innovation_data);
    arm_mat_init_f32(&(ekf->scratch_5x1), 5, 1, ekf->scratch_5x1_data);

    // --- 5x5 Matrices ---
    arm_mat_init_f32(&(ekf->cov),               5, 5, ekf->cov_data);
    arm_mat_init_f32(&(ekf->process_noise),     5, 5, ekf->process_noise_data);
    arm_mat_init_f32(&(ekf->measurement_noise), 5, 5, ekf->measurement_noise_data);
    arm_mat_init_f32(&(ekf->state_to_meas),     5, 5, ekf->state_to_meas_data);
    arm_mat_init_f32(&(ekf->jacobian),          5, 5, ekf->jacobian_data);
    arm_mat_init_f32(&(ekf->innovation_cov),    5, 5, ekf->innovation_cov_data);
    arm_mat_init_f32(&(ekf->kalman_gain),       5, 5, ekf->kalman_gain_data);
    arm_mat_init_f32(&(ekf->identity_matrix),   5, 5, ekf->identity_data);

    // Scratchpads
    arm_mat_init_f32(&(ekf->scratch_5x5_1), 5, 5, ekf->scratch_5x5_1_data);
    arm_mat_init_f32(&(ekf->scratch_5x5_2), 5, 5, ekf->scratch_5x5_2_data);
    arm_mat_init_f32(&(ekf->scratch_5x5_3), 5, 5, ekf->scratch_5x5_3_data);
}

inline float32_t EKF_GetPosX(EKF *ekf) {
	return ekf->state.pData[0];
}

float32_t EKF_GetPosY(EKF *ekf) {
	return ekf->state.pData[1];
}

float32_t EKF_GetYaw(EKF *ekf) {
	return ekf->state.pData[2];
}

float32_t EKF_GetVelo(EKF *ekf) {
	return ekf->state.pData[3];
}

float32_t EKF_GetAngularVelo(EKF *ekf) {
	return ekf->state.pData[4];
}

void EKF_PredictState(EKF *ekf, float32_t dt) {
	//	x' = x + v * cos(theta) * dt
	//	y' = y + v * sin(theta) * dt
	//	theta' = theta + w * dt
	//	v' = v
	//	w' = w

	float32_t posX        = EKF_GetPosX(ekf);
	float32_t posY        = EKF_GetPosY(ekf);
	float32_t yaw         = EKF_GetYaw(ekf);
	float32_t velo        = EKF_GetVelo(ekf);
	float32_t angularVelo = EKF_GetAngularVelo(ekf);

	float32_t *pOut = ekf->pred_state.pData;

	pOut[0] = posX + velo * arm_cos_f32(yaw) * dt;
	pOut[1] = posY + velo * arm_sin_f32(yaw) * dt;
	pOut[2] = yaw + angularVelo * dt;
	pOut[3] = velo;
	pOut[4] = angularVelo;
}

void EKF_GenTransitionJacobian(EKF *ekf, float32_t dt) {
	//    1    0    -v * sin(theta)* dt    cos(theta) * dt    0
	//    0    1    v * cos(theta) * dt    sin(theta) * dt    0
	//    0    0    1                      0                  dt
	//    0    0    0                      1                  0
	//    0    0    0                      0                  1

	float32_t posX        = EKF_GetPosX(ekf);
	float32_t posY        = EKF_GetPosY(ekf);
	float32_t yaw         = EKF_GetYaw(ekf);
	float32_t velo        = EKF_GetVelo(ekf);
	float32_t angularVelo = EKF_GetAngularVelo(ekf);

	float32_t sinYaw = arm_sin_f32(yaw);
	float32_t cosYaw = arm_cos_f32(yaw);

	float32_t *pJacobian = ekf->jacobian.pData;

	// Clear data
	memset(pJacobian, 0, sizeof(float32_t) * 25);

	// Row 1
	pJacobian[EKF_CalculateIndex(5, 0, 0)] = 1.0f;
	pJacobian[EKF_CalculateIndex(5, 0, 2)] = -velo * sinYaw * dt;
	pJacobian[EKF_CalculateIndex(5, 0, 3)] = cosYaw * dt;

	// Row 2
	pJacobian[EKF_CalculateIndex(5, 1, 1)] = 1.0f;
	pJacobian[EKF_CalculateIndex(5, 1, 2)] = velo * cosYaw * dt;
	pJacobian[EKF_CalculateIndex(5, 1, 3)] = sinYaw * dt;

	// Row 3
	pJacobian[EKF_CalculateIndex(5, 2, 2)] = 1.0f;
	pJacobian[EKF_CalculateIndex(5, 2, 4)] = dt;

	// Row 4
	pJacobian[EKF_CalculateIndex(5, 3, 3)] = 1.0f;

	// Row 5
	pJacobian[EKF_CalculateIndex(5, 4, 4)] = 1.0f;
}

void EKF_UpdateFilter(EKF *ekf, float32_t dt) {
	if (ekf->measurement.numRows != ekf->measurement_noise.numRows) {
		// TODO: Impl failure mode
	}

	EKF_PredictState(ekf, dt);
	EKF_GenTransitionJacobian(ekf, dt);

	// pred_cov_P = transition_jacobian @ cov_P @ transition_jacobian.T + process_noise_Q
	arm_mat_mult_f32(&(ekf->jacobian), &(ekf->cov), &(ekf->scratch_5x5_1));
	arm_mat_trans_f32(&(ekf->jacobian), &(ekf->scratch_5x5_2));
	arm_mat_mult_f32(&(ekf->scratch_5x5_1), &(ekf->scratch_5x5_2), &(ekf->cov));
	arm_mat_add_f32(&(ekf->cov), &(ekf->process_noise), &(ekf->cov));

	// innovation = meas - state_to_meas_H @ pred_state
	arm_mat_mult_f32(&(ekf->state_to_meas), &(ekf->pred_state), &(ekf->scratch_5x1));
	arm_mat_sub_f32(&(ekf->measurement), &(ekf->scratch_5x1), &(ekf->innovation));

	// innovation_cov = state_to_meas_H @ pred_cov_P @ state_to_meas_H.T + measurement_noise_R
	arm_mat_mult_f32(&(ekf->state_to_meas), &(ekf->cov), &(ekf->scratch_5x5_1));
	arm_mat_trans_f32(&(ekf->state_to_meas), &(ekf->scratch_5x5_2));
	arm_mat_mult_f32(&(ekf->scratch_5x5_1), &(ekf->scratch_5x5_2), &(ekf->innovation_cov));
	arm_mat_add_f32(&(ekf->innovation_cov), &(ekf->measurement_noise), &(ekf->innovation_cov));

	// kalman_gain = pred_cov_P @ state_to_meas_H.T @ inv(innovation_cov)
	arm_status status = arm_mat_inverse_f32(&(ekf->innovation_cov), &(ekf->scratch_5x5_1));
	if (status == ARM_MATH_SUCCESS) {
		arm_mat_mult_f32(&(ekf->cov), &(ekf->scratch_5x5_2), &(ekf->scratch_5x5_3));
		arm_mat_mult_f32(&(ekf->scratch_5x5_3), &(ekf->scratch_5x5_1), &(ekf->kalman_gain));

		// state = pred_state + kalman_gain @ innovation
		arm_mat_mult_f32(&(ekf->kalman_gain), &(ekf->innovation), &(ekf->scratch_5x1));
		arm_mat_add_f32(&(ekf->pred_state), &(ekf->scratch_5x1), &(ekf->state));

		// cov_P = (I - kalman_gain @ state_to_meas_H) @ pred_cov_P
		arm_mat_mult_f32(&(ekf->kalman_gain), &(ekf->state_to_meas), &(ekf->scratch_5x5_1));
		arm_mat_sub_f32(&(ekf->identity_matrix), &(ekf->scratch_5x5_1), &(ekf->scratch_5x5_2));
		arm_mat_mult_f32(&(ekf->scratch_5x5_2), &(ekf->cov), &(ekf->scratch_5x5_3));
		memcpy(ekf->cov.pData, ekf->scratch_5x5_3.pData, sizeof(float32_t) * 25);
	} else {
		// TODO: Impl inversion failure mode (singular matrix)
	}
}

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
