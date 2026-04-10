/*
 * imu.h
 *
 *  Created on: Apr 1, 2026
 *      Author: rezq
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include "arm_math.h"
#include "stdbool.h"
#include "string.h"

typedef struct {
    // ==========================================
    // 1. Memory Backing Arrays (Float Data)
    // ==========================================

    // --- 5x1 Vectors ---
    float32_t state_data[5];            // x
    float32_t pred_state_data[5];       // x'
    float32_t measurement_data[5];      // z
    float32_t innovation_data[5];       // y
    float32_t scratch_5x1_data[5];      // Intermediate vector math

    // --- 5x5 Matrices ---
    float32_t cov_data[25];             // P
    float32_t process_noise_data[25];   // Q
    float32_t measurement_noise_data[25];// R
    float32_t state_to_meas_data[25];   // H
    float32_t jacobian_data[25];        // F
    float32_t innovation_cov_data[25];  // S
    float32_t kalman_gain_data[25];     // K
    float32_t identity_data[25];        // I
    float32_t scratch_5x5_1_data[25];   // Intermediate matrix math 1
    float32_t scratch_5x5_2_data[25];   // Intermediate matrix math 2
    float32_t scratch_5x5_3_data[25];   // Intermediate matrix math 3

    // ==========================================
    // 2. CMSIS-DSP Matrix Instances
    // ==========================================

    // --- 5x1 Vectors ---
    arm_matrix_instance_f32 state;
    arm_matrix_instance_f32 pred_state;
    arm_matrix_instance_f32 measurement;
    arm_matrix_instance_f32 innovation;
    arm_matrix_instance_f32 scratch_5x1;

    // --- 5x5 Matrices ---
    arm_matrix_instance_f32 cov;
    arm_matrix_instance_f32 process_noise;
    arm_matrix_instance_f32 measurement_noise;
    arm_matrix_instance_f32 state_to_meas;
    arm_matrix_instance_f32 jacobian;
    arm_matrix_instance_f32 innovation_cov;
    arm_matrix_instance_f32 kalman_gain;
    arm_matrix_instance_f32 identity_matrix;
    arm_matrix_instance_f32 scratch_5x5_1;
    arm_matrix_instance_f32 scratch_5x5_2;
    arm_matrix_instance_f32 scratch_5x5_3;

} EKF;

void EKF_Test();

#endif /* INC_EKF_H_ */
