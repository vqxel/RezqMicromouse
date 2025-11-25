#pragma once
#include <cmath> // For fabsf

/**
 * @brief Defines the constraints for a trapezoidal motion profile.
 */
typedef struct {
    float maxVelocity;     // Maximum velocity (must be positive)
    float maxAcceleration; // Maximum acceleration (must be positive)
} MotionProfileConstraints;

/**
 * @brief Represents the state (position and velocity) at a point in time.
 */
typedef struct {
    float position = 0.0f;
    float velocity = 0.0f;
} MotionProfileState;

/**
 * @brief Calculates the next motion profile state after a time step 'dt'.
 *
 * This function implements an iterative trapezoidal motion profile calculator.
 * It determines the position and velocity that should be reached after 'dt'
 * when moving from the 'current' state towards the 'goal' state, while
 * adhering to the specified 'constraints'.
 *
 * @param dt The time step since the last calculation (must be positive).
 * @param current The current state (position and velocity).
 * @param goal The desired goal state (position and velocity).
 * @param constraints The motion constraints (max velocity and acceleration).
 * @return The calculated state (position and velocity) after 'dt'.
 */
MotionProfileState calculate_next_state(
    float dt,
    MotionProfileState current,
    MotionProfileState goal,
    MotionProfileConstraints constraints);

/**
 * @brief Checks if the motion profile has reached its goal state.
 *
 * Compares the current state to the goal state within specified tolerances.
 *
 * @param current The current state (position and velocity).
 * @param goal The desired goal state (position and velocity).
 * @param position_tolerance The acceptable error in position (defaults to 0.01).
 * @param velocity_tolerance The acceptable error in velocity (defaults to 0.01).
 * @return true if the current state is within tolerance of the goal, false otherwise.
 */
inline bool is_finished(
    MotionProfileState current,
    MotionProfileState goal,
    float position_tolerance = 0.01f,
    float velocity_tolerance = 10)
{
    return fabsf(current.position - goal.position) <= position_tolerance
        && fabsf(current.velocity - goal.velocity) <= velocity_tolerance;
}

//#pragma once
//
///**
// * @brief Defines the constraints for a trapezoidal motion profile.
// */
//typedef struct {
//    float maxVelocity;     // Maximum velocity (must be positive)
//    float maxAcceleration; // Maximum acceleration (must be positive)
//} MotionProfileConstraints;
//
///**
// * @brief Represents the state (position and velocity) at a point in time.
// */
//typedef struct {
//    float position = 0.0f;
//    float velocity = 0.0f;
//} MotionProfileState;
//
///**
// * @brief Calculates the next motion profile state after a time step 'dt'.
// *
// * This function implements an iterative trapezoidal motion profile calculator.
// * It determines the position and velocity that should be reached after 'dt'
// * when moving from the 'current' state towards the 'goal' state, while
// * adhering to the specified 'constraints'.
// *
// * @param dt The time step since the last calculation (must be positive).
// * @param current The current state (position and velocity).
// * @param goal The desired goal state (position and velocity).
// * @param constraints The motion constraints (max velocity and acceleration).
// * @return The calculated state (position and velocity) after 'dt'.
// */
//MotionProfileState calculate_next_state(
//    float dt,
//    MotionProfileState current,
//    MotionProfileState goal,
//    MotionProfileConstraints constraints);

/*
typedef struct {
    float end_ramp_time = 0;
    float end_coast_time = 0;
    float end_profile_time = 0;
    float v_max = 0;
    float a_max = 0;
    float x_target = 0;

    float v_initial = 0;
    float v_final = 0;
} MotionProfile;

MotionProfile generate_profile(float v_max, float a_max, float x_target, float v_initial, float v_final);

float profile_velo_wrt_t(float time, float start_time, MotionProfile profile);
*/
