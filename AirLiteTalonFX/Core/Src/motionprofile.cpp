#include "motionprofile.h"
#include <cmath>     // For sqrtf and fabsf
#include <algorithm> // For std::min and std::max

// Internal helper to flip state based on direction
static MotionProfileState direct(MotionProfileState in, float direction) {
    MotionProfileState result = in;
    result.position *= direction;
    result.velocity *= direction;
    return result;
}

// Internal helper to determine if direction should be flipped
static bool should_flip_acceleration(MotionProfileState initial, MotionProfileState goal) {
    return initial.position > goal.position;
}

MotionProfileState calculate_next_state(
    float dt,
    MotionProfileState current,
    MotionProfileState goal,
    MotionProfileConstraints constraints) {

    float direction = should_flip_acceleration(current, goal) ? -1.0f : 1.0f;

    MotionProfileState current_ = direct(current, direction);
    MotionProfileState goal_ = direct(goal, direction);

    float v_max = constraints.maxVelocity;
    float a_max = constraints.maxAcceleration;

    // If current velocity > v_max, use v_max.
    if (current_.velocity > v_max) {
        current_.velocity = v_max;
    }

    // Calculate times and distances for an equivalent 0-Vmax-0 profile
    float cutoffBegin = current_.velocity / a_max;
    float cutoffDistBegin = cutoffBegin * cutoffBegin * a_max / 2.0f;

    float cutoffEnd = goal_.velocity / a_max;
    float cutoffDistEnd = cutoffEnd * cutoffEnd * a_max / 2.0f;

    float fullTrapezoidDist = cutoffDistBegin + (goal_.position - current_.position) + cutoffDistEnd;
    float accelerationTime = v_max / a_max;

    float fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * a_max;

    float endAccel, endFullSpeed, endDecel;

    // Handle triangular profile (doesn't reach v_max)
    if (fullSpeedDist < 0) {
        accelerationTime = sqrtf(fullTrapezoidDist / a_max);
        fullSpeedDist = 0;
        v_max = accelerationTime * a_max; // Actual peak velocity
    }

    // Calculate times for each phase from the current state
    endAccel = accelerationTime - cutoffBegin;
    endFullSpeed = endAccel + fullSpeedDist / v_max;
    endDecel = endFullSpeed + (accelerationTime - cutoffEnd);

    // Ensure times are non-negative
    endAccel = std::max(0.0f, endAccel);
    endFullSpeed = std::max(endAccel, endFullSpeed);
    endDecel = std::max(endFullSpeed, endDecel);

    MotionProfileState result = current_; // Start with current

    // Calculate state based on where 'dt' falls
    if (dt < endAccel) {
        result.velocity += dt * a_max;
        result.position += (current_.velocity + dt * a_max / 2.0f) * dt;
    } else if (dt < endFullSpeed) {
        result.velocity = v_max;
        result.position += (current_.velocity + endAccel * a_max / 2.0f) * endAccel
                           + v_max * (dt - endAccel);
    } else if (dt <= endDecel) {
        float time_left = endDecel - dt;
        result.velocity = goal_.velocity + time_left * a_max;
        result.position = goal_.position
                          - (goal_.velocity + time_left * a_max / 2.0f) * time_left;
    } else {
        result = goal_; // If dt >= total time, jump to goal
    }

    // Ensure velocity doesn't exceed the (potentially reduced) v_max
    result.velocity = std::min(v_max, result.velocity);
    // Ensure velocity doesn't go below goal (in directional terms)
    result.velocity = std::max(goal_.velocity, result.velocity);

    // Flip back to original direction
    return direct(result, direction);
}

//#include "motionprofile.h"
//#include "math.h"     // For sqrtf and fabsf
//#include <algorithm> // For std::min and std::max
//
//// Internal helper to flip state based on direction
//static MotionProfileState direct(MotionProfileState in, float direction) {
//    MotionProfileState result = in;
//    result.position *= direction;
//    result.velocity *= direction;
//    return result;
//}
//
//// Internal helper to determine if direction should be flipped
//static bool should_flip_acceleration(MotionProfileState initial, MotionProfileState goal) {
//    return initial.position > goal.position;
//}
//
//MotionProfileState calculate_next_state(
//    float dt,
//    MotionProfileState current,
//    MotionProfileState goal,
//    MotionProfileConstraints constraints) {
//
//    float direction = should_flip_acceleration(current, goal) ? -1.0f : 1.0f;
//
//    MotionProfileState current_ = direct(current, direction);
//    MotionProfileState goal_ = direct(goal, direction);
//
//    float v_max = constraints.maxVelocity;
//    float a_max = constraints.maxAcceleration;
//
//    // If current velocity > v_max, use v_max.
//    // Note: A real profile might need to decelerate; this simplifies.
//    if (current_.velocity > v_max) {
//        current_.velocity = v_max;
//    }
//
//    // Calculate times and distances for an equivalent 0-Vmax-0 profile
//    float cutoffBegin = current_.velocity / a_max;
//    float cutoffDistBegin = cutoffBegin * cutoffBegin * a_max / 2.0f;
//
//    float cutoffEnd = goal_.velocity / a_max;
//    float cutoffDistEnd = cutoffEnd * cutoffEnd * a_max / 2.0f;
//
//    float fullTrapezoidDist = cutoffDistBegin + (goal_.position - current_.position) + cutoffDistEnd;
//    float accelerationTime = v_max / a_max;
//
//    float fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * a_max;
//
//    float endAccel, endFullSpeed, endDecel;
//
//    // Handle triangular profile (doesn't reach v_max)
//    if (fullSpeedDist < 0) {
//        accelerationTime = sqrtf(fullTrapezoidDist / a_max);
//        fullSpeedDist = 0;
//        v_max = accelerationTime * a_max; // Actual peak velocity
//    }
//
//    // Calculate times for each phase from the current state
//    endAccel = accelerationTime - cutoffBegin;
//    endFullSpeed = endAccel + fullSpeedDist / v_max;
//    endDecel = endFullSpeed + (accelerationTime - cutoffEnd);
//
//    // Ensure times are non-negative
//    endAccel = std::max(0.0f, endAccel);
//    endFullSpeed = std::max(endAccel, endFullSpeed);
//    endDecel = std::max(endFullSpeed, endDecel);
//
//    MotionProfileState result = current_; // Start with current
//
//    // Calculate state based on where 'dt' falls
//    if (dt < endAccel) {
//        result.velocity += dt * a_max;
//        result.position += (current_.velocity + dt * a_max / 2.0f) * dt;
//    } else if (dt < endFullSpeed) {
//        result.velocity = v_max;
//        result.position += (current_.velocity + endAccel * a_max / 2.0f) * endAccel
//                           + v_max * (dt - endAccel);
//    } else if (dt <= endDecel) {
//        float time_left = endDecel - dt;
//        result.velocity = goal_.velocity + time_left * a_max;
//        result.position = goal_.position
//                          - (goal_.velocity + time_left * a_max / 2.0f) * time_left;
//    } else {
//        result = goal_; // If dt >= total time, jump to goal
//    }
//
//    // Ensure velocity doesn't exceed the (potentially reduced) v_max
//    result.velocity = std::min(v_max, result.velocity);
//    // Ensure velocity doesn't go below goal (in directional terms)
//    result.velocity = std::max(goal_.velocity, result.velocity);
//
//
//    // Flip back to original direction
//    return direct(result, direction);
//}
/*#include "motionprofile.h"

#include <math.h>

MotionProfile generate_profile(float v_max, float a_max, float x_target, float v_initial, float v_final) {
    MotionProfile profile = MotionProfile();

    profile.v_max = v_max;
    profile.a_max = a_max;
    profile.x_target = x_target;

    profile.v_initial = v_initial;
    profile.v_final = v_final;

    float t_ramp_up = (v_max - v_initial) / a_max;
    // Ramp up displacement. Includes the accelerating term and the static initial velocity term
    float x_ramp_up = t_ramp_up * (v_max - v_initial) / 2 + t_ramp_up * v_initial;

    float t_ramp_down = (v_max - v_final) / a_max;
    // Ramp down displacement. Includes the accelerating term and the static final velocity term
    float x_ramp_down = t_ramp_down * (v_max - v_final) / 2 + t_ramp_down * v_final;

    // Account for negative motion profiles
    // Check if any time is needed at max velo
    if ((x_target > 0 && x_target >= x_ramp_up + x_ramp_down) || (x_target <= 0 && x_target < -(x_ramp_up + x_ramp_down))) {
        profile.end_ramp_time = t_ramp_up;
        // Distance the robot needs to travel at its max velo (total distance - ramping distance)
        float distance_max_v = x_target - (x_ramp_up + x_ramp_down);
        float time_max_v = distance_max_v / v_max;
        profile.end_coast_time = profile.end_ramp_time + time_max_v;
        profile.end_profile_time = profile.end_coast_time + t_ramp_down;
    } else {
        // No time is needed at max velo, equations are different
        // https://www.desmos.com/calculator/gywqxtrmkc && iPad notes in MotionProfileLogic2-1.png & MotionProfileLogic2-2.png
        profile.end_ramp_time = (-4 * v_initial + sqrt(8 * ((v_initial * v_initial) + (v_final * v_final) + (2 * a_max * x_target)))) / (4 * a_max);
        profile.end_coast_time = profile.end_ramp_time;
        profile.end_profile_time = profile.end_ramp_time + (profile.end_ramp_time * a_max + v_initial - v_final) / a_max;

        // Adjust v_max to be the max attainable v in the space we have
        profile.v_max = profile.end_ramp_time * a_max + v_initial;
    }

    // std::cerr << "End Ramp Time " << profile.end_ramp_time << " End Cost Time " << profile.end_coast_time << " End Profile Time " << profile.end_profile_time << std::endl;

    return profile;
}

float profile_velo_wrt_t(float time, float start_time, MotionProfile profile) {
    float dt = time - start_time; // Elapsed time
    if (dt < profile.end_ramp_time) return profile.a_max * dt + profile.v_initial;
    else if (dt < profile.end_coast_time) return profile.v_max;
    else if (dt < profile.end_profile_time) return (profile.a_max * profile.end_ramp_time + profile.v_initial) - profile.a_max * (dt - profile.end_coast_time);
    else return profile.v_final;
}

//float calculate()
*/
