#include "MotionProfile.h"

#include <math.h>
#include <iostream>

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