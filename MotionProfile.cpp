#include "MotionProfile.h"

#include <math.h>

MotionProfile generate_profile(int v_max, int a_max, int x_target) {
    MotionProfile profile = MotionProfile();

    profile.v_max = v_max;
    profile.a_max = a_max;
    profile.x_target = x_target;

    int t_ramp = v_max / a_max;
    int x_ramp = t_ramp * v_max / 2; // Time it takes to ramp up to/down from max velocity

    // Account for negative motion profiles
    // Check if any time is needed at max velo
    if ((x_target > 0 && x_target >= 2 * x_ramp) || (x_target < 0 && x_target <= 2 * x_ramp)) {
        profile.end_ramp_time = x_ramp;
        int distance_max_v = x_target - 2 * x_ramp;
        int time_max_v = distance_max_v / v_max;
        profile.end_coast_time = profile.end_ramp_time + time_max_v;
        profile.end_profile_time = profile.end_coast_time + x_ramp;
    } else {
        // No time is needed at max velo, equations are different
        profile.end_profile_time = sqrt(4 * x_target / a_max);
        profile.end_ramp_time = profile.end_profile_time / 2;
        profile.end_coast_time = profile.end_ramp_time;
        
        // Adjust v_max to be the max attainable v in the space we have
        profile.v_max = profile.end_profile_time / 2 * a_max;
    }
}

int profile_velo_wrt_t(unsigned short time, unsigned short start_time, MotionProfile profile) {
    int dt = time - start_time; // Elapsed time
    if (dt < profile.end_ramp_time) return profile.a_max * dt;
    else if (dt < profile.end_coast_time) return profile.v_max;
    else if (dt < profile.end_profile_time) return profile.v_max - profile.a_max * (dt - profile.end_coast_time);
    else return 0;
}