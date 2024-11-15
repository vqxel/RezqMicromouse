#include "../MotionProfile.h"
#include <iostream>

// Time unit is ms
#define X_TARGET 1000
#define V_MAX 200
#define A_MAX 20

#define DELTA_T 1

int main(void) {
    MotionProfile profile = generate_profile(V_MAX, A_MAX, X_TARGET);

    int time = 0;
    while (time < profile.end_profile_time) {
        std::cerr << profile_velo_wrt_t(time, 0, profile) << std::endl;

        time += DELTA_T;
    }
}