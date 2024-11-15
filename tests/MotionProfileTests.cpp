#include "../MotionProfile.h"
#include <iostream>

// Time unit is ms
#define X_TARGET 3
#define V_MAX 1
#define A_MAX 1

#define V_INITIAL 0.5
#define V_FINAL 0.2

#define DELTA_T 0.1

int main(void) {
    MotionProfile profile = generate_profile(V_MAX, A_MAX, X_TARGET, V_INITIAL, V_FINAL);

    float time = 0;
    float x = 0;
    while (time <= profile.end_profile_time + DELTA_T) {
        float v = profile_velo_wrt_t(time, 0, profile);
        x += v * DELTA_T;
        std::cerr << "t: " << time << " v: " << v << " x: " << x << std::endl;

        time += DELTA_T;
    }
}