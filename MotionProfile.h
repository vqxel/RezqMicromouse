#pragma once

typedef struct {
    unsigned short end_ramp_time = 0;
    unsigned short end_coast_time = 0;
    unsigned short end_profile_time = 0;
    int v_max = 0;
    int a_max = 0;
    int x_target = 0;
} MotionProfile;

MotionProfile generate_profile(int v_max, int a_max, int x_target);

int profile_velo_wrt_t(unsigned short time, MotionProfile profile);