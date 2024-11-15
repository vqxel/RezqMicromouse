#pragma once

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