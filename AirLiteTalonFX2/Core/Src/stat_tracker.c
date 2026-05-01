/*
 * stat_tracker.c
 *
 *  Created on: Apr 28, 2026
 *      Author: rezq
 */

#include "stat_tracker.h"
#include "string.h"

void StatTracker_Reset(StatTracker *st) {
    memset(st, 0, sizeof(StatTracker));
}

void StatTracker_Update(StatTracker *st, float32_t newValue) {
    st->count++;
    float32_t delta = newValue - st->mean;
    st->mean += delta / st->count;
    float32_t delta2 = newValue - st->mean;
    st->M2 += delta * delta2;
}

float32_t StatTracker_GetVariance(StatTracker *st) {
    if (st->count < 2) return 0.0f;
    return st->M2 / (st->count - 1);
}

float32_t StatTracker_GetMean(StatTracker *st) {
    return st->mean;
}
