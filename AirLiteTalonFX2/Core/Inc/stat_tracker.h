/*
 * stat_tracker.h
 *
 *  Created on: Apr 28, 2026
 *      Author: rezq
 */

#ifndef INC_STAT_TRACKER_H_
#define INC_STAT_TRACKER_H_

#include "arm_math.h"

typedef struct {
    uint32_t count;
    float32_t mean;
    float32_t M2;
} StatTracker;

void StatTracker_Reset(StatTracker *st);
void StatTracker_Update(StatTracker *st, float32_t newValue);
float32_t StatTracker_GetVariance(StatTracker *st);
float32_t StatTracker_GetMean(StatTracker *st);

#endif /* INC_STAT_TRACKER_H_ */
