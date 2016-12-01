#ifndef ATT_EST_H
#define ATT_EST_H
#include "stabilizer_types.h"
void stateEstimator(state_t *state, const marg_t *marg, const uint32_t tick);
#endif
