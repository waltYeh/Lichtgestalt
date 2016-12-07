#ifndef ATT_EST_H
#define ATT_EST_H
#include "stabilizer_types.h"
void stateEstimator(stateAtt_t *state, const marg_t *marg, const vec3f_t *mot_acc);
void quarternion_init(const vec3f_t* acc, const vec3f_t* mag, stateAtt_t* stateAtt);
#endif
