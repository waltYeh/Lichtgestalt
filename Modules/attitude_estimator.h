#ifndef ATT_EST_H
#define ATT_EST_H
#include "stabilizer_types.h"
void stateEstimator(stateAtt_t *state, const marg_t *marg, const vec3f_t *mot_acc, float dt);
void quarternion_init(const vec3f_t* acc, const vec3f_t* mag, stateAtt_t* stateAtt);
void attitude_update(const vec3f_t *gyr, const vec3f_t *pos_acc, const marg_t *marg, vec3f_t *gyr_bias, stateAtt_t *state, float dt);
#endif
