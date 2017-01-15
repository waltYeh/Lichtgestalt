#ifndef COMMANDER_H
#define COMMANDER_H
#include "stabilizer_types.h"
void setpointAcquire(setpoint_t* sp);
void commanderInit(void);
void euler_sp_reset(const stateAtt_t* state);

#endif
