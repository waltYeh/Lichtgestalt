#ifndef COMMANDER_H
#define COMMANDER_H
#include "../MessageTypes/type_methods.h"
void setpointAcquire(attsp_t* sp);
void commanderInit(void);
void euler_sp_reset(const att_t* state);

#endif
