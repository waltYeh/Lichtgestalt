#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "stabilizer_types.h"
void stateController(output_t *output, 
	const state_t *state,const setpoint_t *setpoint,
	const uint32_t tick);
#endif
