#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "stabilizer_types.h"
void stateController(output_t *output, 
	const stateAtt_t *state,const setpoint_t *setpoint);
#endif
