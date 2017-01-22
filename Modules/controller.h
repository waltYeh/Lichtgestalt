#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "../MessageTypes/type_methods.h"
void stateController(output_t *output, 
	const att_t *state,const attsp_t *setpoint, float dt);
void outputBlockingAcquire(output_t *output);

#endif
