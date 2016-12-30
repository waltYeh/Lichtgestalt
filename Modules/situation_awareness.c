#include "situation_awareness.h"
#include "stabilizer_types.h"
#include "../Commons/platform.h"
#include <math.h>
#include "arm_math.h"
#include "stm32f4xx.h"
#include "../MathLib/attitude_lib.h"

//retVal: 0x01010101
//eight bits:
//0,0,0,on air, on land, at rest, upside down, free fall
unsigned char situAwareUpdate(setpoint_t *setpoint, const marg_t *marg, const stateAtt_t *state)
{
	unsigned char retVal=0x00;
	
	
	return retVal;
}
