#ifndef _SENSORS_TASK_
#define _SENSORS_TASK_
#include "stabilizer_types.h"
void mpu6000Callback(void);
void hmc5883lCallback(void);
void sensorsTaskInit(void);
void margAcquire(marg_t *marg);
#endif
