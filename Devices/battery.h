#ifndef BATTERY_H
#define BATTERY_H
#include "../MessageTypes/type_methods.h"
void battery_init(void);
void battery_meas_start(void);
unsigned int battery_get_voltage(void);
void batAcquire(battery_t *bat);
#endif
