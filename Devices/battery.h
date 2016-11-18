#ifndef BATTERY_H
#define BATTERY_H
void battery_init(void);
void battery_meas_start(void);
unsigned int battery_get_voltage(void);
#endif
