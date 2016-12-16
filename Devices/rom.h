#ifndef ROM_H
#define ROM_H
#include "stm32f4xx_hal.h"
#define EEPROM_ADDRESS 0x50
void eeprom_write(void *data, unsigned char len, unsigned char offset);
void eeprom_read(unsigned char *buffer, unsigned char len, unsigned char offset);
void eeprom_readCallback(void);
void eeprom_init(void);
#endif