#ifndef ROM_H
#define ROM_H
#include "stm32f4xx_hal.h"
#define EEPROM_ADDRESS 0x50
#define FLASH_ADDR 0x080E0000
#include "../Modules/stabilizer_types.h"
#define OFFSET_MAG 0
#define OFFSET_ACC 6
#define AGENT_NO 12
#define XBEE_MODE 14
void rom_set_mag_bias(const vec3i16_t *mag_bias);
void rom_set_acc_bias(const vec3i16_t *acc_bias);
void rom_get_mag_bias(vec3i16_t *mag_bias);
void rom_get_acc_bias(vec3i16_t *acc_bias);
void eeprom_write(void *data, unsigned char len, unsigned char offset);
void eeprom_read(unsigned char *buffer, unsigned char len, unsigned char offset);
void eeprom_readCallback(void);
void eeprom_writeCallback(void);
void eeprom_init(void);
#endif
