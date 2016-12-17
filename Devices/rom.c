#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "rom.h"
#include "../config/config.h"
#include "../Modules/stabilizer_types.h"
#include "../Commons/platform.h"
//sensor bias, transp or api, xbee coord and self address
//agent addr
//pid params
extern I2C_HandleTypeDef hi2c2;
#define WRITE_BUF_SIZE 64
#define READ_BUF_SIZE 64
static unsigned char read_byte_cnt = 0;
static unsigned char read_init_offset = 0;
static unsigned char read_len = 0;
static unsigned char write_byte_cnt = 0;
static unsigned char write_init_offset = 0;
static unsigned char write_len = 0;
unsigned char write_buffer[WRITE_BUF_SIZE];
unsigned char read_buffer[READ_BUF_SIZE];
static void vEepromTask( void *pvParameters );
void eeprom_init(void)
{
	xTaskCreate( vEepromTask, "ROM", configMINIMAL_STACK_SIZE, NULL, EEPROM_TASK_PRI, NULL );  	
}
void vEepromTask( void *pvParameters )
{
	int i;
	for(i=0; i<16; i++){
		write_buffer[i] = i;
		read_buffer[i] = 0xEE;
	}
	eeprom_write(write_buffer, 16, 2);
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1000;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1){
		vTaskDelayUntil( &xLastWakeTime, timeIncreament );
		eeprom_read(read_buffer, 16, 0);
	}
}
void eeprom_write(void *data, unsigned char len, unsigned char offset)
{
	write_byte_cnt = 0;
	write_len = len;
	write_init_offset = offset;
	#if EEP_ROM
	HAL_I2C_Mem_Write_IT(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, data, 1);
//	HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, data, len, 5);
	#elif FLASH_ROM
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_SECTORS;
    f.Sector = FLASH_SECTOR_11;//128KB
	f.Banks = FLASH_BANK_1;
	f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    f.NbSectors = 1;
    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&f, &PageError);
	//word 32 bits
	for (write_byte_cnt=0;write_byte_cnt<len;write_byte_cnt++){
		HAL_FLASH_Program(TYPEPROGRAM_BYTE, FLASH_ADDR+offset+write_byte_cnt, *(uint8_t*)data+write_byte_cnt);
	}
	HAL_FLASH_Lock();
	#endif
}
void eeprom_read(unsigned char *pRxData, unsigned char len, unsigned char offset)
{
	read_byte_cnt = 0;
	read_len = len;
	read_init_offset = offset;
	#if EEP_ROM
	HAL_I2C_Mem_Read_IT(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, pRxData, 1);
//	HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, pRxData, len, 5);
	#elif FLASH_ROM
	for (read_byte_cnt=0;read_byte_cnt<len;read_byte_cnt++){
		*(pRxData+read_byte_cnt) = *(__IO uint8_t*)(FLASH_ADDR+offset+read_byte_cnt);
	}
	#endif
}
void EepromReadCpltCallback(void)
{
	
	
}
void eeprom_readCallback(void)
{
	if(read_byte_cnt < read_len - 1){
		read_byte_cnt++;
		HAL_I2C_Mem_Read_IT(&hi2c2, EEPROM_ADDRESS, read_init_offset + read_byte_cnt, I2C_MEMADD_SIZE_8BIT, read_buffer + read_byte_cnt, 1);
	}
	else{
		EepromReadCpltCallback();
	}
}
void eeprom_writeCallback(void)
{
	if(write_byte_cnt < write_len - 1){
		write_byte_cnt++;
		HAL_I2C_Mem_Write_IT(&hi2c2, EEPROM_ADDRESS, write_init_offset + write_byte_cnt, I2C_MEMADD_SIZE_8BIT, write_buffer + write_byte_cnt, 1);
	}
	
}
