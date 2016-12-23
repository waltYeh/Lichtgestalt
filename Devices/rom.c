#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "rom.h"
#include "../config/config.h"
#include "../Modules/stabilizer_types.h"
#include "../Commons/platform.h"
#include "string.h"
//sensor bias, transp or api, xbee coord and self address
//agent addr
//pid params
extern I2C_HandleTypeDef hi2c2;
#define WRITE_BUF_SIZE 64
#define READ_BUF_SIZE 64
bool flash_write = false;
unsigned char write_buffer[WRITE_BUF_SIZE];
unsigned char read_buffer[READ_BUF_SIZE];
static void vEepromTask( void *pvParameters );
void eeprom_init(void)
{
	eeprom_read(read_buffer, 16, 0);
	memcpy(write_buffer, read_buffer, 16);
	xTaskCreate( vEepromTask, "ROM", configMINIMAL_STACK_SIZE, NULL, EEPROM_TASK_PRI, NULL );  
}
void vEepromTask( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1000;
	xLastWakeTime = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil( &xLastWakeTime, timeIncreament );
		if(flash_write){
			flash_write = false;
			vTaskSuspendAll();
			// this will take 1000ms or so
			eeprom_write(write_buffer, 32, 0);
			xTaskResumeAll();
			eeprom_read(read_buffer, 32, 0);
			memcpy(write_buffer, read_buffer, 32);
		}
	}
}
void rom_set_mag_bias(const vec3i16_t *mag_bias)
{
	memcpy(write_buffer + OFFSET_MAG, mag_bias->v, 6);
	flash_write = true;
}
void rom_set_acc_bias(const vec3i16_t *acc_bias)
{
	memcpy(write_buffer + OFFSET_ACC, acc_bias->v, 6);
	flash_write = true;
}
void rom_get_mag_bias(vec3i16_t *mag_bias)
{
	memcpy(mag_bias->v, read_buffer + OFFSET_MAG, 6);
}
void rom_get_acc_bias(vec3i16_t *acc_bias)
{
	memcpy(acc_bias->v, read_buffer + OFFSET_ACC, 6);
}
void eeprom_write(void *data, unsigned char len, unsigned char offset)
{
	int i;
	#if EEP_ROM
	HAL_I2C_Mem_Write_DMA(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, data, len);
//	HAL_I2C_Mem_Write_IT(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, data, 1);
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
	for (i=0;i<len;i++){
		HAL_FLASH_Program(TYPEPROGRAM_BYTE, FLASH_ADDR+offset+i, *((uint8_t*)data+i));
	}
	HAL_FLASH_Lock();
	#endif
}
void eeprom_read(unsigned char *pRxData, unsigned char len, unsigned char offset)
{
	int i;
	#if EEP_ROM
	HAL_I2C_Mem_Read_DMA(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, pRxData, len);
//	HAL_I2C_Mem_Read_IT(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, pRxData, 1);
//	HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, pRxData, len, 5);
	#elif FLASH_ROM
	for (i=0;i<len;i++){
		*(pRxData+i) = *(__IO uint8_t*)(FLASH_ADDR+offset+i);
	}
	#endif
}
void EepromReadCpltCallback(void)
{
	
}
void eeprom_readCallback(void)
{

}
void eeprom_writeCallback(void)
{

}
