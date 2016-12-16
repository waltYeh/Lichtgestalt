#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "rom.h"
#include "../config/config.h"
#include "../Modules/stabilizer_types.h"
//sensor bias, transp or api, xbee coord and self address
//agent addr
//pid params
extern I2C_HandleTypeDef hi2c2;
#define WRITE_BUF_SIZE 64
#define READ_BUF_SIZE 64
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
	HAL_I2C_Mem_Write_IT(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, data, len);
}
void eeprom_read(unsigned char *pRxData, unsigned char len, unsigned char offset)
{
	HAL_I2C_Mem_Read_IT(&hi2c2, EEPROM_ADDRESS, offset, I2C_MEMADD_SIZE_8BIT, pRxData, len);
}
void eeprom_readCallback(void)
{
	int dummy = 0;
	dummy = dummy;
}

