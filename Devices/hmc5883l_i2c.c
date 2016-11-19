#include "hmc5883l_i2c.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
extern I2C_HandleTypeDef hi2c1;

//uint8_t mag_i2c_tx[6] = {0,0,0,0,0,0};
uint8_t mag_i2c_rx[6];
//uint8_t acc_gyr_tx[15];
short mag_data[3];

static void vHMC5883LTask( void *pvParameters ) ;
void hmc5883l_cfg(void)
{
	uint8_t who_am_i = 0;
	uint8_t cfg[3] = {0x18, 0x20, 0x00};
	HAL_I2C_Mem_Read(&hi2c1, HMC5983_ADDRESS, HMC5983_ID_A, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 5);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(&hi2c1, HMC5983_ADDRESS, ADDR_CONF_A, I2C_MEMADD_SIZE_8BIT, cfg, 3, 5);
}
void hmc5883l_read_all(void)
{
	int i;
	for (i=0; i<6; i++){
		HAL_I2C_Mem_Read(&hi2c1, HMC5983_ADDRESS, ADDR_DATA_OUT_X_MSB, I2C_MEMADD_SIZE_8BIT, mag_i2c_rx, 6, 5);
	}
	for (i=0; i<3; i++){
		mag_data[i] = ((short)mag_i2c_rx[2*i]<<8)|(short)mag_i2c_rx[2*i+1];
	}
}
void hmc5883l_dma_start(void)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1, HMC5983_ADDRESS, ADDR_DATA_OUT_X_MSB, I2C_MEMADD_SIZE_8BIT, mag_i2c_rx, 6);
}

void hmc_fast_init(void)
{
	xTaskCreate( vHMC5883LTask, "HMC5883L", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL );  
}
void vHMC5883LTask( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 100;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
  {  
  //  hmc5883l_read_all();
		hmc5883l_dma_start();
    vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
  }  
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	int i;
	for (i=0; i<3; i++){
		mag_data[i] = ((short)mag_i2c_rx[2*i]<<8)|(short)mag_i2c_rx[2*i+1];
	}
}
