#include "ms5611_i2c.h"
#include "../Modules/sensors_task.h"
#include "stm32f4xx_hal.h"
//baro shares the I2C with mag
#define MS5611_ADDRESS (0x77<<1)
#define MS5611_RESET 0x1E
#define MS5611_ROM_C1 0xA2
#define MS5611_ROM_C2 0xA4
#define MS5611_ROM_C3 0xA6
#define MS5611_ROM_C4 0xA8
#define MS5611_ROM_C5 0xAA
#define MS5611_ROM_C6 0xAC
#define MS5611_CONV_D1 0x48
#define MS5611_CONV_D2 0x58
extern I2C_HandleTypeDef hi2c1;
unsigned short setup,C1,C2,C3,C4,C5,C6=0;
unsigned char p_D1 = MS5611_CONV_D1;
unsigned char p_D2 = MS5611_CONV_D2;

unsigned char res[3];
//unsigned int res_D2;

void ms5611_init(void)
{
	uint8_t cmd_reset = MS5611_RESET;
	uint8_t c1_array[2]={0,0};
	uint8_t c2_array[2]={0,0};
	uint8_t c3_array[2]={0,0};
	uint8_t c4_array[2]={0,0};
	uint8_t c5_array[2]={0,0};
	uint8_t c6_array[2]={0,0};
	HAL_StatusTypeDef reset_ret = HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cmd_reset, 1, 5);
	HAL_Delay(100);
	HAL_StatusTypeDef c1_ret = HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C1, I2C_MEMADD_SIZE_8BIT, c1_array, 2, 5);
	HAL_Delay(1);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C2, I2C_MEMADD_SIZE_8BIT, c2_array, 2, 5);
	HAL_Delay(1);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C3, I2C_MEMADD_SIZE_8BIT, c3_array, 2, 5);
	HAL_Delay(1);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C4, I2C_MEMADD_SIZE_8BIT, c4_array, 2, 5);
	HAL_Delay(1);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C5, I2C_MEMADD_SIZE_8BIT, c5_array, 2, 5);
	HAL_Delay(1);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C6, I2C_MEMADD_SIZE_8BIT, c6_array, 2, 5);
	HAL_Delay(1);
	C1 = (c1_array[1]<<8)+c1_array[0];
	C2 = (c2_array[1]<<8)+c2_array[0];
	C3 = (c3_array[1]<<8)+c3_array[0];
	C4 = (c4_array[1]<<8)+c4_array[0];
	C5 = (c5_array[1]<<8)+c5_array[0];
	C6 = (c6_array[1]<<8)+c6_array[0];
	
}

void ms5611_temp_start(void)
{
	HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5611_ADDRESS, &p_D1, 1);
}
void ms5611_pres_start(void)
{
	HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5611_ADDRESS, &p_D2, 1);
}
void ms5611_read(void)
{
//	HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
}
