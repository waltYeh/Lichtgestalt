#ifndef MPU6000_H
#define MPU6000_H
#include <stdint.h>
#include "stm32f4xx_hal.h"
void mpu6000_cfg(void);
void mpu_fast_init(void);
void mpu6000_dma_start(uint8_t *pRxData, uint16_t Size);
#define SEL()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define DSEL()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET) 
//#define SEL()  __HAL_SPI_ENABLE(&hspi1)
//#define DSEL() __HAL_SPI_DISABLE(&hspi1)
#define IMU_ADD 0x00680000
#define MPU_ADDR 0x68
#define RA_SMPLRT_DIV 0x19
#define RA_CONFIG 0x1A
#define RA_GYRO_CONFIG 0x1B
#define RA_ACCEL_CONFIG 0x1C
#define RA_FIFO_EN 0x23
#define RA_INT_PIN_CFG 0x37
#define RA_INT_ENABLE 0x38
#define RA_ACCEL_OUT 0x3B
#define RA_GYRO_OUT 0x43
#define RA_SIGNAL_PATH_RESET 0x68
#define RA_USER_CTRL 0x6A
#define RA_PWR_MGMENT_1 0x6B
#define RA_WHO_AM_I 0x75
#define RA_BYPASS_CFG 0x37

#endif
