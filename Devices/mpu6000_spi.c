#include "mpu6000_spi.h"
#include "../Modules/sensors_task.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../config/config.h"
/*
#define SEL()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define DSEL()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET) 
//#define SEL()  __HAL_SPI_ENABLE(&hspi1)
//#define DSEL() __HAL_SPI_DISABLE(&hspi1)
*/
extern SPI_HandleTypeDef hspi1;
uint8_t acc_gyr_spi_tx[15] = {(1<<7)|(0x3B), 0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//uint8_t acc_gyr_local_rx[15];//uint8_t acc_gyr_spi_rx[15];
//uint8_t acc_gyr_tx[15];
//short a_g_data[6];
//short a_g_queue[6][20];
//static void vMPU6000Task( void *pvParameters ) ;
void mpu6000_cfg(void)
{
	uint8_t who_am_i_tx[2] = {(1<<7)|RA_WHO_AM_I, 0};
	uint8_t dummy_rcv[2] = {0,0};
//	uint8_t mpu_reset[2] = {RA_PWR_MGMENT_1,0x80};
//	uint8_t mpu_wakeup[2] = {RA_PWR_MGMENT_1,0x00};
//	uint8_t pwr_mgment_1[2] = {RA_PWR_MGMENT_1,0x01};
	uint8_t pwr_manage[2] = {RA_PWR_MGMENT_1, 0x1};
	uint8_t smplrt_div[2] = {RA_SMPLRT_DIV, 0x0};
	uint8_t config[2] = {RA_CONFIG, 0x4};
	uint8_t gyro_cfg[2] = {RA_GYRO_CONFIG, 0x08};
	uint8_t accel_cfg[2] = {RA_ACCEL_CONFIG, 0x8};
	uint8_t user_ctrl[2] = {RA_USER_CTRL, 0x0};
	uint8_t int_pin_cfg[2] = {RA_INT_PIN_CFG, 0x0};
	uint8_t int_enable[2] = {RA_INT_ENABLE, 0x0};
	HAL_Delay(10);
	/*
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, mpu_reset, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(50);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, mpu_wakeup, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(10);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, pwr_mgment_1, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(10);
	*/
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, who_am_i_tx, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, pwr_manage, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, smplrt_div, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, config, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, gyro_cfg, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, accel_cfg, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, user_ctrl, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, int_pin_cfg, dummy_rcv, 2, 1);
	DSEL();
	HAL_Delay(1);
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, int_enable, dummy_rcv, 2, 1);
	DSEL();
}
/*
void mpu6000_read_all(void)
{
	uint8_t acc_gyr_tx[15];
	int i;
	acc_gyr_tx[0]=(1<<7)|(0x3B);
	for (i=1; i<15; i++){
		acc_gyr_tx[i] = 0;
	}
	SEL();
	HAL_SPI_TransmitReceive(&hspi1, acc_gyr_tx, acc_gyr_local_rx, 15, 5);
	DSEL();
	for (i=0; i<3; i++){
		a_g_data[i] = ((short)acc_gyr_local_rx[2*i+1]<<8)|(short)acc_gyr_local_rx[2*i+2];
	}
	for (i=0; i<3; i++){
		a_g_data[i+3] = ((short)acc_gyr_local_rx[2*i+9]<<8)|(short)acc_gyr_local_rx[2*i+10];
	}
}
*/
void mpu6000_dma_start(uint8_t *pRxData, uint16_t Size)
{
	SEL();
	HAL_SPI_TransmitReceive_DMA(&hspi1, acc_gyr_spi_tx, pRxData, Size);
//	__HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

}

void mpu_fast_init(void)
{
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	
  HAL_SPI_Init(&hspi1);

//	xTaskCreate( vMPU6000Task, "MPU6000", configMINIMAL_STACK_SIZE, NULL, MPU_TASK_PRI, NULL );  
}
/*
void vMPU6000Task( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1;//1ms
	xLastWakeTime = xTaskGetTickCount();
	for( ;; ){  
//		mpu6000_dma_start(acc_gyr_local_rx, 15);
		mpu6000_read_all();
		vTaskDelayUntil(&xLastWakeTime, timeIncreament); 
	}  
}

*/
/*
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{

//	static int j=0;
	
	DSEL();
	mpu6000Callback();

//	if(j%2 ==0)
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  
//	else
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  
//	j++;
//	if(j==20)
//		j=0;
//	for (i=0; i<6; i++){	
//		a_g_queue[i][j] = a_g_data[i];
//	}
}
*/
