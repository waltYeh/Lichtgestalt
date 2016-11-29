#include "data_link.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
extern UART_HandleTypeDef huart2;
#define TX_BUF_SIZE 29
#define RX_BUF_SIZE 64
unsigned char tx_buffer[TX_BUF_SIZE];
unsigned char rx_buffer0[RX_BUF_SIZE];//change bigger?
unsigned char rx_buffer1[RX_BUF_SIZE];
unsigned int xbee_data_len;
unsigned char xbee_buffer_num=0;
short data[9]={1,2,3,4,5,6,7,8,9};
static void vDataLinkTask( void *pvParameters ) ;
void data_link_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	xTaskCreate( vDataLinkTask, "XBee", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
	if(xbee_buffer_num == 0){
		HAL_UART_Receive_DMA(&huart2,rx_buffer0,RX_BUF_SIZE); 
	}
	else{
		HAL_UART_Receive_DMA(&huart2,rx_buffer0,RX_BUF_SIZE); 
	}	
}
void DataLinkReceive_IDLE(void)
{
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET)){   
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);  
		HAL_UART_DMAStop(&huart2);  
       // temp = huart5.hdmarx->Instance->NDTR;  
		xbee_data_len =  RX_BUF_SIZE - huart2.hdmarx->Instance->NDTR;             
		xbee_buffer_num = (!xbee_buffer_num) & 1;//either 0 or 1			
		if(xbee_buffer_num == 0){
			HAL_UART_Receive_DMA(&huart2,rx_buffer0,RX_BUF_SIZE); 
		}
		else{
			HAL_UART_Receive_DMA(&huart2,rx_buffer1,RX_BUF_SIZE); 
		}		
	}  
}
void vDataLinkTask( void *pvParameters )  
{  
  TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1000;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
  {  
    send_buffer(data, 18);
    vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
  }  
} 
void send_buffer(void *data, unsigned short len)
{
	unsigned char protocol_head[3]={'>','*','>'};
	unsigned char protocol_last[3]={'<','#','<'};
	unsigned char packetdescriptor='c';
	short crc_result = crc16(data,len);
	memcpy(tx_buffer,protocol_head,3);
	memcpy(tx_buffer+3,&len,2);
	memcpy(tx_buffer+5,&packetdescriptor, 1);
	memcpy(tx_buffer+6,data,18);
	memcpy(tx_buffer+24,&crc_result,2);
	memcpy(tx_buffer+26,protocol_last,3);
	HAL_UART_Transmit_DMA(&huart2, tx_buffer, TX_BUF_SIZE);
}
void get_xbee_data(void)
{
	
}
unsigned short crc_update (unsigned short crc, unsigned char data)
{
    data ^= (crc & 0xff);
    data ^= data << 4;
    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
         ^ ((unsigned short )data << 3));
}
unsigned short crc16(void* data, unsigned short cnt)
{
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;
    for (i=0;i<cnt;i++){
        crc=crc_update(crc,*ptr);
        ptr++;
    }
    return crc;
}
