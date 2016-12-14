#include "data_link.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Modules/stabilizer_types.h"
#include "xbee_api.h"
extern UART_HandleTypeDef huart2;
#define TX_BUF_SIZE 29
#define RX_BUF_SIZE 64
unsigned char tx_buffer[TX_BUF_SIZE];
unsigned char rx_buffer0[RX_BUF_SIZE];//change bigger?
unsigned char rx_buffer1[RX_BUF_SIZE];
unsigned char decoding_buffer[RX_BUF_SIZE];
unsigned int xbee_data_len;
unsigned char xbee_buffer_num=0;
unsigned char xbee_buf2read_num=0;
unsigned char data2send[17]=
	{0x10,0x22,0x00,0x13,0xA2,0x00,0x41,0x4E,0x6D,
	0x61,0xFF,0xFE,0x00,0x00,0x41,0x42,0x38};
static xQueueHandle command_q;
static xQueueHandle motion_acc_q;
static command_t command;
static vec3f_t motion_acc;
static xSemaphoreHandle dataReceived;
static void vDataSendTask( void *pvParameters ) ;
static void vDataReceiveTask( void *pvParameters );
void data_link_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	
	xTaskCreate( vDataReceiveTask, "Receive", configMINIMAL_STACK_SIZE, NULL, XBEE_RX_TASK_PRI, NULL );  	
	command_q = xQueueCreate(1, sizeof(command_t));
	motion_acc_q = xQueueCreate(1, sizeof(vec3f_t));
	vSemaphoreCreateBinary( dataReceived );
	if(xbee_buffer_num == 0){
		HAL_UART_Receive_DMA(&huart2,rx_buffer0,RX_BUF_SIZE); 
	}
	else{
		HAL_UART_Receive_DMA(&huart2,rx_buffer0,RX_BUF_SIZE); 
	}	
}
void data_send_start(void)
{
	xTaskCreate( vDataSendTask, "Send", configMINIMAL_STACK_SIZE, NULL, XBEE_TX_TASK_PRI, NULL );
	
}
void DataLinkReceive_IDLE(void)
{
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET)){   
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);  
		HAL_UART_DMAStop(&huart2);  
       // temp = huart5.hdmarx->Instance->NDTR;  
		xbee_data_len =  RX_BUF_SIZE - huart2.hdmarx->Instance->NDTR;             
		
	/*	
		xbee_buf2read_num = xbee_buffer_num;
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(dataReceived, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken)
			portYIELD();
		
	*/	
		xbee_buffer_num = (!xbee_buffer_num) & 1;//either 0 or 1			
		if(xbee_buffer_num == 0){
			HAL_UART_Receive_DMA(&huart2,rx_buffer0,RX_BUF_SIZE); 
		}
		else{
			HAL_UART_Receive_DMA(&huart2,rx_buffer1,RX_BUF_SIZE); 
		}		
	}  
}
void xbee_api_decode(unsigned char * data, unsigned int len, command_t* cmd, vec3f_t* mot_acc)
{
	
	
}
void vDataSendTask( void *pvParameters )  
{  
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1000;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; ){  
		send_buffer(data2send, 17);
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
	}  
}
void vDataReceiveTask( void *pvParameters )  
{  
	for( ;; ){  
		if (pdTRUE == xSemaphoreTake(dataReceived, portMAX_DELAY)){
			if(xbee_buf2read_num == 0)
				memcpy(decoding_buffer,rx_buffer0,xbee_data_len);
			else
				memcpy(decoding_buffer,rx_buffer1,xbee_data_len);
			xbee_api_decode(decoding_buffer, xbee_data_len, &command, &motion_acc);
			xQueueOverwrite(command_q, &command);
			xQueueOverwrite(motion_acc_q, &motion_acc);	
		}
	}  
} 
void send_buffer(void *data, unsigned short len)
{
//	unsigned char protocol_head[3]={'>','*','>'};
//	unsigned char protocol_last[3]={'<','#','<'};
//	unsigned char packetdescriptor='c';
//	short crc_result = crc16(data,len);
	unsigned char start_delimiter = 0x7E;
	unsigned char length = len;
	unsigned char zero = 0;
	unsigned char checksum = 0;
	unsigned char i = 0;
	for(i=0;i<len;i++){
		checksum += *((unsigned char *)(data)+i);
	}
	checksum = 0xFF-checksum;
	memcpy(tx_buffer,&start_delimiter,1);
	memcpy(tx_buffer+1,&zero,1);
	memcpy(tx_buffer+2,&length,1);
	memcpy(tx_buffer+3,data, len);
	memcpy(tx_buffer+3+len,&checksum,1);

	HAL_UART_Transmit_DMA(&huart2, tx_buffer, len+4);
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
void motionAccAcquire(vec3f_t *motion_acc)
{
	xQueueReceive(motion_acc_q, motion_acc, 0);
}
void commandBlockingAcquire(command_t *cmd)
{
	xQueueReceive(command_q, cmd, portMAX_DELAY);
}
