#include "data_link.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Modules/stabilizer_types.h"
#include "xbee_api.h"
extern UART_HandleTypeDef huart2;
#define TX_BUF_SIZE 64
#define RX_BUF_SIZE 64
unsigned char tx_buffer[TX_BUF_SIZE];
unsigned char rx_buffer0[RX_BUF_SIZE];//change bigger?
unsigned char rx_buffer1[RX_BUF_SIZE];
unsigned char decoding_buffer[RX_BUF_SIZE];
static unsigned int rx_len;
static unsigned char buffer_num=0;
static unsigned char buf2read_num=0;
unsigned char data2send[18]=
	{0x10,0x22,0x00,0x13,0xA2,0x00,0x41,0x4E,0x6D,
	0x61,0xFF,0xFE,0x00,0x00,0x41,0x42,0x38,0x37};

unsigned int dest_addr_h = 0x00A21300;
unsigned int dest_addr_l = 0x616D4E41;
static xQueueHandle command_q;
static xQueueHandle motion_acc_q;
static xQueueHandle cal_q;
static command_t command;
static vec3f_t motion_acc;
static calib_t cal;
static xSemaphoreHandle dataReceived;
	
static void vDataSendTask( void *pvParameters ) ;
static void vDataReceiveTask( void *pvParameters );
void data_link_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	
	xTaskCreate( vDataReceiveTask, "Receive", configMINIMAL_STACK_SIZE, NULL, XBEE_RX_TASK_PRI, NULL );  	
	command_q = xQueueCreate(1, sizeof(command_t));
	motion_acc_q = xQueueCreate(1, sizeof(vec3f_t));
	cal_q = xQueueCreate(1, sizeof(calib_t));
	vSemaphoreCreateBinary( dataReceived );
	if(buffer_num == 0){
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


void vDataSendTask( void *pvParameters )  
{  
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1000;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; ){  
		send_buffer(data2send);
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
	}  
}
void vDataReceiveTask( void *pvParameters )  
{  
	unsigned char api_id;
	for( ;; ){  
		if (pdTRUE == xSemaphoreTake(dataReceived, portMAX_DELAY)){
			if(buf2read_num == 0)
				memcpy(decoding_buffer,rx_buffer0,rx_len);
			else
				memcpy(decoding_buffer,rx_buffer1,rx_len);
			api_id = api_pack_decode(decoding_buffer, rx_len);
			switch(api_id){
				case API_ID_TX_REQ:{
					//not for receiving
				}
				break;
				case API_ID_TX_STATUS:{
					api_tx_status_decode(decoding_buffer, rx_len);
				}
				break;
				case API_ID_RX_PACK:{
					unsigned char descriptor = api_rx_decode(decoding_buffer, rx_len);
					switch(descriptor){
						case DSCR_CMD_ACC:{
							decode_cmd_acc(decoding_buffer, rx_len, &command, &motion_acc);
							xQueueOverwrite(command_q, &command);
							xQueueOverwrite(motion_acc_q, &motion_acc);	
						}
						break;
						case DSCR_CAL:{
							decode_calibrate(decoding_buffer, rx_len, &cal);
							xQueueOverwrite(cal_q, &cal);
						}
						break;
						case DSCR_CFG:{
							
						}
						break;
						default:
						break;
					}//switch descriptor
				}//case rx_pack of api_id
				break;
				default:{

				}
				break;
			}//switch api_id
			
		}
	}  
} 
void send_buffer(void *data)
{
	unsigned char content_len = encode_general(tx_buffer, data2send);
	api_tx_encode(tx_buffer, dest_addr_h, dest_addr_l);
	api_pack_encode(tx_buffer, content_len+14);
	HAL_UART_Transmit_DMA(&huart2, tx_buffer, content_len+4+14);
}
void motionAccAcquire(vec3f_t *motion_acc)
{
	xQueueReceive(motion_acc_q, motion_acc, 0);
}
void calibrationAcquire(calib_t *cal)
{
	xQueueReceive(cal_q, cal, 0);
}
void commandBlockingAcquire(command_t *cmd)
{
	xQueueReceive(command_q, cmd, portMAX_DELAY);
}
void DataLinkReceive_IDLE(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET)){   
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);  
		HAL_UART_DMAStop(&huart2);  
       // temp = huart5.hdmarx->Instance->NDTR;  
		rx_len =  RX_BUF_SIZE - huart2.hdmarx->Instance->NDTR;             
		
		
		buf2read_num = buffer_num;
		
		xSemaphoreGiveFromISR(dataReceived, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken)
			portYIELD();
		
		
		buffer_num = (!buffer_num) & 1;//either 0 or 1			
		if(buffer_num == 0){
			HAL_UART_Receive_DMA(&huart2,rx_buffer0,RX_BUF_SIZE); 
		}
		else{
			HAL_UART_Receive_DMA(&huart2,rx_buffer1,RX_BUF_SIZE); 
		}		
	}  
}
