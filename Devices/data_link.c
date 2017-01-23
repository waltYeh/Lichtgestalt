#include "data_link.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../MessageTypes/type_methods.h"
#include "xbee_api.h"
#include "rom.h"
#include "../Commons/platform.h"
#include "../Modules/attitude_estimator.h"
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
short data2send[18];

unsigned int dest_addr_h = 0x00A21300;
unsigned int dest_addr_l = 0x616D4E41;
static xQueueHandle command_q;
static xQueueHandle motion_acc_q;
static xQueueHandle cal_q;
static command_t command;
static vec3f_t motion_acc;
static calib_t cal;
#if XBEE_API
static att_t att;
#endif
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
//	vec3i16_t mag_bias={0,-150,110};
//	rom_set_mag_bias(&mag_bias);
	xTaskCreate( vDataSendTask, "Send", configMINIMAL_STACK_SIZE, NULL, XBEE_TX_TASK_PRI, NULL );
}


void vDataSendTask( void *pvParameters )  
{  
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 100;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; ){
		send_data(data2send);
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
							rom_set_mag_bias(&cal.mag_bias);
							rom_set_acc_bias(&cal.acc_bias);
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
void send_data(void *data)
//no length argument because length is fixed for frames
{
	#if XBEE_API
	if(g_mode != modeCal){
		attAcquire(&att);
		unsigned char content_len = encode_yaw(tx_buffer, &att);
		api_tx_encode(tx_buffer, dest_addr_h, dest_addr_l);
		api_pack_encode(tx_buffer, content_len+14);
		HAL_UART_Transmit_DMA(&huart2, tx_buffer, content_len+4+14);
	}
	#elif XBEE_TRANS
	tx_buffer[0]='h';
	memcpy(tx_buffer+1,data,36);
	tx_buffer[37]='\r';
	tx_buffer[38]='\n';
	HAL_UART_Transmit_DMA(&huart2, tx_buffer, 39);
	#endif
}
void xbee_motionAccAcquire(vec3f_t *motion_acc)
{
	xQueuePeek(motion_acc_q, motion_acc, 0);
}
void xbee_calibrationAcquire(calib_t *cal)
{
	xQueuePeek(cal_q, cal, 0);
}
void xbee_commandBlockingAcquire(command_t *cmd)
{
	xQueuePeek(command_q, cmd, portMAX_DELAY);
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
