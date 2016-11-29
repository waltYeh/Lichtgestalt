#include "GPS.h"
#include "../Modules/commons.h"
#include "../Major/platform.h"
#include "../Major/types.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#define GPS_BUFFER_SIZE 512
extern UART_HandleTypeDef huart1;
struct _gps gps;
unsigned char gps_buffer0[GPS_BUFFER_SIZE];//change bigger?
unsigned char gps_buffer1[GPS_BUFFER_SIZE];
unsigned int data_len;
unsigned char buffer_num=0;
//cleared at timeout, ++ during dma interrupt
static void GPSTask( void *pvParameters );


void GPSReceive_IDLE(void)  
{  
  //  uint32_t temp;  
  
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)){   
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);  
		HAL_UART_DMAStop(&huart1);  
       // temp = huart1.hdmarx->Instance->NDTR;  
		data_len =  GPS_BUFFER_SIZE - huart1.hdmarx->Instance->NDTR;             
		buffer_num = (!buffer_num) & 1;//either 0 or 1			
		if(buffer_num == 0){
			HAL_UART_Receive_DMA(&huart1,gps_buffer0,GPS_BUFFER_SIZE); 
		}
		else{
			HAL_UART_Receive_DMA(&huart1,gps_buffer1,GPS_BUFFER_SIZE); 
		}		
	}  
}  
void GPSInit(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	xTaskCreate( GPSTask, "GPS", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
	if(buffer_num == 0){
		HAL_UART_Receive_DMA(&huart1,gps_buffer0,GPS_BUFFER_SIZE); 
	}
	else{
		HAL_UART_Receive_DMA(&huart1,gps_buffer1,GPS_BUFFER_SIZE); 
	}		
}
void GPSTask( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1000;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
	{  
		vTaskDelayUntil( &xLastWakeTime, timeIncreament );  
	}  
}
float char_to_float(char *a)//transfer ascII code(with)
{
	float value=0.0;
	int i=0;
	int j=1;
	int sign=0;
	float decade=1;
	int point=-1;//if there is no decimal point, point==-1
	if(a[0]=='-')
		sign=1;	
	for(i=0;i<14;i++){ //find decimal position
	 	if(a[i+sign]=='.'){
		   point=i;
		   break;
		}
	}
	for(j=1;j<(point);j++){//find the decade for max bit
		decade=decade*10;
	}
	for(i=0;i<point;i++){		
	   value+=(a[i+sign]-48)*decade;
	   decade=decade*0.1f;
	}//now we have got the int part
	decade=0.1;

	for(i=point+1;i<14;i++){
	 	if(a[i+sign]!=0){
			value+=(a[i+sign]-48)*decade;
			decade=decade*0.1f;
		}
		else{
			break;
		}
	}
	if(sign)
		value=-value;
	return value;
}
void get_gps_data(void)//read the data we want in gps_buffer[160]
{
	char string[15];//to store ascII of a data between two ","
	int string_offset=0;
	int received_type=0;//the type of the data between two ","
	int frame=0;//GGA or RMC
	int gps_temp_data[2]={0,0};//to store unsigned LAT and LON, and wait for the NSEW letter
	unsigned char gps_in;
	unsigned int i,j=0;

	for(j=0;j<data_len;j++){
		if(buffer_num == 0){
			gps_in=gps_buffer0[j]; 
		}
		else{
			gps_in=gps_buffer1[j]; 
		}		
		
		if(gps_in=='$'){
			string_offset=0;
			received_type=0;
		}
		else if(gps_in==','){//deal with one string
			string_offset=0;		
			if(received_type==0){
			#if LEA6H_GPS
				if(string[0]=='G'&&string[1]=='P'&&string[2]=='G'
				&&string[3]=='G'&&string[4]=='A')
					frame=GPGGA;
				else if(string[0]=='G'&&string[1]=='P'&&string[2]=='R'
				&&string[3]=='M'&&string[4]=='C')
					frame=GPRMC;
				else 
				 	frame=OTH_FRM;
			#elif M8N_GPS
				if(string[0]=='G'&&string[1]=='N'&&string[2]=='G'
				&&string[3]=='G'&&string[4]=='A')
					frame=GPGGA;	
				else if(string[0]=='G'&&string[1]=='N'&&string[2]=='R'
				&&string[3]=='M'&&string[4]=='C')
					frame=GPRMC;
				else 
				 	frame=OTH_FRM;
			#endif
			}
			if(frame==OTH_FRM)
				continue;
			else if(frame==GPGGA){
				switch(received_type){
				case 6:
				//gps status: 0 not positioning, 1 non-diff positioning, 
				//2 diff positioning, 3 invalid PPS, 6 estimating
					gps.status=(string[0]-48);
					break;
				case 7://# of sat in use (00~12)
					gps.sat=(string[0]-48)*10+(string[1]-48);
					break;
				case 9://altitude -9,999.9 ~ 99,999.9
					gps.alt=char_to_float(string)*1000;//unit of mm
					break;
				default:
					break;
				}
			}
			else if(frame==GPRMC){
				switch(received_type){
				case 3:
				//latitude ddmm.mmmmm, 0.00001m correspond to 18mm
					gps_temp_data[LAT]=
					(string[0]-48)*100000000
					+(string[1]-48)*10000000
						+((string[2]-48)*10000000
						+(string[3]-48)*1000000
						+(string[5]-48)*100000
						+(string[6]-48)*10000
						+(string[7]-48)*1000
						+(string[8]-48)*100
						+(string[9]-48)*10)/6;//10^7deg
					break;
				case 4://latitude NS
					if(string[0]=='N'){
						gps.lat=gps_temp_data[LAT];
					}
					else if(string[0]=='S'){
						gps.lat=-gps_temp_data[LAT];
					}
					break;
				case 5://longitude dddmm.mmmmm
					gps_temp_data[LON]=
					(string[0]-48)*1000000000
					+(string[1]-48)*100000000
					+(string[2]-48)*10000000
						+((string[3]-48)*10000000
						+(string[4]-48)*1000000
						+(string[6]-48)*100000
						+(string[7]-48)*10000
						+(string[8]-48)*1000
						+(string[9]-48)*100
						+(string[10]-48)*10)/6;
					break;
				case 6://longitude EW			
					if(string[0]=='E'){
						gps.lon=gps_temp_data[LON];
					}
					else if(string[0]=='W'){
						gps.lon=-gps_temp_data[LON];
					}
					break;
				case 7://velocity Knots
					gps.vel=char_to_float(string)*514.4f;//1 knot = 0.5144m/s = 514.4mm/s
					break;
				case 8://azimuth deg			
					gps.azm=char_to_float(string) * DEG2RAD;
					break;
				default:
					break;
				}
			}
			received_type++;
			string_offset=0;
			for(i=0;i<15;i++){
				string[i]='0';
			}
		}
		else if(gps_in=='\r'||gps_in=='\n'){
		}
		else if(gps_in=='*'){
		}
		else{
			if(string_offset<15)
				string[string_offset]=gps_in;
			string_offset++;
		}
	}
}
