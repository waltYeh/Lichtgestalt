#include "receiver_ppm.h"
#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim8;
short rc[9];
//int timePPM=0;
void PPM_init(void)
{
//	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC3);
	HAL_TIM_IC_Start_IT (&htim8, TIM_CHANNEL_3);
}
void PPM_decode(GPIO_PinState state, TIM_HandleTypeDef *htim)
{
	int timePPM=0;
//	static unsigned int l_time=0;
	static unsigned short channel=0;
	if(state){
//		timePPM = __HAL_TIM_GET_COUNTER(htim)/2;
		timePPM = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) / 2;//us
		if(timePPM >= 0 && timePPM <= 4000){
			if(channel < 9){
				rc[channel] = 244*(timePPM-1220)/100;
			}
			channel++;
		}
		else{
			channel = 0;
		}
	}
	else{
		__HAL_TIM_SET_COUNTER(htim, 0);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
  {
		PPM_decode(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8), htim);
	}
	if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC4) != RESET)
  {
		
	}
}
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){  
//		case GPIO_PIN_0:
//			PPM_decode(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0));
//			break;  
//		case GPIO_PIN_1:;break;  
		case GPIO_PIN_2:
			;
		break;  
//		case GPIO_PIN_12:;break;
//		case GPIO_PIN_13:;break; 
//		case GPIO_PIN_15:;break; 
		default:break;  
	}  
}*/

