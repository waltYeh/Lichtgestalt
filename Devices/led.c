#include "led.h"
#include "../Modules/stabilizer_types.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../config/config.h"
#define LED1 PC3
#define LED2 PC14
#define LED3 PB15
#define BUTT PB2

#define LED1_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define LED1_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET) 

#define LED2_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET) 
#define LED2_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)

#define LED3_ON()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) 
#define LED3_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)

bool check_butt(void)
{	
	GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	if(state == GPIO_PIN_RESET)//pressed down
		return true;
	else
		return false;
}

static void vLedTask(void *pvParameters);
void led_init(void)
{
	xTaskCreate( vLedTask, "LED", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRI, NULL );  
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	
}
void but_init(void)
{
	
	
}
void vLedTask(void *pvParameters)
{
//	bool butt_state = false;
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 20;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
	{  
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
//		butt_state = check_butt();
/*		if(butt_state)
			LED1_ON();
		else
			LED1_OFF();
		*/
	}  
}
