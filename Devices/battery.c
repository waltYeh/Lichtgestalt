#include "battery.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Modules/stabilizer_types.h"
extern ADC_HandleTypeDef hadc1;
//unsigned int bat_volt = 0;
static xQueueHandle bat_q;
battery_t bat;
static void vAdcTask(void *pvParameters);
void battery_init(void)
{
	bat_q = xQueueCreate(1, sizeof(battery_t));
	xTaskCreate( vAdcTask, "Adc", configMINIMAL_STACK_SIZE, NULL, ADC_TASK_PRI, NULL );  
	battery_meas_start();
}
void battery_meas_start(void)
{
	HAL_ADC_Start(&hadc1);
	
}
unsigned int battery_get_voltage(void)
{
	unsigned int val; 
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)){
    val = HAL_ADC_GetValue(&hadc1) * 23 / 20;
		//considering the voltage separation for battery
	}
//	bat_volt = val;
	return val;
}
void vAdcTask(void *pvParameters)
{
	
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 400;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
	{  
    vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  
		HAL_ADC_PollForConversion(&hadc1, 50);//less than 2us
	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		bat.voltage = battery_get_voltage();
		xQueueOverwrite(bat_q, &bat);
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
		battery_meas_start();
	}  
}
void batAcquire(battery_t *bat)
{
	xQueueReceive(bat_q, bat, 0);
}
