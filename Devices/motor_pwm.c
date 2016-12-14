#include "stm32f4xx_hal.h"
#include "motor_pwm.h"
extern TIM_HandleTypeDef htim3;
//RCC_PeriphCLKInitTypeDef RCC_InitStruct;
//GPIO_InitTypeDef GPIO_InitStruct;


void MX_TIM2_Init(void);
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void motor_init(void)
{
//	MX_TIM2_Init();
	__HAL_TIM_SET_AUTORELOAD(&htim3, 4000);//4ms
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}
void motor_pwm_output(const unsigned short duty[4])
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty[0]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty[1]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty[2]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty[3]);
}
