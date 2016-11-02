#include "stm32f4xx_hal.h"
#include "motor_pwm.h"
TIM_HandleTypeDef htim2;
//RCC_PeriphCLKInitTypeDef RCC_InitStruct;
GPIO_InitTypeDef GPIO_InitStruct;


void MX_TIM2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void motor_init(void)
{
	MX_TIM2_Init();
}
/* TIM2 init function */
void MX_TIM2_Init(void)
{
/*	RCC_InitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
//@ref RCCEx_Periph_Clock_Selection	
	RCC_InitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV2;
//@ref RCC_RTC_Clock_Source
	HAL_RCCEx_PeriphCLKConfig(&RCC_InitStruct);
*/	
/*	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	*/
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
	
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  HAL_TIM_Base_Init(&htim2);
	HAL_TIM_PWM_Init(&htim2);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_MspPostInit(&htim2);

	
	__HAL_TIM_SET_AUTORELOAD(&htim2, 4000);//4ms
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 2500);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

}
