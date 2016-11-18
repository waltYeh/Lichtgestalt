#ifndef RECEIVER_PPM_H
#define RECEIVER_PPM_H
#include "stm32f4xx_hal.h"
void PPM_init(void);
void HAL_GPIO_EXTI_Callback(unsigned short GPIO_Pin);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
#endif
