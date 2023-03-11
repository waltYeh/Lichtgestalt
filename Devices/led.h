#ifndef LED_H
#define LED_H
#include <stdbool.h>
#include "stm32f4xx_hal.h"
void led_init(void);
void but_init(void);
bool check_butt(void);
void setLed(unsigned char index, unsigned int duty, unsigned int period);

#define LED1_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define LED1_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET) 

#define LED2_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET) 
#define LED2_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)

#define LED3_ON()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) 
#define LED3_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)

#endif
