#ifndef RECEIVER_H
#define RECEIVER_H
#include "stm32f4xx_hal.h"
#include "../Modules/stabilizer_types.h"
#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00
void rcBlockingAcquire(rc_t *rc);
void rcAcquire(rc_t *rc);
void receiver_init(void);
void sbus_init(void);
void sbusCallback(void);
void ppm_init(void);
void ppmCallback(GPIO_PinState state);
void sbus_IDLE(void);
#endif
