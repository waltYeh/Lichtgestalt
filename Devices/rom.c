#include "hmc5883l_i2c.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Modules/stabilizer_types.h"
//sensor bias, transp or api, xbee coord and self address
//agent addr
extern I2C_HandleTypeDef hi2c2;
