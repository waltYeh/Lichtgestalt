#include <stdbool.h>
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../config/config.h"
#include "system.h"
#include "../MessageTypes/type_methods.h"
/* Private variable */
//static bool selftestPassed;
//static bool canFly;
mode_t g_mode;
statusLock_t g_statusLock;
statusFlight_t g_statusFlight;
statusRC_t g_statusRC;
statusGS_t g_statusGS;
static bool isInit;
void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

//  xSemaphoreTake(canStartMutex, portMAX_DELAY);
//  xSemaphoreGive(canStartMutex);
}
