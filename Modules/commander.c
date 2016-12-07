#include "commander.h"
#include "../Commons/utils.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Devices/data_link.h"
static xQueueHandle setpoint_q;
static setpoint_t setpoint;
static command_t command;
static void commanderTask( void *pvParameters ) ;
void commanderTaskInit(void)
{
	setpoint_q = xQueueCreate(1,sizeof(setpoint_t));
	xTaskCreate(commanderTask, "cmdTask", CMD_TASK_STACKSIZE, NULL, CMD_TASK_PRI, NULL);
	
}
void commands2setpoint(setpoint_t* sp, const command_t* cmd)
{
	quaternion2rotation(&cmd->Q, &sp->R);
	sp->thrust = cmd->thrust;
}
void commanderTask( void *pvParameters )
{
	for(;;){
		commandBlockingAcquire(&command);
		commands2setpoint(&setpoint, &command);
	}
}


void setpointAcquire(setpoint_t* sp)
{
	xQueueReceive(setpoint_q, sp, 0);
}
