#include "commander.h"
#include "../Commons/utils.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Devices/data_link.h"
#include "../Devices/receiver.h"
#include "../Commons/platform.h"
static xQueueHandle setpoint_q;
static setpoint_t setpoint;
static vec3f_t euler_sp;
#if CMD_XBEE
static command_t command;
#else
static rc_t rc;
#endif
static void commanderTask( void *pvParameters ) ;
void commanderTaskInit(void)
{
	setpoint_q = xQueueCreate(1,sizeof(setpoint_t));
	xTaskCreate(commanderTask, "cmdTask", CMD_TASK_STACKSIZE, NULL, CMD_TASK_PRI, NULL);
	
}
void xbee_commands2setpoint(setpoint_t* sp, const command_t* cmd)
{
	quaternion2rotation(&cmd->Q, &sp->R);
	sp->thrust = cmd->thrust;
}
void euler_sp_reset(float yaw)
{
	euler_sp.Y = yaw;
}
void sbus_channel2setpoint(setpoint_t* sp, const rc_t* rc, float dt)
{
	float yaw_moverate;
	float thrust;
	float throttle;//0~1.0f
	euler_sp.P = rc->channels[1] * MAX_ATT_MANUEL;
	euler_sp.R = -rc->channels[0] * MAX_ATT_MANUEL;
	yaw_moverate = dead_zone_f(rc->channels[3] * MAX_YAW_RATE_MANEUL, YAWRATE_DEADZONE);
	euler_sp.Y += yaw_moverate * dt;		
	throttle = dead_zone_f((rc->channels[2] + 1024)/2048.0f, 0.05);
	thrust = VEHICLE_MASS * GRAVITY_M_S2 * throttle * 2.0f;
	
	euler2rotation(&euler_sp, &sp->R);
	sp->thrust = thrust;
}
void commanderTask( void *pvParameters )
{

	float dt;
	uint32_t lastWakeTime, currWakeTime;
	lastWakeTime = xTaskGetTickCount ();

	for(;;){
#if CMD_XBEE
		xbee_commandBlockingAcquire(&command);
		xbee_commands2setpoint(&setpoint, &command);
#else
		rcBlockingAcquire(&rc);
		currWakeTime = xTaskGetTickCount ();
		dt = (float)(currWakeTime - lastWakeTime) / (float)configTICK_RATE_HZ;
		lastWakeTime = currWakeTime;
		sbus_channel2setpoint(&setpoint, &rc, dt);
#endif
		xQueueOverwrite(setpoint_q, &setpoint);
	}
}


void setpointAcquire(setpoint_t* sp)
{
	xQueueReceive(setpoint_q, sp, 0);
}
