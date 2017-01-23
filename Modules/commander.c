#include "commander.h"
#include "../MessageTypes/type_methods.h"
#include "../Mathlib/comparison.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Devices/data_link.h"
#include "../Devices/receiver.h"
#include "../Commons/platform.h"
static xQueueHandle setpoint_q;
static attsp_t setpoint;
static vec3f_t euler_sp;
//float euler_setpoint[3];
#if CMD_XBEE
static command_t command;
#else
static rc_t rc;
//short rc_int[16];
#endif
static void commanderTask( void *pvParameters ) ;
void commanderInit(void)
{
	setpoint_q = xQueueCreate(1,sizeof(attsp_t));
	xTaskCreate(commanderTask, "cmdTask", CMD_TASK_STACKSIZE, NULL, CMD_TASK_PRI, NULL);
}
void xbee_commands2setpoint(attsp_t* sp, const command_t* cmd)
{
	quaternion2rotation(&cmd->Q, &sp->R);
	sp->thrust = cmd->thrust;
}

void rc_channel2setpoint(attsp_t* sp, const rc_t* rc, float dt)
{
	float yaw_moverate;
	float thrust;
	float throttle;//0~1.0f
//	for(int i=0;i<16;i++){
//		rc_int[i] = rc->channels[i];
//	}
	
	euler_sp.P = rc->channels[1] * MAX_ATT_MANUEL / 1024.0f;
	euler_sp.R = -rc->channels[0] * MAX_ATT_MANUEL / 1024.0f;
	yaw_moverate = dead_zone_f(rc->channels[3] * MAX_YAW_RATE_MANEUL / 1024.0f, YAWRATE_DEADZONE);
	euler_sp.Y += yaw_moverate * dt;		
	throttle = dead_zone_f((rc->channels[2] + 1024)/2048.0f, 0.05);
	thrust = VEHICLE_MASS * GRAVITY * throttle * 2.0f;
//	euler_setpoint[0] = euler_sp.R;
//	euler_setpoint[1] = euler_sp.P;
//	euler_setpoint[2] = euler_sp.Y;
	euler2rotation(&euler_sp, &sp->R);
	sp->thrust = thrust;
}
void commanderTask( void *pvParameters )
{
#if CMD_PPM
	float dt;
	uint32_t lastWakeTime, currWakeTime;
	lastWakeTime = xTaskGetTickCount ();
#endif
	for(;;){
#if CMD_XBEE
		xbee_commandBlockingAcquire(&command);
		xbee_commands2setpoint(&setpoint, &command);
#else
		rcBlockingAcquire(&rc);
		currWakeTime = xTaskGetTickCount ();
		dt = (float)(currWakeTime - lastWakeTime) / (float)configTICK_RATE_HZ;
		if(dt > 0.2f)
			dt = 0.2f;
		/*
		for(int i=0;i<3;i++){
			data2send[i+6] = rc.channels[i];//state.Euler.v[i]*573.0f;
			data2send[i+9] = rc.channels[i+3];
		//	data2send[i+12] = _marg.acc.v[i];
		//	data2send[i+15] = _marg.gyr.v[i];
		}*/
		lastWakeTime = currWakeTime;
		rc_channel2setpoint(&setpoint, &rc, dt);
#endif
		xQueueOverwrite(setpoint_q, &setpoint);
	}
}

void attsp_reset(const att_t* state)
{
	euler_sp.P = state->Euler.P;
	euler_sp.R = state->Euler.R;
	euler_sp.Y = state->Euler.Y;
	euler2rotation(&euler_sp, &setpoint.R);
	setpoint.thrust = 0;
	xQueueOverwrite(setpoint_q, &setpoint);
}
void setpointAcquire(attsp_t* sp)
{
	xQueuePeek(setpoint_q, sp, 0);
}
