/*
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "../config/config.h"
//#include "log.h"
//#include "param.h"
#include "../Devices/data_link.h"
#include "../Devices/battery.h"
#include "../Devices/receiver.h"
#include "attitude_estimator.h"
#include "controller.h"
#include "motor_mixer.h"
#include "sensors_task.h"
#include "stabilizer.h"
#include "commander.h"
#include "situation_awareness.h"
#include "stabilizer_types.h"
#include "sensors_calibration.h"
#include "../MathLib/attitude_lib.h"
#include "../Commons/platform.h"
#include "../Devices/led.h"
#include "../Devices/motor_pwm.h"

//extern short data2send[18];
static marg_t marg;
static stateAtt_t state;
static output_t output;
static setpoint_t setpoint;
static vec3f_t motion_acc;
static vec3f_t euler_sp;
static battery_t bat={4000};

mode_t g_mode;
status_t g_status;
static void stabilizerTask(void* param);
static void stabilizerInitTask(void* param);
void stabilizerInit(void)
{
	unsigned int i,j;
	for(i=0;i<3;i++){
		marg.acc.v[i] = 0;
		marg.gyr.v[i] = 0;
		marg.mag.v[i] = 0;
		state.Euler.v[i] = 0;
		state.rate.v[i] = 0;
		output.moment.v[i] = 0;
		motion_acc.v[i] = 0;
		for(j=0;j<3;j++){
			state.R.R[i][j] = 0;
			setpoint.R.R[i][j] = 0;
		}
		state.R.R[i][i] = 1.0f;
		setpoint.R.R[i][i] = 1.0f;
	}
	output.thrust = 0;
	setpoint.thrust = 0;
	marg.mag_updated = false;
	motor_cut();
	if(g_mode == modeAtt){
		xTaskCreate(stabilizerInitTask, "stabilizerInit", STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
	}
	else if(g_mode == modeCal){
		calibration_manager_init();
	}
}
bool stabilizerTest(void)
{
	bool pass = true;
	if(output.thrust > 50.0f){
		pass = false;
	}
	return pass;
}
void stabilizerReady2Fly(void)
{
//	while(stabilizerTest()==false);
	xTaskCreate(stabilizerTask, "stabilizer", STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
}


static void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while(1) {
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));
		margAcquire(&marg);
		xbee_motionAccAcquire(&motion_acc);
		stateEstimator(&state, &marg, &motion_acc, 1.0f/RATE_1000_HZ);
		setpointAcquire(&setpoint);
		rotation2euler(&setpoint.R, &euler_sp);
		situAwareUpdate(&setpoint, &marg, &state);
		stateController(&output, &state, &setpoint, 1.0f/RATE_1000_HZ);
		batAcquire(&bat);
		powerDistribution(&output, &bat);

		for(int i=0;i<3;i++){
			data2send[i] = state.Euler.v[i]*573.0f;
			data2send[i+3] = state.rate.v[i]*573.0f;
			data2send[i+6] = state.Euler.v[i]*573.0f;
			data2send[i+9] = euler_sp.v[i]*573.0f;
		//	data2send[i+12] = output.moment.v[i]*573.0f;
		//	data2send[i+15] = output.moment.v[i]*573.0f;
		}
		if(g_status == motorUnlocking){
			if(stabilizerTest())
				g_status = motorUnlocked;
		}
		if(g_status == motorLocked)
			setLed(0, 500, 1000);
		else
			setLed(0, 200, 500);
		if(bat.voltage < BAT_WARNING)
			setLed(1, 300, 1000);
		tick++;	
	}
}
*/
