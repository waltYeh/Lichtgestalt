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

static void stabilizerInitTask(void* param)
{
	//task killed when static time is enough, quarternion_init before quit
	//record only after stable
//	uint32_t tick = 0;
	#define STD_BLOCK_LEN 25
	#define AVERAGE_SAMPLES 150//2sec
	#define ACC_STEADY_STD 50.0f
	#define GYR_STEADY_STD 50.0f
	#define MAG_STEADY_STD 50.0f
	int i;
	unsigned int p_block = 0;
	bool state_steady = false;
	unsigned int usable_data_cnt = 0;
	float acc_block[3][STD_BLOCK_LEN];
	float gyr_block[3][STD_BLOCK_LEN];
	float mag_block[3][STD_BLOCK_LEN];
	vec3f_t avr_acc = {0,0,0};
	vec3f_t avr_gyr = {0,0,0};
	vec3f_t avr_mag = {0,0,0};
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while(1){
		margAcquire(&marg);
		if(marg.mag_updated){
			p_block++;
			if(p_block == STD_BLOCK_LEN)
				p_block = 0;
			state_steady = true;
			for(i = 0; i<3; i++){
				acc_block[i][p_block] = marg.acc.v[i];
				gyr_block[i][p_block] = marg.gyr.v[i];
				mag_block[i][p_block] = marg.mag.v[i];
				state_steady &= judge_steady(acc_block[i], STD_BLOCK_LEN, ACC_STEADY_STD);
				state_steady &= judge_steady(gyr_block[i], STD_BLOCK_LEN, GYR_STEADY_STD);
				state_steady &= judge_steady(mag_block[i], STD_BLOCK_LEN, MAG_STEADY_STD);
			}
			if(state_steady){
				setLed(0, 1000, 1000);
				for(i = 0; i<3; i++){
					avr_acc.v[i] += marg.acc.v[i];
					avr_gyr.v[i] += marg.gyr.v[i];
					avr_mag.v[i] += marg.mag.v[i];
				}
				usable_data_cnt++;
			}
			else{
				setLed(0, 0, 1000);
				for(i = 0; i<3; i++){
					avr_acc.v[i] = 0;
					avr_gyr.v[i] = 0;
					avr_mag.v[i] = 0;
				}
				usable_data_cnt = 0;
			}
			if(usable_data_cnt == AVERAGE_SAMPLES){
				setLed(0, 0, 1000);
				for(i = 0; i<3; i++){
					avr_acc.v[i] /= AVERAGE_SAMPLES;
					avr_gyr.v[i] /= AVERAGE_SAMPLES;
					avr_mag.v[i] /= AVERAGE_SAMPLES;
				}
				quarternion_init(&avr_acc, &avr_mag, &state);
				gyro_calibrate(&avr_gyr);
				euler_sp_reset(&state);
				stabilizerReady2Fly();
				data_send_start();
				vTaskDelete(NULL);
			}
		}
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));
	}
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
