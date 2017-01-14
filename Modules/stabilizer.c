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
//#include "sensors.h"
//#include "commander.h"
//#include "sitaw.h"
//#include "controller.h"
//#include "power_distribution.h"
//static bool isInit = false;

extern short data2send[18];
static marg_t marg;
static stateAtt_t state;
static output_t output;
static setpoint_t setpoint;
static vec3f_t motion_acc;
//static rc_t rc;
static battery_t bat={4000};
/*
static sensorData_t sensorData;
static control_t control;
*/
//float mag[3],acc[3],gyr[3];//debugging
//short bat_v;
short rc_v[8];
mode_t g_mode;
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
	}
	output.thrust = 0;
	setpoint.thrust = 0;
	marg.mag_updated = false;
	motor_cut();
	if(g_mode == modeAtt){
		xTaskCreate(stabilizerInitTask, "stabilizerInit",
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
	}
	else if(g_mode == modeCal){
		calibration_manager_init();
		
	}
	/*  if(isInit)
    return;

//stateControllerInit();
//powerDistributionInit();
  isInit = true;*/

}
bool stabilizerTest(void)
{
  bool pass = true;

//  pass &= sensorsTest();
  //pass &= stateEstimatorTest();
  //pass &= stateControllerTest();
  //pass &= powerDistributionTest();

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
//	#define ACC_STEADY_STD 0.1f
//	#define GYR_STEADY_STD 0.05f
//	#define MAG_STEADY_STD 0.1f
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
				LED1_ON();
				for(i = 0; i<3; i++){
					avr_acc.v[i] += marg.acc.v[i];
					avr_gyr.v[i] += marg.gyr.v[i];
					avr_mag.v[i] += marg.mag.v[i];
				}
				usable_data_cnt++;
			}
			else{
				LED1_OFF();
				for(i = 0; i<3; i++){
					avr_acc.v[i] = 0;
					avr_gyr.v[i] = 0;
					avr_mag.v[i] = 0;
				}
				usable_data_cnt = 0;
			}
			if(usable_data_cnt == AVERAGE_SAMPLES){
				LED1_OFF();
				for(i = 0; i<3; i++){
					avr_acc.v[i] /= AVERAGE_SAMPLES;
					avr_gyr.v[i] /= AVERAGE_SAMPLES;
					avr_mag.v[i] /= AVERAGE_SAMPLES;
				}
				quarternion_init(&avr_acc, &avr_mag, &state);
				gyro_calibrate(&avr_gyr);
				stabilizerReady2Fly();
				data_send_start();
//				isInit = true;
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
		for(int i=0;i<3;i++){
			data2send[i] = state.Euler.v[i]*573.0f;
			data2send[i+3] = state.rate.v[i]*573.0f;
			data2send[i+6] = marg.mag_updated;
			data2send[i+9] = marg.mag.v[i];
			data2send[i+12] = marg.acc.v[i];
			data2send[i+15] = marg.gyr.v[i];
		}
		setpointAcquire(&setpoint);
		situAwareUpdate(&setpoint, &marg, &state);
		stateController(&output, &state, &setpoint, 1.0f/RATE_1000_HZ);
		batAcquire(&bat);
		
		powerDistribution(&output, &bat);

		tick++;
		if(tick==500){
			LED1_ON();
			if(bat.voltage < BAT_WARNING)
				LED2_ON();
			if(g_mode == modeCal)
				LED3_ON();
		}
		if(tick==1000){
			LED1_OFF();
			LED2_OFF();
			LED3_OFF();
			tick=0;
		}
	}
}
