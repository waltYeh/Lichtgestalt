#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "../config/config.h"
//#include "log.h"
//#include "param.h"
#include "attitude_estimator.h"
#include "controller.h"
#include "motor_mixer.h"
#include "sensors_task.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
//#include "sensors.h"
//#include "commander.h"
//#include "sitaw.h"
//#include "controller.h"
//#include "power_distribution.h"
//static bool isInit;
static marg_t marg;
static state_t state;
static output_t output;
static setpoint_t setpoint;
/*

static sensorData_t sensorData;

static control_t control;
*/
float mag[3],acc[3],gyr[3];//debugging

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
/*  if(isInit)
    return;

//sensorsInit();
//stateEstimatorInit();
//stateControllerInit();
//powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, "stabilizer",
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;*/
	xTaskCreate(stabilizerTask, "stabilizer",
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
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
static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
//  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
//  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
//  while(!sensorsAreCalibrated()) {
//    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
//  }

	while(1) {
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
		margAcquire(&marg);
		for(int i=0;i<3;i++){
			mag[i] = marg.mag.v[i];
			acc[i] = marg.acc.v[i];
			gyr[i] = marg.gyr.v[i];
		}
		stateEstimator(&state, &marg, tick);
		stateController(&output, &state, &setpoint, tick);
		powerDistribution(&output);
/*
#ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorUpdate(&state, &sensorData, &control);
#else
    sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, &sensorData, tick);
#endif

    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    stateController(&control, &sensorData, &state, &setpoint, tick);
    powerDistribution(&control);
*/
    tick++;
  }
}
