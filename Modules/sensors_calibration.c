#include "sensors_calibration.h"
#include "../MathLib/calibration_lib.h"
#include <math.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "../config/config.h"
#include "sensors_task.h"
#include "stabilizer_types.h"
#include "../MathLib/attitude_lib.h"
#include "../MathLib/type_math.h"
#include "../Commons/platform.h"
#include "../Devices/led.h"
#include "../Devices/data_link.h"
#include "../Devices/rom.h"
extern short data2send[18];
static void magCalTask(void* param);
	float x[MAG_STORE];
	float y[MAG_STORE];
	float z[MAG_STORE];
void calibration_manager_init(void)
{
	xTaskCreate(magCalTask, "magCal",
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
}
static void magCalTask(void* param)
{
	float gyro_scale = 7509.9f * 0.5f;
	static marg_t marg;

	vec3f_t mag_bias;
	vec3i16_t mag_bias_i;
	float radius;
	unsigned int mag_store_pt = 0;
	uint32_t tick = 0;
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 200;
	xLastWakeTime = xTaskGetTickCount ();
	data_send_start();
	tick = 0;
	while(1) {
		
		margAcquire(&marg);
		for(int i=0;i<3;i++){
			data2send[i] = mag_bias.v[i];
			data2send[i+3] = marg.mag.v[i];
		}
		if(absolute_f(marg.gyr.x/gyro_scale)> ROTATION_THRES|| 
			absolute_f(marg.gyr.y/gyro_scale)> ROTATION_THRES|| 
			absolute_f(marg.gyr.z/gyro_scale)> ROTATION_THRES){
			//take this data down
			if(vec3f_length(&marg.mag)>MAG_LEN_MIN_THRES){
				x[mag_store_pt] = marg.mag.x;
				y[mag_store_pt] = marg.mag.y;
				z[mag_store_pt] = marg.mag.z;
				mag_store_pt++;
				if(mag_store_pt==MAG_STORE){
					mag_store_pt = 0;
					sphere_fit_least_squares(x, y, z,
						MAG_STORE,
						100, 0.0f,
						&mag_bias.x, &mag_bias.y, &mag_bias.z,
						&radius);
					mag_bias_i.x = mag_bias.x;
					mag_bias_i.y = mag_bias.y;
					mag_bias_i.z = mag_bias.z;
					rom_set_mag_bias(&mag_bias_i);
				}
			}
		}
		
		tick++;
		if(tick==2){
			if(mag_store_pt < 42)
				LED1_ON();
			else if(mag_store_pt < 85)
				LED2_ON();
			else
				LED3_ON();
		}
		if(tick==5){
			if(mag_store_pt < 42){
				LED1_OFF();
				LED2_OFF();
				LED3_OFF();
			}
			else if(mag_store_pt < 85){
				LED1_ON();
				LED2_OFF();
				LED3_OFF();
			}
			else{
				LED1_ON();
				LED2_ON();
				LED3_OFF();
			}
			tick=0;
		}
		vTaskDelayUntil(&xLastWakeTime, timeIncreament);
	}
	
}

