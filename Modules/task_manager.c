#include "../MessageTypes/type_methods.h"

#include "cmsis_os.h"
#include "../config/config.h"

#include "commander.h"
#include "sensors_task.h"
#include "attitude_estimator.h"
#include "attitude_controller.h"
#include "motor_mixer.h"
#include "sensors_calibration.h"
#include "task_manager.h"

#include "../Commons/platform.h"
#include "../MessageTypes/type_methods.h"
#include "../config/config.h"

#include "../Devices/data_link.h"
#include "../Devices/battery.h"
#include "../Devices/led.h"
#include "../Devices/rom.h"
#include "../Devices/motor_pwm.h"
#include "../Devices/mpu6000_spi.h"
#include "../Devices/hmc5883l_i2c.h"
#include "../Devices/receiver.h"
#include "../Devices/GPS.h"
mode_t g_mode;
statusLock_t g_statusLock;
statusFlight_t g_statusFlight;
statusRC_t g_statusRC;
statusGS_t g_statusGS;
/*
static marg_t _marg;
static att_t _att;
static output_t _output;
static attsp_t _setpoint;
static vec3f_t _motion_acc;
static vec3f_t _euler_sp;

static command_t _command;
*/
//static battery_t _bat;
static void managerTask(void* param);
//static void managerInitTask(void* param);
void taskManagerInit(void)
{
	if(check_butt()){
		g_mode = modeCal;
	}
	else{
		g_mode = modeAtt;
	}
	g_statusLock = motorLocked;
	motor_init();
	motor_cut();
	mpu6000_cfg();
	hmc5883l_cfg();
	mpu_fast_init();
	hmc_fast_init();
	receiver_init();
	led_init();
	data_link_init();
	data_send_start();
	GPSInit();
	eeprom_init();
	battery_init();
	sensorsTaskInit();
	
//	motor_mixer_init();
	
	if(g_mode == modeAtt){
		
		commanderInit();
		
		attitude_init();
		start_manager();
	}
	else if(g_mode == modeCal){
		calibration_manager_init();
		data_send_start();
	}
	
}
void attitude_initialized_callback(att_t * att)
{
	attsp_reset(att);
	attitude_estimator_start();
	attitude_controller_init();
	motor_mixer_init();
	
}
uint32_t self_check(void)
{
	
	return 0;
}
void start_manager(void)
{
	xTaskCreate(managerTask, "managerTask", MANAGER_TASK_STACKSIZE, NULL, MANAGER_TASK_PRI, NULL);
}
static void managerTask(void* param)
{
	TickType_t xLastWakeTime;
	unsigned int tick = 0, but_cnt = 0;
	const TickType_t timeIncreament = 20;
	
	
	
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
	{  
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
		tick ++;
		if(check_butt())
			but_cnt++;
		else
			but_cnt = 0;
		if(but_cnt == 25 && g_mode == modeAtt){
			if(g_statusLock == motorLocked)
				g_statusLock = motorUnlocking;
			else if(g_statusLock == motorUnlocked||g_statusLock == motorUnlocking)
				g_statusLock = motorLocked;
		}
		if(g_statusLock == motorUnlocking){
			if(self_check() == 0)
				g_statusLock = motorUnlocked;
		}
		if(g_statusLock == motorLocked)
			setLed(0, 500, 1000);
		else
			setLed(0, 200, 500);		
	}
}
