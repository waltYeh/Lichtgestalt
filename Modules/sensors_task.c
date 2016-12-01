#include "../Devices/hmc5883l_i2c.h"
#include "../Devices/mpu6000_spi.h"
#include "low_pass_filter.h"
#include "sensors_task.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "../config/config.h"
#include "stabilizer_types.h"

uint8_t acc_gyr_spi_rx[15];
uint8_t mag_i2c_rx[6];
	
vec3i16_t acc_raw;
vec3i16_t gyr_raw;	
vec3i16_t mag_raw;
	
marg_t marg_data;

//static xQueueHandle acc_raw_q;
//static xQueueHandle gyr_raw_q;
//static xQueueHandle mag_raw_q;
static xQueueHandle marg_q;
static xSemaphoreHandle imuDataReady;
static bool magDataReady;
IIRFilter iir_ax={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_ay={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_az={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
	
static void sensorsTrigerTask( void *pvParameters ) ;
static void sensorsProcessTask( void *pvParameters ) ;
void acc_process(vec3i16_t* input, vec3f_t* output);
void gyr_process(vec3i16_t* input, vec3f_t* output);	
void mag_process(vec3i16_t* input, vec3f_t* output);
				
void imu_IIR_init(void)
{	
	float cutoff_freq = 40.0;//, cutoff_freq2 = 40.0;
	float smpl_freq = 500.0;
	float dummy;
	IIR_set_cutoff_freq(&iir_ax, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_ay, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_az, 10.0, smpl_freq);
	dummy = IIR_reset(&iir_ax, 0);
	dummy = IIR_reset(&iir_ay, 0);
	dummy = IIR_reset(&iir_az, 8192);
	dummy = dummy;
/*	IIR_set_cutoff_freq(&iir_gx, cutoff_freq2, smpl_freq);
	IIR_set_cutoff_freq(&iir_gy, cutoff_freq2, smpl_freq);
	IIR_set_cutoff_freq(&iir_gz, cutoff_freq2, smpl_freq);
	sens.gx = IIR_reset(&iir_gx, 0);
	sens.gy = IIR_reset(&iir_gy, 0);
	sens.gz = IIR_reset(&iir_gz, 0);
	*/
}
void sensorsTaskInit(void)
{
//  acc_raw_q = xQueueCreate(1, sizeof(acc_raw_t));
//  gyr_raw_q = xQueueCreate(1, sizeof(gyr_raw_t));
	marg_q = xQueueCreate(1, sizeof(marg_t));
	imu_IIR_init();
  
	vSemaphoreCreateBinary( imuDataReady );
//	vSemaphoreCreateBinary( magDataReady );
	xTaskCreate(sensorsTrigerTask, "sensTrigTask", SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
	xTaskCreate(sensorsProcessTask, "sensProcessTask", SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
}
void sensorsTrigerTask( void *pvParameters )
{
	uint32_t mag_cnt = 0;
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1;//1ms
	xLastWakeTime = xTaskGetTickCount();
	for( ;; ){  
		mpu6000_dma_start(acc_gyr_spi_rx, 15);
		mag_cnt ++;
		if(mag_cnt == 14){
			mag_cnt = 0;
			hmc5883l_dma_start(mag_i2c_rx, 6);
		}
		vTaskDelayUntil(&xLastWakeTime, timeIncreament); 
	}  
}
void sensorsProcessTask( void *pvParameters )
{
	for( ;; ){  
		if (pdTRUE == xSemaphoreTake(imuDataReady, portMAX_DELAY)){
			int i;
//			vec3i16_t acc_raw;
//			vec3i16_t gyr_raw;
			for (i=0; i<3; i++){
				acc_raw.v[i] = ((short)acc_gyr_spi_rx[2*i+1]<<8)|(short)acc_gyr_spi_rx[2*i+2];
			}
			for (i=0; i<3; i++){
				gyr_raw.v[i] = ((short)acc_gyr_spi_rx[2*i+9]<<8)|(short)acc_gyr_spi_rx[2*i+10];
			}
			acc_raw.timestamp = xTaskGetTickCount();
			gyr_raw.timestamp = xTaskGetTickCount();
			acc_process(&acc_raw, &(marg_data.acc));
			gyr_process(&gyr_raw, &(marg_data.gyr));
			
	//		xQueueOverwrite(acc_raw_q, &acc_raw);
	//		xQueueOverwrite(gyr_raw_q, &gyr_raw);
			if (magDataReady == true){
				magDataReady = false;
				int i;
			//	mag_raw_t mag_raw;
				for (i=0; i<3; i++){
					mag_raw.v[i] = ((short)mag_i2c_rx[2*i]<<8)|(short)mag_i2c_rx[2*i+1];
				}
				mag_raw.timestamp = xTaskGetTickCount();
				mag_process(&mag_raw, &(marg_data.mag));
				marg_data.mag_updated = true;
			//	xQueueOverwrite(mag_raw_q, &mag_raw);
			}
			xQueueOverwrite(marg_q, &marg_data);
			marg_data.mag_updated = false;
		}
	}  
}

void acc_process(vec3i16_t* input, vec3f_t* output)
{
	//swap, bias, lpf
	output->v[0] = input->v[0];
	output->v[1] = input->v[1];
	output->v[2] = input->v[2];
}
void gyr_process(vec3i16_t* input, vec3f_t* output)
{
	//swap, bias, lpf
	output->v[0] = input->v[0];
	output->v[1] = input->v[1];
	output->v[2] = input->v[2];
}
void mag_process(vec3i16_t* input, vec3f_t* output)
{
	//swap, bias
	output->v[0] = input->v[0];
	output->v[1] = input->v[1];
	output->v[2] = input->v[2];
}
void mpu6000Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(imuDataReady, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken)
		portYIELD();
}

void hmc5883lCallback(void)
{
	magDataReady = true;
}





void margAcquire(marg_t *marg)
{
	xQueueReceive(marg_q, marg, 0);
}
