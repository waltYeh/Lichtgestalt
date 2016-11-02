#ifndef PLATFORM_H
#define PLATFORM_H

/******Board Version*****/
#define BOARD_V5 1
#define BOARD_V4 0

/******Air Frame*****/
#define AIRFRAME_F450 0
#define AIRFRAME_XINSONG 0
#define AIRFRAME_F330 0
#define AIRFRAME_F240 1

#if AIRFRAME_F450
	#define THRCMDRATIO 1
	#define VEHICLE_MASS 1300//g
	#define ROTOR_DIST 450//mm
	#define D2_SQRT2 636//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15//torq=c*f
	#define CROSS 1
#elif AIRFRAME_XINSONG
	#define THRCMDRATIO 1
	#define VEHICLE_MASS 1200//g
	#define ROTOR_DIST 450//mm
	#define D2_SQRT2 636//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15//torq=c*f
	#define CROSS 1
#elif AIRFRAME_F330
	#define THRCMDRATIO 1
	#define VEHICLE_MASS 1100//actually 920g
	#define ROTOR_DIST 330//mm
	#define D2_SQRT2 467//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15
	#define CROSS 1
#elif AIRFRAME_F240
	#define THRCMDRATIO 1
	#define VEHICLE_MASS 720
	#define ROTOR_DIST 240//mm
	#define D2_SQRT2 339//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15
	#define PLUS 1
#endif
/******Sensors Used*****/
#define IMU_MPU6000 1
#define IMU_ANALOG 0

#define GPS_M8N 0
#define GPS_LEA6H 1

#define COMPASS_HMC 1
#define COMPASS_MPU 0

#define BARO_I2C 1
#define BARO_SPI 0

/******Driver Options*****/
#define USB_TEST 0
#define I2C_CONTI 0
#define PPM_STORE 1
#define PWM16 1

/******Algorithm Options*****/
#define ATT_OLD 0
#define ATT_NEW 0
#define ATT_MADGWICK 1
#define ATT_ORIGINAL_MAD 0
#define MAG_PITCH_ROLL 0

/******Flight Modes*****/
#define INDOOR 0
#define OUTDOOR 1
#define ON_FLIGHT 1
#define OFF_FLIGHT 0
#define WAIT_GPS 0
#define OFFBOARD_AVAIL 0

#define FREQ_1000HZ 1

#endif
