#ifndef MESSAGES_H
#define MESSAGES_H
#include "basic_types.h"

typedef struct margData_s {
	vec3f_t acc;
	vec3f_t gyr;
	vec3f_t mag;
	bool mag_updated;
} marg_t;
typedef struct calib_s {
	vec3i16_t acc_bias;
	vec3i16_t gyr_bias;	
	vec3i16_t mag_bias;
} calib_t;
typedef struct baro_s {
	float pressure;
	float temp;
	float alt;
} baro_t;
typedef struct gpsRaw_s {
	int64_t lat;
	int64_t lon;
	float azm;
	float vel;
	bool vel_valid;
	float alt;
	uint8_t sat;
	uint32_t date;
	uint32_t time;
	enum statusGPS_e {
		noGPS = 0,
		realtimeGPS = 1,
		difGPS = 2
	}status;
} gpsRaw_t;
typedef struct gpsProcessed_s {
	vec3f_t pos;
	vec3f_t vel;
	vec3f_t acc;
} gpsProcessed_t;
typedef struct rc_s {
	float channels[16];
} rc_t;
typedef struct battery_s {
	uint16_t voltage;
} battery_t;
typedef struct att_s {
	vec3f_t Euler;
	quaternion_t Q;
	rotation_t R;
	vec3f_t rate;
} att_t;
typedef struct pos_s {
	vec3f_t pos;
	vec3f_t vel;
	vec3f_t acc;
} pos_t;
typedef struct command_s {
	quaternion_t Q;
	float thrust;
} command_t;
typedef struct attsp_s {
	rotation_t R;
	float thrust;
	float yaw_ff;
} attsp_t;
typedef struct possp_s {
	vec3f_t pos_sp;
	vec3f_t vel_ff;
	vec3f_t acc_ff;
	float yaw_sp;
} posCtrlsp_t;
typedef struct altCtrlsp_s {
	float pos_sp_z;
	float vel_ff_z;
	vec3f_t euler;
} altCtrlsp_t;
typedef struct manuelCtrlsp_s {
	float throttle;
	vec3f_t euler;
} manCtrlsp_t;
typedef struct generalCtrlsp_s {
	float throttle;
	vec3f_t euler;
	vec3f_t pos_sp;
	vec3f_t vel_ff;
	vec3f_t acc_ff;
} generalCtrlsp_t;
typedef struct output_s {
	vec3f_t moment;
	float thrust;
} output_t;
typedef struct motors_s {
	float thrust[8];
} motors_t;
typedef struct agent_s {
	unsigned int coord_addr_h;
	unsigned int coord_addr_l;
	unsigned int self_addr_h;
	unsigned int self_addr_l;
	unsigned int agent_addr;
} agent_t;
typedef enum modeVehicle_e {
	modeCal = 0,
	modeRate,
	modeAtt,
	modeAlt,
	modePos,
	modeTrj
} mode_t;
typedef enum statusLock_e {
	motorLocked = 0,
	motorUnlocking,
	motorUnlocked,
} statusLock_t;
typedef enum statusFlight_e {
	stautsLanded = 0,
	stautsFlying,
	stautsFreefall,
	stautsUpsidedown,
	stautsTumble
} statusFlight_t;
typedef enum statusGS_e {
	gs_normal = 0,

} statusGS_t;
typedef enum statusRC_e {
	rc_normal = 0,
} statusRC_t;
extern mode_t g_mode;
extern statusLock_t g_statusLock;
extern statusFlight_t g_statusFlight;
extern statusRC_t g_statusRC;
extern statusGS_t g_statusGS;
extern short data2send[18];
#endif
