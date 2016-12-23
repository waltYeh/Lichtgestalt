#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__
#pragma anon_unions
#include <stdint.h>
#include <stdbool.h>
//#include "../../Devices/imu_types.h"

/* Orientation as a quaternion */
typedef struct quaternion_s {
//  uint32_t timestamp;
  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
	float q[4];
  };
} quaternion_t;

typedef struct rotation_s {
//  uint32_t timestamp;  // Timestamp when the data was computed
	float R[3][3];
} rotation_t;

/* x,y,z vector */
struct vec3f_s {
//  uint32_t timestamp; // Timestamp when the data was computed
	union {
    struct {
      float R;
      float P;
      float Y;
    };
    struct {
      float x;
      float y;
      float z;
    };
		float v[3];
  };
};
typedef struct vec3f_s vec3f_t;
//typedef struct vec3f_s moment_t;
//typedef struct vec3f_s euler_t;
//typedef struct vec3f_s angularRate_t;
struct vec3i16_s {
//	uint32_t timestamp;
	union {
   struct {
         int16_t x;
         int16_t y;
         int16_t z;
   };
   int16_t v[3];
 };
};
//typedef struct vec3i16_s acc_raw_t;
//typedef struct vec3i16_s gyr_raw_t;
//typedef struct vec3i16_s mag_raw_t;
typedef struct vec3i16_s vec3i16_t;

typedef struct margData_s {
	vec3f_t acc;
	vec3f_t gyr;
	vec3f_t mag;
	bool mag_updated;
} marg_t;//used in queue

typedef struct rc_s {
	int channels[16];
} rc_t;

typedef struct battery_s {
  uint16_t voltage;
} battery_t;//used in queue

typedef struct stateAtt_s {
	vec3f_t Euler;
	quaternion_t Q;
	rotation_t R;
	vec3f_t rate;
} stateAtt_t;//used in queue

typedef struct command_s {
	quaternion_t Q;
	float thrust;
} command_t;//used in queue

typedef struct setpoint_s {
	rotation_t R;
	float thrust;
} setpoint_t;//used in queue

typedef struct output_s {
	vec3f_t moment;
	float thrust;
} output_t;//used in queue

typedef struct agent_s {
	unsigned int coord_addr_h;
	unsigned int coord_addr_l;
	unsigned int self_addr_h;
	unsigned int self_addr_l;
	unsigned int agent_addr;
} agent_t;

typedef struct calib_s {
	vec3i16_t acc_bias;
	vec3i16_t gyr_bias;	
	vec3i16_t mag_bias;
} calib_t;

typedef enum mode_e {
  modeCal = 0,
  modeAtt,
  modeRate
} mode_t;
extern mode_t g_mode;
typedef struct PID_s {
	float Err;
	float RateErr;
	float l_RateErr;
	float int_RateErr;
	float P;		
	float Prate;
	float Irate;
	float Drate;
	uint32_t loop_rate;
}PID_t;
//extern PID_t rollPID;
//extern PID_t pitchPID;
//extern PID_t yawPID;
//extern PID_t altPID;
//extern PID_t pos_xPID;
//extern PID_t pos_yPID;
#endif
