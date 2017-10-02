#ifndef TYPES_H
#define TYPES_H
#pragma anon_unions
#include <stdint.h>
#include <stdbool.h>
typedef struct {
  unsigned int timestamp; // Timestamp when the data was computed
	union {
    struct {
      float x;
      float y;
      float z;
    };
    struct {
      float v[3];
    };
  };
}V3_f;
typedef struct  {
  unsigned int timestamp; // Timestamp when the data was computed
	union {
    struct {
      int x;
      int y;
      int z;
    };
    struct {
      int v[3];
    };
  };
}V3_i;
//typedef struct vec3_f point_t;
//typedef struct vec3_f velocity_t;
//typedef struct vec3_f acc_t;

/* Orientation as a quaternion */
typedef struct  {
  unsigned int timestamp;
  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float q[4];
    };
  };
}Q_f;
typedef struct {
  unsigned int timestamp;
  union {
    struct {
      int x;
      int y;
      int z;
      int w;
    };
    struct {
      int q[4];
    };
  };
}Q_i ;
typedef struct  {
  unsigned int timestamp;
  union {
    struct {
      float R[3][3];
    };
		struct {
      float R9[9];
    };
  };
}R_f;
typedef struct  {
  unsigned int timestamp;
  union {
    struct {
      int R[3][3];
    };
		struct {
      int R9[9];
    };
  };
}R_i;
/*
struct {
	int lat;
	int lon;
	int alt;
	int speed;
	float azm; //azimuth, rad
	short sat;
	short status;
	int year;
	unsigned char month;
	unsigned char date;
	unsigned int time;
	int x;
	int y;
	int z;
	int vx;
	int vy;
	unsigned char v_valid;
	char gpsflag;
	#define GPS_PERIOD 200
	#define GPS_DELAY 400
}_gps;
*/
/*
typedef struct gpsRaw_s {
	uint32_t timestamp;
	int64_t lat;
	int64_t lon;
	float azm;
	float vel;
	bool vel_valid;
	float alt;
	uint8_t sat;
	uint32_t date;
	uint32_t time;
	float eph;
	enum statusGPS_e {
		noGPS = 0,
		realtimeGPS = 1,
		difGPS = 2
	}status;
} gpsRaw_t;
*/
struct _vicon{
	int x;//mm
	int y;
	int z;
	int vx;//mm/s
	int vy;
	int vz;
	char viconflag;
	#define VICON_PERIOD 100//ms
	#define VICON_DELAY 120
};

struct _sens {
	short gx;
	short gy;
	short gz;
	short ax;
	short ay;
	short az;
	short mx;
	short my;
	short mz;
	unsigned char mag_trig;	 //triggered by process250HzB, 
	//in MPU_I2C mode, cleared after gyro twi finishes, and starts mag measurement
	//in MPU_SPI mode, cleared in process250HzB, and starts mag measurement
	unsigned char mag_updated;
//	unsigned char spi_ready;
};
struct _baro {
	int temp;
	int pressure;
	int refPressure;	 
	int alt;
	char temp_pres_switch;
	unsigned long D1;
	unsigned long D2;
	unsigned char updated;
	#define PRES_SWITCH 0
	#define TEMP_SWITCH 1
	#define BARO_STEPS 4
	#define D1_UPDATED 1
	#define D2_UPDATED 2
};


struct _att {
	
	float roll;
	float pitch;
	float yaw;
	
	float rollspeed;
	float pitchspeed;
	float yawspeed;
	float R[3][3];//glob=R*body//1 ~ 2^14 ~ 16384
	float q[4];
	#define DSCRT_14 0
	#define DSCRT_15 1
	
	#if DSCRT_15
		#define DSCRT 15
		#define DSCRT_F 32768.0f
		#define DSCRT_I 32768
	#elif DSCRT_14
		#define DSCRT 14
		#define DSCRT_F 16384.0f
		#define DSCRT_I 16384
	#endif
	//when discrete is 14, the min detectable angular rate
	//at 500Hz for attitude is 229 units or 1.75 deg/s
	//max discrete can be 15 for int variables, min is 0.875 deg/s
};





struct _pos {	 //unit mm
	float x_est[2];
	float y_est[2];
	float z_est[2];
	float Acc_x;
	float Acc_y;
	float Acc_z;
};

struct _cmd {
	short rc[9];//-1024 ~ +1024
	int throttle;//0~1024
	char SonarEnable;
	char data2send;
	#define SonarOFF 0
	#define SonarON 1
	#define sendNON 0
	#define sendSENS 1
	#define sendGPS 2
	#define sendATT 3
	#define sendPOS 4
	#define sendPID 5
	#define sendCMD 6
	#define sendOUT 7
	#define sendUSB 8
	#define sendROS 9
	#define KILL 11
};

struct _ctrl {
	
	float pitch_sp;
	float roll_sp;
	float yaw_sp;	
			
	float pos_x_sp;
	float pos_y_sp;
	float pos_z_sp;
	
	float vel_x_sp;
	float vel_y_sp;
	float vel_z_sp;
	
	float vel_x_ff;
	float vel_y_ff;
	float vel_z_ff;
	
};

struct _output {
	float thrustForce;//U1=F1+F2+F3+F4, 1 mNewton
	float pitchMmt;//U1=(F3-F1)L
	float rollMmt;//U2=(F2-F4)L
	float yawMmt;//U4=M2+M4-M1-M3, 1 mNewton*mm
};

struct _mode {
	char FlightMode;
	char CalibrationMode;
	char l_FlightMode;
	char l_CalMode;
	char offboard;
	char locked;
	#define MANUEL 1
	#define ACROBATIC 0
	#define ALT_CTRL 2	
	#define POS_CTRL 4
	#define RASP_MANUEL 5
	#define RASP_ALT 6
	#define RASP_POS 7
	#define RASP_NURBS 8
	#define MOTOR_CUT 9
};


struct _adc {
	unsigned int battery;//in mV
};
struct _smpl{
#define ATT_PERIOD 2
#define POS_CTRL_PERIOD 4
#define RADIO_PERIOD 20
	unsigned int error_code;
	char Flag500Hz;
	char tx_finished;
	unsigned char d_ctrl_disable;
	char sens_rdy;
};
struct _comm {
	#define USB_PLUG_IN 1
	#define USB_PLUG_OUT 2
	#define USB_PLUG_CLEAR 0
	#define USB_TIMEOUT_MS 500
	char usb_connect_flag;
	char usb_connected;
	short usb_data_got[3];
	char usb_out_coming;
	char usb_out_read_flag;
	short usb_rcv_timeout;
	
};

extern short data2[9];
typedef struct _PID {
	int Err;
	int RateErr;
	int l_RateErr;
	int int_RateErr;
	float P;		
	float Prate;
	float Irate;
	float Drate;
}PID;
extern PID rollPID;
extern PID pitchPID;
extern PID yawPID;
extern PID altPID;
extern PID pos_xPID;
extern PID pos_yPID;


#if DSCRT_15
	#define MAX_ATT_MANUEL 0.698f//28874 40deg,0.698rad
	#define MAX_YAW_RATE 0.698f//28606 50.0
	#define MAX_YAW_RATE_MANEUL 0.698f
#elif DSCRT_14
	#define MAX_ATT_MANUEL 0.698f//11437 40deg,0.698rad
	#define MAX_YAW_RATE 0.698f//14303 50.0
	#define MAX_YAW_RATE_MANEUL 0.698f
#endif

#define MAX_ALT_VEL 1.0f//in loop
#define MAX_ALT_VEL_MANUEL 0.5f//manuel
#define MAX_XY_VEL 8.0f
#define MAX_XY_VEL_MANUEL 1.5f

//#define BAT_WARNING 3500

#endif
