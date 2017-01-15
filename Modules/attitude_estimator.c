#include "stabilizer_types.h"
#include "attitude_estimator.h"
#include "../Commons/platform.h"
#include <math.h>
#include "arm_math.h"
#include "stm32f4xx.h"
#include "../MathLib/attitude_lib.h"
#include "../MathLib/type_math.h"
extern short data2send[9];

#if ATT_MADGWICK
stateAtt_t state_madg;
float R_trans[3][3] = {{0,1,0},{1,0,0},{0,0,-1}};
// System constants
//#define deltat 0.001f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError PI * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift PI * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta
// Global system variables
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
float rollspeed = 0, pitchspeed = 0, yawspeed = 0;
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error
// Function to compute one filter iteration
void margUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float deltat);
void imuUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float deltat);
void imuPredict(float w_x, float w_y, float w_z, float deltat);
#elif ATT_COMP
float w_mag = 0.4f;
float w_acc = 0.2f;
float w_gyr_bias = 0.1f;
float bias_max = 0.05f;
vec3f_t gyro_bias;
#endif
void stateEstimator(stateAtt_t *state, const marg_t *marg, const vec3f_t *mot_acc, float dt)
{
	vec3f_t w;
	int i;
	float gyro_scale = 7509.9f * 0.5f;
	for(i = 0; i<3; i++){
		w.v[i] = marg->gyr.v[i] / gyro_scale;
	}
#if ATT_COMP
	attitude_update(&w, mot_acc, marg, &gyro_bias, state, dt);
#elif ATT_MADGWICK
	if(marg->mag_updated){
	//	imuPredict(w[0], w[1], w[2]);
		margUpdate(w.v[0], w.v[1], w.v[2], marg->acc.x, marg->acc.y, marg->acc.z, marg->mag.x, marg->mag.y, marg->mag.z, dt);
	//	imuUpdate(w[0], w[1], w[2], marg->acc.x, marg->acc.y, marg->acc.z);
	}
	else{
	//	imuPredict(w[0], w[1], w[2]);
	//	margUpdate(w[0], w[1], w[2], marg->acc.x, marg->acc.y, marg->acc.z, marg->mag.x, marg->mag.y, marg->mag.z);
		imuUpdate(w.v[0], w.v[1], w.v[2], marg->acc.x, marg->acc.y, marg->acc.z, dt);
	}
//	for(i=0;i<3;i++){
//	data2send[6] = w_bx;
//	data2send[7] = w_by;
//	data2send[8] = w_bz;
//	}
	state_madg.Q.q0 = SEq_1;
	state_madg.Q.q1 = SEq_2;
	state_madg.Q.q2 = SEq_3;
	state_madg.Q.q3 = SEq_4;
	state_madg.rate.x = rollspeed;
	state_madg.rate.y = pitchspeed;
	state_madg.rate.z = yawspeed;
	quaternion2rotation(&state_madg.Q, &state_madg.R);
	
	arm_matrix_instance_f32 arm_R_madg = {3,3,(float *)state_madg.R.R};
	arm_matrix_instance_f32 arm_R = {3,3,(float *)state->R.R};
	arm_matrix_instance_f32 arm_R_trans = {3,3,(float *)R_trans};
	arm_mat_mult_f32(&arm_R_trans,&arm_R_madg,&arm_R);
//	rotation2quaternion(&state_madg.R, &state_madg.Q);
	rotation2euler(&state->R, &state->Euler);
	state->rate.x = state_madg.rate.x;
	state->rate.y = state_madg.rate.y;
	state->rate.z = state_madg.rate.z;
#endif
}

void quarternion_init(const vec3f_t* acc, const vec3f_t* mag, stateAtt_t* stateAtt)
{
	float xh,yh;
	float pitch,roll,yaw; 
	for(int i = 0; i<3; i++)
		gyro_bias.v[i] = 0;
//	float q0,q1,q2,q3;
	pitch = -data_2_angle(acc->x,acc->y,acc->z);//rad
	roll = data_2_angle(acc->y,acc->x,acc->z); //rad
	xh = mag->y*arm_cos_f32(roll)+mag->x*arm_sin_f32(roll)*arm_sin_f32(pitch)-mag->z*arm_cos_f32(pitch)*arm_sin_f32(roll);		 
	yh = mag->x*arm_cos_f32(pitch)+mag->z*arm_sin_f32(pitch);
	yaw = -atan2(xh,yh)+1.57f;
	if(yaw < -PI )
		yaw = yaw + 2*PI;
	if(yaw > PI )
		yaw = yaw - 2*PI;
	stateAtt->Euler.R = roll;
	stateAtt->Euler.P = pitch;
	stateAtt->Euler.Y = yaw;
	stateAtt->rate.R = 0;
	stateAtt->rate.P = 0;
	stateAtt->rate.Y = 0;
	euler2quaternion(&stateAtt->Euler, &stateAtt->Q);
	quaternion2rotation(&stateAtt->Q, &stateAtt->R);
	/*
	q0 = (cos(0.5f*roll)*cos(0.5f*pitch)*cos(0.5f*yaw) + sin(0.5f*roll)*sin(0.5f*pitch)*sin(0.5f*yaw));  
	q1 = (sin(0.5f*roll)*cos(0.5f*pitch)*cos(0.5f*yaw) - cos(0.5f*roll)*sin(0.5f*pitch)*sin(0.5f*yaw));    
	q2 = (cos(0.5f*roll)*sin(0.5f*pitch)*cos(0.5f*yaw) + sin(0.5f*roll)*cos(0.5f*pitch)*sin(0.5f*yaw)); 
	q3 = (cos(0.5f*roll)*cos(0.5f*pitch)*sin(0.5f*yaw) - sin(0.5f*roll)*sin(0.5f*pitch)*cos(0.5f*yaw));  
	stateAtt->Q.q0 = q0;
	stateAtt->Q.q1 = q1;
	stateAtt->Q.q2 = q2;
	stateAtt->Q.q3 = q3;
	
	stateAtt->R.R[0][0]=1.0f - ((q2 * q2 + q3 * q3) * 2.0f);
	stateAtt->R.R[0][1]=(q1 * q2 - q0 * q3) * 2.0f;
	stateAtt->R.R[0][2]=(q1 * q3 + q0 * q2) * 2.0f;
	stateAtt->R.R[1][0]=(q1 * q2 + q0 * q3) * 2.0f;
	stateAtt->R.R[1][1]=1.0f - ((q1 * q1 + q3 * q3) * 2.0f);
	stateAtt->R.R[1][2]=(q2 * q3 - q0 * q1) * 2.0f;
	stateAtt->R.R[2][0]=(q1 * q3 - q0 * q2) * 2.0f;
	stateAtt->R.R[2][1]=(q2 * q3 + q0 * q1) * 2.0f;
	stateAtt->R.R[2][2]=1.0f - ((q1 * q1 + q2 * q2) * 2.0f);
	*/
	#if ATT_MADGWICK
	arm_matrix_instance_f32 arm_R_madg = {3,3,(float *)state_madg.R.R};
	arm_matrix_instance_f32 arm_R = {3,3,(float *)stateAtt->R.R};
	arm_matrix_instance_f32 arm_R_trans = {3,3,(float *)R_trans};
	arm_mat_mult_f32(&arm_R_trans,&arm_R,&arm_R_madg);
	rotation2quaternion(&state_madg.R, &state_madg.Q);
	SEq_1 = state_madg.Q.q0;
	SEq_2 = state_madg.Q.q1;
	SEq_3 = state_madg.Q.q2;
	SEq_4 = state_madg.Q.q3;
	#endif
}
#if ATT_COMP
void attitude_update(const vec3f_t *gyr, const vec3f_t *pos_acc, const marg_t *marg, vec3f_t *gyr_bias, stateAtt_t *state, float dt)
{
	vec3f_t corr;
	vec3f_t k;
	vec3f_t grav_acc;
	vec3f_t acc_err;
//	vec3f_t rates;
	quaternion_t derQ;
	int i;
	float spinRate = vec3f_length(gyr);
	for(i=0;i<3;i++)
		corr.v[i] = 0;
	if(marg->mag_updated){
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;
		vec3f_t mag_err_earth;
		vec3f_t mag_err_body;
		vec3f_t mag_earth;
		body2glob(&state->R, &marg->mag, &mag_earth, 3);
		mag_err_earth.z = atan2f(mag_earth.x, mag_earth.y);
		mag_err_earth.x = 0;
		mag_err_earth.y = 0;
		if (spinRate > fifty_dps) 
			gainMult = minimum_f(spinRate / fifty_dps, 10.0f);
		glob2body(&state->R, &mag_err_earth, &mag_err_body, 3);
		for(i=0;i<3;i++)
			corr.v[i] += mag_err_body.v[i] * gainMult * w_mag;
	}
	quaternion_normalize(&state->Q);
	if(1){
		k.x = 2.0f * (state->Q.q1 * state->Q.q3 - state->Q.q0 * state->Q.q2);
		k.y = 2.0f * (state->Q.q2 * state->Q.q3 + state->Q.q0 * state->Q.q1);
		k.z = 1.0f - ((state->Q.q1 * state->Q.q1 + state->Q.q2 * state->Q.q2) * 2.0f);
		for(i=0;i<3;i++)
			grav_acc.v[i] = marg->acc.v[i]-pos_acc->v[i];
		vec3f_normalize(&grav_acc);
		vec3f_cross(&grav_acc, &k, &acc_err);
		for(i=0;i<3;i++)
			corr.v[i] += acc_err.v[i] * w_acc;
	}
	if (spinRate < 0.175f) {
		for(i=0;i<3;i++)
			gyr_bias->v[i] += corr.v[i] * (w_gyr_bias * dt);
		for (int i = 0; i < 3; i++) 
			gyr_bias->v[i] = constrain_f(gyr_bias->v[i], -bias_max, bias_max);
	}
	for (int i = 0; i < 3; i++) 
		state->rate.v[i] = gyr->v[i] + gyr_bias->v[i];
	for (int i = 0; i < 3; i++) 
		corr.v[i] += state->rate.v[i];
	quaternion_derivative(&state->Q, &derQ, &corr);
	for (int i = 0; i < 4; i++) 
		state->Q.q[i] += derQ.q[i] * dt;
	quaternion_normalize(&state->Q);
	quaternion2rotation(&state->Q, &state->R);
	rotation2euler(&state->R, &state->Euler);
}
#elif ATT_MADGWICK
/***Frame different****/

void margUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float deltat)
{
	// local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
	J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z; // computed flux in the earth frame
	// axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	float twoSEq_4 = 2.0f * SEq_4;
	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;
	float twob_xSEq_1 = 2.0f * b_x * SEq_1;
	float twob_xSEq_2 = 2.0f * b_x * SEq_2;
	float twob_xSEq_3 = 2.0f * b_x * SEq_3;
	float twob_xSEq_4 = 2.0f * b_x * SEq_4;
	float twob_zSEq_1 = 2.0f * b_z * SEq_1;
	float twob_zSEq_2 = 2.0f * b_z * SEq_2;
	float twob_zSEq_3 = 2.0f * b_z * SEq_3;
	float twob_zSEq_4 = 2.0f * b_z * SEq_4;
	float SEq_1SEq_2;
	float SEq_1SEq_3 = SEq_1 * SEq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = SEq_2 * SEq_4;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;
	// normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// normalise the magnetometer measurement
	norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x /= norm;
	m_y /= norm;
	m_z /= norm;
	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	J_41 = twob_zSEq_3; // negated in matrix multiplication
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;
	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
	// normalise the gradient to estimate direction of the gyroscope error
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 = SEqHatDot_1 / norm;
	SEqHatDot_2 = SEqHatDot_2 / norm;
	SEqHatDot_3 = SEqHatDot_3 / norm;
	SEqHatDot_4 = SEqHatDot_4 / norm;
	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
	// compute and remove the gyroscope baises
	w_bx += w_err_x * deltat * zeta;
	w_by += w_err_y * deltat * zeta;
	w_bz += w_err_z * deltat * zeta;
	w_x -= w_bx;
	w_y -= w_by;
	w_z -= w_bz;
	rollspeed = w_x;
	pitchspeed = w_y;
	yawspeed = w_z;
	// compute the quaternion rate measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// compute then integrate the estimated quaternion rate
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
	// compute flux in the earth frame
	SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
	SEq_1SEq_3 = SEq_1 * SEq_3;
	SEq_1SEq_4 = SEq_1 * SEq_4;
	SEq_3SEq_4 = SEq_3 * SEq_4;
	SEq_2SEq_3 = SEq_2 * SEq_3;
	SEq_2SEq_4 = SEq_2 * SEq_4;
	h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
	// normalise the flux vector to have only components in the x and z
	b_x = sqrt((h_x * h_x) + (h_y * h_y));
	b_z = h_z;
}

void imuUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float deltat)
{
	// Local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	float twoSEq_4 = 2.0f * SEq_4;
	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;
	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
	// compute and remove the gyroscope baises
	w_bx += w_err_x * deltat * zeta;
	w_by += w_err_y * deltat * zeta;
	w_bz += w_err_z * deltat * zeta;
	w_x -= w_bx;
	w_y -= w_by;
	w_z -= w_bz;
	rollspeed = w_x;
	pitchspeed = w_y;
	yawspeed = w_z;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
}
void imuPredict(float w_x, float w_y, float w_z, float deltat)
{
	// Local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;

	// Normalise the accelerometer measurement
	w_x -= w_bx;
	w_y -= w_by;
	w_z -= w_bz;
	rollspeed = w_x;
	pitchspeed = w_y;
	yawspeed = w_z;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
}
#endif
