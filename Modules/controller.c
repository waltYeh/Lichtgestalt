#include "controller.h"
#include "arm_math.h"
#include "string.h"
#include <math.h>
#include "../Commons/utils.h"
/*PID_t rollPID;
PID_t pitchPID;
PID_t yawPID;*/
PID_t pitchPID={0,0,0,0,
			1.6,
			16.0,0.8,0.0,
			1000};
PID_t rollPID={0,0,0,0,
			1.6,
			16.0,0.8,0.0,
			1000};
PID_t yawPID={0,0,0,0,
			0.75,
			10.0,0.0,0.0,
			1000};
float external_err_pid(PID_t *pid, float err)
{
 	 int retValue;								
 	 pid->Err = err;
 	 retValue = pid->P * err;
	 return retValue;
}
float internal_err_pid(PID_t *pid, float rate_err)
{
   	float dt = 1.0f / (float)pid->loop_rate;
	float rateOuput;								
  	pid->RateErr = rate_err;
  	rateOuput = pid->Prate * rate_err + pid->Drate * (rate_err - pid->l_RateErr) / dt + pid->Irate * pid->int_RateErr;
  	pid->l_RateErr = rate_err;
	pid->int_RateErr += rate_err * dt;
  	return rateOuput;
}


void reset_pid(PID_t *pid)
{
	pid->int_RateErr = 0.0f;
}
void init_pid(PID_t *pid, float P, float Prate, float Irate, float Drate, uint32_t loop_rate)
{
	pid->P = P;
	pid->Prate = Prate;
	pid->Irate = Irate;
	pid->Drate = Drate;
	pid->loop_rate = loop_rate;
	pid->Err = 0.0f;
	pid->RateErr = 0.0f;
	pid->l_RateErr = 0.0f;
	pid->int_RateErr = 0.0f;
}

void stateController(output_t *output, 
	const stateAtt_t *state,const setpoint_t *setpoint)
{
	float p_e_R_hat[3][3];//measured body to desired body
	float p_R_est[3][3];
	float p_R_sp[3][3];
	float p_R_est_t[3][3];
	float p_R_sp_t[3][3];
	float p_temp1[3][3];
	float p_temp2[3][3];
	float p_temp3[3][3];
	float e_R[3], e_w[3];
	float w_sp[3];
	memcpy(p_R_est,state->R.R,sizeof(p_R_est));
	memcpy(p_R_sp,setpoint->R.R,sizeof(p_R_sp));
	arm_matrix_instance_f32 e_R_hat = {3,3,(float *)p_e_R_hat};
	arm_matrix_instance_f32 R_est = {3,3,(float *)p_R_est};
	arm_matrix_instance_f32 R_sp = {3,3,(float *)p_R_sp};
	arm_matrix_instance_f32 R_est_t = {3,3,(float *)p_R_est_t};
	arm_matrix_instance_f32 R_sp_t = {3,3,(float *)p_R_sp_t};
	arm_matrix_instance_f32 temp1 = {3,3,(float *)p_temp1};
	arm_matrix_instance_f32 temp2 = {3,3,(float *)p_temp2};
	arm_matrix_instance_f32 temp3 = {3,3,(float *)p_temp3};
//	e_R_hat = (R_sp.transposed() * R - R.transposed() * R_sp) * (0.5f);
	arm_mat_trans_f32(&R_est, &R_est_t);
	arm_mat_trans_f32(&R_sp, &R_sp_t);
	arm_mat_mult_f32(&R_sp_t,&R_est,&temp1);
	arm_mat_mult_f32(&R_est_t,&R_sp,&temp2);
	arm_mat_sub_f32(&temp1,&temp2,&temp3);
	arm_mat_scale_f32(&temp3,0.5f,&e_R_hat);
	e_R[0] = (p_e_R_hat[1][2] - p_e_R_hat[2][1]) / 2.0f;//roll err in rad<<14
	e_R[1] = (p_e_R_hat[2][0] - p_e_R_hat[0][2]) / 2.0f;//pitch err in rad<<14
	e_R[2] = (p_e_R_hat[0][1] - p_e_R_hat[1][0]) / 2.0f;//yaw err in rad<<14
	w_sp[0] = external_err_pid(&rollPID, e_R[0]);//RollRate_sp
	w_sp[1] = external_err_pid(&pitchPID, e_R[1]);//PitchRate_sp
	w_sp[2] = external_err_pid(&yawPID, e_R[2]);//YawRate_sp
	e_w[0] = w_sp[0] - state->rate.R;
	e_w[1] = w_sp[1] - state->rate.P;
	e_w[2] = w_sp[2] - state->rate.Y;
	output->moment.R = internal_err_pid(&rollPID, e_w[0]);
	output->moment.P = internal_err_pid(&pitchPID, e_w[1]);
	output->moment.Y = internal_err_pid(&yawPID, e_w[2]);
	output->thrust = setpoint->thrust;
}

