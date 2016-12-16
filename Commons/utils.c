#include "utils.h"
#include <math.h>
#include "arm_math.h"
#include "../Modules/stabilizer_types.h"
void body2glob(const rotation_t* R, const vec3f_t* body, vec3f_t* glob, short dimension)
{	
	if(dimension == 2){
		float yaw = -atan2(R->R[0][1], R->R[1][1]);
		glob->x = body->x*arm_cos_f32(yaw) + body->y*arm_sin_f32(-yaw);
		glob->y = body->x*arm_sin_f32(yaw) + body->y*arm_cos_f32(yaw); 
	}
	else if(dimension == 3){
		glob->x = (body->x*R->R[0][0] + body->y*R->R[0][1] + body->z*R->R[0][2]);
		glob->y = (body->x*R->R[1][0] + body->y*R->R[1][1] + body->z*R->R[1][2]);
		glob->z = (body->x*R->R[2][0] + body->y*R->R[2][1] + body->z*R->R[2][2]);
	}
}
void glob2body(const rotation_t* R, const vec3f_t* glob, vec3f_t* body, short dimension)//body=inv(R)*glob
{	
	if(dimension == 2){
		float yaw = -atan2(R->R[0][1], R->R[1][1]);
		body->x = glob->x*arm_cos_f32(yaw) + glob->y*arm_sin_f32(yaw);
		body->y = glob->x*arm_sin_f32(-yaw) + glob->y*arm_cos_f32(yaw); 
	}
	else if(dimension == 3){
		body->x = (glob->x*R->R[0][0] + glob->y*R->R[1][0] + glob->z*R->R[2][0]);
		body->y = (glob->x*R->R[0][1] + glob->y*R->R[1][1] + glob->z*R->R[2][1]);
		body->z = (glob->x*R->R[0][2] + glob->y*R->R[1][2] + glob->z*R->R[2][2]);
	}
}
void quaternion2rotation(const quaternion_t* Q, rotation_t* R)
{
	float q0,q1,q2,q3;
	q0 = Q->q0;
	q1 = Q->q1;
	q2 = Q->q2;
	q3 = Q->q3;
	R->R[0][0]=1.0f - ((q2 * q2 + q3 * q3) * 2.0f);
	R->R[0][1]=(q1 * q2 - q0 * q3) * 2.0f;
	R->R[0][2]=(q1 * q3 + q0 * q2) * 2.0f;
	R->R[1][0]=(q1 * q2 + q0 * q3) * 2.0f;
	R->R[1][1]=1.0f - ((q1 * q1 + q3 * q3) * 2.0f);
	R->R[1][2]=(q2 * q3 - q0 * q1) * 2.0f;
	R->R[2][0]=(q1 * q3 - q0 * q2) * 2.0f;
	R->R[2][1]=(q2 * q3 + q0 * q1) * 2.0f;
	R->R[2][2]=1.0f - ((q1 * q1 + q2 * q2) * 2.0f);
}
void rotation2euler(const rotation_t* R, vec3f_t* Euler)
{
	Euler->P = -asin(R->R[2][0]);
	Euler->R  = atan2(R->R[2][1], R->R[2][2]);
	Euler->Y = -atan2(R->R[0][1], R->R[1][1]);
}
void euler2rotation(const vec3f_t* Euler, rotation_t* R)
{
	float cp = arm_cos_f32(Euler->P);
	float sp = arm_sin_f32(Euler->P);
	float sr = arm_sin_f32(Euler->R);
	float cr = arm_cos_f32(Euler->R);
	float sy = arm_sin_f32(Euler->Y);
	float cy = arm_cos_f32(Euler->Y);
	R->R[0][0] = cp * cy;
	R->R[0][1] = ((sr * sp * cy) - (cr * sy));
	R->R[0][2] = ((cr * sp * cy) + (sr * sy));
	R->R[1][0] = cp * sy;
	R->R[1][1] = ((sr * sp * sy) + (cr * cy));
	R->R[1][2] = ((cr * sp * sy) - (sr * cy));
	R->R[2][0] = -sp;
	R->R[2][1] = sr * cp;
	R->R[2][2] = cr * cp;	
}
bool judge_steady(float* block, unsigned int len, float threshold)
{
	float std;
	arm_std_f32(block,len,&std);
	if(std < threshold)
		return true;
	else
		return false;
}
