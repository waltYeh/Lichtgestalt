#include "type_methods.h"

float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void vec3f_normalize(vec3f_t * v)
{
	float inv_norm;
	inv_norm = inv_sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	v->x *= inv_norm;
	v->y *= inv_norm;
	v->z *= inv_norm;
}
void vec3f_cross(const vec3f_t * a, const vec3f_t * b, vec3f_t * d)
{
/*
a X b = | i		j		k	|
		| ax	ay		az	|
		| bx	by		bz	|
	where |.| is the determinant
*/	
	d->x = a->y * b->z - a->z * b->y;
	d->y = a->z * b->x - a->x * b->z;
	d->z = a->x * b->y - a->y * b->x;
}
float vec3f_dot(const vec3f_t * a, const vec3f_t * b)
{
	return (a->x * b->x + a->y * b->y + a->z * b->z);
}
vec3f_t vec3f_normalized(vec3f_t * v)
{
	vec3f_t ret;
	float inv_norm;
	inv_norm = inv_sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	ret.x = v->x * inv_norm;
	ret.y = v->y * inv_norm;
	ret.z = v->z * inv_norm;
	return ret;
}

float vec3f_length(const vec3f_t * v)
{
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}
void body2earth(const rotation_t* R, const vec3f_t* body, vec3f_t* earth, short dimension)
{	
	if(dimension == 2){
		float yaw = -atan2(R->R[0][1], R->R[1][1]);
		earth->x = body->x*arm_cos_f32(yaw) + body->y*arm_sin_f32(-yaw);
		earth->y = body->x*arm_sin_f32(yaw) + body->y*arm_cos_f32(yaw); 
	}
	else if(dimension == 3){
		earth->x = (body->x*R->R[0][0] + body->y*R->R[0][1] + body->z*R->R[0][2]);
		earth->y = (body->x*R->R[1][0] + body->y*R->R[1][1] + body->z*R->R[1][2]);
		earth->z = (body->x*R->R[2][0] + body->y*R->R[2][1] + body->z*R->R[2][2]);
	}
}
void earth2body(const rotation_t* R, const vec3f_t* earth, vec3f_t* body, short dimension)//body=inv(R)*earth
{	
	if(dimension == 2){
		float yaw = -atan2(R->R[0][1], R->R[1][1]);
		body->x = earth->x*arm_cos_f32(yaw) + earth->y*arm_sin_f32(yaw);
		body->y = earth->x*arm_sin_f32(-yaw) + earth->y*arm_cos_f32(yaw); 
	}
	else if(dimension == 3){
		body->x = (earth->x*R->R[0][0] + earth->y*R->R[1][0] + earth->z*R->R[2][0]);
		body->y = (earth->x*R->R[0][1] + earth->y*R->R[1][1] + earth->z*R->R[2][1]);
		body->z = (earth->x*R->R[0][2] + earth->y*R->R[1][2] + earth->z*R->R[2][2]);
	}
}
float data_2_angle(float x, float y, float z)	 //in rad
{
	float res;
	res = atan2(x,sqrtf(y*y+z*z));
	return res;
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
void euler2quaternion(const vec3f_t* Euler, quaternion_t* Q)
{
	float hr = Euler->R * 0.5f;
	float hp = Euler->P * 0.5f;
	float hy = Euler->Y * 0.5f;
	float q0,q1,q2,q3;
	q0 = (arm_cos_f32(hr)*arm_cos_f32(hp)*arm_cos_f32(hy) + arm_sin_f32(hr)*arm_sin_f32(hp)*arm_sin_f32(hy));  
	q1 = (arm_sin_f32(hr)*arm_cos_f32(hp)*arm_cos_f32(hy) - arm_cos_f32(hr)*arm_sin_f32(hp)*arm_sin_f32(hy));    
	q2 = (arm_cos_f32(hr)*arm_sin_f32(hp)*arm_cos_f32(hy) + arm_sin_f32(hr)*arm_cos_f32(hp)*arm_sin_f32(hy)); 
	q3 = (arm_cos_f32(hr)*arm_cos_f32(hp)*arm_sin_f32(hy) - arm_sin_f32(hr)*arm_sin_f32(hp)*arm_cos_f32(hy));
	Q->q0 = q0;
	Q->q1 = q1;
	Q->q2 = q2;
	Q->q3 = q3;
}
void rotation2quaternion(const rotation_t* R, quaternion_t* Q)
{
	float q[4];
	float tr = R->R[0][0] + R->R[1][1] + R->R[2][2];
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f);
		q[0] = s * 0.5f;
		s = 0.5f / s;
		q[1] = (R->R[2][1] - R->R[1][2]) * s;
		q[2] = (R->R[0][2] - R->R[2][0]) * s;
		q[3] = (R->R[1][0] - R->R[0][1]) * s;
	} 
	else {
		/* Find maximum diagonal element in dcm
		* store index in dcm_i */
		int dcm_i = 0;
		for (int i = 1; i < 3; i++) {
			if (R->R[i][i] > R->R[dcm_i][dcm_i]) {
				dcm_i = i;
			}
		}
		int dcm_j = (dcm_i + 1) % 3;
		int dcm_k = (dcm_i + 2) % 3;
		float s = sqrtf((R->R[dcm_i][dcm_i] - R->R[dcm_j][dcm_j] - R->R[dcm_k][dcm_k]) + 1.0f);
		q[dcm_i + 1] = s * 0.5f;
		s = 0.5f / s;
		q[dcm_j + 1] = (R->R[dcm_i][dcm_j] + R->R[dcm_j][dcm_i]) * s;
		q[dcm_k + 1] = (R->R[dcm_k][dcm_i] + R->R[dcm_i][dcm_k]) * s;
		q[0] = (R->R[dcm_k][dcm_j] - R->R[dcm_j][dcm_k]) * s;
	}
	for(int i = 0; i < 4; i++)
		Q->q[i] = q[i];
}

void quaternion_derivative(const quaternion_t* Q, quaternion_t* derQ, const vec3f_t* w)
{
	int i;
	float dataQ[] = {
		Q->q[0], -Q->q[1], -Q->q[2], -Q->q[3],
		Q->q[1],  Q->q[0], -Q->q[3],  Q->q[2],
		Q->q[2],  Q->q[3],  Q->q[0], -Q->q[1],
		Q->q[3], -Q->q[2],  Q->q[1],  Q->q[0]
	};
	float V[4] = {0,w->x,w->y,w->z};
	float Q_V[4];
	float result[4];
	arm_matrix_instance_f32 matQ = {4,4,(float *)dataQ};
	arm_matrix_instance_f32 matV = {4,1,(float *)V};
	arm_matrix_instance_f32 matQ_V = {4,1,(float *)Q_V};
	arm_matrix_instance_f32 matRes = {4,1,(float *)result};
	arm_mat_mult_f32(&matQ,&matV,&matQ_V);
	arm_mat_scale_f32(&matQ_V,0.5f,&matRes);
	for(i=0;i<4;i++){
		derQ->q[i] = result[i];
	}
}
void quaternion_normalize(quaternion_t* Q)
{
	float q0,q1,q2,q3;
	float inv_norm;
	q0 = Q->q0;
	q1 = Q->q1;
	q2 = Q->q2;
	q3 = Q->q3;
	inv_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= inv_norm;
	q1 *= inv_norm;
	q2 *= inv_norm;
	q3 *= inv_norm;
	Q->q0 = q0;
	Q->q1 = q1;
	Q->q2 = q2;
	Q->q3 = q3;
}
