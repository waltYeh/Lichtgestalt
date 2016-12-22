#ifndef UTILS_H
#define UTILS_H
#include "../Modules/stabilizer_types.h"
//#define cos arm_cos_f32
//#define sin arm_sin_f32
static int constrain_int32(int a, int b, int c){
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
static float constrain_f(float a, float b, float c){
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
static int dead_zone_int32(int a, int b){
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}
static int dead_zone_f(float a, float b){
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}

static int minimum_int32(int a, int b){
	return (a>b?b:a);
}
static int minimum_f(float a, float b){
	return (a>b?b:a);
}
static int maximum_int32(int a, int b){
	return (a>b?a:b);
}
static int maximum_f(float a, float b){
	return (a>b?a:b);
}
float data_2_angle(float x, float y, float z);
void body2glob(const rotation_t* R, const vec3f_t* body, vec3f_t* glob, short dimension);
void glob2body(const rotation_t* R, const vec3f_t* glob, vec3f_t* body, short dimension);//body=inv(R)*glob;
void quaternion2rotation(const quaternion_t* Q, rotation_t* R);
void rotation2euler(const rotation_t* R, vec3f_t* Euler);
void rotation2quaternion(const rotation_t* R, quaternion_t* Q);
void euler2rotation(const vec3f_t* Euler, rotation_t* R);
void euler2quaternion(const vec3f_t* Euler, quaternion_t* Q);

bool judge_steady(float* block, unsigned int len, float threshold);
void quaternion_normalize(quaternion_t* Q);
void vec3f_normalize(vec3f_t * v);
void vec3f_cross(const vec3f_t * a, const vec3f_t * b, vec3f_t * d);

void quaternion_derivative(const quaternion_t* Q, quaternion_t* derQ, const vec3f_t* w);
float vec3f_length(const vec3f_t * v);
#define GRAVITY_M_S2 9.81f
/*
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}
*/
#endif
