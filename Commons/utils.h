#ifndef UTILS_H
#define UTILS_H
#include "../Modules/stabilizer_types.h"
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

void body2glob(const rotation_t* R, const vec3f_t* body, vec3f_t* glob, short dimension);
void glob2body(const rotation_t* R, const vec3f_t* glob, vec3f_t* body, short dimension);//body=inv(R)*glob;
void quaternion2rotation(const quaternion_t* Q, rotation_t* R);
void rotation2euler(const rotation_t* R, vec3f_t* Euler);
void euler2rotation(const vec3f_t* Euler, rotation_t* R);
bool judge_steady(float* block, unsigned int len, float threshold);

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
