#ifndef TYPE_MATH_H
#define TYPE_MATH_H
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
static float dead_zone_f(float a, float b){
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}

static int minimum_int32(int a, int b){
	return (a>b?b:a);
}
static float minimum_f(float a, float b){
	return (a>b?b:a);
}
static int maximum_int32(int a, int b){
	return (a>b?a:b);
}
static float maximum_f(float a, float b){
	return (a>b?a:b);
}
static float absolute_f(float a){
	return (a>0?a:-a);
}
static int absolute_int32(float a){
	return (a>0?a:-a);
}
float vec3f_length(const vec3f_t * v);
void vec3f_normalize(vec3f_t * v);
void vec3f_cross(const vec3f_t * a, const vec3f_t * b, vec3f_t * d);
float inv_sqrt(float x) ;
#endif
