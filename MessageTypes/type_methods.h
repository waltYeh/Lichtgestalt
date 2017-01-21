#ifndef TYPE_METHODS_H
#define TYPE_METHODS_H
#include "messages.h"
#include "basic_types.h"
#include "arm_math.h"
#include <math.h>
float inv_sqrt(float x);

void vec3f_normalize(vec3f_t * v);

void vec3f_cross(const vec3f_t * a, const vec3f_t * b, vec3f_t * d);
float vec3f_dot(const vec3f_t * a, const vec3f_t * b);
vec3f_t vec3f_normalized(vec3f_t * v);

float vec3f_length(const vec3f_t * v);

void body2earth(const rotation_t* R, const vec3f_t* body, vec3f_t* earth, short dimension);

void earth2body(const rotation_t* R, const vec3f_t* earth, vec3f_t* body, short dimension);//body=inv(R)*earth

float data_2_angle(float x, float y, float z);	 //in rad

void quaternion2rotation(const quaternion_t* Q, rotation_t* R);

void rotation2euler(const rotation_t* R, vec3f_t* Euler);

void euler2rotation(const vec3f_t* Euler, rotation_t* R);

void euler2quaternion(const vec3f_t* Euler, quaternion_t* Q);

void rotation2quaternion(const rotation_t* R, quaternion_t* Q);

void quaternion_derivative(const quaternion_t* Q, quaternion_t* derQ, const vec3f_t* w);

void quaternion_normalize(quaternion_t* Q);

#endif
