#include "type_math.h"
#include "attitude_lib.h"
#include <math.h>
#include "arm_math.h"
#include "../Modules/stabilizer_types.h"
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
