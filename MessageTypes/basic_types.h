#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H
#pragma anon_unions
#include <stdint.h>
#include <stdbool.h>
typedef struct vec2i16_s {
//	uint32_t timestamp;
	union {
		struct {
			int16_t x;
			int16_t y;
		};
		int16_t v[2];
	};
}vec2i16_t;
typedef struct vec2f_s {
//  uint32_t timestamp; 
	union {
		struct {
			float x;
			float y;
		};
		float v[2];
	};
}vec2f_t;
/* x,y,z vector */
typedef struct vec3i16_s {
//	uint32_t timestamp;
	union {
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
		int16_t v[3];
	};
}vec3i16_t;
typedef struct vec3f_s {
//  uint32_t timestamp; // Timestamp when the data was computed
	union {
		struct {
			float R;
			float P;
			float Y;
		};
		struct {
			float x;
			float y;
			float z;
		};
		float v[3];
	};
}vec3f_t;

typedef struct quaternion_s {
//  uint32_t timestamp;
	union {
		struct {
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct {
			float x;
			float y;
			float z;
			float w;
		};
		float q[4];
	};
} quaternion_t;

typedef struct rotation_s {
//  uint32_t timestamp;  // Timestamp when the data was computed
	float R[3][3];
} rotation_t;


#endif
