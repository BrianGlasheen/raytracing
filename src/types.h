#pragma once

#include "math.h"

struct ray {
	vec3 pos;
	vec3 dir;
};

struct hit {
	bool hit;
	vec3 pos;
	float t;
	vec3 norm;
	uint32 material_idx;
};

hit RAY_MISS = { false, vec3(0.0f), FLT_MAX, vec3(0.0f) };

struct light {
	vec3 pos;
	float intensity;
	vec3 color;
};

enum shape_type {
	SPHERE = 0,
	PLANE,
	PARALLELOGRAM
};

struct material {
	vec3 color;
	float reflectivity;
	float roughness;
	bool refractive;
	float refraction_index;
	float emission;
};

struct sphere {
	vec3  center;
	float radius;
};

struct plane {
	vec3 point;
	vec3 normal;
};

struct parallelogram {
	vec3 point;
	vec3 p;
	vec3 q;
};

struct box {
	vec3 point;
};

// struct cube
// struct torus
// struct parallelogram
// struct infinite cylinder
// struct disk (finite cylinder)
// struct cone?

struct analytical_shape {
	// todo aabb
	shape_type type;
	union {
		sphere m_sphere;
		// ellipse m_ellipse;
		plane m_plane;
		parallelogram m_parallelogram;
	};
	uint32 material_idx;
};
