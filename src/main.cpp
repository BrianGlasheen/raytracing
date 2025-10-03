#include <omp.h>
#include <iostream>
#include <string>
#include <cmath>
#include <unistd.h>
#include <vector>
#include <random>
#include <float.h>
#include <algorithm>

#include "Image.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

// #define TINYOBJLOADER_IMPLEMENTATION
// #include "tiny_obj_loader.h"

using glm::vec3, glm::vec4, glm::mat4, glm::mat3, glm::dot, glm::reflect, glm::inverse, glm::determinant, glm::clamp;

using std::vector, std::cout, std::endl, std::cerr, std::stoi, std::string, std::to_string, std::max;

typedef uint32_t uint32;

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

// todo mesh
// struct mesh {
	// blas / bvh
	// aabb
	// transform
	// vertices
	// indices
	// material
	// backfaces(?)
// };

hit sphere_ray_intersection(const sphere& s, const ray& r) {
    vec3 oc = r.pos - s.center;
    
    float a = dot(r.dir, r.dir);
    float half_b = dot(oc, r.dir); // !!
    float c = dot(oc, oc) - s.radius * s.radius;
    
    float discriminant = half_b * half_b - a * c;
    
    if (discriminant < 0.0f)
        return {false, vec3(0.0f), FLT_MAX, vec3(0.0f)};
    
    float sqrtd = sqrtf(discriminant);
    float t = (-half_b - sqrtd) / a;
    
    if (t < 0.001f) {
        t = (-half_b + sqrtd) / a;
        if (t < 0.001f)
            return {false, vec3(0.0f), FLT_MAX, vec3(0.0f)};
    }
    
    vec3 hit_pos = r.pos + r.dir * t;
    vec3 normal = (hit_pos - s.center) / s.radius;
    return { true, hit_pos, t, normal, 0 };
}

hit plane_ray_intersection(const plane& p, const ray& r) {
    float denom = dot(p.normal, r.dir);
    
    if (fabsf(denom) < 1e-6f)
        return {false, vec3(0.0f), FLT_MAX, vec3(0.0f)};
    
    float t = dot(p.point - r.pos, p.normal) / denom;
    
    if (t < 0.001f)
        return {false, vec3(0.0f), FLT_MAX, vec3(0.0f)};
    
    vec3 hit_point = r.pos + r.dir * t;

    vec3 normal = p.normal;
    if (dot(normal, r.dir) > 0.0f) // flip normal if pointing away
        normal = -normal;
    
    return { true, hit_point, t, normal, 0 };
}

hit parallelogram_ray_intersection(const parallelogram& para, const ray& r) {
    vec3 offset = r.pos - para.point;

    mat3 A(-r.dir, para.p, para.q);

    float detA = determinant(A);
    if (abs(detA) < 1e-6)
        return { false };

    vec3 tuv = inverse(A) * offset;

    float t = tuv.x;
    float u = tuv.y;
    float v = tuv.z;

    if (t < 0 || u < 0 || u > 1 || v < 0 || v > 1)
        return { false };

    vec3 hit_point = r.pos + t * r.dir;
    vec3 normal = normalize(cross(para.p, para.q));

	if (dot(normal, r.dir) > 0.0f)
        normal = -normal;

    return { true, hit_point, t, normal, 0 };
}

hit ray_analytical_shape_intersection(const analytical_shape& shape, const ray& r) {
	// using enum shape_type; c++ 20
	hit hit;

	switch (shape.type) {
		case shape_type::SPHERE:
			hit = sphere_ray_intersection(shape.m_sphere, r);
			break;
		case shape_type::PLANE:
			hit = plane_ray_intersection(shape.m_plane, r);
			break;
		case shape_type::PARALLELOGRAM:
			hit = parallelogram_ray_intersection(shape.m_parallelogram, r);
			break;
		default:
			assert(false);
	}

	hit.material_idx = shape.material_idx;
	return hit;
}

vec3 randomSmallVector() { // todo wtf is this code even CHANGE!!
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dis(-0.1f, 0.1f); // Adjust the range as needed

    return vec3(dis(gen), dis(gen), dis(gen));
}

vec3 random_in_unit_sphere() {
    while (true) {
        vec3 p = vec3(
            (float)rand() / RAND_MAX * 2.0f - 1.0f,
            (float)rand() / RAND_MAX * 2.0f - 1.0f,
            (float)rand() / RAND_MAX * 2.0f - 1.0f
        );
        if (glm::length(p) < 1.0f) {
            return p;
        }
    }
}

vec3 cosine_weighted_hemisphere(vec3 normal) {
    // random point on unit disk
    float r = sqrt((float)rand() / RAND_MAX);
    float theta = 2.0f * 3.14159265f * ((float)rand() / RAND_MAX);
    float x = r * cos(theta);
    float y = r * sin(theta);
    
    // project to hemisphere
    float z = sqrt(max(0.0f, 1.0f - x*x - y*y));
    
    vec3 tangent, bitangent;
    if (abs(normal.x) > abs(normal.y)) {
        tangent = vec3(normal.z, 0.0f, -normal.x) / sqrtf(normal.x * normal.x + normal.z * normal.z);
    } else {
        tangent = vec3(0.0f, -normal.z, normal.y) / sqrtf(normal.y * normal.y + normal.z * normal.z);
    }
    bitangent = glm::cross(normal, tangent);
    
    return x * tangent + y * bitangent + z * normal;
}

// fresnel approx
float schlick(float cosine, float ref_idx) {
    float r0 = (1.0f - ref_idx) / (1.0f + ref_idx);
    r0 = r0 * r0;
    return r0 + (1.0f - r0) * powf(1.0f - cosine, 5.0f);
}

bool refract(const vec3& v, const vec3& n, float ni_over_nt, vec3& refracted) {
    vec3 uv = glm::normalize(v);
    float dt = dot(uv, n);
    float discriminant = 1.0f - ni_over_nt * ni_over_nt * (1.0f - dt * dt);
    
    if (discriminant > 0.0f) {
        refracted = ni_over_nt * (uv - n * dt) - n * sqrtf(discriminant);
        return true;
    }

	// total internal reflection
    return false;
}

vec3 shade(const hit& ray_hit, const vector<analytical_shape>& scene, const vector<light>& lights) {
	float ambient = 0.00f;
    vec3 color = vec3(ambient);
    
    for (const light& l : lights) {
        vec3 to_light = l.pos - ray_hit.pos;
        float light_distance = glm::length(to_light);
        vec3 light_dir = to_light / light_distance;
        
        ray shadow_ray;
        shadow_ray.pos = ray_hit.pos + ray_hit.norm * 0.001f;
        shadow_ray.dir = light_dir;
        
        bool in_shadow = false;
        for (const analytical_shape& shape : scene) {
            hit shadow_hit = ray_analytical_shape_intersection(shape, shadow_ray);
            
            if (shadow_hit.hit && shadow_hit.t < light_distance) {
                in_shadow = true;
                break;
            }
        }
        
        if (!in_shadow) {
			// lambert diffuse
			float diffuse = glm::max(dot(ray_hit.norm, light_dir), 0.0f);
            float attenuation = 1.0f / (1.0f + 0.09f * light_distance + 0.032f * light_distance * light_distance);
            color += l.color * l.intensity * diffuse * attenuation;
        }
    }
    
    return clamp(color, 0.0f, 1.0f);
}

// todo maybe move these guys
vector<analytical_shape> scene;
vector<light> lights;
vector<material> materials;

vec3 trace_whitted(const vector<analytical_shape>& scene, const vector<light>& lights, const ray& r, int depth = 0) {
	hit closest_hit = { false, vec3(0.0f), FLT_MAX, vec3(0.0f) };

	for (const analytical_shape& shape : scene) {
		hit hit_info = ray_analytical_shape_intersection(shape, r);
		if (hit_info.hit && hit_info.t < closest_hit.t)
			closest_hit = hit_info;
	}

	if (!closest_hit.hit) {
		vec3 unit_dir = glm::normalize(r.dir);
		float t = 0.5f * (unit_dir.y + 1.0f);
		return (1.0f - t) * vec3(0.01f, 0.01f, 0.03f) + t * vec3(0.05f, 0.05f, 0.15f);
	}

	vec3 diffuse_color = shade(closest_hit, scene, lights);
    diffuse_color *= materials[closest_hit.material_idx].color;
	// return diffuse_color;
	// return 0.5f * (closest_hit.norm + vec3(1.0f, 1.0f, 1.0f)); 	// normal shading

	float reflectivity = materials[closest_hit.material_idx].reflectivity;
	float roughness = materials[closest_hit.material_idx].reflectivity;
	bool refractive = materials[closest_hit.material_idx].refractive;
	float refraction_index = materials[closest_hit.material_idx].refraction_index;

	if (refractive) {
        vec3 outward_normal;
        vec3 reflected = reflect(r.dir, closest_hit.norm);
        float ni_over_nt;
        vec3 refracted;
        float reflect_prob;
        float cosine;
        
        if (dot(r.dir, closest_hit.norm) > 0.0f) {
            // exiting
            outward_normal = -closest_hit.norm;
            ni_over_nt = refraction_index;
            cosine = dot(r.dir, closest_hit.norm) / glm::length(r.dir);
        } else {
            // entering
            outward_normal = closest_hit.norm;
            ni_over_nt = 1.0f / refraction_index;
            cosine = -dot(r.dir, closest_hit.norm) / glm::length(r.dir);
        }
        
        if (refract(r.dir, outward_normal, ni_over_nt, refracted)) {
            reflect_prob = schlick(cosine, refraction_index);
        } else {
			// total internal reflection
			reflect_prob = 1.0f;
        }
        
        if ((float)rand() / RAND_MAX < reflect_prob) {
            // reflect
            ray reflect_ray;
            reflect_ray.pos = closest_hit.pos + outward_normal * 0.001f;
            reflect_ray.dir = reflected;
            return trace_whitted(scene, lights, reflect_ray, depth - 1);
        } else {
            // refract
            ray refract_ray;
            refract_ray.pos = closest_hit.pos - outward_normal * 0.001f;
            refract_ray.dir = refracted;
            return trace_whitted(scene, lights, refract_ray, depth - 1);
        }
    }

	vec3 reflection_color = vec3(0.0f);
    if (reflectivity > 0.0f) {
        vec3 reflect_dir = reflect(r.dir, closest_hit.norm);

		if (roughness > 0.0f) {
			int num_samples = 4;
			vec3 accumulated_color = vec3(0.0f);
			
			for (int i = 0; i < num_samples; i++) {
				vec3 perturbed_dir = reflect_dir + roughness * randomSmallVector();
				perturbed_dir = glm::normalize(perturbed_dir);
				
				if (dot(perturbed_dir, closest_hit.norm) > 0.0f) {
					ray reflect_ray;
					reflect_ray.pos = closest_hit.pos + closest_hit.norm * 0.001f;
					reflect_ray.dir = perturbed_dir;
					
					accumulated_color += trace_whitted(scene, lights, reflect_ray, depth + 1);
				}
			}
			
			reflection_color = accumulated_color / (float)num_samples;
		}
		else {
			ray reflect_ray;
			reflect_ray.pos = closest_hit.pos + closest_hit.norm * 0.001f;
			reflect_ray.dir = reflect_dir;
			
			reflection_color = trace_whitted(scene, lights, reflect_ray, depth + 1);
    	}
    } 
    
    return (1.0f - reflectivity) * diffuse_color + reflectivity * reflection_color;
}

vec3 trace_monte(const vector<analytical_shape>& scene, const ray& r, int depth = 0) {
	const int MAX_DEPTH = 4;
    if (depth >= MAX_DEPTH) {
        return vec3(0.0f);
    }

	hit closest_hit = { false, vec3(0.0f), FLT_MAX, vec3(0.0f) };

	for (const analytical_shape& shape : scene) {
		hit hit_info = ray_analytical_shape_intersection(shape, r);
		if (hit_info.hit && hit_info.t < closest_hit.t)
			closest_hit = hit_info;
	}

	if (!closest_hit.hit) {
		vec3 unit_dir = glm::normalize(r.dir);
		float t = 0.5f * (unit_dir.y + 1.0f);
		return (1.0f - t) * vec3(0.01f, 0.01f, 0.03f) + t * vec3(0.05f, 0.05f, 0.15f);
	}

	const material& mat = materials[closest_hit.material_idx];
	vec3 emitted = mat.emission * mat.color;

	if (depth > 3) {
        float survival_prob = 0.8f;
        if ((float)rand() / RAND_MAX > survival_prob) {
            return emitted;
        }
    }

	if (mat.refractive) {
        vec3 outward_normal;
        vec3 reflected = reflect(r.dir, closest_hit.norm);
        float ni_over_nt;
        vec3 refracted;
        float reflect_prob;
        float cosine;
        
        if (dot(r.dir, closest_hit.norm) > 0.0f) {
            // exiting
            outward_normal = -closest_hit.norm;
            ni_over_nt = mat.refraction_index;
            cosine = dot(r.dir, closest_hit.norm) / glm::length(r.dir);
        } else {
            // entering
            outward_normal = closest_hit.norm;
            ni_over_nt = 1.0f / mat.refraction_index;
            cosine = -dot(r.dir, closest_hit.norm) / glm::length(r.dir);
        }
        
        if (refract(r.dir, outward_normal, ni_over_nt, refracted)) {
            reflect_prob = schlick(cosine, mat.refraction_index);
        } else {
            // total internal reflection
            reflect_prob = 1.0f;
        }
        
        if ((float)rand() / RAND_MAX < reflect_prob) {
            // reflect
            ray reflect_ray;
            reflect_ray.pos = closest_hit.pos + outward_normal * 0.001f;
            reflect_ray.dir = reflected;
            return emitted + trace_monte(scene, reflect_ray, depth + 1);
        } else {
            // refract
            ray refract_ray;
            refract_ray.pos = closest_hit.pos - outward_normal * 0.001f;
            refract_ray.dir = refracted;
            return emitted + trace_monte(scene, refract_ray, depth + 1);
        }
    }
    
    if (mat.reflectivity > 0.9f) {
        vec3 reflect_dir = reflect(r.dir, closest_hit.norm);
        
        if (mat.roughness > 0.0f) {
            reflect_dir = reflect_dir + mat.roughness * random_in_unit_sphere();
            reflect_dir = glm::normalize(reflect_dir);
            
            if (dot(reflect_dir, closest_hit.norm) <= 0.0f) {
                return emitted;
            }
        }
        
        ray reflect_ray;
        reflect_ray.pos = closest_hit.pos + closest_hit.norm * 0.001f;
        reflect_ray.dir = reflect_dir;
        
        vec3 incoming = trace_monte(scene, reflect_ray, depth + 1);
        return emitted + mat.color * incoming;
    }
    
    // importance sampling
    vec3 scatter_dir = cosine_weighted_hemisphere(closest_hit.norm);
    
    ray scattered;
    scattered.pos = closest_hit.pos + closest_hit.norm * 0.001f;
    scattered.dir = scatter_dir;
    
    vec3 incoming_light = trace_monte(scene, scattered, depth + 1);
    
    // BRDF for Lambertian (diffuse) surface
    // For cosine-weighted sampling, the pdf cancels with cos_theta/PI
    // So we just multiply by albedo
    vec3 reflected = mat.color * incoming_light;
    
    return emitted + reflected;
}

void setup_scene() {
	analytical_shape& p1 = scene.emplace_back();
	p1.type = shape_type::PLANE;
	p1.m_plane.point = vec3(0.0f, -1.0f, 0.0f);
	p1.m_plane.normal = vec3(0.0f, 1.0f, 0.0f);

	p1.material_idx = materials.size();
	materials.push_back({ vec3(0.75f), 0.0f, 0.0f });

	analytical_shape& s = scene.emplace_back();
	s.type = shape_type::SPHERE;
	s.m_sphere.center = vec3(0, 0, 0);
	s.m_sphere.radius = 1;
	s.material_idx = materials.size();
	materials.push_back({ vec3(1.0f, 0.0f, 0.0f), 0.0f, 0.0f });

	analytical_shape& s2 = scene.emplace_back();
	s2.type = shape_type::SPHERE;
	s2.m_sphere.center = vec3(1.0f, 0.0f, 1.5f);
	s2.m_sphere.radius = 0.5f;
	s2.material_idx = materials.size();
	materials.push_back({ vec3(0.0f, 1.0f, 0.0f), 0.0f, 0.0f, false, 1.0f, 2.0f });

	analytical_shape& s3 = scene.emplace_back();
	s3.type = shape_type::SPHERE;
	s3.m_sphere.center = vec3(-1.0f, -0.5f, 1.5f);
	s3.m_sphere.radius = 0.5f;
	s3.material_idx = materials.size();
	materials.push_back({ vec3(0.0f, 0.0f, 1.0f), 0.5f, 0.5f });

	analytical_shape& s4 = scene.emplace_back();
	s4.type = shape_type::SPHERE;
	s4.m_sphere.center = vec3(0.0f, -0.25f, 3.0f);
	s4.m_sphere.radius = 0.5f;
	s4.material_idx = materials.size();
	materials.push_back({ vec3(0.0f, 1.0f, 1.0f), 0.0f, 0.0f, true, 1.5f });

	analytical_shape& p2 = scene.emplace_back();
	p2.type = shape_type::PARALLELOGRAM;
	p2.m_parallelogram.point = vec3(-1.0f, 1.5f, -1.0f);
	p2.m_parallelogram.p = vec3(2.0f, 0.0f, 0.0f);
	p2.m_parallelogram.q = vec3(0.0f, 0.0f, 2.0f);
	p2.material_idx = materials.size();
	materials.push_back({ vec3(1.0f, 1.0f, 1.0f), 0.0f, 0.0f, true, 1.00f, 1.0f});

	// struct material {
	// 	vec3 color;
	// 	float reflectivity;
	// 	float roughness;
	// 	bool refractive;
	// 	float refraction_index;
	// 	float emission;
	// };

	light& l1 = lights.emplace_back();
	l1.pos = vec3(1.0f, 2.0f, 2.0f);
	l1.intensity = 0.75f;
	l1.color = vec3(1.5f);

	light& l2 = lights.emplace_back();
	l2.pos = vec3(-1.0f, 2.0f, -1.0f);
	l2.intensity = 0.5f;
	l2.color = vec3(1.5f);
}

void setup_cornell_box() {
    analytical_shape& right_wall = scene.emplace_back();
    right_wall.type = shape_type::PARALLELOGRAM;
    right_wall.m_parallelogram.point = vec3(555.0f, 0.0f, 0.0f);
    right_wall.m_parallelogram.p = vec3(0.0f, 555.0f, 0.0f);
    right_wall.m_parallelogram.q = vec3(0.0f, 0.0f, 555.0f);
    right_wall.material_idx = materials.size();
    materials.push_back({ vec3(0.12f, 0.45f, 0.15f), 0.0f, 0.0f, false, 1.0f, 0.0f });

    analytical_shape& left_wall = scene.emplace_back();
    left_wall.type = shape_type::PARALLELOGRAM;
    left_wall.m_parallelogram.point = vec3(0.0f);
    left_wall.m_parallelogram.p = vec3(0.0f, 555.0f, 0.0f);
    left_wall.m_parallelogram.q = vec3(0.0f, 0.0f, 555.0f);
    left_wall.material_idx = materials.size();
    materials.push_back({ vec3(0.65f, 0.05f, 0.05f), 0.0f, 0.0f, false, 1.0f, 0.0f });
    
    analytical_shape& light = scene.emplace_back();
    light.type = shape_type::PARALLELOGRAM;
    light.m_parallelogram.point = vec3(343.0f, 554.0f, 332.0f);
    light.m_parallelogram.p = vec3(-130.0f, 0.0f, 0.0f);
    light.m_parallelogram.q = vec3(0.0f, 0.0f, -105.0f);
    light.material_idx = materials.size();
    materials.push_back({ vec3(1.0f), 0.0f, 0.0f, false, 1.0f, 10.0f });

    size_t white_wall_mat = materials.size();
    materials.push_back({ vec3(0.73f), 0.0f, 0.0f, false, 1.0f, 0.0f });

    analytical_shape& floor = scene.emplace_back();
    floor.type = shape_type::PARALLELOGRAM;
    floor.m_parallelogram.point = vec3(0.0f);
    floor.m_parallelogram.p = vec3(555.0f, 0.0f, 0.0f);
    floor.m_parallelogram.q = vec3(0.0f, 0.0f, 555.0f);
    floor.material_idx = white_wall_mat;

    analytical_shape& top = scene.emplace_back();
    top.type = shape_type::PARALLELOGRAM;
    top.m_parallelogram.point = vec3(555.0f);
    top.m_parallelogram.p = vec3(-555.0f, 0.0f, 0.0f);
    top.m_parallelogram.q = vec3(0.0f, 0.0f, -555.0f);
    top.material_idx = white_wall_mat;

    // world.add(make_shared<quad>(point3(0,0,555), vec3(555,0,0), vec3(0,555,0), white));
    analytical_shape& back = scene.emplace_back();
    back.type = shape_type::PARALLELOGRAM;
    back.m_parallelogram.point = vec3(0.0f, 0.0f, 555.0f);
    back.m_parallelogram.p = vec3(555.0f, 0.0f, 0.0f);
    back.m_parallelogram.q = vec3(0.0f, 555.0f, 0.0f);
    back.material_idx = white_wall_mat;
}

// usage ./RT <IMAGE SIZE> <IMAGE FILENAME>
int main(int argc, char **argv) {
	int imageSize(stoi(argv[1]));
	string fileName(argv[2]);
	
	int width = imageSize;
	int height = imageSize;

	int res = imageSize;

	vector<ray> rays; // todo alloc total rays


    vec3 camera_pos = vec3(278, 278, -800);
    vec3 camera_target = vec3(278, 278, 0);
    vec3 camera_up = vec3(0.0f, 1.0f, 0.0f);

    vec3 forward = glm::normalize(camera_target - camera_pos);
    vec3 right = glm::normalize(glm::cross(forward, camera_up));
    vec3 up = glm::cross(right, forward);
    
    float fov = 40.0f;
    float aspect = (float)width / (float)height;
    float h = tan((fov * 0.5f * M_PI) / 180.0f);
    float viewport_height = 2.0f * h;
    float viewport_width = viewport_height * aspect;
    
    for (int row = 0; row < res; row++) {
        for (int col = 0; col < res; col++) {
            float u = (col + 0.5f) / res;
            float v = (row + 0.5f) / res;
            
            float x = (2.0f * u - 1.0f) * viewport_width * 0.5f;
            float y = (1.0f - 2.0f * v) * viewport_height * 0.5f; // Flip Y
            
            ray r;
            r.pos = camera_pos;
            r.dir = glm::normalize(forward + x * right + y * up);
            
            rays.push_back(r);
        }
    }

    // setup_scene();
	setup_cornell_box();

	Image image = Image(width, height);
	int w = width / res;

    #pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < width; i++) {
        // #pragma omp critical
		printf("%d / %d\n", i, width);
		for (int j = 0; j < height; j++) {
			int x = j / w;
			int y = i / w;

			int idx = x * res + y;
			// vec3 c = trace_whitted(scene, lights, rays[idx]);

            int samples_per_pixel = 200;

            vec3 c(0.0f);
            for (int s = 0; s < samples_per_pixel; s++) {
                c += trace_monte(scene, rays[idx], 0);
    			// vec3 c = clamp(trace_monte(scene, rays[idx]), 0.0f, 1.0f);
            }

            c = clamp(c / (float)samples_per_pixel, 0.0f, 1.0f);
			image.setPixel(i, height - j - 1, c.r * 255, c.g * 255, c.b * 255);
		}
	}
	
	image.writeToFile("../resources/" + fileName + ".png");
	return 0;
}

// gonna want this later
// int intersectTri(ray* r, tri* triangle, float* t, float* u, float* v) { 
// 	vec3 orig = r->pos;
// 	vec3 dir = r->dir;
// 	vec3 vert0 = triangle->v1;
// 	vec3 vert1 = triangle->v2;
// 	vec3 vert2 = triangle->v3;

// 	vec3 edge1(0.0f), edge2(0.0f), tvec(0.0f), pvec(0.0f), qvec(0.0f);
// 	float det, inv_det;

// 	edge1 = vert1 - vert0;
// 	edge2 = vert2 - vert0;

// 	pvec = glm::cross(dir, edge2);

// 	det = dot(edge1, pvec);

// 	tvec = orig - vert0;
// 	inv_det = 1.0 / det;

// 	qvec = glm::cross(tvec, edge1);

// 	if (det > 0.000001) {
// 		*u = dot(tvec, pvec);
// 		if (*u < 0.0 || *u > det)
// 			return 0;

// 		*v = dot(dir, qvec);
// 		if (*v < 0.0 || *u + *v > det)
// 			return 0;
// 	} else if (det < -0.000001) {
// 		*u = dot(tvec, pvec);
// 		if (*u > 0.0 || *u < det)
// 			return 0;
		
// 		*v = dot(dir, qvec);
// 		if (*v > 0.0 || *u + *v < det)
// 			return 0;
// 	} else {
// 		return 0;
// 	}

// 	*t = dot(edge2, qvec) * inv_det;
// 	(*u) *= inv_det;
// 	(*v) *= inv_det;

// 	return 1;

// }
