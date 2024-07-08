#include <iostream>
#include <string>
#include <cmath>
#include <unistd.h>

#include "Image.h"
#include <glm/glm.hpp>
#include "glm/gtc/matrix_transform.hpp"
#include <algorithm>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <vector>
#include <random>

using glm::vec3;
using glm::vec4;
using glm::mat4;

using namespace std;

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

bool PHONG_AND_REFLECTIVE = false;
bool ROUGH = false;
float REFLECT_WEIGHT = .3;
float PHONG_WEIGHT = .7;

class ThreadPool {
public:
    ThreadPool(size_t numThreads) : stop(false) {
        for (size_t i = 0; i < numThreads; ++i) {
            threads.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty())
                            return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task();
                    {
                        std::lock_guard<std::mutex> lock(queue_mutex);
                        --tasks_left;
                        if (tasks_left == 0) {
                            condition.notify_all();
                        }
                    }
                }
            });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : threads) {
            worker.join();
        }
    }

    template<class F>
    void enqueue(F &&f) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            tasks.emplace(std::forward<F>(f));
            ++tasks_left;
        }
        condition.notify_one();
    }

    void waitAll() {
        std::unique_lock<std::mutex> lock(queue_mutex);
        condition.wait(lock, [this] { return tasks_left == 0; });
    }

private:
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
    size_t tasks_left = 0;
};

enum {SPHERE, PLANE, ELLIPSE, MESH};
enum {NO_REFLECT, REFLECT, REFRACT};

struct ray {
	vec3 pos;
	vec3 dir;
	vec3 color;
};

struct sphere {
	vec3 pos;
	float r;
	vec3 diff;
	vec3 spec;
	vec3 amb;
	float s;
};

struct light {
	vec3 pos;
	float i;
};

struct tri {
	vec3 v1;
	vec3 v2;
	vec3 v3;
	vec3 n1;
	vec3 n2;
	vec3 n3;
};

struct BB {
	vec3 min;
	vec3 max;
};

struct tn { // t and norm
	float t;
	vec3 n;
};


class Shape {
	public:	
		int type;
		int reflective;
		vec3 pos;
		vec3 amb;
		vec3 diff;
		vec3 spec;
		float s;
		mat4 transforms;

		Shape(const int type, const int reflective, const vec3& pos, const vec3& amb, const vec3& diff, const vec3& spec, const float s)
        	: type(type), reflective(reflective), pos(pos), amb(amb), diff(diff), spec(spec), s(s) {}

		virtual tn* intersect(ray* r) { return new tn{-1, vec3(0.0f)}; } 
		virtual vec3 normalAt(vec3 point) const { return vec3(0.0f); } 
		virtual vec3 getNorm() const { return vec3(0.0f); } 
		virtual mat4 getTransforms() const { return mat4(0.0f); } 
		virtual void setTransforms(mat4 t) { transforms = t; } 
		virtual Shape* getBs() const { return 0; } 
};

class Sphere : public Shape {
	public:
		float radius;
		mat4 transforms = mat4(1.0f);

		Sphere(const int type, const int reflective, const vec3& pos, const vec3& amb, const vec3& diff, const vec3& spec, const float s, const float radius)
        : Shape(type, reflective, pos, amb, diff, spec, s), radius(radius) {
			// transforms[3][0] = 1;
		}

		virtual void setTransforms(mat4 t) override { transforms = t; } 

		tn* intersect(ray* r) override {
			// Apply inverse transformation to the ray

			ray transformed_ray;

			transformed_ray.pos = inverse(transforms) * vec4(r->pos, 1.0f);
			transformed_ray.dir = inverse(transforms) * vec4(r->dir, 0.0f);

			vec3 oc = transformed_ray.pos - pos;
			float a = dot(transformed_ray.dir, transformed_ray.dir);
			float b = 2 * dot(transformed_ray.dir, oc);
			float c = dot(oc, oc) - radius * radius;
			float disc = b * b - 4 * a * c;

			if (disc < 0) {
				return new tn{-1.0, vec3(0.0f)};
			} else {
				float t = (-b - sqrt(disc)) / (2 * a);
				// Transform the intersection point back to the original coordinate system
				vec3 intersection_point = transformed_ray.pos + t * transformed_ray.dir;
				vec3 normal = normalize(intersection_point - pos);
				// Apply transformation to the normal vector
				normal = glm::transpose(inverse(glm::mat3(transforms))) * normal;
				return new tn{t, normal};
			}
		}

};

class Ellipse : public Sphere {
	public:
		float radius;
		mat4 transforms;

		Ellipse(const int type, const int reflective, const vec3& pos, const vec3& amb, const vec3& diff, const vec3& spec, const float s, const float radius, const mat4 transforms)
        : Sphere(type, reflective, pos, amb, diff, spec, s, radius), transforms(transforms) {}

		virtual mat4 getTransforms() const { return transforms; }

		float unitSphereIntersect(ray* ray) const {

			vec3 oc = ray->pos - vec3(0, 0, 0);
			float a = dot(ray->dir, ray->dir);
			float b = 2 * dot(ray->dir, oc);
			float c = dot(oc, oc) - 1;
			float disc = b * b - 4 * a * c;

			if (disc < 0) {
				return -1.0;
			} else {
				return (-b - sqrt(disc) ) / (2.0*a);
			}
		} 

		tn* intersect(ray* r) override {
			ray* transformedRay = new ray;
			transformedRay->pos = inverse(transforms) * vec4(r->pos, 1.0f);
			transformedRay->dir = inverse(transforms) * vec4(r->dir, 0.0f);

			float t = unitSphereIntersect(transformedRay);

			if (t > 0) {
				vec4 intersectionPoint = vec4(transformedRay->pos + t * transformedRay->dir, 1.0f);
				vec4 worldIntersectionPoint = transforms * intersectionPoint;

				vec4 hitPosLocal = inverse(transforms) * vec4(r->pos + t * r->dir, 1.0f);
				hitPosLocal /= hitPosLocal.w; // homo cord
				vec3 n = normalize(normalAt(vec3(hitPosLocal)));

				return new tn{distance(r->pos, vec3(worldIntersectionPoint)), n};
			}

			return new tn{-1.0f, vec3(0.0f)};
		}

		vec3 normalAt(vec3 point) const override {
			vec3 normalLocal = normalize(point);
			vec3 normalWorld = vec3(transpose(inverse(transforms)) * vec4(normalLocal, 0.0f));

			return normalWorld;
		}
};

class Plane : public Shape {
	public:
		vec3 norm;

		Plane(const int type, const int reflective, const vec3& pos, const vec3& amb, const vec3& diff, const vec3& spec, const float s, const vec3& norm)
        	: Shape(type, reflective, pos, amb, diff, spec, s), norm(norm) {}

		tn* intersect(ray* r) override {
			float denom = dot(norm, r->dir);

			if (abs(denom) > 0.0001f) {
				float t = dot(pos - r->pos, norm) / denom;
				if (t >= 0) return new tn{t, norm};
			}
			return new tn{-1, vec3(0.0f)};
		}

		vec3 getNorm() const override { return norm; } 
};

int intersectTri(ray* r, tri* triangle, float* t, float* u, float* v) { 
	vec3 orig = r->pos;
	vec3 dir = r->dir;
	vec3 vert0 = triangle->v1;
	vec3 vert1 = triangle->v2;
	vec3 vert2 = triangle->v3;

	vec3 edge1(0.0f), edge2(0.0f), tvec(0.0f), pvec(0.0f), qvec(0.0f);
	float det, inv_det;

	edge1 = vert1 - vert0;
	edge2 = vert2 - vert0;

	pvec = glm::cross(dir, edge2);

	det = dot(edge1, pvec);

	tvec = orig - vert0;
	inv_det = 1.0 / det;

	qvec = glm::cross(tvec, edge1);

	if (det > 0.000001) {
		*u = dot(tvec, pvec);
		if (*u < 0.0 || *u > det)
			return 0;

		*v = dot(dir, qvec);
		if (*v < 0.0 || *u + *v > det)
			return 0;
	} else if (det < -0.000001) {
		*u = dot(tvec, pvec);
		if (*u > 0.0 || *u < det)
			return 0;
		
		*v = dot(dir, qvec);
		if (*v > 0.0 || *u + *v < det)
			return 0;
	} else {
		return 0;
	}

	*t = dot(edge2, qvec) * inv_det;
	(*u) *= inv_det;
	(*v) *= inv_det;

	return 1;

}

class Mesh : public Shape {
	public:
		vector<tri*> triangles;
		Sphere* boundingSphere;
		mat4 transforms = mat4(1.0f);

		Mesh(const int type, const int reflective, const vec3& pos, const vec3& amb, const vec3& diff, const vec3& spec, const float s, const vector<tri*>& triangles, Sphere* bs)
        	: Shape(type, reflective, pos, amb, diff, spec, s), triangles(triangles), boundingSphere(bs) {}

		virtual void setTransforms(mat4 t) override { transforms = t; }
		virtual Shape* getBs() const override { return boundingSphere; } 

		tn* intersect(ray* r) override {

			if (boundingSphere->intersect(r)->t > 0) { // hits bounding sphere
				ray* transformedRay = new ray;
				transformedRay->pos = inverse(transforms) * vec4(r->pos, 1.0f);
				transformedRay->dir = inverse(transforms) * vec4(r->dir, 0.0f);
				
				// check every triangle
				float maxT = INT32_MAX;
				float maxTu = 0;
				float maxTv = 0; 
				float t;
				float u;
				float v;

				int triIdx = 0;
				bool hitTri = false;

				for (long unsigned int i = 0; i < triangles.size(); i++) {

					if (intersectTri(transformedRay, triangles[i], &t, &u, &v) > 0) {
						if (t <= 0) continue ;

						hitTri = true;
						if (t < maxT) {
							maxT = t;
							maxTu = u;
							maxTv = v;
							triIdx = i;
						}
					}
				}
				if (!hitTri) return new tn{-1, vec3(0.0f)};

				tri* hit = triangles[triIdx];
				float w = 1 - maxTu - maxTv;
				vec3 interp = normalize((hit->n1 * w) + (hit->n2 * maxTu) + (hit->n3 * maxTv));
				
				return new tn{t, interp};

			} else {		

				return new tn{-1, vec3(0.0f)};
			}
		}

};

struct hit {
	Shape* shape;
	vec3 pos;
	vec3 norm;
};

vector<Shape*> scene;
vector<light> lights;

void normalize(ray* r) { // normalize ray dist
    r->dir = glm::normalize(r->dir);
}

void colorImageRay(Image& image, vector<ray*> rays, int res, int width, int height) {
	int w = width / res;

	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			int x = j / w;
			int y = i / w;	

			vec3 c = rays[x * res + y]->color;
			image.setPixel(i, height - j - 1, c.r * 255, c.g * 255, c.b * 255);
		}
	}
	
}

vec3 blinnphong(vector<light> lights, vec3 eyePos, vec3 fragPos, vec3 fragNorm, Shape* s1) {
 
    vec3 viewDir = normalize(eyePos - fragPos);
    vec3 result(0.0f);

    for (light l : lights) {
        vec3 lightDir = normalize(l.pos - fragPos);
        float diff = max(0.0f, dot(lightDir, fragNorm));
        vec3 kd = s1->diff * diff;

        vec3 halfwayDir = normalize(lightDir + viewDir);
        float spec = pow(max(0.0f, dot(fragNorm, halfwayDir)), s1->s);
        vec3 ks = spec * s1->spec;

        result += (kd + ks) * l.i;
    }

	result += s1->amb;
    return result;
}

vec3 randomSmallVector() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dis(-0.1f, 0.1f); // Adjust the range as needed

    return vec3(dis(gen), dis(gen), dis(gen));
}

void castRay(ray* r, vector<Shape*>& scene, int depth) {

	hit* closestHit = new hit;
	closestHit->pos.z = INT32_MIN;

	for (long unsigned int i = 0; i < scene.size(); i++) {
		Shape* shape = scene[i];

		tn* t = shape->intersect(r);

		if (t->t > 0) {
			vec3 hitPos = (r->pos + t->t * r->dir);

			if (distance(r->pos, hitPos) < distance(r->pos, closestHit->pos)) {
				closestHit->shape = shape;
				closestHit->pos = hitPos;
				closestHit->norm = t->n;
			}
		}
	}

	if (closestHit->pos.z == INT32_MIN) { // background, i.e no hits
		r->color = vec3(0, 0, 0);
		return ;
	}

	ray* rayFromHit = new ray;
	rayFromHit->pos = closestHit->pos;
	Shape* shape = closestHit->shape;
	vec3 n = closestHit->norm;

	vector<light> valid_lights;
	for (light l : lights) {
		bool hitByThisLight = true;
		rayFromHit->dir = normalize(l.pos - rayFromHit->pos);

		for (long unsigned int j = 0; j < scene.size(); j++) {
			if (shape->type != MESH && shape == scene[j]) continue ;

			tn* tlight = scene[j]->intersect(rayFromHit);
			if (tlight->t > 0) {
				if (scene[j]->type == ELLIPSE) {
					hitByThisLight = false;
					break;
				}
				
				if (distance(l.pos, rayFromHit->pos) > distance(rayFromHit->pos, rayFromHit->pos + tlight->t * n)) {
					if (scene[j]->reflective == REFRACT) continue;
					hitByThisLight = false;
					break;
				}
			}
			// delete tlight;
		}
		if (hitByThisLight) {
			valid_lights.push_back(l);
		}
	}

	vec3 bp = blinnphong(valid_lights, r->pos, rayFromHit->pos, n, shape);
	
	if (shape->reflective && depth > 0) {

		if (shape->reflective == REFLECT) {
		
			ray* reflectedRay = new ray;
			reflectedRay->pos = closestHit->pos;
			reflectedRay->dir = normalize(r->dir - 2 * dot(r->dir, n) * n);

			if (ROUGH) {
				int numRays = 4;
				vec3 averageColor(0.0f, 0.0f, 0.0f);

				for (int i = 0; i < numRays; ++i) {

					ray* extra = new ray;
					extra->dir = reflectedRay->dir + randomSmallVector();
					extra->dir = normalize(extra->dir);
					extra->pos = reflectedRay->pos;

					castRay(extra, scene, depth - 1);

					averageColor += extra->color;
					delete extra;
				}

				averageColor /= numRays;
				reflectedRay->color = averageColor;

			} else { // cast one ray
				castRay(reflectedRay, scene, depth - 1);
			}

			if (PHONG_AND_REFLECTIVE) {
				vec3 combined = REFLECT_WEIGHT * reflectedRay->color + PHONG_WEIGHT * bp;
				combined.r = clamp(combined.r, 0.0f, 1.0f);
				combined.g = clamp(combined.g, 0.0f, 1.0f);
				combined.b = clamp(combined.b, 0.0f, 1.0f);
				r->color = combined; 
			} else {
				r->color = reflectedRay->color; 
			}

			// delete reflectedRay;

		} else if (shape->reflective == REFRACT) {
			ray* refractedRay = new ray;
			refractedRay->pos = closestHit->pos;
			refractedRay->dir = normalize(glm::refract(r->dir, n, 1.0f/1.02f));
			refractedRay->pos += refractedRay->dir * .01f;
			castRay(refractedRay, scene, depth - 1);
			r->color = refractedRay->color;

			delete refractedRay;
		}

	} else if (shape->reflective) { // hit reflective but no depth
		r->color = vec3(0.0f);

	} else {
		r->color.r = clamp(bp.r, 0.0f, 1.0f);
		r->color.g = clamp(bp.g, 0.0f, 1.0f);
		r->color.b = clamp(bp.b, 0.0f, 1.0f);
	}

}

void setup2() {

	sphere s1;
	s1.pos = vec3(-.5f, -1, 1);
	s1.diff = vec3(1, 0, 0);
	s1.spec = vec3(1, 1, .5);
	s1.amb = vec3(.1, .1, .1);
	s1.s = 100;
	s1.r = 1;
	Sphere* s1_ = new Sphere(SPHERE, NO_REFLECT, s1.pos, s1.amb, s1.diff, s1.spec, s1.s, s1.r);
	scene.push_back(s1_);

	sphere s2;
	s2.pos = vec3(.5, -1, -1);
	s2.r = 1;
	s2.diff = vec3(0, 1, 0);
	s2.spec = vec3(1, 1, .5);
	s2.amb = vec3(.1, .1, .1);
	s2.s = 100;
	Sphere* s2_ = new Sphere(SPHERE, NO_REFLECT, s2.pos, s2.amb, s2.diff, s2.spec, s2.s, s2.r);
	scene.push_back(s2_);

	sphere s3;
	s3.pos = vec3(0, 1, 0);
	s3.r = 1;
	s3.diff = vec3(0, 0, 1);
	s3.spec = vec3(1, 1, .5);
	s3.amb = vec3(.1, .1, .1);
	s3.s = 100;
	Sphere* s3_ = new Sphere(SPHERE, NO_REFLECT, s3.pos, s3.amb, s3.diff, s3.spec, s3.s, s3.r);
	scene.push_back(s3_);

	light l;
	l.pos = vec3(-2, 1, 1);
	l.i = 1;
	lights.push_back(l);
}

void setup3() {

	mat4 transforms(1.0f);
	transforms[0][0] = 0.5f;
	transforms[1][1] = 0.6f;
	transforms[2][2] = 0.2f;
	transforms[3][3] = 1.0f;

	transforms[3][0] = .5; // wrong should be .5
	transforms[3][1] = 0;
	transforms[3][2] = .5;

	sphere e;
	e.pos = vec3(0, 0, 0);
	e.diff = vec3(1, 0, 0);
	e.spec = vec3(1, 1, .5);
	e.amb = vec3(.1, .1, .1);
	e.s = 100;
	e.r = 1;
	Ellipse* e_ = new Ellipse(ELLIPSE, NO_REFLECT, e.pos, e.amb, e.diff, e.spec, e.s, e.r, transforms);
	scene.push_back(e_);

	sphere s1;
	s1.pos = vec3(-.5, 0, -.5);
	s1.diff = vec3(0, 1, 0);
	s1.spec = vec3(1, 1, .5);
	s1.amb = vec3(.1, .1, .1);
	s1.s = 100;
	s1.r = 1;
	Sphere* s1_ = new Sphere(SPHERE, NO_REFLECT, s1.pos, s1.amb, s1.diff, s1.spec, s1.s, s1.r);
	scene.push_back(s1_);

	vec3 ppos(0, -1, 0);
	vec3 pn(0, 1, 0);
	vec3 diff(1, 1, 1);
	vec3 spec(0, 0, 0);
	vec3 amb(.1, .1, .1);
	float s = 0;
	Plane* p = new Plane(PLANE, NO_REFLECT, ppos, amb, diff, spec, s, pn);
	scene.push_back(p);

	light l1;
	l1.pos = vec3(1, 2, 2);
	l1.i = .5;
	lights.push_back(l1);

	light l2;
	l2.pos = vec3(-1, 2, -1);
	l2.i = .5;
	lights.push_back(l2);

}

void setup5() {

	sphere s1;
	s1.pos = vec3(.5, -.7, .5);
	s1.r = .3;
	s1.diff = vec3(1, 0, 0);
	s1.spec = vec3(1, 1, .5);
	s1.amb = vec3(.1, .1, .1);
	s1.s = 100;
	Sphere* s1_ = new Sphere(SPHERE, NO_REFLECT, s1.pos, s1.amb, s1.diff, s1.spec, s1.s, s1.r);
	scene.push_back(s1_);

	sphere s2;
	s2.pos = vec3(1, -.7, 0);
	s2.r = .3;
	s2.diff = vec3(0, 0, 1);
	s2.spec = vec3(1, 1, .5);
	s2.amb = vec3(.1, .1, .1);
	s2.s = 100;
	Sphere* s2_ = new Sphere(SPHERE, NO_REFLECT, s2.pos, s2.amb, s2.diff, s2.spec, s2.s, s2.r);
	scene.push_back(s2_);

	sphere r1;
	r1.pos = vec3(-.5, 0, -.5);
	r1.r = 1;	
	r1.diff = vec3(0, .9, .9);
	r1.spec = vec3(0, .9, .9);
	r1.amb = vec3(0, 0, 0);
	r1.s = 20;
	Sphere* r1_ = new Sphere(SPHERE, REFLECT, r1.pos, r1.amb, r1.diff, r1.spec, r1.s, r1.r);
	scene.push_back(r1_);

	sphere r2;
	r2.pos = vec3(1.5, 0, -1.5);
	r2.r = 1;	
	r2.diff = vec3(.9, 0, .9);
	r2.spec = vec3(.9, 0, .9);
	r2.amb = vec3(0, 0, 0);
	r2.s = 20;
	Sphere* r2_ = new Sphere(SPHERE, REFLECT, r2.pos, r2.amb, r2.diff, r2.spec, r2.s, r2.r);
	scene.push_back(r2_);

	vec3 ppos(0, -1, 0);
	vec3 pn(0, 1, 0);
	vec3 diff(1, 1, 1);
	vec3 spec(0, 0, 0);
	vec3 amb(.1, .1, .1);
	float s = 0;
	Plane* p = new Plane(PLANE, NO_REFLECT, ppos, amb, diff, spec, s, pn);
	scene.push_back(p);

	vec3 ppos2(0, 0, -3);
	vec3 pn2(0, 0, 1);
	vec3 diff2(1, 1, 1);
	vec3 spec2(0, 0, 0);
	vec3 amb2(.1, .1, .1);
	float ss2 = 0;
	Plane* p2 = new Plane(PLANE, NO_REFLECT, ppos2, amb2, diff2, spec2, ss2, pn2);
	scene.push_back(p2);

	light l1;
	l1.pos = vec3(-1, 2, 1);
	l1.i = .5;
	lights.push_back(l1);

	light l2;
	l2.pos = vec3(.5, -.5, 0);
	l2.i = .5;
	lights.push_back(l2);

}

BB getMeshBB(vector<float>& posBuf) {
	BB b {vec3(INT32_MAX), vec3(INT32_MIN)};

	for (unsigned int i = 0; i < posBuf.size(); i += 3) {
		float x = posBuf[i];
		float y = posBuf[i + 1];
		float z = posBuf[i + 2];
		if (x > b.max.x) b.max.x = x;
		if (x < b.min.x) b.min.x = x;
		if (y > b.max.y) b.max.y = y;
		if (y < b.min.y) b.min.y = y;
		if (z > b.max.z) b.max.z = z;
		if (z < b.min.z) b.min.z = z;
	}
	
	return b;
}

sphere sphereFromBB(BB b) {
	sphere s;
	s.pos = 0.5f * (b.max + b.min);
	s.r = max(max(b.max.x - b.min.x, b.max.y - b.min.y), b.max.z - b.min.z) / 2;

	return s;
}

void loadTriangles(vector<tri*>& triangles, vector<float>& posBuf, vector<float>& norBuf) {

	for (long unsigned int num = 0; num < posBuf.size() / 9; num++) {
		int i = num * 9;
		tri* t = new tri;
		// get coordinates
		t->v1.x = posBuf[i];
		t->v1.y = posBuf[i + 1];
		t->v1.z = posBuf[i + 2];
		t->v2.x = posBuf[i + 3];
		t->v2.y = posBuf[i + 4];
		t->v2.z = posBuf[i + 5];
		t->v3.x = posBuf[i + 6];
		t->v3.y = posBuf[i + 7];
		t->v3.z = posBuf[i + 8];

		// get normals
		t->n1.x = norBuf[i]; 
		t->n1.y = norBuf[i + 1];
		t->n1.z = norBuf[i + 2];
		t->n2.x = norBuf[i + 3]; 
		t->n2.y = norBuf[i + 4];
		t->n2.z = norBuf[i + 5];
		t->n3.x = norBuf[i + 6]; 
		t->n3.y = norBuf[i + 7];
		t->n3.z = norBuf[i + 8];

		triangles.push_back(t);
	}

}	

void setup6(string meshName) {

	vector<float> posBuf; // list of vertex positions
	vector<float> norBuf; // list of vertex normals
	vector<float> texBuf; // list of vertex texture coords
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	string errStr;
	bool rc = tinyobj::LoadObj(&attrib, &shapes, &materials, &errStr, meshName.c_str());
	if(!rc) {
		cerr << errStr << endl;
	} else {
		for(size_t s = 0; s < shapes.size(); s++) {
			// Loop over faces (polygons)
			size_t index_offset = 0;
			for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
				size_t fv = shapes[s].mesh.num_face_vertices[f];
				// Loop over vertices in the face.
				for(size_t v = 0; v < fv; v++) {
					// access to vertex
					tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+0]);
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+1]);
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+2]);
					if(!attrib.normals.empty()) {
						norBuf.push_back(attrib.normals[3*idx.normal_index+0]);
						norBuf.push_back(attrib.normals[3*idx.normal_index+1]);
						norBuf.push_back(attrib.normals[3*idx.normal_index+2]);
					}
					if(!attrib.texcoords.empty()) {
						texBuf.push_back(attrib.texcoords[2*idx.texcoord_index+0]);
						texBuf.push_back(attrib.texcoords[2*idx.texcoord_index+1]);
					}
				}
				index_offset += fv;
				// per-face material (IGNORE)
				shapes[s].mesh.material_ids[f];
			}
		}
	}

	cout << meshName << " has " << posBuf.size() << " verticies" << endl;

	vector<tri*> triangles;
	loadTriangles(triangles, posBuf, norBuf); // load info into tri objects
	BB bb = getMeshBB(posBuf); // get bounding box of mesh
	sphere bs = sphereFromBB(bb); // get bounding sphere of bounding box
	Sphere* realBs = new Sphere(SPHERE, NO_REFLECT, bs.pos, vec3(1, 1, 1), vec3(1, 1, 1), vec3(1, 1, 1), 100, bs.r * 1.25);
	
	Mesh* bunny = new Mesh(MESH, NO_REFLECT, vec3(0), vec3(.1, .1, .1), vec3(0, 0, 1), vec3(1, 1, .5), 100, triangles, realBs);
	scene.push_back(bunny);

	light l2;
	l2.pos = vec3(-1, 1, 1);
	l2.i = 1;
	lights.push_back(l2);
}	

void setup7() {

	mat4 T = glm::translate(glm::mat4(1.0f), vec3(0.3f, -1.5f, 0.0f));
	mat4 R = glm::rotate(glm::mat4(1.0f), glm::radians(20.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(1.5f));
	mat4 M = T * R * S;
	
	scene[0]->setTransforms(M);
	scene[0]->getBs()->setTransforms(M);

	lights.pop_back();
	light l2;
	l2.pos = vec3(1, 1, 2);
	l2.i = 1;
	lights.push_back(l2);
}

void setup8(vector<ray*>& rays, int res) {
	rays.clear();

	float h = tan((M_PI * 0.5 * 55) / 180.0);
	float step = 2 * h / res;
	for (float i = h; i > -h; i -= step) {
		for (float j = -h; j < h; j += step) {
			ray* v = new ray;
			v->dir.x = j - step / 2;
			v->dir.y = i - step / 2;
			v->dir.z = -1;

			v->pos.x = j - step / 2;
			v->pos.y = i - step / 2;
			v->pos.z = -.7;

			normalize(v);
			rays.push_back(v);
		}
	}


	glm::vec3 cameraPos(-3.0f, 0.0f, 0.0f);
	glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f); // Look at the origin
	glm::vec3 up(0.0f, 1.0f, 0.0f); // Up vector

	// change shapes
	glm::mat4 viewMatrix = glm::lookAt(cameraPos, cameraTarget, up);
	scene[0]->setTransforms(viewMatrix);
	scene[1]->setTransforms(viewMatrix);
	scene[2]->setTransforms(viewMatrix);
	// change lights
	lights[0].pos = viewMatrix * vec4(lights[0].pos, 1.0);
}

void setup11() {
	sphere s1;
	s1.pos = vec3(.1, -.25, 2.25);
	s1.r = .4;
	s1.diff = vec3(0, 0, 0);
	s1.spec = vec3(0, 0, 0);
	s1.amb = vec3(0, 0, 0);
	s1.s = 100;
	Sphere* s1_ = new Sphere(SPHERE, REFRACT, s1.pos, s1.amb, s1.diff, s1.spec, s1.s, s1.r);
	scene.push_back(s1_);
}

void animationSetup() {
	sphere s1;
	s1.pos = vec3(-1.9, -.5, 1.75);
	// s1.pos = vec3(1.9, -.5, 1.75);
	s1.r = .5;
	s1.diff = vec3(0, 0, 0);
	s1.spec = vec3(0, 0, 0);
	s1.amb = vec3(0, 0, 0);
	s1.s = 100;
	Sphere* s1_ = new Sphere(SPHERE, REFRACT, s1.pos, s1.amb, s1.diff, s1.spec, s1.s, s1.r);
	scene.push_back(s1_);
}

// void animate(vector<ray*>& rays, int reflections, int res, int width, int height) {
	// cast all rays for scene
	// int t = rays.size();
	// int scenes = 38 * 2 / 8;

// }

int main(int argc, char **argv)
{
	// ./A6 <SCENE> <IMAGE SIZE> <IMAGE FILENAME>
	string task = argv[1];
    int taskNum(stoi(task));

	int imageSize(stoi(argv[2]));

	string fileName(argv[3]);
	
	int width = imageSize;
	int height = imageSize;

	int res = imageSize;

	vector<ray*> rays;

	float h = tan((M_PI * 0.5 * 45) / 180.0);
	float step = 2 * h / res;
	for (float i = h; i > -h; i -= step) {
		for (float j = -h; j < h; j += step) {
			ray* v = new ray;
			v->dir.x = j + step / 2;
			v->dir.y = i - step / 2;
			v->dir.z = -1;

			v->pos.x = j - step / 2;
			v->pos.y = i - step / 2;
			v->pos.z = 4;

			normalize(v);
			rays.push_back(v);
		}
	}

	int t = rays.size();
	int reflections = 0;

	switch (taskNum) {
		case 1:
		case 2:
			setup2();	
			break;

		case 3:
			setup3();
			break;
		
		case 4:
			setup5();	
			reflections = 1;
			break;		

		case 5:
			setup5();	
			reflections = 100;
			break;		
			
		case 6:
			setup6("../resources/bunny.obj");	
			break;

		case 7:
			setup6("../resources/bunny.obj");
			setup7();
			break;

		case 8:
			setup2();
			setup8(rays, res);
			break;

		case 9:
			setup5();	
			reflections = 100;
			PHONG_AND_REFLECTIVE = true;
			break;	

		case 10:
			setup5();	
			reflections = 10;
			PHONG_AND_REFLECTIVE = true;
			ROUGH = true;
			break;	

		case 11:
			setup5();
			setup11();
			reflections = 10;
			break;

		case 12:
			setup5();
			setup11();
			reflections = 10;
			PHONG_AND_REFLECTIVE = true;
			ROUGH = true;
			break;

		case 13:
			setup5();
			animationSetup();
			reflections = 5;
			PHONG_AND_REFLECTIVE = true;
			// ROUGH = true;
			// animate(rays, res, reflections, width, height);
			break;
	}

	if (taskNum == 13) {
		int scenes = 38 * 2;
		float step = 3.8f / scenes;

		for (int i = 0; i < scenes; i++) {
			cout << "redering scene " << i << "/" << scenes << endl;

			ThreadPool pool(8);

			for (ray* r : rays) {
				pool.enqueue([&, r, t] {
					castRay(r, scene, reflections);
				});
			}
		
			pool.waitAll();
			Image image = Image(width, height);
			colorImageRay(image, rays, res, width, height);
			image.writeToFile("../resources/animation_pngs/" + to_string(i) + ".png");

			scene.back()->pos.x += step;
		}
		
		return 0;
	}


	ThreadPool pool(8);

	for (ray* r : rays) {
		pool.enqueue([&, r, t] {
			castRay(r, scene, reflections);
		});
	}
	
	pool.waitAll();
	Image image = Image(width, height);
	colorImageRay(image, rays, res, width, height);
	image.writeToFile("../resources/" + fileName + ".png");

	return 0;
}
