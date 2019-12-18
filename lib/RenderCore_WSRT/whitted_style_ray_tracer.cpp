
#include "whitted_style_ray_tracer.h"

using namespace lh2core;

constexpr float kEpsilon = 1e-8;
constexpr float defaultRayBounces = 2;
constexpr float bias = 0.00001;
constexpr float refractiveIndexGlass = 1.5168;
constexpr float refractiveIndexAir = 1.0;
constexpr uint softLightRays = 10;

// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
bool IntersectsWithTriangle(const Ray &ray, const float3 &v0, const float3 &v1, const float3 &v2, float &t, Side &side, float &u, float &v) {
	float3 v0v1 = v1 - v0;
	float3 v0v2 = v2 - v0;
	float3 pvec = cross(ray.direction, v0v2);
	float det = dot(v0v1, pvec);
	// if the determinant is negative the triangle is backfacing
	// if the determinant is close to 0, the ray misses the triangle
	if (det > kEpsilon) {
		side = Front;
	}
	else if (det < -kEpsilon) {
		side = Back;
	}
	else {
		return false;
	}

	float invDet = 1 / det;

	float3 tvec = ray.origin - v0;
	u = dot(tvec, pvec) * invDet;
	if (u < 0 || u > 1) return false;

	float3 qvec = cross(tvec, v0v1);
	v = dot(ray.direction, qvec) * invDet;
	if (v < 0 || u + v > 1) return false;

	t = dot(v0v2, qvec) * invDet;

	return true;
}

// https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
bool BoundingBoxIntersection(const Ray &ray, const aabb &bounds, float &tmin, float &tmax) {
	float3 invD = 1 / ray.direction;
	float3 t0s = (bounds.bmin3 - ray.origin) * invD;
	float3 t1s = (bounds.bmax3 - ray.origin) * invD;

	float3 tsmaller = fminf(t0s, t1s);
	float3 tbigger = fmaxf(t0s, t1s);

	tmin = max(tsmaller.x, max(tsmaller.y, tsmaller.z));
	tmax = min(tbigger.x, min(tbigger.y, tbigger.z));

	return tmin < tmax;
}

void WhittedStyleRayTracer::Render(const ViewPyramid& view, Bitmap* screen) {
	float3 xDirection = (view.p2 - view.p1) / screen->width;
	float3 yDirection = (view.p3 - view.p1) / screen->height;

	float3 p1 = view.p1 - view.pos + 0.5 * xDirection + 0.5 * yDirection;

	Ray ray;

	for (uint u = 0; u < screen->width; u++) {
		for (uint v = 0; v < screen->height; v++) {
			ray.direction = normalize(p1 + u * xDirection + v * yDirection);
			ray.origin = view.pos;
			ray.bounces = defaultRayBounces;

			float3 color = Trace(ray);
			int colorHex = (int(0xff * min(color.x, 1.0f)) + (int(0xff * min(color.y, 1.0f)) << 8) + (int(0xff * min(color.z, 1.0f)) << 16));
			screen->Plot(u, v, colorHex);
		}
	}
}

float3 WhittedStyleRayTracer::Trace(Ray ray) {
	if (ray.bounces < 0) return make_float3(0); //SkyDomeColor(ray, skyDome);

	int numberIntersections = 0;

	Intersection intersection;
	intersection.t = 1e34f;
	bool foundIntersection = NearestIntersection(ray, intersection, numberIntersections);

	//return HSVtoRGB(numberIntersections, 1, 1);

	// normal view
	if (!foundIntersection) return SkyDomeColor(ray, *skyDome);
	if (!foundIntersection) return make_float3(0); else return (intersection.normal + 1.0f) * 0.5f;

	//return HSVtoRGB((int)(intersection.t * 400) % 360, 1, 1);


	Material*material = materials[intersection.tri->material];

	float3 diffuse;
	if (material->texture == NULL) {
		diffuse = material->diffuse;
	}
	else {
		float2 uv0 = make_float2(intersection.tri->u0, intersection.tri->v0);
		float2 uv1 = make_float2(intersection.tri->u1, intersection.tri->v1);
		float2 uv2 = make_float2(intersection.tri->u2, intersection.tri->v2);
		float2 uv = (1 - intersection.u - intersection.v) * uv0 + intersection.u * uv1 + intersection.v * uv2;
		GetColor(uv, *(material->texture));
	}

	// return diffuse;

	//return diffuse * Directllumination(intersection);

	//return Trace(Reflect(ray, intersection));

	return Dielectrics(ray, intersection);

	//// just render the color of the light
	//if (diffuse.x > 1.0 || diffuse.y > 1.0 || diffuse.z > 1.0) return diffuse;

	//float refractive = material.transmission;
	//float reflective = (1 - refractive) * material.specularity;
	//float diffusive = 1 - refractive - reflective;

	//float3 color = make_float3(0.0);

	//if (refractive > 0) {
	//	color += refractive * Dielectrics(ray, intersection);
	//}
	//if (reflective > 0) {
	//	color += diffuse * reflective * Trace(Reflect(ray, intersection));
	//}
	//if (diffusive > 0) {
	//	color += diffusive * diffuse * Directllumination(intersection);
	//}

	//return color;
}

bool WhittedStyleRayTracer::HasIntersection(const Ray &ray, const bool bounded, const float distance) {
	return false;
}

bool WhittedStyleRayTracer::NearestIntersection(const Ray &ray, Intersection &intersection, int &numberIntersections) {
	bool foundIntersection = false;

	if (bvhTop->bvhCount > 0) if (NearestIntersection(*bvhTop->root, ray, intersection, numberIntersections)) foundIntersection = true;

	if (foundIntersection) {
		intersection.position = ray.origin + intersection.t * ray.direction;
		intersection.normal = (1 - intersection.u - intersection.v) * intersection.tri->vN0 + intersection.u * intersection.tri->vN1 + intersection.v * intersection.tri->vN2;
		if (intersection.side == Back) intersection.normal = -intersection.normal;
		intersection.normal = make_float3(make_float4(intersection.normal, 0.0f) * intersection.transform);
	}

	return foundIntersection;
}

bool WhittedStyleRayTracer::NearestIntersection(const BVHTopNode &node, const Ray &ray, Intersection &intersection, int &numberIntersections) {
	float tmin, tmax;
	numberIntersections++;
	if (!BoundingBoxIntersection(ray, node.bounds, tmin, tmax)) return false;
	if (tmin < 0 || tmin > intersection.t) return false;

	if (node.bvh != nullptr) {
		Ray transfomedRay;
		mat4 transform = node.transform.Inverted();
		transfomedRay.origin = make_float3(make_float4(ray.origin, 1.0f) * transform);
		transfomedRay.direction = make_float3(make_float4(ray.direction, 0.0f) * transform);
		bool foundIntersection = NearestIntersection(*(node.bvh), 0, transfomedRay, intersection, numberIntersections);
		if (foundIntersection) intersection.transform = node.transform;
		return foundIntersection;
	}
	else {
		bool foundIntersection = false;
		if (NearestIntersection(*node.left, ray, intersection, numberIntersections)) foundIntersection = true;
		if (NearestIntersection(*node.right, ray, intersection, numberIntersections)) foundIntersection = true;
		return foundIntersection;
	}
}

bool WhittedStyleRayTracer::NearestIntersection(const BVH &bvh, const uint nodeIndex, const Ray &ray, Intersection &intersection, int &numberIntersections) {
	BVHNode *node = &(bvh.pool[nodeIndex]);

	float tmin, tmax;
	numberIntersections++;
	if (!BoundingBoxIntersection(ray, node->bounds, tmin, tmax)) return false;
	if (tmin < 0 || tmin > intersection.t) return false;

	float foundIntersection = false;

	if (node->count == 0) {
		int splitAxis = node->bounds.LongestAxis();
		bool signedRayDirection = get_axis(splitAxis, ray.direction) > 0;

		int left;
		int right;
		if (signedRayDirection) {
			left = node->leftFirst;
			right = left + 1;
		}
		else {
			right = node->leftFirst;
			left = right + 1;
		}

		if (NearestIntersection(bvh, left, ray, intersection, numberIntersections)) foundIntersection = true;
		if (NearestIntersection(bvh, right, ray, intersection, numberIntersections)) foundIntersection = true;
	}
	else {
		uint first = node->leftFirst;
		uint last = first + node->count;

		float bestT = intersection.t;
		float bestU, bestV;
		Side bestSide;
		int bestTri;

		float t, u, v;
		Side side;

		for (int i = first; i < last; i++) {
			int index = bvh.indices[i] * 3;
			float3 a = make_float3(bvh.vertices[index]);
			float3 b = make_float3(bvh.vertices[index + 1]);
			float3 c = make_float3(bvh.vertices[index + 2]);

			if (IntersectsWithTriangle(ray, a, b, c, t, side, u, v) && t > kEpsilon && t < bestT) {
				bestT = t;
				bestU = u;
				bestV = v;
				bestSide = side;
				bestTri = i;
				foundIntersection = true;
			}
		}

		if (foundIntersection) {
			intersection.t = bestT;
			intersection.u = bestU;
			intersection.v = bestV;
			intersection.tri = &(bvh.triangles[bvh.indices[bestTri]]);
			intersection.side = bestSide;
		}
	}

	return foundIntersection;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
Ray WhittedStyleRayTracer::Reflect(const Ray &ray, const Intersection &intersection) {
	Ray reflectRay;

	float3 N = intersection.normal;
	float3 P = intersection.position;

	// taken from lecture slides "whitted-style" slide 13
	reflectRay.direction = ray.direction - 2 * dot(N, ray.direction) * N;
	reflectRay.origin = P + bias * reflectRay.direction;
	reflectRay.bounces = ray.bounces - 1;

	return reflectRay;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
float3 WhittedStyleRayTracer::Directllumination(const Intersection &intersection) {
	float3 illumination = make_float3(0.0);
	Ray ray;

	float3 N = intersection.normal;
	float3 P = intersection.position;

	for (CoreLightTri*areaLight : areaLights) {
		for (int i = 0; i < softLightRays; i++) {
			float r1 = ((double)rand() / RAND_MAX);
			float r2 = ((double)rand() / RAND_MAX);
			float sqrtR1 = sqrt(r1);
			float3 position = (1 - sqrtR1) * areaLight->vertex0 + (sqrtR1 * (1 - r2)) * areaLight->vertex1 + (sqrtR1 * r2) * areaLight->vertex2;

			float3 intersectionLight = position - P;
			float3 lightDirection = normalize(intersectionLight);
			float lightDistance = length(intersectionLight);

			float contribution = areaLight->area * dot(N, lightDirection) * dot(areaLight->N, -lightDirection) / (lightDistance * lightDistance);
			if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

			ray.origin = P + bias * lightDirection;
			ray.direction = lightDirection;

			if (!HasIntersection(ray, true, lightDistance - 2 * bias)) {
				illumination += areaLight->radiance * contribution / softLightRays;
			}
		}
	}

	for (CorePointLight*pointLight : pointLights) {
		float3 intersectionLight = pointLight->position - P;
		float3 lightDirection = normalize(intersectionLight);
		float lightDistance = length(intersectionLight);

		// Code taken from: https://www.gamedev.net/blogs/entry/2260865-shadows-and-point-lights/
		float contribution = dot(N, lightDirection) / (lightDistance * lightDistance);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = P + bias * lightDirection;
		ray.direction = lightDirection;

		if (!HasIntersection(ray, true, lightDistance)) {
			illumination += pointLight->radiance * contribution;
		}
	}

	for (CoreDirectionalLight*directionLight : directionLights) {
		// TODO move normalization to SetLights
		float3 lightDirection = -normalize(directionLight->direction);

		float contribution = dot(N, lightDirection);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = P + bias * lightDirection;
		ray.direction = lightDirection;

		if (!HasIntersection(ray, false, 0)) {
			illumination += directionLight->radiance * contribution;
		}
	}

	for (CoreSpotLight*spotLight : spotLights) {
		float3 intersectionLight = spotLight->position - P;
		float3 lightDirection = normalize(intersectionLight);
		float lightDistance = length(intersectionLight);

		float angle = dot(-lightDirection, spotLight->direction);
		float contribution;

		if (angle < spotLight->cosOuter) {
			continue;
		}
		else if (angle < spotLight->cosInner) {
			contribution = 1 - ((angle - spotLight->cosInner) / (spotLight->cosOuter - spotLight->cosInner));
		}
		else {
			contribution = 1;
		}

		contribution *= dot(N, lightDirection) / (lightDistance * lightDistance);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = P + bias * lightDirection;
		ray.direction = lightDirection;

		if (!HasIntersection(ray, true, lightDistance)) {
			illumination += spotLight->radiance * contribution;
		}

	}

	return illumination;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
float3 WhittedStyleRayTracer::GetColor(const float2 &uv, const Texture &texture) {
	int width = texture.width;
	int height = texture.height;
	float3* pixels = texture.pixels;

	float u = uv.x * (width - 1);
	float v = uv.y * (height - 1);

	uint u1 = floor(u);
	uint u2 = ceil(u);
	uint v1 = floor(v);
	uint v2 = ceil(v);

	float3 a = pixels[u1 + v1 * width];
	float3 b = pixels[u2 + v1 * width];
	float3 c = pixels[u2 + v2 * width];
	float3 d = pixels[u2 + v1 * width];

	float x = u - u1;
	float y = v - v1;

	// bi-linear interpolation
	return lerp(lerp(a, b, x), lerp(c, d, x), y);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
float3 WhittedStyleRayTracer::SkyDomeColor(const Ray &ray, const Texture &texture) {
	float2 uv;
	uv.x = atan2(ray.direction.x, ray.direction.z) / (2 * PI);
	if (uv.x < 0) uv.x += 1;
	uv.y = acos(ray.direction.y) / PI;

	return GetColor(uv, texture);
}

float3 WhittedStyleRayTracer::Dielectrics(const Ray &ray, const Intersection &intersection) {
	float3 N = intersection.normal;
	float3 P = intersection.position;

	float n1, n2;
	switch (intersection.side) {
	case Front:
		n1 = refractiveIndexAir;
		n2 = refractiveIndexGlass;
		break;
	case Back:
		n1 = refractiveIndexGlass;
		n2 = refractiveIndexAir;
		break;
	}

	float n1n2 = n1 / n2;
	float cosO1 = dot(N, -1 * ray.direction);

	float k = 1 - n1n2 * n1n2 * (1 - cosO1 * cosO1);

	// total internal reflection
	if (k < 0) return Trace(Reflect(ray, intersection));

	Ray refractRay;
	refractRay.direction = n1n2 * ray.direction + N * (n1n2 * cosO1 - sqrt(k));
	refractRay.origin = P + bias * -1 * refractRay.direction;
	refractRay.bounces = ray.bounces - 1;

	float fr = Fresnel(ray, intersection, n1, n2, cosO1);
	float ft = 1 - fr;

	float3 diffuse = make_float3(0);
	if (fr > kEpsilon) {
		diffuse += fr * Trace(Reflect(ray, intersection));
	}
	if (ft > kEpsilon) {
		diffuse += ft * Beer(ray, intersection, Trace(refractRay));
	}

	return diffuse;
}

float3 WhittedStyleRayTracer::Beer(const Ray ray, const Intersection &intersection, float3 diffuse) {
	switch (intersection.side) {
	case Front:
		return diffuse;
	case Back:
		Material*material = materials[intersection.tri->material];

		float3 absorption;
		absorption.x = exp(-material->transmittance.x * intersection.t);
		absorption.y = exp(-material->transmittance.y * intersection.t);
		absorption.z = exp(-material->transmittance.z * intersection.t);

		return diffuse * absorption;
	}
}

float WhittedStyleRayTracer::Fresnel(const Ray &ray, const Intersection &intersection, const float n1, const float n2, const float cosOi) {
	float n1n2 = n1 / n2;

	float n1n2SinCos01 = n1n2 * sin(acos(cosOi));
	float cosOt = sqrt(1 - n1n2SinCos01 * n1n2SinCos01);

	float sPolarizedLight = (n1 * cosOi - n2 * cosOt) / (n1 * cosOi + n2 * cosOt);
	float pPolarizedLight = (n1 * cosOt - n2 * cosOi) / (n1 * cosOt + n2 * cosOi);
	float fr = (sPolarizedLight * sPolarizedLight + pPolarizedLight * pPolarizedLight) / 2;

	return fr;
}
