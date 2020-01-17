
#include "whitted_style_ray_tracer.h"
#include <vector>
#include <iostream>

using namespace lh2core;

constexpr float kEpsilon = 1e-8;
constexpr float bias = 0.0001;
constexpr float refractiveIndexGlass = 1.5168;
constexpr float refractiveIndexAir = 1.0;

void RandomDirection(float3&N) {
	N.x = ((float)rand() / RAND_MAX) * 2 - 1;
	N.y = ((float)rand() / RAND_MAX) * 2 - 1;
	N.z = ((float)rand() / RAND_MAX) * 2 - 1;

	while (N.x * N.x + N.y * N.y + N.z * N.z > 1) {
		N.x = ((float)rand() / RAND_MAX) * 2 - 1;
		N.y = ((float)rand() / RAND_MAX) * 2 - 1;
		N.z = ((float)rand() / RAND_MAX) * 2 - 1;
	}
}

void RandomDirectionHemisphere(const float3&N, float3&dir) {
	RandomDirection(dir);
	dir = dot(N, dir) > 0 ? dir : -dir;
}

float3 GetRandomPointOnAreaLight(CoreLightTri* areaLight) {
	float r1 = ((float)rand() / RAND_MAX);
	float r2 = ((float)rand() / RAND_MAX);
	float sqrtR1 = sqrt(r1);
	return (1 - sqrtR1) * areaLight->vertex0 + (sqrtR1 * (1 - r2)) * areaLight->vertex1 + (sqrtR1 * r2) * areaLight->vertex2;
}

CoreLightTri* WhittedStyleRayTracer::GetRandomLight() {
	if (areaLights.size() == 0) return nullptr;

	int lightIndex = rand() % areaLights.size();
	CoreLightTri* areaLight = areaLights[lightIndex];
	return areaLight;
}

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
bool BoundingBoxIntersection(const Ray &ray, const aabb &bounds, float&tmin, float&tmax) {
	float3 invD = 1 / ray.direction;
	float3 t0s = (bounds.bmin3 - ray.origin) * invD;
	float3 t1s = (bounds.bmax3 - ray.origin) * invD;

	float3 tsmaller = fminf(t0s, t1s);
	float3 tbigger = fmaxf(t0s, t1s);

	tmin = max(tsmaller.x, max(tsmaller.y, tsmaller.z));
	tmax = min(tbigger.x, min(tbigger.y, tbigger.z));

	return tmin < tmax;
}

void WhittedStyleRayTracer::ResizeScreen(const int width, const int height) {
	delete accumulator;
	accumulator = new float3[width * height];
}

IntersectionShading WhittedStyleRayTracer::intersectionTraverseToIntersectionShading(const IntersectionTraverse&intersectionTraverse, const Ray&ray) {
	IntersectionShading intersection;

	if (intersectionTraverse.t == 1e34f) {
		intersection.hasIntersection = false;
		return intersection;
	}

	BVHTopNode topBvhNode = bvhTop->pool[intersectionTraverse.tri >> 24];
	mat4 transform = bvhTop->transforms[intersectionTraverse.tri >> 24];

	BVH*bvh = bvhs[topBvhNode.MeshIndex()];
	intersection.hasIntersection = true;
	intersection.tri = &bvh->triangles[bvh->indices[intersectionTraverse.tri & ((2 << 23) - 1)]];
	intersection.t = intersectionTraverse.t;
	intersection.position = ray.origin + intersectionTraverse.t * ray.direction;
	intersection.normal = normalize((1 - intersectionTraverse.u - intersectionTraverse.v) * intersection.tri->vN0 + intersectionTraverse.u * intersection.tri->vN1 + intersectionTraverse.v * intersection.tri->vN2);
	intersection.side = dot(ray.direction, intersection.normal) < 0 ? Front : Back;
	if (intersection.side == Back) intersection.normal = -intersection.normal;
	intersection.normal = make_float3(make_float4(intersection.normal, 0.0f) * transform);

	Material*material = materials[intersection.tri->material];

	if (material->texture == NULL) {
		intersection.diffuse = material->diffuse;
	}
	else {
		float2 uv0 = make_float2(intersection.tri->u0, intersection.tri->v0);
		float2 uv1 = make_float2(intersection.tri->u1, intersection.tri->v1);
		float2 uv2 = make_float2(intersection.tri->u2, intersection.tri->v2);
		float2 uv = (1 - intersectionTraverse.u - intersectionTraverse.v) * uv0 + intersectionTraverse.u * uv1 + intersectionTraverse.v * uv2;
		intersection.diffuse = GetColor(uv, *(material->texture));
	}

	return intersection;
}

void lh2core::WhittedStyleRayTracer::ShootLightRays(const uint numberOfRays)
{
	Ray ray;
	IntersectionTraverse* intersectionTraverse = (IntersectionTraverse*)_aligned_malloc(sizeof(IntersectionTraverse), 64);
	int numberOfIntersections;
	photons = (Photon*)_aligned_malloc(numberOfRays * areaLights.size() * sizeof(Photon), 64);
	int photonPtr = 0;
	Photon photon;
	for (CoreLightTri* areaLight : areaLights) {
		for (int i = 0; i < numberOfRays; i++) {
			float r1 = ((float)rand() / RAND_MAX);
			float r2 = ((float)rand() / RAND_MAX);
			float sqrtR1 = sqrt(r1);

			float3 position = (1 - sqrtR1) * areaLight->vertex0 + (sqrtR1 * (1 - r2)) * areaLight->vertex1 + (sqrtR1 * r2) * areaLight->vertex2;
			float3 direction;
			RandomDirectionHemisphere(areaLight->N, direction);

			ray.origin = position + bias * direction;
			ray.direction = direction;
			intersectionTraverse->t = 1e34f;
			NearestIntersection(ray, *intersectionTraverse, numberOfIntersections);

			if (intersectionTraverse->t != 1e34f) {
				photon.power = areaLight->radiance;
				photon.position = ray.origin + ray.direction * intersectionTraverse->t;
				photon.L = -ray.direction;

				photons[photonPtr++] = photon;
			}
		}
	}
}

void WhittedStyleRayTracer::Render(const ViewPyramid&view, Bitmap*screen, const Convergence converge) {
	float3 xStep = (view.p2 - view.p1) / screen->width;
	float3 yStep = (view.p3 - view.p1) / screen->height;
	float3 xDirection = normalize(xStep);
	float3 yDirection = normalize(yStep);

	if (converge == Restart) {
		accumulatorIndex = 0;
		for (int i = 0, l = screen->width * screen->height; i < l; i++) {
			accumulator[i] = make_float3(0.0f);
		}
	}

	float t = (float)accumulatorIndex / (float)(accumulatorIndex + 1);

	Ray ray;
	float3 rayTarget;
	float2 apertureOffset;
	float3 albedo;
	for (uint u = 0; u < screen->width; u++) {
		for (uint v = 0; v < screen->height; v++) {
			rayTarget = view.p1 + (u + ((float)rand() / RAND_MAX)) * xStep + (v + ((float)rand() / RAND_MAX)) * yStep;

			apertureOffset = make_float2(((float)rand() / RAND_MAX) * 2 - 1, ((float)rand() / RAND_MAX) * 2 - 1);
			while (dot(apertureOffset, apertureOffset) > 1) {
				apertureOffset.x = ((float)rand() / RAND_MAX) * 2 - 1;
				apertureOffset.y = ((float)rand() / RAND_MAX) * 2 - 1;
			}  
			apertureOffset *= view.aperture;

			ray.origin = view.pos + apertureOffset.x * xDirection + apertureOffset.y * yDirection;
			ray.direction = normalize(rayTarget - ray.origin);

			albedo = lerp(Trace(ray, true), accumulator[v * screen->width + u], t);
			accumulator[v * screen->width + u] = albedo;
			int colorHex = (int(0xff * min(albedo.x, 1.0f)) + (int(0xff * min(albedo.y, 1.0f)) << 8) + (int(0xff * min(albedo.z, 1.0f)) << 16));
			screen->Plot(u, v, colorHex);
		}
	}

	accumulatorIndex = accumulatorIndex + 1;
}

float3 WhittedStyleRayTracer::Trace(Ray ray, bool lastSpecular) {
	IntersectionTraverse intersectionTraverse;
	int numberIntersections = 0;

	intersectionTraverse.Reset();
	NearestIntersection(ray, intersectionTraverse, numberIntersections);
	IntersectionShading intersection = intersectionTraverseToIntersectionShading(intersectionTraverse, ray);

	// heatmap
	//return HSVtoRGB(numberIntersections, 1, 1);

	// normal view
	// return foundIntersection ? (intersection.normal + 1.0f) * 0.5f : make_float3(0);

	// depth view
	//return HSVtoRGB((int)(intersection.t * 400) % 360, 1, 1);

	if (!intersection.hasIntersection) {
		//return SkyDomeColor(ray, *skyDome);
		return make_float3(0.0f);
	}
	else {
		if (intersection.diffuse.x > 1.0f || intersection.diffuse.y > 1.0f || intersection.diffuse.x > 1.0f) {
			//return intersection.diffuse;
			return lastSpecular ? intersection.diffuse : make_float3(0.0f);
		} else {
			float3 BRDF = intersection.diffuse * INVPI;

			RandomDirectionHemisphere(intersection.normal, ray.direction);
			ray.origin = intersection.position + bias * ray.direction;

			float3 albedo = PI * 2.0f * BRDF * Trace(ray, false) * dot(ray.direction, intersection.normal);

			// NEE
			CoreLightTri* areaLight = GetRandomLight();
			if (areaLight != nullptr) {
				float3 position = GetRandomPointOnAreaLight(areaLight);

				float3 intersectionLight = position - intersection.position;
				float3 lightDirection = normalize(intersectionLight);
				float lightDistance = length(intersectionLight);

				float contribution = areaLight->area * dot(intersection.normal, lightDirection) * dot(areaLight->N, -lightDirection) / (lightDistance * lightDistance);
				if (contribution > 0) { // don't calculate illumination for intersections facing away from the light
					ray.origin = intersection.position + bias * lightDirection;
					ray.direction = lightDirection;

					if (!HasIntersection(ray, true, lightDistance - 2 * bias)) {
						albedo += BRDF * areaLight->radiance * contribution;
					}
				}
			}
			return albedo;
		}
	}
}

bool WhittedStyleRayTracer::HasIntersection(const Ray &ray, const bool bounded, const float distance) {
	BVHTopNode node;
	Ray transfomedRay;
	mat4 transform;
	int nodeIndex;
	float tmin, tmax;

	vector<int> stack;
	stack.reserve(bvhTop->bvhCount * 2 - 1);

	if (bvhTop->bvhCount > 0) stack.push_back(bvhTop->bvhCount * 2 - 2);

	while (!stack.empty()) {
		nodeIndex = stack.back();
		stack.pop_back();

		node = bvhTop->pool[nodeIndex];

		if (!BoundingBoxIntersection(ray, node.bounds, tmin, tmax)) continue;
		if (tmax < kEpsilon) continue;

		if (nodeIndex < bvhTop->bvhCount) {
			// leaf
			transform = bvhTop->transforms[nodeIndex];

			transfomedRay.origin = make_float3(make_float4(ray.origin, 1.0f) * transform);
			transfomedRay.direction = make_float3(make_float4(ray.direction, 0.0f) * transform);

			if (HasIntersection(*bvhs[node.MeshIndex()], transfomedRay, bounded, distance)) return true;
		}
		else {
			stack.push_back(node.LeftIndex());
			stack.push_back(node.RightIndex());
		}
	}

	return false;
}

bool WhittedStyleRayTracer::HasIntersection(const BVH &bvh, const Ray &ray, const bool bounded, const float distance) {
	BVHNode node;
	vector<int> stack;
	int nodeId;
	float tmin, tmax;
	float tminLeft, tmaxLeft;
	float tminRight, tmaxRight;
	int left, right;
	float t, u, v;
	Side side;
	float3 a, b, c;
	int index, i;
	uint first, last;
	bool intersectLeft, intersectRight;

	//stack.reserve(30);
	stack.reserve(log2(bvh.vcount / 3) * 1.5);

	if (BoundingBoxIntersection(ray, bvh.pool[0].bounds, tmin, tmax) && tmax > kEpsilon && (!bounded || tmin < distance)) stack.push_back(0);

	while (!stack.empty()) {
		node = bvh.pool[stack.back()];
		stack.pop_back();

		if (node.count == 0) {
			left = node.leftFirst;
			right = left + 1;

			intersectLeft = BoundingBoxIntersection(ray, bvh.pool[left].bounds, tminLeft, tmaxLeft);
			intersectRight = BoundingBoxIntersection(ray, bvh.pool[right].bounds, tminRight, tmaxRight);

			if (tminLeft < tminRight) {
				if (intersectRight && tmaxRight > kEpsilon) stack.push_back(right);
				if (intersectLeft && tmaxLeft > kEpsilon) stack.push_back(left);
			}
			else {
				if (intersectLeft && tmaxLeft > kEpsilon) stack.push_back(left);
				if (intersectRight && tmaxRight > kEpsilon) stack.push_back(right);
			}
		}
		else {
			first = node.leftFirst;
			last = first + node.count;

			for (i = first; i < last; i++) {
				index = bvh.indices[i] * 3;
				a = make_float3(bvh.vertices[index]);
				b = make_float3(bvh.vertices[index + 1]);
				c = make_float3(bvh.vertices[index + 2]);

				if (IntersectsWithTriangle(ray, a, b, c, t, side, u, v) && t > kEpsilon && (!bounded || t < distance)) return true;
			}
		}
	}

	return false;
}

void WhittedStyleRayTracer::NearestIntersection(const Ray&ray, IntersectionTraverse&intersection, int &numberIntersections) {
	BVHTopNode node;
	Ray transfomedRay;
	mat4 transform;
	int nodeIndex;
	float tmin, tmax;

	vector<int> stack;
	stack.reserve(bvhTop->bvhCount * 2 - 1);

	if (bvhTop->bvhCount > 0) stack.push_back(bvhTop->bvhCount * 2 - 2);

	while (!stack.empty()) {
		nodeIndex = stack.back();
		stack.pop_back();

		node = bvhTop->pool[nodeIndex];

		if (!BoundingBoxIntersection(ray, node.bounds, tmin, tmax)) continue;
		if (tmax < kEpsilon || tmin > intersection.t) continue;

		if (nodeIndex < bvhTop->bvhCount) {
			// leaf
			transform = bvhTop->transforms[nodeIndex];

			transfomedRay.origin = make_float3(make_float4(ray.origin, 1.0f) * transform);
			transfomedRay.direction = make_float3(make_float4(ray.direction, 0.0f) * transform);

			NearestIntersection(*bvhs[node.MeshIndex()], transfomedRay, intersection, nodeIndex, numberIntersections);
		}
		else {
			stack.push_back(node.LeftIndex());
			stack.push_back(node.RightIndex());
		}
	}
}

void WhittedStyleRayTracer::NearestIntersection(const BVH&bvh, const Ray &ray, IntersectionTraverse&intersection, const int instanceIdx, int &numberIntersections) {
	BVHNode node;
	vector<tuple<float, int>> stack;
	int nodeId;
	float tmin, tmax;
	float tminLeft, tmaxLeft;
	float tminRight, tmaxRight;
	int left, right;
	float t, u, v;
	Side side;
	float3 a, b, c;
	int index, i;
	uint first, last;
	bool intersectLeft, intersectRight;

	//stack.reserve(30);
	stack.reserve(log2(bvh.vcount / 3) * 1.5);

	numberIntersections++;
	if (BoundingBoxIntersection(ray, bvh.pool[0].bounds, tmin, tmax) && tmax > kEpsilon) stack.push_back(make_tuple(tmin, 0));

	while (!stack.empty()) {
		tie(tmin, nodeId) = stack.back();
		stack.pop_back();

		if (tmin > intersection.t) continue;

		node = bvh.pool[nodeId];

		if (node.count == 0) {
			left = node.leftFirst;
			right = left + 1;

			numberIntersections+=2;
			intersectLeft = BoundingBoxIntersection(ray, bvh.pool[left].bounds, tminLeft, tmaxLeft);
			intersectRight = BoundingBoxIntersection(ray, bvh.pool[right].bounds, tminRight, tmaxRight);

			if (tminLeft < tminRight) {
				if (intersectRight && tmaxRight > kEpsilon) stack.push_back(make_tuple(tminRight, right));
				if (intersectLeft && tmaxLeft > kEpsilon) stack.push_back(make_tuple(tminLeft, left));
			}
			else {
				if (intersectLeft && tmaxLeft > kEpsilon) stack.push_back(make_tuple(tminLeft, left));
				if (intersectRight && tmaxRight > kEpsilon) stack.push_back(make_tuple(tminRight, right));
			}
		}
		else {
			first = node.leftFirst;
			last = first + node.count;

			for (i = first; i < last; i++) {
				index = bvh.indices[i] * 3;
				a = make_float3(bvh.vertices[index]);
				b = make_float3(bvh.vertices[index + 1]);
				c = make_float3(bvh.vertices[index + 2]);

				if (IntersectsWithTriangle(ray, a, b, c, t, side, u, v) && t > kEpsilon && t < intersection.t) {
					intersection.t = t;
					intersection.u = u;
					intersection.v = v;

					intersection.tri = i | (instanceIdx << 24);
				}
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
Ray WhittedStyleRayTracer::Reflect(const Ray &ray, const IntersectionShading&intersection) {
	Ray reflectRay;

	float3 N = intersection.normal;
	float3 P = intersection.position;

	// taken from lecture slides "whitted-style" slide 13
	reflectRay.direction = ray.direction - 2 * dot(N, ray.direction) * N;
	reflectRay.origin = P + bias * reflectRay.direction;

	return reflectRay;
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

float3 WhittedStyleRayTracer::Dielectrics(const Ray &ray, const IntersectionShading &intersection) {
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
	if (k < kEpsilon) return Trace(Reflect(ray, intersection), true);

	Ray refractRay;
	refractRay.direction = n1n2 * ray.direction + N * (n1n2 * cosO1 - sqrt(k));
	refractRay.origin = P + bias * -1 * refractRay.direction;

	float fr = Fresnel(ray, intersection, n1, n2, cosO1);
	float ft = 1 - fr;

	float3 diffuse = make_float3(0);
	if (fr > kEpsilon) {
		diffuse += fr * Trace(Reflect(ray, intersection), true);
	}
	if (ft > kEpsilon) {
		diffuse += ft * Beer(intersection, Trace(refractRay, true));
	}

	return diffuse;
}

float3 WhittedStyleRayTracer::Beer(const IntersectionShading&intersection, float3 diffuse) {
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

float WhittedStyleRayTracer::Fresnel(const Ray &ray, const IntersectionShading&intersection, const float n1, const float n2, const float cosOi) {
	float n1n2 = n1 / n2;

	float n1n2SinCos01 = n1n2 * sin(acos(cosOi));
	float cosOt = sqrt(1 - n1n2SinCos01 * n1n2SinCos01);

	float sPolarizedLight = (n1 * cosOi - n2 * cosOt) / (n1 * cosOi + n2 * cosOt);
	float pPolarizedLight = (n1 * cosOt - n2 * cosOi) / (n1 * cosOt + n2 * cosOi);
	float fr = (sPolarizedLight * sPolarizedLight + pPolarizedLight * pPolarizedLight) / 2;

	return fr;
}
