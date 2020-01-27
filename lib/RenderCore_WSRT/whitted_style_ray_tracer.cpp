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

	N = normalize(N);
}

void RandomDirectionHemisphere(const float3&N, float3&dir) {
	RandomDirection(dir);
	dir = dot(N, dir) > 0 ? dir : -dir;
}

void TangentSpace(const float3&n, float3&N, float3&T, float3&B) {
	float3 W = abs(n.x) > 0.99 ? make_float3(0.0f, 1.0f, 0.0f) : make_float3(1.0f, 0.0f, 0.0f);

	N = n;
	T = normalize(cross(N, W));
	B = cross(T, N);
}

void ToLocalSpace(const float3&N, const float3&T, const float3&B, const float3&PWorld, float3&PLocal) {
	PLocal.x = dot(PWorld, T);
	PLocal.y = dot(PWorld, B);
	PLocal.z = dot(PWorld, N);
}

void ToWorldSpace(const float3&N, const float3&T, const float3&B, const float3&PLocal, float3&Pworld) {
	Pworld = PLocal.x * T + PLocal.y * B + PLocal.z * N;
}

void SampleDiffuseReflection(const float3&n, float3&dir, float&PDF) {
	RandomDirectionHemisphere(n, dir);
	PDF = 1 / (2 * PI);
}

void SampleCosineWeightedDiffuseReflection(const float3&n, float3&dir, float&PDF) {
	float r0 = ((float)rand() / RAND_MAX), r1 = ((float)rand() / RAND_MAX);
	float r = sqrt(r0);
	float theta = 2 * PI * r1;
	dir.x = r * cosf(theta);
	dir.y = r * sinf(theta);
	dir.z = sqrt(1 - r0);

	float3 N, T, B;
	TangentSpace(n, N, T, B);
	ToWorldSpace(N, T, B, dir, dir);

	PDF = max(kEpsilon, dot(n, dir) / PI);
}

void SampleGlossyReflection(const float3&n, float const&a, float3&dir, float&PDF) {
	float3 reflectDir = reflect(dir, n);

	float r1 = ((float)rand() / RAND_MAX), r2 = ((float)rand() / RAND_MAX);

	float t = pow(r2, 2 / (a + 1));
	dir.x = cos(2 * PI * r1) * sqrt(1 - t);
	dir.y = sin(2 * PI * r1) * sqrt(1 - t);
	dir.z = sqrt(t);

	float3 N, T, B;
	TangentSpace(n, N, T, B);
	ToWorldSpace(N, T, B, dir, dir);

	PDF = max(kEpsilon, (a + 2) * pow(dot(reflectDir, dir), a));
}

float3 MicroFacetBRDF(const float3&N, const float&a, const float3&L, const float3&V) {
	float3 H = normalize(V + L);
	float D = (a + 2.0f) / (2.0 * PI) * max(0.0f, pow(dot(N, H), a));

	float NDotH = dot(N, H);
	float NDotV = dot(N, V);
	float VDotH = dot(V, H);
	float NDotL = dot(N, L);
	float LDotH = dot(L, H);
	float G = min(1.0f, min(
		max(0.0f, 2.0f * NDotH * NDotV) / max(kEpsilon, VDotH),
		max(0.0f, 2.0f * NDotH * NDotL) / max(kEpsilon, VDotH)
	));

	float3 kSpecular = make_float3(0.56, 0.57, 0.58);
	float3 F = kSpecular + (1 - kSpecular) * pow(1 - LDotH, 5);

	return (F * G * D) / max(kEpsilon, (4 * NDotL * NDotV));
}

float3 GetColor(const float2 &uv, const Texture &texture) {
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

float3 SkyDomeColor(const Ray &ray, const Texture &texture) {
	float2 uv;
	uv.x = atan2(ray.direction.x, ray.direction.z) / (2 * PI);
	if (uv.x < 0) uv.x += 1;
	uv.y = acos(ray.direction.y) / PI;

	return GetColor(uv, texture);
}

void WhittedStyleRayTracer::GetRandomLight(const IntersectionShading&intersection, CoreLightTri*&areaLight, float3&lightPosition, float&p) {
	if (areaLights.size() == 0) {
		areaLight = nullptr;
		return;
	}

	aabb dim = bvhTop->pool[bvhTop->bvhCount * 2 - 2].bounds;
	int gridIndex = photonMap->CoordToIndex(intersection.position, dim);
	double random = ((double)rand() / RAND_MAX);

	float lastProbability = 0.0f;
	for (int i = 0; i < NrCDFLights; i++) {
		float probability = photonMap->cdfGrid[gridIndex].probabilities[i];
		if (probability >= random) {
			int lightIndex = photonMap->cdfGrid[gridIndex].lightIndices[i];
			if (lightIndex == -1) {
				areaLight = areaLights[rand() % areaLights.size()];
				p = max(kEpsilon, 1.0f / (float)areaLights.size());
			}
			else {
				areaLight = areaLights[lightIndex];
				p = probability - lastProbability;
			}
			break;
		}
		lastProbability = probability;
	}


	float r1 = ((float)rand() / RAND_MAX);
	float r2 = ((float)rand() / RAND_MAX);
	float sqrtR1 = sqrt(r1);
	lightPosition = (1 - sqrtR1) * areaLight->vertex0 + (sqrtR1 * (1 - r2)) * areaLight->vertex1 + (sqrtR1 * r2) * areaLight->vertex2;
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
	CoreTri*tri = &bvh->triangles[bvh->indices[intersectionTraverse.tri & ((2 << 23) - 1)]];

	intersection.hasIntersection = true;
	intersection.t = intersectionTraverse.t;
	intersection.position = ray.origin + intersectionTraverse.t * ray.direction;
	intersection.normal = normalize((1 - intersectionTraverse.u - intersectionTraverse.v) * tri->vN0 + intersectionTraverse.u * tri->vN1 + intersectionTraverse.v * tri->vN2);
	intersection.side = dot(ray.direction, intersection.normal) < 0 ? Front : Back;
	if (intersection.side == Back) intersection.normal = -intersection.normal;
	intersection.normal = make_float3(make_float4(intersection.normal, 0.0f) * transform);

	Material*material = materials[tri->material];

	if (material->texture == NULL) {
		intersection.diffuse = material->diffuse;
	}
	else {
		float2 uv0 = make_float2(tri->u0, tri->v0);
		float2 uv1 = make_float2(tri->u1, tri->v1);
		float2 uv2 = make_float2(tri->u2, tri->v2);
		float2 uv = (1 - intersectionTraverse.u - intersectionTraverse.v) * uv0 + intersectionTraverse.u * uv1 + intersectionTraverse.v * uv2;
		intersection.diffuse = GetColor(uv, *(material->texture));
	}

	return intersection;
}

void lh2core::WhittedStyleRayTracer::ShootLightRays() {
	Ray ray;
	IntersectionTraverse intersectionTraverse;
	IntersectionShading intersection;
	int numberOfIntersections;
	int emittedNumberOfPhotons = 0;
	int* photonsPerLight = (int*)_aligned_malloc(areaLights.size() * sizeof(int), 64);

	for (int i = 0; i < areaLights.size(); i++) {
		int numberOfPhotons = length(areaLights[i]->radiance) * areaLights[i]->area * 10000;
		photonsPerLight[i] = numberOfPhotons;
		emittedNumberOfPhotons += numberOfPhotons;
	}

	cout << "Photons emmitted:" << emittedNumberOfPhotons << endl;

	Photon* photons = (Photon*)_aligned_malloc(emittedNumberOfPhotons * sizeof(Photon), 64);
	int photonPtr = 0;
	Photon photon;

	for (int i = 0; i < areaLights.size(); i++) {

		CoreLightTri* areaLight = areaLights[i];
		uint numberOfRays = photonsPerLight[i];

		for (int j = 0; j < numberOfRays; j++) {
			float r1 = ((float)rand() / RAND_MAX);
			float r2 = ((float)rand() / RAND_MAX);
			float sqrtR1 = sqrt(r1);

			float3 position = (1 - sqrtR1) * areaLight->vertex0 + (sqrtR1 * (1 - r2)) * areaLight->vertex1 + (sqrtR1 * r2) * areaLight->vertex2;
			float3 direction;
			RandomDirectionHemisphere(areaLight->N, direction);

			ray.origin = position + bias * direction;
			ray.direction = direction;
			intersectionTraverse.Reset();
			NearestIntersection(ray, intersectionTraverse);
			intersection = intersectionTraverseToIntersectionShading(intersectionTraverse, ray);

			if (intersection.hasIntersection) {
				photon.energy = length(areaLight->radiance) * dot(intersection.normal, -ray.direction);
				photon.position = ray.origin + ray.direction * intersectionTraverse.t;
				photon.lightIndex = i;

				photons[photonPtr++] = photon;
			}
		}
	}

	cout << "Photons landed:" << photonPtr << endl;

	aabb dim = bvhTop->pool[bvhTop->bvhCount * 2 - 2].bounds;

	photonMap = new PhotonMap(photons, photonPtr, dim, areaLights.size());
	_aligned_free(photons);
	_aligned_free(photonsPerLight);
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

	float totalEnergy = 0;

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

			albedo = lerp(Trace(ray), accumulator[v * screen->width + u], t);
			accumulator[v * screen->width + u] = albedo;
			int colorHex = int(0xff * min(albedo.x, 1.0f)) + (int(0xff * min(albedo.y, 1.0f)) << 8) + (int(0xff * min(albedo.z, 1.0f)) << 16);
			screen->Plot(u, v, colorHex);

			totalEnergy += albedo.x + albedo.y + albedo.z;
		}
	}

	//cout << totalEnergy << endl;

	accumulatorIndex = accumulatorIndex + 1;
}

//float3 WhittedStyleRayTracer::Trace(Ray ray) {
//	IntersectionShading intersection;
//	IntersectionTraverse intersectionTraverse;
//
//	intersectionTraverse.Reset();
//	NearestIntersection(ray, intersectionTraverse);
//	intersection = intersectionTraverseToIntersectionShading(intersectionTraverse, ray);
//
//	if (!intersection.hasIntersection) return SkyDomeColor(ray, *skyDome);
//	//if (!intersection.hasIntersection) return make_float3(0.0f);
//	if (intersection.diffuse.x > 1.0f || intersection.diffuse.y > 1.0f || intersection.diffuse.z > 1.0f) return intersection.diffuse;
//
//	float3 oldRay = ray.direction;
//	float a = 10.0f;
//
//	float PDF;
//	//SampleCosineWeightedDiffuseReflection(intersection.normal, ray.direction, PDF);
//	SampleDiffuseReflection(intersection.normal, ray.direction, PDF);
//
//	//float3 BRDF = MicroFacetBRDF(intersection.normal, a, -oldRay, ray.direction);
//	float3 BRDF = intersection.diffuse * INVPI;
//
//	ray.origin = intersection.position + bias * ray.direction;
//
//	return BRDF * dot(ray.direction, intersection.normal) * Trace(ray) / PDF;
//}

float3 WhittedStyleRayTracer::Trace(Ray ray) {
	//return HSVtoRGB(photonMap->NumberOfIntersections(ray), 1, 1);
	Timer t{};
	IntersectionShading intersection;
	IntersectionTraverse intersectionTraverse;

	t.reset(); intersectionTraverse.Reset();

	NearestIntersection(ray, intersectionTraverse);
	intersection = intersectionTraverseToIntersectionShading(intersectionTraverse, ray);

	coreStats->primaryRayCount++; coreStats->traceTime0 += t.elapsed();

	//return intersection.hasIntersection ? intersection.diffuse : make_float3(0.0f);
	//return intersection.hasIntersection ? (intersection.normal + 1.0f) / 2.0f : make_float3(0.0f);

	float3 albedo = make_float3(1.0f);
	float3 albedoLight = make_float3(0.0f);

	if (intersection.hasIntersection && (intersection.diffuse.x > 1.0f || intersection.diffuse.y > 1.0f || intersection.diffuse.z > 1.0f)) {
		return intersection.diffuse;
	}

	while (intersection.hasIntersection) {
		if (intersection.diffuse.x > 1.0f || intersection.diffuse.y > 1.0f || intersection.diffuse.z > 1.0f) {
			return albedoLight;
		}

		float3 BRDF = intersection.diffuse * INVPI;

		// NEE
		CoreLightTri* areaLight;
		float3 lightPosition;
		float probability;
		GetRandomLight(intersection, areaLight, lightPosition, probability);

		if (areaLight != nullptr) {
			float3 intersectionLight = lightPosition - intersection.position;
			float3 lightDirection = normalize(intersectionLight);
			float lightDistance = length(intersectionLight);

			float nDotL = dot(intersection.normal, lightDirection);
			float nLDotMinL = dot(areaLight->N, -lightDirection);

			if (nLDotMinL > 0 && nDotL > 0) { // don't calculate illumination for intersections facing away from the light
				ray.origin = intersection.position + bias * lightDirection;
				ray.direction = lightDirection;
				t.reset();
				bool lightObstructed = HasIntersection(ray, true, lightDistance - 2 * bias);
				coreStats->totalShadowRays++; coreStats->shadowTraceTime += t.elapsed();

				if (!lightObstructed) {
					float solidAngle = (nLDotMinL * areaLight->area) / (lightDistance * lightDistance);
					float lightPDF = 1 / solidAngle;
					albedoLight += (albedo * (nDotL / lightPDF) * BRDF * areaLight->radiance) / probability;
				}
			}
		}

		// russian roulette
		float pSurvive = clamp(max(max(albedo.x, albedo.y), albedo.z), 0.0f, 1.0f);
		if ((float)rand() / RAND_MAX > pSurvive) {
			return albedoLight;
		}
		albedo /= max(kEpsilon, pSurvive);

		float PDF;
		SampleCosineWeightedDiffuseReflection(intersection.normal, ray.direction, PDF);
		ray.origin = intersection.position + bias * ray.direction;

		albedo *= BRDF * dot(ray.direction, intersection.normal) / PDF;

		intersectionTraverse.Reset(); t.reset();

		NearestIntersection(ray, intersectionTraverse);
		intersection = intersectionTraverseToIntersectionShading(intersectionTraverse, ray);

		coreStats->bounce1RayCount++; coreStats->traceTime1 += t.elapsed();
	}

	return albedoLight + SkyDomeColor(ray, *skyDome) * albedo;
	//return albedoLight;
}

bool WhittedStyleRayTracer::HasIntersection(const Ray&ray, const bool bounded, const float distance) {
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
	vector<int> stack;

	//stack.reserve(30);
	stack.reserve(log2(bvh.vcount / 3) * 1.5);

	float tmin, tmax;
	if (BoundingBoxIntersection(ray, bvh.pool[0].bounds, tmin, tmax) && tmax > kEpsilon && (!bounded || tmin < distance)) stack.push_back(0);

	while (!stack.empty()) {
		BVHNode node = bvh.pool[stack.back()];
		stack.pop_back();

		if (node.count == 0) {
			int left = node.leftFirst;
			int right = left + 1;

			float tmin, tmax;

			if (BoundingBoxIntersection(ray, bvh.pool[left].bounds, tmin, tmax) && tmax > kEpsilon) stack.push_back(left);
			if (BoundingBoxIntersection(ray, bvh.pool[right].bounds, tmin, tmax) && tmax > kEpsilon) stack.push_back(right);
		}
		else {
			for (int i = node.leftFirst, l = node.leftFirst + node.count; i < l; i++) {
				int index = bvh.indices[i] * 3;
				float3 a = make_float3(bvh.vertices[index]);
				float3 b = make_float3(bvh.vertices[index + 1]);
				float3 c = make_float3(bvh.vertices[index + 2]);
				Side side;
				float t, u, v;

				if (IntersectsWithTriangle(ray, a, b, c, t, side, u, v) && t > kEpsilon && (!bounded || t < distance)) return true;
			}
		}
	}

	return false;
}

void WhittedStyleRayTracer::NearestIntersection(const Ray&ray, IntersectionTraverse&intersection) {
	vector<int> stack;
	if (bvhTop->bvhCount > 0) {
		stack.push_back(bvhTop->bvhCount * 2 - 2);
		stack.reserve(bvhTop->bvhCount * 2 - 1);
	}

	while (!stack.empty()) {
		int nodeIndex = stack.back();
		stack.pop_back();

		BVHTopNode node = bvhTop->pool[nodeIndex];

		float tmin, tmax;
		if (!BoundingBoxIntersection(ray, node.bounds, tmin, tmax)) continue;
		if (tmax < kEpsilon || tmin > intersection.t) continue;

		if (nodeIndex < bvhTop->bvhCount) {
			// leaf
			mat4 transform = bvhTop->transforms[nodeIndex];

			Ray transfomedRay;
			transfomedRay.origin = make_float3(make_float4(ray.origin, 1.0f) * transform);
			transfomedRay.direction = make_float3(make_float4(ray.direction, 0.0f) * transform);

			NearestIntersection(*bvhs[node.MeshIndex()], transfomedRay, intersection, nodeIndex);
		}
		else {
			stack.push_back(node.LeftIndex());
			stack.push_back(node.RightIndex());
		}
	}
}

void WhittedStyleRayTracer::NearestIntersection(const BVH&bvh, const Ray &ray, IntersectionTraverse&intersection, const int instanceIdx) {
	vector<tuple<float, int>> stack;

	stack.reserve(30);
	//stack.reserve(log2(bvh.vcount / 3) * 1.5);

	float tmin, tmax;
	if (BoundingBoxIntersection(ray, bvh.pool[0].bounds, tmin, tmax) && tmax > kEpsilon) stack.push_back(make_tuple(tmin, 0));

	while (!stack.empty()) {
		int nodeId;
		float tmin;
		tie(tmin, nodeId) = stack.back();
		stack.pop_back();

		if (tmin > intersection.t) continue;

		BVHNode node = bvh.pool[nodeId];

		if (node.count == 0) {
			int left, right;
			left = node.leftFirst;
			right = left + 1;

			float tminLeft, tmaxLeft;
			float tminRight, tmaxRight;
			bool intersectLeft = BoundingBoxIntersection(ray, bvh.pool[left].bounds, tminLeft, tmaxLeft);
			bool intersectRight = BoundingBoxIntersection(ray, bvh.pool[right].bounds, tminRight, tmaxRight);

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
			int first = node.leftFirst;
			int last = first + node.count;

			for (int i = first; i < last; i++) {
				int index = bvh.indices[i] * 3;
				
				float3 a = make_float3(bvh.vertices[index]);
				float3 b = make_float3(bvh.vertices[index + 1]);
				float3 c = make_float3(bvh.vertices[index + 2]);

				float t, u, v;
				Side side;
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
