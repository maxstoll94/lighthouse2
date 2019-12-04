
#include "whitted_style_ray_tracer.h"

using namespace lh2core;

constexpr float kEpsilon = 1e-8;
constexpr float defaultRayBounces = 2;
constexpr float bias = 0.00001;
constexpr float refractiveIndexGlass = 1.5168;
constexpr float refractiveIndexAir = 1.0;
constexpr uint softLightRays = 10;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
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

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
float3 WhittedStyleRayTracer::Trace(Ray ray) {
	if (ray.bounces < 0) return make_float3(0.0);

	Intersection intersection;
	bool hasIntersection = NearestIntersection(ray, intersection);

	if (!hasIntersection) return SkyDomeColor(ray, skyDome);

	Material material = materials[intersection.materialIndex];

	float3 diffuse = material.texture == NULL ? material.diffuse : GetColor(intersection.uv, *material.texture);

	// just render the color of the light
	if (diffuse.x > 1.0 || diffuse.y > 1.0 || diffuse.z > 1.0) return diffuse;

	float refractive = material.transmission;
	float reflective = (1 - refractive) * material.specularity;
	float diffusive = 1 - refractive - reflective;

	float3 color = make_float3(0.0);


	if (refractive > 0) {
		color += refractive * Dielectrics(ray, intersection);
	}
	if (reflective > 0) {
		color += diffuse * reflective * Trace(Reflect(ray, intersection));
	}
	if (diffusive > 0) {
		color += diffusive * diffuse * Directllumination(intersection);
	}

	return color;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
bool WhittedStyleRayTracer::HasIntersection(const Ray &ray, const bool bounded, const float distance) {
	float t;
	float u;
	float v;
	side side;

	for (Mesh& mesh : meshes) for (int i = 0; i < mesh.vcount; i += 3) {
		float3 a = make_float3(mesh.vertices[i]);
		float3 b = make_float3(mesh.vertices[i + 1]);
		float3 c = make_float3(mesh.vertices[i + 2]);

		if (IntersectsWithTriangle(ray, a, b, c, t, side, u, v) && t > kEpsilon && (!bounded || t < distance)) {
			return true;
		}
	}
	return false;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
bool WhittedStyleRayTracer::NearestIntersection(const Ray &ray, Intersection &intersection) {
	//float3 pos = make_float3(0, 0, 0);
	//float radius = 1.0f;
	//float a = dot(ray.direction, ray.direction);
	//float b = dot(2 * ray.direction, ray.origin - pos);
	//float c = dot(ray.origin - pos, ray.origin - pos) - radius * radius;
	//if (b * b - 4 * a * c < 0) {
	//	return false;
	//}
	//float t;
	//if (b * b - 4 * a * c == 0) {
	//	t = -b / (2 * a);
	//}
	//else {
	//	float t1 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
	//	float t2 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
	//	if (t1 > kEpsilon) {
	//		t = t1;
	//		intersection.side = Front;
	//	}
	//	else if (t2 > kEpsilon) {
	//		t = t2;
	//		intersection.side = Back;
	//	}
	//	else {
	//		return false;
	//	}
	//}

	//intersection.position = ray.origin + ray.direction * t;
	//intersection.normal = normalize(intersection.position - pos);
	//if (intersection.side == Back) intersection.normal = -intersection.normal;
	//intersection.materialIndex = 0;
	//intersection.distance = t;
	//return true;

	float currentT, currentU, currentV;
	float nearestT, nearestU, nearestV;
	side nearestSide, currentSide;
	CoreTri nearestTriangle;
	bool hasIntersection = false;

	for (Mesh& mesh : meshes) for (int i = 0; i < mesh.vcount; i += 3) {
		float3 a = make_float3(mesh.vertices[i]);
		float3 b = make_float3(mesh.vertices[i + 1]);
		float3 c = make_float3(mesh.vertices[i + 2]);

		if (IntersectsWithTriangle(ray, a, b, c, currentT, currentSide, currentU, currentV)
			&& currentT > kEpsilon
			&& (!hasIntersection || currentT < nearestT)
			) {
			nearestT = currentT;
			nearestU = currentU;
			nearestV = currentV;
			nearestSide = currentSide;
			nearestTriangle = mesh.triangles[i / 3];
			hasIntersection = true;
		}
	}

	if (hasIntersection) {
		intersection.position = ray.origin + ray.direction * nearestT;
		intersection.normal = (1 - nearestU - nearestV) * nearestTriangle.vN0 + nearestU * nearestTriangle.vN1 + nearestV * nearestTriangle.vN2;
		if (nearestSide == Back) intersection.normal = -intersection.normal;
		intersection.materialIndex = nearestTriangle.material;
		intersection.side = nearestSide;
		intersection.distance = nearestT;

		if (materials[intersection.materialIndex].texture != NULL) {
			float2 uv0 = make_float2(nearestTriangle.u0, nearestTriangle.v0);
			float2 uv1 = make_float2(nearestTriangle.u1, nearestTriangle.v1);
			float2 uv2 = make_float2(nearestTriangle.u2, nearestTriangle.v2);
			intersection.uv = (1 - nearestU - nearestV) * uv0 + nearestU * uv1 + nearestV * uv2;
		}
	}

	return hasIntersection;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
bool WhittedStyleRayTracer::IntersectsWithTriangle(const Ray &ray, const float3 &v0, const float3 &v1, const float3 &v2, float &t, side &side, float &u, float &v) {
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

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
Ray WhittedStyleRayTracer::Reflect(const Ray &ray, const Intersection &intersection) {
	Ray reflectRay;
	// taken from lecture slides "whitted-style" slide 13
	reflectRay.direction = ray.direction - 2 * dot(intersection.normal, ray.direction) * intersection.normal;
	reflectRay.origin = intersection.position + bias * reflectRay.direction;
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

	for (CoreLightTri areaLight : areaLights) {
		for (int i = 0; i < softLightRays; i++) {
			float r1 = ((double)rand() / RAND_MAX);
			float r2 = ((double)rand() / RAND_MAX);
			float sqrtR1 = sqrt(r1);
			float3 position = (1 - sqrtR1) * areaLight.vertex0 + (sqrtR1 * (1 - r2)) * areaLight.vertex1 + (sqrtR1 * r2) * areaLight.vertex2;

			float3 intersectionLight = position - intersection.position;
			float3 lightDirection = normalize(intersectionLight);
			float lightDistance = length(intersectionLight);

			float contribution = areaLight.area * dot(intersection.normal, lightDirection) * dot(areaLight.N, -lightDirection) / (lightDistance * lightDistance);
			if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

			ray.origin = intersection.position + bias * lightDirection;
			ray.direction = lightDirection;

			if (!HasIntersection(ray, true, lightDistance - 2 * bias)) {
				illumination += areaLight.radiance * contribution / softLightRays;
			}
		}
	}

	for (CorePointLight pointLight : pointLights) {
		float3 intersectionLight = pointLight.position - intersection.position;
		float3 lightDirection = normalize(intersectionLight);
		float lightDistance = length(intersectionLight);

		// Code taken from: https://www.gamedev.net/blogs/entry/2260865-shadows-and-point-lights/
		float contribution = dot(intersection.normal, lightDirection) / (lightDistance * lightDistance);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = intersection.position + bias * lightDirection;
		ray.direction = lightDirection;

		if (!HasIntersection(ray, true, lightDistance)) {
			illumination += pointLight.radiance * contribution;
		}
	}

	for (CoreDirectionalLight directionLight : directionLights) {
		// TODO move normalization to SetLights
		float3 lightDirection = -normalize(directionLight.direction);

		float contribution = dot(intersection.normal, lightDirection);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = intersection.position + bias * lightDirection;
		ray.direction = lightDirection;

		if (!HasIntersection(ray, false, 0)) {
			illumination += directionLight.radiance * contribution;
		}
	}

	for (CoreSpotLight spotLight : spotLights) {
		float3 intersectionLight = spotLight.position - intersection.position;
		float3 lightDirection = normalize(intersectionLight);
		float lightDistance = length(intersectionLight);

		float angle = dot(-lightDirection, spotLight.direction);
		float contribution;

		if (angle < spotLight.cosOuter) {
			continue;
		}
		else if (angle < spotLight.cosInner) {
			contribution = 1 - ((angle - spotLight.cosInner) / (spotLight.cosOuter - spotLight.cosInner));
		}
		else {
			contribution = 1;
		}

		contribution *= dot(intersection.normal, lightDirection) / (lightDistance * lightDistance);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = intersection.position + bias * lightDirection;
		ray.direction = lightDirection;

		if (!HasIntersection(ray, true, lightDistance)) {
			illumination += spotLight.radiance * contribution;
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
	float cosO1 = dot(intersection.normal, -1 * ray.direction);

	float k = 1 - n1n2 * n1n2 * (1 - cosO1 * cosO1);

	// total internal reflection
	if (k < 0) return Trace(Reflect(ray, intersection));

	Ray refractRay;
	refractRay.direction = n1n2 * ray.direction + intersection.normal * (n1n2 * cosO1 - sqrt(k));
	refractRay.origin = intersection.position + bias * -1 * refractRay.direction;
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
		Material material = materials[intersection.materialIndex];

		float3 absorption;
		absorption.x = exp(-material.transmittance.x * intersection.distance);
		absorption.y = exp(-material.transmittance.y * intersection.distance);
		absorption.z = exp(-material.transmittance.z * intersection.distance);

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
