/* rendercore.cpp - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include "core_settings.h"
#include <iostream>
#include <math.h>
#include "rendersystem.h"

using namespace lh2core;

constexpr float kEpsilon = 1e-8;
constexpr float defaultRayDistance = 1000;
constexpr float bias = 0.00001;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init()
{
	// initialize core
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget( GLTexture* target )
{
	// synchronize OpenGL viewport
	targetTextureID = target->ID;
	if (screen != 0 && target->width == screen->width && target->height == screen->height) return; // nothing changed
	delete screen;
	screen = new Bitmap( target->width, target->height );
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData, const uint* alphaFlags )
{
	if (meshIdx >= meshes.size())
	{
		Mesh newMesh;
		// copy the supplied vertices; we cannot assume that the render system does not modify
		// the original data after we leave this function.
		newMesh.vertices = new float4[vertexCount];
		newMesh.vcount = vertexCount;
		memcpy(newMesh.vertices, vertexData, vertexCount * sizeof(float4));
		// copy the supplied 'fat triangles'
		newMesh.triangles = new CoreTri[vertexCount / 3];
		memcpy(newMesh.triangles, triangleData, (vertexCount / 3) * sizeof(CoreTri));
		meshes.push_back(newMesh);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetLights                                                    |
//  |  Update the point lights, spot lights and directional lights.                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetLights(const CoreLightTri* areaLights, const int areaLightCount, const CorePointLight* corePointLights, const int pointLightCount,
	const CoreSpotLight* coreSpotLights, const int spotLightCount, const CoreDirectionalLight* coreDirectionalLights, const int directionalLightCount)
{
	for (int i = 0; i < pointLightCount; i++) {
		pointLights.push_back(corePointLights[i]);
	}

	for (int i = 0; i < directionalLightCount; i++) {
		directionLights.push_back(coreDirectionalLights[i]);
	}

	for (int i = 0; i < spotLightCount; i++) {
		spotLights.push_back(coreSpotLights[i]);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTextures                                                    |
//  |  Set the texture data.                                                LH2'19|
//  +-----------------------------------------------------------------------------+
// Copied from soft-rasterizer
void RenderCore::SetTextures(const CoreTexDesc* tex, const int textures) {
	// copy the supplied array of texture descriptors
	for (int i = 0; i < textures; i++) {
		Texture* t;
		if (i < texList.size()) t = texList[i];
		else texList.push_back(t = new Texture());
		t->pixels = (uint*)MALLOC64(tex[i].pixelCount * sizeof(uint));
		if (tex[i].idata) memcpy(t->pixels, tex[i].idata, tex[i].pixelCount * sizeof(uint));
		else memcpy(t->pixels, 0, tex[i].pixelCount * sizeof(uint) /* assume integer textures */);
		// Note: texture width and height are not known yet, will be set when we get the materials.
	}
}

void RenderCore::SetMaterials(CoreMaterial* mat, const CoreMaterialEx* matEx, const int materialCount) {
	for (int i = 0; i < materialCount; i++) {
		CoreMaterial coreMaterial = mat[i];

		Material newMaterial;

		int texId = matEx[i].texture[TEXTURE0];
		if (texId == -1) {
			newMaterial.texture = 0;
			float r = coreMaterial.diffuse_r, g = coreMaterial.diffuse_g, b = coreMaterial.diffuse_b;
			newMaterial.diffuse = make_float3(r, g, b);
		}
		else {
			newMaterial.texture = texList[texId];
			// we know this only now, so set it properly
			newMaterial.texture->width = mat[i].texwidth0; 
			newMaterial.texture->height = mat[i].texheight0;
		}

		materials.push_back(newMaterial);
	}
}

void RenderCore::SetSkyData(const float3* pixels, const uint width, const uint height) {
	skyDome.height = height;
	skyDome.width = width;
	for (int i = 0; i < (width * height); i++) {
		skyDome.pixels.push_back(make_float3(pixels[i].x, pixels[i].y, pixels[i].z));
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
void RenderCore::Render(const ViewPyramid& view, const Convergence converge)
{
	// render
	screen->Clear();

	float3 xDirection = (view.p2 - view.p1) / screen->width;
	float3 yDirection = (view.p3 - view.p1) / screen->height;

	float3 p1 = view.p1 - view.pos + 0.5 * xDirection + 0.5 * yDirection;

	Ray ray;

	for (uint u = 0; u < screen->width; u++) {
		for (uint v = 0; v < screen->height; v++) {
			ray.direction = normalize(p1 + u * xDirection + v * yDirection);
			ray.origin = view.pos;
			ray.distance = defaultRayDistance;

			float3 color = Trace(ray);
			int colorHex = (int(0xff * min(color.x, 1.0f)) + (int(0xff * min(color.y, 1.0f)) << 8) + (int(0xff * min(color.z, 1.0f)) << 16));
			screen->Plot(u, v, colorHex);
		}
	}

	// copy pixel buffer to OpenGL render target texture
	glBindTexture(GL_TEXTURE_2D, targetTextureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown()
{
	delete screen;
}

float3 RenderCore::Trace(Ray &ray) {
	Intersection intersection;
	bool hasIntersection = NearestIntersection(ray, intersection);

	if (!hasIntersection) return SkyDomeColor(ray);

	Material material = materials[intersection.materialIndex];

	float3 diffuse;
	if (material.texture == NULL) {
		diffuse = material.diffuse;
	}
	else {
		int w = material.texture->width;
		int h = material.texture->height;

		int u = intersection.uv.x * w;
		int v = intersection.uv.y * h;
		int i = u + w * v;

		int hexvalue = material.texture->pixels[i];
		diffuse.x = ((hexvalue >> 16) & 0xff) / 255.0;  // extract the rr byte
		diffuse.y = ((hexvalue >> 8)  & 0xff) / 255.0;  // extract the gg byte
		diffuse.z = ((hexvalue)       & 0xff) / 255.0;  // extract the bb byte
	}
	
	return diffuse * Directllumination(intersection);

	float refractiveIndexGlass = 1.5168;
	float refractiveIndexAir = 1.0;

	float cosO1 = dot(intersection.normal, -ray.direction);

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


	float k = 1 - n1n2 * n1n2 * (1 - cosO1 * cosO1);

	// total internal reflection
	if (k < 0) make_float3(0.0);

	Ray refractRay;
	refractRay.direction = n1n2 * ray.direction + intersection.normal * (n1n2 * cosO1 - sqrt(k));
	refractRay.origin = intersection.position + bias * -intersection.normal;
	refractRay.distance = ray.distance - intersection.distance;

	switch (intersection.side) {
	case Front:
		return Trace(refractRay);
	case Back:
		float3 absorption;
		absorption.x = exp(-8.0 * intersection.distance);
		absorption.y = exp(-2.0 * intersection.distance);
		absorption.z = exp(-0.1 * intersection.distance);

		return absorption * Trace(refractRay);
	}

	// return diffuse * Directllumination(intersection);

	// return diffuse * Trace(Reflect(ray, intersection));
}

float3 RenderCore::SkyDomeColor(const Ray &ray) {
	float u = atan2(ray.direction.x, ray.direction.z) / (2 * PI);
	if (u < 0) u += 1;
	float v = acos(ray.direction.y) / PI;

	uint i = floor(v * skyDome.height) * skyDome.width + floor(u * skyDome.width);
	return skyDome.pixels[i];
}

bool RenderCore::HasIntersection(const Ray &ray) {
	float t;
	float u;
	float v;
	side side;

	for (Mesh& mesh : meshes) for (int i = 0; i < mesh.vcount; i += 3) {
		float3 a = make_float3(mesh.vertices[i]);
		float3 b = make_float3(mesh.vertices[i + 1]);
		float3 c = make_float3(mesh.vertices[i + 2]);

		if (IntersectsWithTriangle(ray, a, b, c, t, side, u, v)
			&& t > kEpsilon
			&& t < ray.distance
		) return true;
	}
	return false;
}

bool RenderCore::NearestIntersection(const Ray &ray, Intersection &intersection) {
	//float3 pos = make_float3(0, 0, 0);
	//float radius = 1.0f;

	//float a = dot(ray.direction, ray.direction);
	//float b = dot(2 * ray.direction, ray.origin - ray.direction);
	//float c = dot(ray.origin - ray.direction, ray.origin - ray.direction) - radius * radius;

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

	//if (t > ray.distance) return false;

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
			&& currentT < ray.distance
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

		// Only do this when material has a texture
		float2 uv0 = make_float2(nearestTriangle.u0, nearestTriangle.v0);
		float2 uv1 = make_float2(nearestTriangle.u1, nearestTriangle.v1);
		float2 uv2 = make_float2(nearestTriangle.u2, nearestTriangle.v2);
		intersection.uv = (1 - nearestU - nearestV) * uv0 + nearestU * uv1 + nearestV * uv2;
	}

	return hasIntersection;
}

// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
bool RenderCore::IntersectsWithTriangle(const Ray &ray, const float3 &v0, const float3 &v1, const float3 &v2, float &t, side &side, float &u, float &v) {
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
Ray RenderCore::Reflect(const Ray &ray, const Intersection &intersection) {
	Ray reflectRay;
	// taken from lecture slides "whitted-style" slide 13
	reflectRay.direction = ray.direction - 2 * dot(intersection.normal, ray.direction) * intersection.normal;
	reflectRay.origin = intersection.position + bias * intersection.normal;
	reflectRay.distance = ray.distance - intersection.distance;

	return reflectRay;
}

float3 RenderCore::Directllumination(const Intersection &intersection) {
	float3 illumination = make_float3(0, 0, 0);
	Ray ray;

	for (CorePointLight pointLight : pointLights) {
		float3 intersectionLight = pointLight.position - intersection.position;
		float3 lightDirection = normalize(intersectionLight);
		float lightDistance = length(intersectionLight);

		// Code taken from: https://www.gamedev.net/blogs/entry/2260865-shadows-and-point-lights/
		float contribution = dot(intersection.normal, lightDirection) / pow(lightDistance, 2);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = intersection.position + bias * intersection.normal;
		ray.direction = lightDirection;
		ray.distance = lightDistance;

		if (!HasIntersection(ray)) {
			illumination += pointLight.radiance * contribution;
		}
	}

	for (CoreDirectionalLight directionLight : directionLights) {
		// TODO move normalization to SetLights
		float3 lightDirection = -normalize(directionLight.direction);

		float contribution = dot(intersection.normal, lightDirection);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = intersection.position + bias * intersection.normal;
		ray.direction = lightDirection;
		ray.distance = std::numeric_limits<float>::infinity();

		if (!HasIntersection(ray)) {
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
		} else if (angle < spotLight.cosInner) {
			contribution = 1 - ((angle - spotLight.cosInner) / (spotLight.cosOuter - spotLight.cosInner));
		} else {
			contribution = 1;
		}

		contribution *= dot(intersection.normal, lightDirection) / pow(lightDistance, 2);
		if (contribution <= 0) continue; // don't calculate illumination for intersections facing away from the light

		ray.origin = intersection.position + bias * intersection.normal;
		ray.direction = lightDirection;
		ray.distance = lightDistance;

		if (!HasIntersection(ray)) {
			illumination += spotLight.radiance * contribution;
		}

	}

	return illumination;
}

void RenderCore::printFloat3(float3 value) {
	cout << "{ x:" << value.x << ", y: " << value.y << ", z: " << value.z << " }" << endl;
}

// EOF
