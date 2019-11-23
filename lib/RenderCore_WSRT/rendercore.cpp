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
	Mesh newMesh;
	// copy the supplied vertices; we cannot assume that the render system does not modify
	// the original data after we leave this function.
	newMesh.vertices = new float4[vertexCount];
	newMesh.vcount = vertexCount;
	memcpy( newMesh.vertices, vertexData, vertexCount * sizeof( float4 ) );
	// copy the supplied 'fat triangles'
	newMesh.triangles = new CoreTri[vertexCount / 3];
	memcpy( newMesh.triangles, triangleData, (vertexCount / 3) * sizeof( CoreTri ) );
	meshes.push_back( newMesh );
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetLights                                                    |
//  |  Update the point lights, spot lights and directional lights.                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetLights(const CoreLightTri* areaLights, const int areaLightCount, const CorePointLight* corePointLights, const int pointLightCount,
	const CoreSpotLight* spotLights, const int spotLightCount, const CoreDirectionalLight* coreDirectionalLights, const int directionalLightCount) 
{
	for (int i = 0; i < pointLightCount; i++) {
		pointLights.push_back(corePointLights[i]);
	}

	for (int j = 0; j < directionalLightCount; j++) {
		directionLights.push_back(coreDirectionalLights[j]);
	}
}

void RenderCore::SetMaterials(CoreMaterial* mat, const CoreMaterialEx* matEx, const int materialCount) {
	for (int i = 0; i < materialCount; i++) {
		CoreMaterial coreMaterial = mat[i];
		Material newMaterial;
		newMaterial.diffuse = make_float3(coreMaterial.diffuse_r, coreMaterial.diffuse_g, coreMaterial.diffuse_b);
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
//  +-----------------------------------------------------------------------------+
//void RenderCore::Render( const ViewPyramid& view, const Convergence converge, const float brightness, const float contrast )
//{
//	// render
//	screen->Clear();
//	for( Mesh& mesh : meshes ) for( int i = 0; i < mesh.vcount; i++ )
//	{
//		// convert a vertex position to a screen coordinate
//		int screenx = mesh.vertices[i].x / 80 * (float)screen->width + screen->width / 2;
//		int screeny = mesh.vertices[i].z / 80 * (float)screen->height + screen->height / 2;
//		screen->Plot( screenx, screeny, 0xffffff /* white */ );
//	}
//	// copy pixel buffer to OpenGL render target texture
//	glBindTexture( GL_TEXTURE_2D, targetTextureID );
//	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels );
//}

void RenderCore::Render(const ViewPyramid& view, const Convergence converge, const float brightness, const float contrast)
{
	// render
	screen->Clear();

	float3 xDirection = (view.p2 - view.p1) / screen->width;
	float3 yDirection = (view.p3 - view.p1) / screen->height;

	float3 p1 = view.p1 - view.pos;

	Ray ray;

	for (uint u = 0; u < screen->width; u++) {
		for (uint v = 0; v < screen->height; v++) {
			ray.direction = normalize(p1 + u * xDirection + v * yDirection);
			ray.origin = view.pos;

			float3 color = Trace(ray);
			int colorHex = (int(0xff * color.x) + (int(0xff * color.y) << 8) + (int(0xff * color.z) << 16));
			screen->Plot(u, v, colorHex);
		}
		// copy pixel buffer to OpenGL render target texture
		glBindTexture(GL_TEXTURE_2D, targetTextureID);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels);
	}
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

	if (!hasIntersection) {
		float u = atan2(ray.direction.x, ray.direction.z) / (2 * PI);
		if (u < 0) u += 1;
		float v = acos(ray.direction.y) / PI;

		//return make_float3(u, v, 0);
		 uint i = uint(v * skyDome.height) * skyDome.width + uint(u * skyDome.width);
		 return skyDome.pixels[i];
	}
	Material material = materials[intersection.materialIndex];
	return material.diffuse * Directllumination(intersection);
}

bool RenderCore::HasIntersection(const Ray &ray) {
	float t;
	float u;
	float v;

	for (Mesh& mesh : meshes) for (int i = 0; i < mesh.vcount; i += 3) {
		float3 a = make_float3(mesh.vertices[i]);
		float3 b = make_float3(mesh.vertices[i + 1]);
		float3 c = make_float3(mesh.vertices[i + 2]);

		if (IntersectsWithTriangle(ray, a, b, c, t, u, v)
		&& t > kEpsilon
		// && t < ray.distance
		) return true;
	}
	return false;
}

bool RenderCore::NearestIntersection(const Ray &ray, Intersection &intersection) {
	float currentT, currentU, currentV;
	float nearestT, nearestU, nearestV;
	CoreTri nearestTriangle;
	bool hasIntersection = false;

	for (Mesh& mesh : meshes) for (int i = 0; i < mesh.vcount; i += 3) {
		float3 a = make_float3(mesh.vertices[i]);
		float3 b = make_float3(mesh.vertices[i + 1]);
		float3 c = make_float3(mesh.vertices[i + 2]);

		if (IntersectsWithTriangle(ray, a, b, c, currentT, currentU, currentV) && (!hasIntersection || currentT < nearestT)) {
			nearestT = currentT;
			nearestU = currentU;
			nearestV = currentV;
			nearestTriangle = mesh.triangles[i / 3];
			hasIntersection = true;
		}
	}

	if (hasIntersection) {
		intersection.intersection = ray.origin + ray.direction * nearestT;
		intersection.normal = (1 - nearestU - nearestV) * nearestTriangle.vN0 + nearestU * nearestTriangle.vN1 + nearestV * nearestTriangle.vN2;
		intersection.materialIndex = nearestTriangle.material;
	}

	return hasIntersection;
}

// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
bool RenderCore::IntersectsWithTriangle(const Ray &ray, const float3 &v0, const float3 &v1, const float3 &v2, float &t, float &u, float &v) {
	float3 v0v1 = v1 - v0;
	float3 v0v2 = v2 - v0;
	float3 pvec = cross(ray.direction, v0v2);
	float det = dot(v0v1, pvec);
	// if the determinant is negative the triangle is backfacing
	// if the determinant is close to 0, the ray misses the triangle
	if (det < kEpsilon) return false;

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

float RenderCore::Directllumination(Intersection intersection) {
	float illumination = 0;
	Ray ray;
	ray.origin = intersection.intersection;

	for (CorePointLight pointLight : pointLights) {
		float3 intersectionLight = pointLight.position - intersection.intersection;

		ray.direction = normalize(intersectionLight);
		// ray.distance = length(intersectionLight); // objects position further then the light can't occlude the light

		if (!HasIntersection(ray)) {
			illumination += fmaxf(dot(intersection.normal, ray.direction), 0);
		}
	}

	return illumination;
}

void printFloat3(float3 value) {
	cout << "{ x:" << value.x << " y: " << value.y << " z: " << value.z << "}";
}

// EOF