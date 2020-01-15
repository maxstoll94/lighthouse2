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
#include "rendercore.h"
#include "rendersystem.h"
#include <ctime>

using namespace lh2core;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init()
{
	rayTracer.bvhTop = new BVHTop();
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget(GLTexture* target)
{
	// synchronize OpenGL viewport
	targetTextureID = target->ID;
	if (screen != 0 && target->width == screen->width && target->height == screen->height) return; // nothing changed
	delete screen;
	screen = new Bitmap(target->width, target->height);

	rayTracer.ResizeScreen(target->width, target->height);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry(const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData, const AnimationType animationType, const uint* alphaFlags)
{
	BVH *bvh;
	if (meshIdx >= rayTracer.bvhs.size()) {
		// copy the supplied vertices; we cannot assume that the render system does not modify
		// the original data after we leave this function.

		bvh = new BVH;
		bvh->vertices = (float4*)_aligned_malloc(vertexCount * sizeof(float4), 64); // new float4[vertexCount];
		rayTracer.bvhs.push_back(bvh);

		bvh->indices = (int*)_aligned_malloc(triangleCount * sizeof(int), 64); // new int[triangleCount];
		for (int i = 0; i < triangleCount; i++) {
			bvh->indices[i] = i;
		}

		bvh->centroids = (float4*)_aligned_malloc(triangleCount * sizeof(float4), 64); // new float4[triangleCount];
		for (int i = 0; i < triangleCount; i++) {
			bvh->centroids[i] = (vertexData[i * 3] + vertexData[i * 3 + 1] + vertexData[i * 3 + 2]) / 3;
		}

		bvh->pool = (BVHNode*)_aligned_malloc(triangleCount * 2 * sizeof(BVHNode), 64);

		bvh->vcount = triangleCount;
	}
	else {
		bvh = rayTracer.bvhs[meshIdx];
	}

	bvh->vcount = vertexCount;
	memcpy(bvh->vertices, vertexData, vertexCount * sizeof(float4));
	// copy the supplied 'fat triangles'
	bvh->triangles = (CoreTri*)_aligned_malloc((vertexCount / 3) * sizeof(CoreTri), 64); // new CoreTri[vertexCount / 3];
	memcpy(bvh->triangles, triangleData, (vertexCount / 3) * sizeof(CoreTri));

	bvh->Update(animationType);
}

void RenderCore::SetInstance(const int instanceIdx, const int modelIdx, const mat4& transform) {
	if (modelIdx == -1) {
		if (rayTracer.instances.size() > instanceIdx) rayTracer.instances.resize(instanceIdx);
		if (rayTracer.bvhTop->bvhCount != instanceIdx) {
			_aligned_free(rayTracer.bvhTop->pool);
			rayTracer.bvhTop->pool = (BVHTopNode*)_aligned_malloc((instanceIdx - 1) * sizeof(BVHTopNode), 64);
			rayTracer.bvhTop->bvhCount = instanceIdx;
		}
		return;
	}

	BVHTopNode *bvhTopNode = nullptr;

	if (instanceIdx >= rayTracer.instances.size()) {
		bvhTopNode = (BVHTopNode*)_aligned_malloc(sizeof(BVHTopNode), 64);
		rayTracer.instances.push_back(bvhTopNode);
	}
	else {
		bvhTopNode = rayTracer.instances[instanceIdx];
	}

	bvhTopNode->instanceIdx = instanceIdx;
	bvhTopNode->bvh = rayTracer.bvhs[modelIdx];
	bvhTopNode->transform = transform;
	float3 bmin = bvhTopNode->bvh->pool[0].bounds.bmin3;
	float3 bmax = bvhTopNode->bvh->pool[0].bounds.bmax3;
	bvhTopNode->bounds.Reset();
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmin.x, bmin.y, bmin.z, 1.0f) * transform));
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmin.x, bmax.y, bmin.z, 1.0f) * transform));
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmin.x, bmax.y, bmax.z, 1.0f) * transform));
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmin.x, bmin.y, bmax.z, 1.0f) * transform));
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmax.x, bmin.y, bmin.z, 1.0f) * transform));
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmax.x, bmax.y, bmin.z, 1.0f) * transform));
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmax.x, bmax.y, bmax.z, 1.0f) * transform));
	bvhTopNode->bounds.Grow(make_float3(make_float4(bmax.x, bmin.y, bmax.z, 1.0f) * transform));
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetLights													  |
//  |  Update the point lights, spot lights and directional lights.         LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetLights(const CoreLightTri* areaLights, const int areaLightCount, const CorePointLight* corePointLights, const int pointLightCount,
	const CoreSpotLight* coreSpotLights, const int spotLightCount, const CoreDirectionalLight* coreDirectionalLights, const int directionalLightCount)
{
	rayTracer.areaLights.clear();
	for (int i = 0; i < areaLightCount; i++) {
		CoreLightTri*light = new CoreLightTri;
		memcpy(light, &(areaLights[i]), sizeof(CoreLightTri));
		rayTracer.areaLights.push_back(light);
	}

	rayTracer.pointLights.clear();
	for (int i = 0; i < pointLightCount; i++) {
		CorePointLight*light = new CorePointLight;
		memcpy(light, &(corePointLights[i]), sizeof(CorePointLight));
		rayTracer.pointLights.push_back(light);
	}

	rayTracer.directionLights.clear();
	for (int i = 0; i < directionalLightCount; i++) {
		CoreDirectionalLight*light = new CoreDirectionalLight;
		memcpy(light, &(coreDirectionalLights[i]), sizeof(CoreDirectionalLight));
		rayTracer.directionLights.push_back(light);
	}

	rayTracer.spotLights.clear();
	for (int i = 0; i < spotLightCount; i++) {
		CoreSpotLight*light = new CoreSpotLight;
		memcpy(light, &(coreSpotLights[i]), sizeof(CoreSpotLight));
		rayTracer.spotLights.push_back(light);
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
		if (i < rayTracer.texList.size()) {
			t = rayTracer.texList[i];
		}
		else {
			rayTracer.texList.push_back(t = new Texture());
		}
		t->pixels = (float3*)MALLOC64(tex[i].pixelCount * sizeof(float3));
		t->width = tex[i].width, t->height = tex[i].height;

		uint* hexPixels = (uint*)MALLOC64(tex[i].pixelCount * sizeof(uint));
		memcpy(hexPixels, tex[i].idata, tex[i].pixelCount * sizeof(uint));

		for (int j = 0; j < tex[i].pixelCount; j++) {
			int hexValue = hexPixels[j];

			t->pixels[j].x = ((hexValue >> 16) & 0xff) / 255.0;  // extract the rr byte
			t->pixels[j].y = ((hexValue >> 8) & 0xff) / 255.0;  // extract the gg byte
			t->pixels[j].z = ((hexValue) & 0xff) / 255.0;  // extract the bb byte
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetMaterials													  |
//  |  Update the material list used by the RenderCore. Textures referenced by    | 
//  |  the materials must be set in advance.								LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetMaterials(CoreMaterial* mat, const int materialCount) {
	for (int i = 0; i < materialCount; i++) {
		CoreMaterial coreMaterial = mat[i];

		Material*newMaterial = new Material;

		int texId = coreMaterial.color.textureID;
		if (texId == -1) {
			newMaterial->texture = 0;
		}
		else {
			newMaterial->texture = rayTracer.texList[texId];
		}

		newMaterial->diffuse = coreMaterial.color.value;
		newMaterial->transmittance = coreMaterial.absorption.value;
		newMaterial->specularity = coreMaterial.specular.value;
		newMaterial->transmission = coreMaterial.transmission.value;

		rayTracer.materials.push_back(newMaterial);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetSkyData                                                     |
//  |  Specify the data required for sky dome rendering..                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetSkyData(const float3* pixels, const uint width, const uint height, const mat4& worldToLight) {
	Texture*texture = new Texture;

	texture->height = height;
	texture->width = width;
	texture->pixels = (float3*)MALLOC64(width * height * sizeof(float3));
	for (int i = 0; i < (width * height); i++) {
		texture->pixels[i] = make_float3(pixels[i].x, pixels[i].y, pixels[i].z);
	}

	rayTracer.skyDome = texture;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render(const ViewPyramid& view, const Convergence converge)
{
	Timer frameTime{};
	screen->Clear();
	frameTime.reset();

	coreStats.traceTime0 = 0;
	coreStats.traceTime1 = 0;
	coreStats.shadowTraceTime = 0;
	coreStats.primaryRayCount = 0;
	coreStats.bounce1RayCount = 0;
	coreStats.totalShadowRays = 0;
	rayTracer.coreStats = &coreStats;

	rayTracer.Render(view, screen, converge);

	// copy pixel buffer to OpenGL render target texture
	glBindTexture(GL_TEXTURE_2D, targetTextureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels);

	coreStats.renderTime = frameTime.elapsed();
}

void lh2core::RenderCore::UpdateTopLevel() {
	rayTracer.bvhTop->UpdateTopLevel(rayTracer.instances);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown()
{
	delete screen;
}
// EOF
