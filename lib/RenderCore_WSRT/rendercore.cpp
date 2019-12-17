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

int FindBestMatch(int a, const vector<BVHTopNode*>&topNodes) {
	float3 centerA = (topNodes[a]->bounds.bmin3 + topNodes[a]->bounds.bmax3) * 0.5f;

	int bestNode = a;
	float bestDistance = 1e34f;

	for (int b = 0; b < topNodes.size(); b++) {
		if (b == a) continue;

		float3 centerB = (topNodes[b]->bounds.bmin3 + topNodes[b]->bounds.bmax3) * 0.5f;
		float distance = length(centerB - centerA);
		if (distance < bestDistance) {
			bestDistance = distance;
			bestNode = b;
		}
	}

	return bestNode;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init()
{
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
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry(const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData, const uint* alphaFlags)
{
	if (meshIdx >= rayTracer.bvhs.size()) {
		// copy the supplied vertices; we cannot assume that the render system does not modify
		// the original data after we leave this function.

		BVH *bvh = new BVH;
		bvh->vertices = new float4[vertexCount];
		bvh->vcount = vertexCount;
		memcpy(bvh->vertices, vertexData, vertexCount * sizeof(float4));
		// copy the supplied 'fat triangles'
		bvh->triangles = new CoreTri[vertexCount / 3];
		memcpy(bvh->triangles, triangleData, (vertexCount / 3) * sizeof(CoreTri));

		clock_t begin = clock();
		bvh->ConstructBVH();
		clock_t end = clock();

		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		cout << "constructed bvh of " << vertexCount << " triangles in " << elapsed_secs << "s" << endl;

		rayTracer.bvhs.push_back(bvh);
	}
}

void RenderCore::SetInstance(const int instanceIdx, const int modelIdx, const mat4& transform) {
	if (modelIdx == -1) {
		if (rayTracer.instances.size() > instanceIdx) rayTracer.instances.resize(instanceIdx);
		if (rayTracer.topLevelBHVCount != instanceIdx) {
			rayTracer.topLevelBVHs = (BVHTopNode*)MALLOC64(instanceIdx * sizeof(BVHTopNode));
			rayTracer.topLevelBHVCount = instanceIdx;
		}
		return;
	}

	BVHTopNode *bvhTopNode = nullptr;

	if (instanceIdx >= rayTracer.instances.size()) {
		bvhTopNode = new BVHTopNode;
		rayTracer.instances.push_back(bvhTopNode);
	}
	else {
		bvhTopNode = rayTracer.instances[instanceIdx];
	}

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
void RenderCore::SetMaterials(CoreMaterial* mat, const CoreMaterialEx* matEx, const int materialCount) {
	for (int i = 0; i < materialCount; i++) {
		CoreMaterial coreMaterial = mat[i];

		Material*newMaterial = new Material;

		int texId = matEx[i].texture[TEXTURE0];
		if (texId == -1) {
			newMaterial->texture = 0;
		}
		else {
			newMaterial->texture = rayTracer.texList[texId];
			// we know this only now, so set it properly
			newMaterial->texture->width = mat[i].texwidth0;
			newMaterial->texture->height = mat[i].texheight0;
		}

		newMaterial->diffuse.x = coreMaterial.diffuse_r;
		newMaterial->diffuse.y = coreMaterial.diffuse_g;
		newMaterial->diffuse.z = coreMaterial.diffuse_b;

		newMaterial->transmittance.x = coreMaterial.transmittance_r;
		newMaterial->transmittance.y = coreMaterial.transmittance_g;
		newMaterial->transmittance.z = coreMaterial.transmittance_b;

		newMaterial->specularity = coreMaterial.specular();
		newMaterial->transmission = coreMaterial.transmission();

		rayTracer.materials.push_back(newMaterial);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetSkyData                                                     |
//  |  Specify the data required for sky dome rendering..                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetSkyData(const float3* pixels, const uint width, const uint height) {
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
	Timer t, frameTime{};
	screen->Clear();
	t.reset();
	frameTime.reset();

	rayTracer.Render(view, screen);

	coreStats.traceTime0 = t.elapsed();
	coreStats.primaryRayCount = screen->width * screen->height * 1;

	// copy pixel buffer to OpenGL render target texture
	glBindTexture(GL_TEXTURE_2D, targetTextureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels);

	coreStats.renderTime = frameTime.elapsed();
}

void RenderCore::UpdateTopLevel() {

	vector<BVHTopNode*> topNodes(rayTracer.instances);
	int topLevelBVHsPtr = 1;

	int a = 0;
	int b = FindBestMatch(a, topNodes);

	while (topNodes.size() > 1) {
		int c = FindBestMatch(b, topNodes);

		if (a == c) {
			BVHTopNode topNode = rayTracer.topLevelBVHs[topLevelBVHsPtr ++];
			topNode.bvh = nullptr;
			topNode.left = topNodes[a];
			topNode.right = topNodes[b];
			topNode.bounds = topNodes[a]->bounds.Union(topNodes[b]->bounds);

			topNodes.erase(topNodes.begin() + a);
			topNodes.erase(topNodes.begin() + b);
			topNodes.push_back(&topNode);

			a = topNodes.size() - 1;
			b = FindBestMatch(a, topNodes);
		}
		else {
			a = b;
			b = c;
		}
	}

	rayTracer.topLevelBVHs[0] = *topNodes[0];
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
