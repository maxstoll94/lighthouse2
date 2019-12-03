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

using namespace lh2core;

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
	if (meshIdx >= rayTracer.meshes.size())
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
		rayTracer.meshes.push_back(newMesh);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetLights													  |
//  |  Update the point lights, spot lights and directional lights.         LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetLights(const CoreLightTri* areaLights, const int areaLightCount, const CorePointLight* corePointLights, const int pointLightCount,
	const CoreSpotLight* coreSpotLights, const int spotLightCount, const CoreDirectionalLight* coreDirectionalLights, const int directionalLightCount)
{
	for (int i = 0; i < areaLightCount; i++) {
		rayTracer.areaLights.push_back(areaLights[i]);
	}

	for (int i = 0; i < pointLightCount; i++) {
		rayTracer.pointLights.push_back(corePointLights[i]);
	}

	for (int i = 0; i < directionalLightCount; i++) {
		rayTracer.directionLights.push_back(coreDirectionalLights[i]);
	}

	for (int i = 0; i < spotLightCount; i++) {
		rayTracer.spotLights.push_back(coreSpotLights[i]);
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
			t->pixels[j].y = ((hexValue >> 8)  & 0xff) / 255.0;  // extract the gg byte
			t->pixels[j].z = ((hexValue)       & 0xff) / 255.0;  // extract the bb byte
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

		Material newMaterial;

		int texId = matEx[i].texture[TEXTURE0];
		if (texId == -1) {
			newMaterial.texture = 0;
		}
		else {
			newMaterial.texture = rayTracer.texList[texId];
			// we know this only now, so set it properly
			newMaterial.texture->width  = mat[i].texwidth0; 
			newMaterial.texture->height = mat[i].texheight0;
		}

		newMaterial.diffuse.x = coreMaterial.diffuse_r;
		newMaterial.diffuse.y = coreMaterial.diffuse_g;
		newMaterial.diffuse.z = coreMaterial.diffuse_b;

		newMaterial.transmittance.x = coreMaterial.transmittance_r;
		newMaterial.transmittance.y = coreMaterial.transmittance_g;
		newMaterial.transmittance.z = coreMaterial.transmittance_b;

		newMaterial.specularity = coreMaterial.specular();
		newMaterial.transmission = coreMaterial.transmission();

		rayTracer.materials.push_back(newMaterial);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetSkyData                                                     |
//  |  Specify the data required for sky dome rendering..                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetSkyData(const float3* pixels, const uint width, const uint height) {
	rayTracer.skyDome.height = height;
	rayTracer.skyDome.width = width;
	rayTracer.skyDome.pixels = (float3*)MALLOC64(width * height * sizeof(float3));
	for (int i = 0; i < (width * height); i++) {
		rayTracer.skyDome.pixels[i] = make_float3(pixels[i].x, pixels[i].y, pixels[i].z);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render(const ViewPyramid& view, const Convergence converge)
{
	// render
	screen->Clear();
			
	rayTracer.Render(view, screen);

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
// EOF
