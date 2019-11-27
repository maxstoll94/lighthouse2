/* rendercore.h - Copyright 2019 Utrecht University

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

#pragma once
#include "rendersystem.h"

namespace lh2core
{

// -----------------------------------------------------------
// Texture class
// encapsulates a palettized pixel surface with pre-scaled
// palettes for fast shading
// -----------------------------------------------------------
// copied from software rasterizer
class Texture {
public:
	// constructor / destructor
	Texture() = default;
	Texture(int w, int h) : width(w), height(h) { pixels = (uint*)MALLOC64(w * h * sizeof(uint)); }
	~Texture() { FREE64(pixels); }
	// data members
	int width = 0, height = 0;
	uint* pixels = 0;
};

//  +-----------------------------------------------------------------------------+
//  |  Mesh                                                                       |
//  |  Minimalistic mesh storage.                                           LH2'19|
//  +-----------------------------------------------------------------------------+
class Mesh
{
public:
	float4* vertices = 0;							// vertex data received via SetGeometry
	int vcount = 0;									// vertex count
	CoreTri* triangles = 0;							// 'fat' triangle data
};

class Material {
public:
	float reflection;
	float3 transmission;
	float3 diffuse;
	Texture* texture = 0;			// texture
};

class Ray
{
public:
	float3 origin;
	float3 direction;
	float distance;
};

class Sky
{
public:
	vector<float3> pixels;
	uint width;
	uint height;
};

struct Intersection
{
	float3 position;
	float3 normal;
	float2 uv;
	uint materialIndex;
	float distance;
};

//  +-----------------------------------------------------------------------------+
//  |  RenderCore                                                                 |
//  |  Encapsulates device code.                                            LH2'19|
//  +-----------------------------------------------------------------------------+
class RenderCore
{
public:
	// methods
	void Init();
	void SetTarget( GLTexture* target );
	static HostSkyDome* sky;

	void SetGeometry( const int meshIdx, 
		const float4* vertexData, 
		const int vertexCount, 
		const int triangleCount, 
		const CoreTri* triangles, 
		const uint* alphaFlags = 0 );

	// SetLights: update the point lights, spot lights and directional lights.
	void SetLights(const CoreLightTri* areaLights, 
		const int areaLightCount, 
		const CorePointLight* pointLights, 
		const int pointLightCount, 
		const CoreSpotLight* spotLights, 
		const int spotLightCount, 
		const CoreDirectionalLight* directionalLights, 
		const int directionalLightCount);

	void SetSkyData(const float3* pixels, const uint width, const uint height);

	void SetMaterials(CoreMaterial* mat, const CoreMaterialEx* matEx, const int materialCount);

	void SetTextures(const CoreTexDesc* tex, const int textureCount);

	void Render( const ViewPyramid& view, 
		const Convergence converge, 
		const float brightness, 
		const float contrast );

	void Shutdown();
	// internal methods
	float3 Trace(Ray &ray);
	bool NearestIntersection(const Ray &ray, Intersection &intersection); // Returns the nearest intersection point, the normal and the material type.
	bool HasIntersection(const Ray &ray);
	bool RenderCore::IntersectsWithTriangle(const Ray &ray,
		const float3 &v0, 
		const float3 &v1, 
		const float3 &v2, 
		float &t, float &u, 
		float &v);
	Ray Reflect(const Ray &ray, const Intersection &intersection);
	float3 SkyDomeColor(const Ray &ray);
	float3 Directllumination(const Intersection &intersection);
	void printFloat3(float3 value);

private:
	// data members
	Bitmap* screen = 0;								// temporary storage of RenderCore output; will be copied to render target
	int targetTextureID = 0;						// ID of the target OpenGL texture
	vector<Mesh> meshes;							// mesh data storage
	vector<CorePointLight> pointLights;				// point lights of the scene
	vector<CoreDirectionalLight> directionLights;	// direction lights of the scene
	vector<CoreSpotLight> spotLights;				// spot lights of the scene
	vector<Material> materials;
	vector<Texture*> texList;
	Sky skyDome;
public:
	CoreStats coreStats;							// rendering statistic
};

} // namespace lh2core


// EOF