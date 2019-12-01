#pragma once
#include "rendersystem.h"

namespace lh2core
{
	//  +-----------------------------------------------------------------------------+
	//  |  Ray	                                                                      |
	//  |  Center piece of this Ray Tracer                                      LH2'19|
	//  +-----------------------------------------------------------------------------+
	class Ray
	{
	public:
		float3 origin;		// origin point of the ray
		float3 direction;	// direction the ray is travelling in
		int bounces;		// maximum number of bounces of a ray before it is discarded.
	};

	//  +-----------------------------------------------------------------------------+
	//  |  Texture                                                                    |
	//  |  Encapsulates a palettized pixel surface with pre-scaled paletted for fast  |
	//	|  shading. Partially copied from Software Rasterizer					LH2'19|
	//  +-----------------------------------------------------------------------------+
	class Texture {
	public:
		int width = 0, height = 0;
		float3* pixels = 0;
	};

	//  +-----------------------------------------------------------------------------+
	//  |  Mesh                                                                       |
	//  |  Minimalistic mesh storage.                                           LH2'19|
	//  +-----------------------------------------------------------------------------+
	class Mesh
	{
	public:
		float4* vertices = 0;		// vertex data received via SetGeometry
		int vcount = 0;				// vertex count
		CoreTri* triangles = 0;		// 'fat' triangle data
	};

	//  +-----------------------------------------------------------------------------+
	//  |  Material                                                                   |
	//  |  Minimalistic mesh storage.                                           LH2'19|
	//  +-----------------------------------------------------------------------------+
	class Material {
	public:
		float transmission;
		float specularity;
		float3 diffuse;
		float3 transmittance;
		Texture* texture = 0;			// texture
		bool isDielectic;
	};

	enum side { Front, Back };

	class Intersection
	{
	public:
		float3 position;
		float3 normal;
		float2 uv;
		side side;
		uint materialIndex;
		float distance;
	};
}
