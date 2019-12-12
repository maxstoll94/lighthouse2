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
		Texture* texture = 0;
	};

	enum Side { Front, Back };

	class Intersection
	{
	private:
		Side side;
		float distance;
		float u, v;
		CoreTri *tri;
		bool hasIntersection;
		bool hasPosition; float3 position;
		bool hasUv; float2 uv;
		bool hasNormal; float3 normal;
	public:
		__inline bool Improves(float _distance) { return !hasIntersection || _distance < distance; };
		__inline bool HasIntersection() { return hasIntersection; };
		__inline void Reset() { hasIntersection = false; };
		__inline void Set(Side _side, float _distance, float _u, float _v, CoreTri *_tri) {
			hasIntersection = true;
			hasUv = hasNormal = false;
			side = _side;
			distance = _distance;
			u = _u; v = _v;
			tri = _tri;
		}
		__inline float GetDistance() { return distance; }
		__inline Side GetSide() { return side; }
		//__inline float GetU() { return u; }
		//__inline float GetV() { return v; }
		__inline float3 GetPosition() {
			if (!hasUv) {
				position = (1 - u - v) * tri->vertex0 + u * tri->vertex1 + v * tri->vertex2;
				hasPosition = true;
			}
			return position;
		}
		__inline CoreTri* GetTri() { return tri; }
		__inline float3 GetNormal() {
			if (!hasNormal) {
				normal = (1 - u - v) * tri->vN0 + u * tri->vN1 + v * tri->vN2;
				if (side == Back) normal = -normal;
				hasNormal = true;
			}
			return normal;
		}
		__inline float2 GetUV() {
			if (!hasUv) {
				uv = (1 - u - v) * make_float2(tri->u0, tri->v0) + u * make_float2(tri->u1, tri->v1) + v * make_float2(tri->u2, tri->v2);
				hasUv = true;
			}
			return uv;
		}
	};
}
