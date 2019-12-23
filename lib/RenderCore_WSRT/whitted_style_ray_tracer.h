#pragma once
#include "rendersystem.h"
#include "classes.h"
#include "BVH.h"

namespace lh2core
{
	//  +-----------------------------------------------------------------------------+
	//  |  Whitted Style Ray Tracer													  |
	//  |  Center piece of this Ray Tracer										LH2'19|
	//  +-----------------------------------------------------------------------------+
	class WhittedStyleRayTracer
	{
	public: 
		BVHTop* bvhTop;
		vector<BVHTopNode*> instances;
		vector<BVH*> bvhs;								// storing all bvh's
		vector<CoreLightTri*> areaLights;				// point lights of the scene
		vector<CorePointLight*> pointLights;			// point lights of the scene
		vector<CoreDirectionalLight*> directionLights;	// direction lights of the scene
		vector<CoreSpotLight*> spotLights;				// spot lights of the scene
		vector<Material*> materials;					// materials of the scene
		vector<Texture*> texList;						// 2D representation of the texture
		Texture*skyDome;								// sky dome of the scene

		void Render(const ViewPyramid& view, Bitmap* screen, CoreStats&coreStats);
	private:
		float3 Directllumination(const Intersection &intersection);
		float3 Trace(Ray ray, CoreStats*coreStats);
		bool NearestIntersection(const Ray&ray, Intersection&intersection, int&numberIntersections); // Returns the nearest intersection point, the normal and the material type.
		bool NearestIntersection(const BVHTopNode&bvh, const Ray&ray, Intersection&intersection, int&numberIntersections);
		bool NearestIntersection(const BVH&bvh, const uint nodeIndex, const Ray&ray, Intersection&intersection, int&numberIntersections);
		bool HasIntersection(const Ray &ray, const bool isBounded, const float distance);
		bool HasIntersection(const BVHTopNode & node, const Ray & ray, const bool bounded, const float distance);
		bool HasIntersection(const BVH & bvh, const uint nodeIndex, const Ray & ray, const bool bounded, const float distance);
		// bool HasIntersection(const Ray &ray, const aabb &aabb, const bool isBounded, const float distance);
		Ray Reflect(const Ray &ray, const Intersection &intersection);
		float3 SkyDomeColor(const Ray &ray, const Texture &texture);
		float3 GetColor(const float2 &uv, const Texture &texture);
		float3 Dielectrics(const Ray &ray, const Intersection &intersection);
		float Fresnel(const Ray &ray, const Intersection &intersection, const float n1, const float n2, const float cosOi);
		float3 Beer(Ray ray, const Intersection &intersection, float3 diffuse);
	};
}