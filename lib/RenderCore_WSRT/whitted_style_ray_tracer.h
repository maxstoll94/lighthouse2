#pragma once
#include "rendersystem.h"
#include "classes.h"

namespace lh2core
{
	//  +-----------------------------------------------------------------------------+
	//  |  Whitted Style Ray Tracer													  |
	//  |  Center piece of this Ray Tracer										LH2'19|
	//  +-----------------------------------------------------------------------------+
	class WhittedStyleRayTracer
	{
	public:
		vector<Mesh> meshes;							// mesh data storage
		vector<CorePointLight> pointLights;				// point lights of the scene
		vector<CoreDirectionalLight> directionLights;	// direction lights of the scene
		vector<CoreSpotLight> spotLights;				// spot lights of the scene
		vector<Material> materials;						// materials of the scene
		vector<Texture*> texList;						// 2D representation of the texture
		Texture skyDome;								// sky dome of the scene

		void Render(const ViewPyramid& view, Bitmap* screen);
	private:
		float3 Directllumination(const Intersection &intersection);
		float3 Trace(Ray ray);
		bool NearestIntersection(const Ray &ray, Intersection &intersection); // Returns the nearest intersection point, the normal and the material type.
		bool HasIntersection(const Ray &ray, const bool isBounded, const float distance);
		bool IntersectsWithTriangle(const Ray &ray, const float3 &v0, const float3 &v1,
			const float3 &v2, float &t, side &side, float &u, float &v);
		Ray Reflect(const Ray &ray, const Intersection &intersection);
		float3 SkyDomeColor(const Ray &ray, const Texture &texture);
		float3 GetColor(const float2 &uv, const Texture &texture);
	};
}