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
		CoreStats*coreStats;
		BVHTop* bvhTop;
		vector<tuple<BVHTopNode*, mat4*>> instances;
		vector<BVH*> bvhs;								// storing all bvh's
		vector<CoreLightTri*> areaLights;				// point lights of the scene
		vector<CorePointLight*> pointLights;			// point lights of the scene
		vector<CoreDirectionalLight*> directionLights;	// direction lights of the scene
		vector<CoreSpotLight*> spotLights;				// spot lights of the scene
		vector<Material*> materials;					// materials of the scene
		vector<Texture*> texList;						// 2D representation of the texture
		Texture*skyDome;								// sky dome of the scene

		float3*accumulator;
		int accumulatorIndex;

		Photon*photons;
		int totalNumberOfPhotons;
		float*lightsProbabilities;

		void Render(const ViewPyramid& view, Bitmap* screen, const Convergence converge);
		IntersectionShading intersectionTraverseToIntersectionShading(const IntersectionTraverse&intersectionTraverse, const Ray&ray);
		void GetRandomLight(const IntersectionShading&intersection, CoreLightTri*&areaLight, float3&lightPosition, float&p);
		void ResizeScreen(const int width, const int height);
		void ShootLightRays();
	private:
		float3 Trace(Ray ray);
		void NearestIntersection(const Ray&ray, IntersectionTraverse&intersection, int&numberIntersections); // Returns the nearest intersection point, the normal and the material type.
		void NearestIntersection(const BVH&bvh, const Ray&ray, IntersectionTraverse&intersection, const int meshIdx, int&numberIntersections);
		bool HasIntersection(const Ray &ray, const bool isBounded, const float distance);
		bool HasIntersection(const BVH & bvh, const Ray & ray, const bool bounded, const float distance);
	};
}