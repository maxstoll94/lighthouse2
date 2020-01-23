#pragma once

#include "rendersystem.h"
#include "classes.h"
#include "BVH.h"


namespace lh2core
{
	class PhotonMap {
	public:
		BVHNode* pool;
		int* indices;
		Photon* photons;
		int photonCount = 0;

		PhotonMap(const Photon * photons, const int photonCount);
		void Subdivide(const int nodeIndex, const int first, const int last, int&poolPtr);
		void CalculateBounds(const int first, const int last, aabb&aabb);
		int BinningSurfaceAreaHeuristic(BVHNode&node, const int first, const int last);
		vector<int> NearestNeighbours(const float3 &position, const float &distance);
		int NumberOfIntersections(const Ray & ray);
	};

}