#pragma once

#include "rendersystem.h"
#include "classes.h"
#include "BVH.h"

const int NrGridCellSplits = 2;
const int GridCellSize = NrGridCellSplits * NrGridCellSplits * NrGridCellSplits;

namespace lh2core
{
	class PhotonMap {
	public:
		BVHNode* pool;
		int* indices;
		Photon* photons;
		int photonCount = 0;
		CDF cdfGrid[GridCellSize];

		PhotonMap(const Photon * photons, const int photonCount, const aabb dim, const int nrOfLights);
		int CoordToIndex(const float3&coord, const aabb&dim);
		void Subdivide(const int nodeIndex, const int first, const int last, int&poolPtr);
		void CalculateBounds(const int first, const int last, aabb&aabb);
		int BinningSurfaceAreaHeuristic(BVHNode&node, const int first, const int last);
		vector<int> NearestNeighbours(const float3 &position, const float &distance);
		int NumberOfIntersections(const Ray & ray);

	};

}