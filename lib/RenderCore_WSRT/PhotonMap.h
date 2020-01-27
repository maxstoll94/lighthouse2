#pragma once

#include "rendersystem.h"
#include "classes.h"
#include "BVH.h"

const int NrCDFLights = 16;
const int NrGridCellsAxis = 16;
const int NrGridCells = NrGridCellsAxis * NrGridCellsAxis * NrGridCellsAxis;

namespace lh2core
{
	struct CDF {
		float probabilities[NrCDFLights];
		int lightIndices[NrCDFLights];
	};

	class PhotonMap {
	public:
		CDF*cdfGrid;

		PhotonMap(const Photon*photons, const int photonCount, const aabb dim, const int nrOfLights);
		int CoordToIndex(const float3&coord, const aabb&dim);
	};

}