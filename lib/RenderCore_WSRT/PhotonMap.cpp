#include "PhotonMap.h"
#include <iostream>

using namespace lh2core;

lh2core::PhotonMap::PhotonMap(const Photon* photons, const int photonCount, const aabb dim, const int nrOfLights)
{
	cdfGrid = (CDF*)_aligned_malloc(NrGridCells * sizeof(CDF), 64);

	int*cellPhotonCount = (int*)_aligned_malloc(NrGridCells * sizeof(int), 64);
	for (int i = 0; i < NrGridCells; i++) cellPhotonCount[i] = 0;

	for (int i = 0; i < photonCount; i++) {
		Photon photon = photons[i];
		int index = CoordToIndex(photon.position, dim);
		cellPhotonCount[index] ++;
	}

	int communativeCounter = 0;
	int communativePhotonCount[NrGridCells];
	for (int i = 0; i < NrGridCells; i++) {
		communativePhotonCount[i] = communativeCounter;
		communativeCounter += cellPhotonCount[i];
	}
	 
	// sort photons such that photons with the same cellIndex are in sequence
	Photon*sortedPhotons = (Photon*)_aligned_malloc(photonCount * sizeof(Photon), 64);
	int offsetPhotonCount[NrGridCells] = { 0 };
	for (int i = 0; i < photonCount; i++) {
		Photon photon = photons[i];
		int cellIndex = CoordToIndex(photon.position, dim);
		int sortedIndex = communativePhotonCount[cellIndex] + offsetPhotonCount[cellIndex];
		sortedPhotons[sortedIndex] = photon;
		offsetPhotonCount[cellIndex] ++;
	}

	float*totalEnergyPerLight = (float*)_aligned_malloc(nrOfLights * sizeof(float), 64);
	for (int i = 0; i < NrGridCells; i++) {

		// calculate contribution of each light in grid cell i
		for (int j = 0; j < nrOfLights; j++) totalEnergyPerLight[j] = 0.0f; // reset totalEnergyPerLight
		int startCell = communativePhotonCount[i];
		int endCell = communativePhotonCount[i] + cellPhotonCount[i];
		for (int j = startCell; j < endCell; j ++) {
			Photon photon = sortedPhotons[j];
			totalEnergyPerLight[photon.lightIndex] += photon.energy;
		}

		// totalCdfEnergy is used to later normalize the probabilites
		float totalCdfEnergy = 0.0f;
		// find the 16 lights with the highest probability
		for (int j = 0; j < NrCDFLights; j++) {
			// find light with the highes contribution
			// if no light is found because no photons
			// landed in the grid cell or there are less
			// then 16 lights the lightIndex stays at -1
			int lightIndex = -1;
			float energy = 0.0f;
			for (int k = 0; k < nrOfLights; k++) {
				if (totalEnergyPerLight[k] > energy) {
					lightIndex = k;
					energy = totalEnergyPerLight[k];
				}
			}

			// set energy to 0 so we don't find the same light in the next iteration
			if (lightIndex != -1) totalEnergyPerLight[lightIndex] = 0;

			cdfGrid[i].lightIndices[j] = lightIndex;
			cdfGrid[i].probabilities[j] = energy;

			totalCdfEnergy += energy;
		}

		float communativeEnergy = 0.0f;
		// normalize the lights and make the probabilities communative
		for (int j = 0; j < NrCDFLights; j++) {
			if (cdfGrid[i].lightIndices[j] == -1) {
				cdfGrid[i].probabilities[j] = 1;
			}
			else {
				communativeEnergy += cdfGrid[i].probabilities[j] / totalCdfEnergy;
				cdfGrid[i].probabilities[j] = communativeEnergy;
			}
		}
	}

	//for (int i = 0; i < NrGridCells; i++) {
	//	cout << "Probability for gridcell: " << i << "  ---  ";
	//	for (int j = 0; j < NrCDFLights; j++) {
	//		cout << cdfGrid[i].lightIndices[j] << ":=" << cdfGrid[i].probabilities[j] << ", ";
	//	}
	//	cout << endl;
	//}

	_aligned_free(totalEnergyPerLight);
	_aligned_free(sortedPhotons);
	_aligned_free(cellPhotonCount);
}

int lh2core::PhotonMap::CoordToIndex(const float3&coord, const aabb&dim) {
	float3 t = (coord - dim.bmin3) / (dim.bmax3 - dim.bmin3);
	int3 cell = make_int3(floorf(t * NrGridCellsAxis));
	return (cell.z * NrGridCellsAxis + cell.y) * NrGridCellsAxis + cell.x;
}
