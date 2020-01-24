#include "PhotonMap.h"
#include <iostream>

using namespace lh2core;

constexpr int numberOfBins = 32;

void Swap2(int* a, int* b) {
	int t = *a;
	*a = *b;
	*b = t;
}

// https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection
bool BoundingBoxIntersection(const aabb &a, const aabb &b) {
	return (a.bmin3.x <= b.bmax3.x && a.bmax3.x >= b.bmin3.x) &&
		(a.bmin3.y <= b.bmax3.y && a.bmax3.y >= b.bmin3.y) &&
		(a.bmin3.z <= b.bmax3.z && a.bmax3.y >= b.bmin3.z);
}

// https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
bool BoundingBoxIntersection2(const Ray &ray, const aabb &bounds, float&tmin, float&tmax) {
	float3 invD = 1 / ray.direction;
	float3 t0s = (bounds.bmin3 - ray.origin) * invD;
	float3 t1s = (bounds.bmax3 - ray.origin) * invD;

	float3 tsmaller = fminf(t0s, t1s);
	float3 tbigger = fmaxf(t0s, t1s);

	tmin = max(tsmaller.x, max(tsmaller.y, tsmaller.z));
	tmax = min(tbigger.x, min(tbigger.y, tbigger.z));

	return tmin < tmax;
}

//lh2core::PhotonMap::PhotonMap(const Photon* photons, const int photonCount)
//{
//	this->photonCount = photonCount;
//	this->photons = (Photon*)_aligned_malloc(photonCount * sizeof(Photon), 64);
//	memcpy(this->photons, photons, photonCount * sizeof(Photon));
//
//	indices = (int*)_aligned_malloc(photonCount * sizeof(int), 64); // new int[triangleCount];
//
//	for (int i = 0; i < photonCount; i++) {
//		indices[i] = i;
//	}
//
//	pool = (BVHNode*)_aligned_malloc(photonCount * 2 * sizeof(BVHNode), 64);
//
//	int nodeIndex = 0;
//	int first = 0;
//	int last = photonCount - 1;
//	int poolPtr = 2;
//
//	Subdivide(nodeIndex, first, last, poolPtr);
//}

lh2core::PhotonMap::PhotonMap(const Photon* photons, const int photonCount, const aabb dim, const int nrOfLights)
{
	int gridPhotonCount[GridCellSize];

	for (int i = 0; i < GridCellSize; i++) {
		gridPhotonCount[i] = 0;
	}

	for (int i = 0; i < photonCount; i++) {
		Photon photon = photons[i];
		int index = CoordToIndex(photon.position, dim);
		cout << index << endl;
		gridPhotonCount[index] = gridPhotonCount[index] + 1;
	}

	int photonCellCount = 0;
	int gridCommunativePhotonCount[GridCellSize];
	for (int i = 0; i < GridCellSize; i++) {
		gridCommunativePhotonCount[i] = photonCellCount;
		photonCellCount += gridPhotonCount[i];
	}
	 
	Photon*sortedPhotons = new Photon[photonCount];
	int gridCommunativePhotonCount2[GridCellSize];
	for (int i = 0; i < GridCellSize; i++) gridCommunativePhotonCount2[i] = gridCommunativePhotonCount[i];
	for (int i = 0; i < photonCount; i++) {
		Photon photon = photons[i];
		int cellIndex = CoordToIndex(photon.position, dim);
		int sortedIndex = gridCommunativePhotonCount2[cellIndex];
		gridCommunativePhotonCount2[cellIndex] ++;

		sortedPhotons[sortedIndex] = photon;
	}

	float*totalEnergyPerLight = new float[nrOfLights];
	for (int i = 0; i < GridCellSize; i++) {

		for (int j = 0; j < nrOfLights; j++) {
			totalEnergyPerLight[j] = 0.0f;
		}

		int startGrid = gridCommunativePhotonCount[i];
		int endGrid = gridCommunativePhotonCount[i] + gridPhotonCount[i];
		for (int j = startGrid; j < endGrid; j ++) {
			Photon photon = sortedPhotons[j];
			totalEnergyPerLight[photon.lightIndex] += photon.energy;
		}

		float totalCdfEnergy = 0.0f;
		for (int j = 0; j < CDFLightSize; j++) {
			int lightIndex = -1;
			float energy = 0.0f;
			for (int k = 0; k < nrOfLights; k++) {
				if (totalEnergyPerLight[k] > energy) {
					lightIndex = k;
					energy = totalEnergyPerLight[k];
				}
			}

			if (lightIndex != -1) totalEnergyPerLight[lightIndex] = 0;

			cdfGrid[i].lightIndices[j] = lightIndex;
			cdfGrid[i].probabilities[j] = energy;

			totalCdfEnergy += energy;
		}

		float communativeEnergy = 0.0f;
		for (int j = 0; j < CDFLightSize; j++) {
			if (cdfGrid[i].lightIndices[j] == -1) {
				cdfGrid[i].probabilities[j] = 1;
			}
			else {
				communativeEnergy += cdfGrid[i].probabilities[j] / totalCdfEnergy;
				cdfGrid[i].probabilities[j] = communativeEnergy;
			}
		}
	}
	delete totalEnergyPerLight;

	for (int i = 0; i < GridCellSize; i++) {
		cout << "Probability for gridcell: " << i << "  ---  ";
		for (int j = 0; j < CDFLightSize; j++) {
			cout << cdfGrid[i].lightIndices[j] << ":=" << cdfGrid[i].probabilities[j] << ", ";
		}
		cout << endl;
	}

	delete sortedPhotons;
}

int lh2core::PhotonMap::CoordToIndex(const float3&coord, const aabb&dim) {
	float3 t = (coord - dim.bmin3) / (dim.bmax3 - dim.bmin3);
	int3 cell = make_int3(floorf(t * NrGridCellSplits));
	return (cell.z * NrGridCellSplits + cell.y) * NrGridCellSplits + cell.x;
}

void lh2core::PhotonMap::Subdivide(const int nodeIndex, const int first, const int last, int & poolPtr)
{
	BVHNode&node = pool[nodeIndex];
	CalculateBounds(first, last, node.bounds);
	int splitIndex = BinningSurfaceAreaHeuristic(node, first, last);

	if (splitIndex == -1) {
		node.count = last - first + 1;
		node.leftFirst = first;
	}
	else {
		node.leftFirst = poolPtr;
		poolPtr += 2;
		node.count = 0;

		Subdivide(node.GetLeft(), first, splitIndex, poolPtr);
		Subdivide(node.GetRight(), splitIndex + 1, last, poolPtr);
	}
}

void lh2core::PhotonMap::CalculateBounds(const int first, const int last, aabb & bounds)
{
	bounds.Reset();

	for (int i = first; i <= last; i++) {
		int index = indices[i];
		bounds.Grow(photons[index].position);
	}
}

int lh2core::PhotonMap::BinningSurfaceAreaHeuristic(BVHNode & node, const int first, const int last)
{
	if (last - first <= 3) return -1;

	int longestAxis = node.bounds.LongestAxis();

	float minAxis = get_axis(longestAxis, node.GetBounds().bmin3);
	float maxAxis = get_axis(longestAxis, node.GetBounds().bmax3);

	int splitIndices[numberOfBins - 1];

	for (int i = 0, l = numberOfBins - 1; i < l; i++) {
		float splitPlane = minAxis + (maxAxis - minAxis) / numberOfBins * (i + 1);
		int splitIndex = i == 0 ? first - 1 : splitIndices[i - 1];
		for (int j = splitIndex + 1; j <= last; j++) {
			if (get_axis(longestAxis, photons[indices[j]].position) < splitPlane) {
				splitIndex++;
				Swap2(&indices[splitIndex], &indices[j]);
			}
		}
		splitIndices[i] = splitIndex;
	}

	float a = node.GetBounds().Area();
	int n = last - first + 1;
	float lowestCost = a * n;
	int bestSplitIndex = -1;

	aabb lBounds;
	aabb rBounds;

	for (int i = 0, l = numberOfBins - 1; i < l; i++) {
		int currentSplitIndex = splitIndices[i];

		int nLeft = currentSplitIndex - first + 1;
		if (nLeft == 0) continue;
		int nRight = last - currentSplitIndex;
		if (nRight == 0) continue;

		CalculateBounds(first, currentSplitIndex, lBounds);
		CalculateBounds(currentSplitIndex + 1, last, rBounds);

		float aLeft = lBounds.Area();
		float aRight = rBounds.Area();

		float currentCost = aLeft * nLeft + aRight * nRight;

		if (currentCost < lowestCost) {
			bestSplitIndex = currentSplitIndex;
			lowestCost = currentCost;
		}
	}

	return bestSplitIndex;
}

vector<int> lh2core::PhotonMap::NearestNeighbours(const float3 & position, const float &radius)
{
	BVHNode node;
	vector<int> stack;
	int left, right;
	uint first, last;
	vector<int> indices;

	stack.reserve(log2(photonCount) * 1.5);

	aabb source = aabb(position - radius, position + radius);

	stack.push_back(0);

	while (!stack.empty()) {
		node = pool[stack.back()];
		stack.pop_back();

		if (!BoundingBoxIntersection(source, node.bounds)) continue;

		if (node.count == 0) {
			left = node.leftFirst;
			right = left + 1;
			
			stack.push_back(right);
			stack.push_back(left);
		}
		else {
			first = node.leftFirst;
			last = first + node.count;

			for (int i = first; i < last; i++) {
				//Photon photon = photons[this->indices[i]];

				//if (dot(photon.position, position) < radius * radius) {
					indices.push_back(this->indices[i]);
				//}
			}
		}
	}

	return indices;
}

int PhotonMap::NumberOfIntersections(const Ray&ray) {
	BVHNode node;
	vector<int> stack;
	int left, right;
	int numberOfIntersections = 0;
	float tmin, tmax;

	stack.reserve(log2(photonCount) * 1.5);

	stack.push_back(0);

	while (!stack.empty()) {
		node = pool[stack.back()];
		stack.pop_back();

		numberOfIntersections++;

		if (!BoundingBoxIntersection2(ray, node.bounds, tmin, tmax)) continue;

		if (node.count == 0) {
			left = node.leftFirst;
			right = left + 1;

			stack.push_back(right);
			stack.push_back(left);
		}
	}

	return numberOfIntersections;
}