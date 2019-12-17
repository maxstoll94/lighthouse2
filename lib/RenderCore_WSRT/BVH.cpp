#include "BVH.h"
#include <iostream>

using namespace lh2core;

constexpr int numberOfBins = 8;

void Swap(int* a, int* b) {
	int t = *a;
	*a = *b;
	*b = t;
}

void BVH::ConstructBVH() {
	int primitiveCount = vcount / 3;
	indices = new int[primitiveCount];
	for (int i = 0; i < primitiveCount; i++) {
		indices[i] = i;
	}

	pool = new BVHNode[primitiveCount * 2];

	int nodeIndex = 0;
	int first = 0;
	int last = primitiveCount - 1;
	int poolPtr = 2;
	Subdivide(nodeIndex, first, last, poolPtr);

	//for (int i = 0; i < primitiveCount * 2; i++) {
	//	if (pool[i].IsLeaf()) {
	//		cout << i << ": LEAF[" << pool[i].GetFirst() << ": " << pool[i].GetCount() << "], ";
	//	}
	//	else {
	//		cout << i << ": BRANCH[" << pool[i].GetLeft() << ", " << pool[i].GetRight() << "], ";
	//	}
	//}
	//cout << endl;
}

void BVH::Subdivide(const int nodeIndex, const int first, const int last, int &poolPtr) {
	BVHNode &node = pool[nodeIndex];
	CalculateBounds(first, last, node.bounds);
	//int splitIndex = Median(node, first, last);
	int splitIndex = BinningSurfaceAreaHeuristic(node, first, last);
	//int splitIndex = SurfaceAreaHeuristic(node, first, last);

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

void BVH::CalculateBounds(const int first, const int last, aabb &bounds) {
	bounds.Reset();

	for (int i = first; i <= last; i ++) {
		int index = indices[i] * 3;
		bounds.Grow(make_float3(vertices[index]));
		bounds.Grow(make_float3(vertices[index + 1]));
		bounds.Grow(make_float3(vertices[index + 2]));
	}
}

// Quicksort algorithm based on https://www.geeksforgeeks.org/cpp-program-for-quicksort/
void BVH::QuickSortPrimitives(const int axis, const int first, const int last) {
	if (first < last) {
		float pivot = get_axis(axis, vertices[indices[(first + last) / 2] * 3]);

		int i = first - 1;

		for (int j = first; j <= last - 1; j ++) {
			if (get_axis(axis, vertices[indices[j] * 3]) <= pivot) {
				i++;
				Swap(&indices[i], &indices[j]);
			}
		}
		Swap(&indices[i + 1], &indices[last]);

		int pivotIndex = i + 1;

		QuickSortPrimitives(axis, first, pivotIndex - 1);
		QuickSortPrimitives(axis, pivotIndex + 1, last);
	}
}

int BVH::Median(BVHNode &node, int first, int last) {
	if (last - first <= 3) return -1;

	int longestAxis = node.bounds.LongestAxis();

	float minAxis = get_axis(longestAxis, node.GetBounds().bmin3);
	float maxAxis = get_axis(longestAxis, node.GetBounds().bmax3);

	int splitIndex = first - 1;
	float splitPlane = (minAxis + maxAxis) / 2;
	for (int i = first; i <= last; i ++) {
		if (get_axis(longestAxis, vertices[indices[i] * 3]) < splitPlane) {
			splitIndex++;
			Swap(&indices[splitIndex], &indices[i]);
		}
	}

	aabb lBounds;
	aabb rBounds;

	int nLeft = splitIndex - first + 1;
	if (nLeft == 0) return -1;
	int nRight = last - splitIndex;
	if (nRight == 0) return -1;

	CalculateBounds(first, splitIndex, lBounds);
	CalculateBounds(splitIndex + 1, last, rBounds);

	float aLeft = lBounds.Area();
	float aRight = rBounds.Area();

	float splitCost = aLeft * nLeft + aRight * nRight;

	float a = node.GetBounds().Area();
	int n = last - first + 1;
	float noSplitCost = a * n;

	return splitCost < noSplitCost ? splitIndex : -1;
}

int BVH::BinningSurfaceAreaHeuristic(BVHNode &node, int first, int last) {
	if (last - first <= 3) return -1;

	int longestAxis = node.bounds.LongestAxis();

	float minAxis = get_axis(longestAxis, node.GetBounds().bmin3);
	float maxAxis = get_axis(longestAxis, node.GetBounds().bmax3);

	int splitIndices[numberOfBins - 1];

	for (int i = 0, l = numberOfBins - 1; i < l; i++) {
		float splitPlane = minAxis + (maxAxis - minAxis) / numberOfBins * (i + 1);
		int splitIndex = i == 0 ? first - 1 : splitIndices[i - 1];
		for (int j = splitIndex + 1; j <= last; j++) {
			if (get_axis(longestAxis, vertices[indices[j] * 3]) < splitPlane) {
				splitIndex++;
				Swap(&indices[splitIndex], &indices[j]);
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

int BVH::SurfaceAreaHeuristic(BVHNode &node, int first, int last) {
	int longestAxis = node.bounds.LongestAxis();
	QuickSortPrimitives(longestAxis, first, last);

	float a = node.GetBounds().Area();
	int n = last - first + 1;
	float lowestCost = a * n;
	int bestSplitIndex = -1;

	aabb lBounds;
	aabb rBounds;

	for (int currentSplitIndex = first; currentSplitIndex < last; currentSplitIndex++) {
		CalculateBounds(first, currentSplitIndex, lBounds);
		CalculateBounds(currentSplitIndex + 1, last, rBounds);

		float aLeft = lBounds.Area();
		float aRight = rBounds.Area();
		int nLeft = currentSplitIndex - first + 1;
		int nRight = last - currentSplitIndex;

		float currentCost = aLeft * nLeft + aRight * nRight;

		if (currentCost < lowestCost) {
			bestSplitIndex = currentSplitIndex;
			lowestCost = currentCost;
		}
	}

	return bestSplitIndex;
}
