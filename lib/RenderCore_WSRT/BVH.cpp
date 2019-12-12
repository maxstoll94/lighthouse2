#include "BVH.h"
#include <iostream>

using namespace lh2core;

constexpr int numberOfBins = 4;

void Swap(uint* a, uint* b) {
	uint t = *a;
	*a = *b;
	*b = t;
}

float GetAxis(const Axis axis, const float4 &vector) {
	switch (axis) {
	case Xaxis:
		return vector.x;
	case Yaxis:
		return vector.y;
	case Zaxis:
		return vector.z;
	}
}

float GetAxis(const Axis axis, const float3 &vector) {
	switch (axis) {
	case Xaxis:
		return vector.x;
	case Yaxis:
		return vector.y;
	case Zaxis:
		return vector.z;
	}
}

void BVH::ConstructBVH(Mesh* _mesh) {
	mesh = _mesh;
	uint primitiveCount = mesh->vcount / 3;
	indices = new uint[primitiveCount];
	for (int i = 0; i < primitiveCount; i++) {
		indices[i] = i;
	}

	pool = new BVHNode[primitiveCount * 2];

	uint nodeIndex = 0;
	uint first = 0;
	uint last = primitiveCount - 1;
	uint poolPtr = 2;
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

void BVH::Subdivide(const uint nodeIndex, const uint first, const uint last, uint &poolPtr) {
	BVHNode &node = pool[nodeIndex];
	CalculateBounds(first, last, node.bounds);
	int splitIndex = SurfaceAreaHeuristic(node, first, last);

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

void BVH::CalculateBounds(const uint first, const uint last, aabb &bounds) {
	bounds = aabb(make_float3(mesh->vertices[indices[first] * 3]), make_float3(mesh->vertices[indices[first] * 3]));

	for (int i = first; i <= last; i ++) {
		int index = indices[i] * 3;
		bounds.Grow(make_float3(mesh->vertices[index]));
		bounds.Grow(make_float3(mesh->vertices[index + 1]));
		bounds.Grow(make_float3(mesh->vertices[index + 2]));
	}
}

// Quicksort algorithm based on https://www.geeksforgeeks.org/cpp-program-for-quicksort/
void BVH::QuickSortPrimitives(const Axis axis, const uint first, const uint last) {
	if (first < last) {
		float pivot = GetAxis(axis, mesh->vertices[indices[first] * 3]);

		int i = first - 1;

		for (int j = first; j <= last - 1; j ++) {
			if (GetAxis(axis, mesh->vertices[indices[j] * 3]) <= pivot) {
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

//int BVH::PartitionSAH(BVHNode &node, uint first, uint last) {
//	float a = node.GetBounds().Area();
//	uint n = last - first + 1;
//
//	int lowestCost = a * n;
//	int bestSplitIndex = -1;
//
//	Axis splitAxis = (Axis)(node.bounds.LongestAxis());
//	QuickSortPrimitives(splitAxis, first, last);
//
//	aabb lBounds;
//	aabb rBounds;
//
//	for (int currentSplitIndex = first; currentSplitIndex < last; currentSplitIndex++) {
//		CalculateBounds(first, currentSplitIndex, lBounds);
//		CalculateBounds(currentSplitIndex + 1, last, rBounds);
//
//		float aLeft = lBounds.Area();
//		float aRight = rBounds.Area();
//		float nLeft = currentSplitIndex - first + 1;
//		float nRight = last - currentSplitIndex;
//
//		float currentCost = aLeft * nLeft + aRight * nRight;
//
//		if (currentCost < lowestCost) {
//			bestSplitIndex = currentSplitIndex;
//			lowestCost = currentCost;
//		}
//	}
//
//	return bestSplitIndex;
//}

int BVH::SurfaceAreaHeuristic(BVHNode &node, uint first, uint last) {
	Axis longestAxis = (Axis)node.bounds.LongestAxis();

	float minAxis = GetAxis(longestAxis, node.GetBounds().bmin3);
	float maxAxis = GetAxis(longestAxis, node.GetBounds().bmax3);

	uint splitIndices[numberOfBins - 1];

	for (int i = 0, l = numberOfBins - 1; i < l; i++) {
		float splitPlane = minAxis + (maxAxis - minAxis) / numberOfBins * (i + 1);
		int splitIndex = i == 0 ? first : splitIndices[i - 1];
		for (int j = splitIndex; j < last; j++) {
			if (GetAxis(longestAxis, mesh->vertices[indices[j] * 3]) < splitPlane) {
				Swap(&indices[splitIndex], &indices[j]);
				splitIndex++;
			}
		}
		splitIndices[i] = splitIndex;
	}

	float a = node.GetBounds().Area();
	uint n = last - first + 1;
	int lowestCost = a * n;
	int bestSplitIndex = -1;

	aabb lBounds;
	aabb rBounds;

	for (uint i = 0, l = numberOfBins - 1; i < l; i++) {
		uint currentSplitIndex = splitIndices[i];
		CalculateBounds(first, currentSplitIndex, lBounds);
		CalculateBounds(currentSplitIndex + 1, last, rBounds);

		float aLeft = lBounds.Area();
		float aRight = rBounds.Area();
		float nLeft = currentSplitIndex - first + 1;
		float nRight = last - currentSplitIndex;

		float currentCost = aLeft * nLeft + aRight * nRight;

		if (currentCost < lowestCost) {
			bestSplitIndex = currentSplitIndex;
			lowestCost = currentCost;
		}
	}

	return bestSplitIndex;
}

uint* BVH::CalculateSplitIndices(BVHNode &node, uint first, uint last) {
	Axis longestAxis = (Axis)node.bounds.LongestAxis();

	float minAxis = GetAxis(longestAxis, node.GetBounds().bmin3);
	float maxAxis = GetAxis(longestAxis, node.GetBounds().bmax3);

	uint splitIndices[numberOfBins - 1];

	for (int i = 0, l = numberOfBins - 1; i < l; i++) {
		float splitPlane = minAxis + (maxAxis - minAxis) / numberOfBins * (i + 1);
		int splitIndex = i == 0 ? first : splitIndices[i - 1];
		for (int j = splitIndex; j < last; j++) {
			if (GetAxis(longestAxis, mesh->vertices[indices[j] * 3]) < splitPlane) {
				Swap(&indices[splitIndex], &indices[j]);
				splitIndex++;
			}
		}
		splitIndices[i] = splitIndex;
	}

	return splitIndices;
}

int BVH::PartitionNever(BVHNode &node, uint first, uint last) {
	return -1;
}