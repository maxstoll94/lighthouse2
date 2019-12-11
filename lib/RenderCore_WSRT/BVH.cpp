#include "BVH.h"
#include <iostream>

using namespace lh2core;

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
	// int splitIndex = PartitionNever(node, first, last);
	// int splitIndex = PartitionMedian(node, first, last);
	int splitIndex = PartitionSAH(node, first, last);

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
	uint first3 = first * 3;
	uint last3 = (last + 1) * 3;

	bounds = aabb(make_float3(mesh->vertices[first3]), make_float3(mesh->vertices[first3]));

	for (int i = first3 + 1; i < last3; i ++) {
		bounds.Grow(make_float3(mesh->vertices[i]));
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

int BVH::PartitionMedian(const BVHNode &node, const uint first, const uint last) {
	if (last - first < 3) {
		return -1;
	}

	Axis splitAxis = (Axis)(node.bounds.LongestAxis());
	QuickSortPrimitives(splitAxis, first, last);
	return floor((last + first) / 2);
}

int BVH::PartitionSAH(BVHNode &node, uint first, uint last) {
	float a = node.GetBounds().Area();
	uint n = last - first + 1;

	int lowestCost = a * n;
	int bestSplitIndex = -1;

	Axis splitAxis = (Axis)(node.bounds.LongestAxis());
	QuickSortPrimitives(splitAxis, first, last);

	aabb lBounds;
	aabb rBounds;

	for (int currentSplitIndex = first; currentSplitIndex < last; currentSplitIndex++) {
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

int BVH::PartitionNever(BVHNode &node, uint first, uint last) {
	return -1;
}