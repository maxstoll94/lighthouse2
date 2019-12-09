#include "BVH.h"

using namespace lh2core;

void BVH::ConstructBVH(CoreTri* _primitives, uint primitiveCount) {
	indices = new uint[primitiveCount];
	primitives = _primitives;

	for (int i = 0; i < primitiveCount; i++) {
		indices[i] = i;
	}

	pool = new BVHNode[primitiveCount * 2];

	uint nodeIndex = 0;
	uint first = 0;
	uint last = primitiveCount - 1;
	uint poolPtr = 2;
	Subdivide(nodeIndex, first, last, poolPtr);
}

void BVH::Subdivide(uint nodeIndex, uint first, uint last, uint &poolPtr) {
	BVHNode node = pool[nodeIndex];
	node.bounds = CalculateBounds(first, last);
	uint splitIndex;
	bool hasSplit = Partition(node.bounds, splitIndex, first, last);

	if (hasSplit) {
		node.leftFirst = poolPtr;
		poolPtr += 2;
		node.count = 0;

		Subdivide(node.GetLeft(), first, splitIndex, poolPtr);
		Subdivide(node.GetRight(), splitIndex + 1, last, poolPtr);
	}
	else {
		node.count = last - first;
		node.leftFirst = first;
	}
}

bool BVH::Partition(aabb bounds, uint &splitIndex, uint first, uint last) {
	Axis splitAxis = (Axis)(bounds.LongestAxis());
	QuickSortPrimitives(splitAxis, first, last);
	splitIndex = floor((last + first) / 2);

	return last - first < 3;
}

aabb BVH::CalculateBounds(uint first, uint last) {
	aabb bounds = aabb(primitives[first].vertex0, primitives[first].vertex0);

	for (int i = first; i < last; i++) {
		bounds.Grow(primitives[i].vertex0);
		bounds.Grow(primitives[i].vertex1);
		bounds.Grow(primitives[i].vertex2);
	}

	return bounds;
}

// Quicksort algorithm based on https://www.geeksforgeeks.org/cpp-program-for-quicksort/
void BVH::QuickSortPrimitives(Axis axis, uint first, uint last) {
	if (first < last) {
		float pivot = GetAxis(axis, primitives[indices[first]].vertex0);

		int i = (first - 1);  // Index of smaller element 

		for (int j = first; j <= last - 1; j++) {
			// If current element is smaller than or 
			// equal to pivot 
			if (GetAxis(axis, primitives[indices[j]].vertex0) <= pivot) {
				i++;    // increment index of smaller element 
				Swap(&indices[i], &indices[j]);
			}
		}
		Swap(&indices[i + 1], &indices[last]);

		int pivotIndex = (i + 1);

		QuickSortPrimitives(axis, first, pivotIndex - 1);
		QuickSortPrimitives(axis, pivotIndex + 1, last);
	}
}

float BVH::GetAxis(Axis axis, float3 vector) {
	switch (axis) {
	case Xaxis:
		return vector.x;
	case Yaxis:
		return vector.y;
	case Zaxis:
		return vector.z;
	}
}

void BVH::Swap(uint* a, uint* b) {
	uint t = *a;
	*a = *b;
	*b = t;
}