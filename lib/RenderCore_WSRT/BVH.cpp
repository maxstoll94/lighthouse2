#include "BVH.h"

using namespace lh2core;

void BVH::ConstructBVH(Mesh* _mesh) {
	mesh = _mesh;
	uint primitiveCount = mesh->vcount;
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
}

void BVH::Subdivide(const uint nodeIndex, const uint first, const uint last, uint &poolPtr) {
	BVHNode node = pool[nodeIndex];
	CalculateBounds(first, last, node.bounds);
	uint splitIndex;
	bool hasSplit = Partition(node, first, last, splitIndex);

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

bool BVH::Partition(const BVHNode &node, const uint first, const uint last, uint &splitIndex) {
	Axis splitAxis = (Axis)(node.bounds.LongestAxis());
	QuickSortPrimitives(splitAxis, first, last);
	splitIndex = floor((last + first) / 2);

	return last - first < 3;
}

void BVH::CalculateBounds(const uint first, const uint last, aabb &bounds) {
	uint first3 = first * 3;
	uint last3 = last * 3;

	bounds = aabb(make_float3(mesh->vertices[first3]), make_float3(mesh->vertices[first3]));

	for (int i = first3 + 1; i < last3; i++) {
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

float BVH::GetAxis(const Axis axis, const float4 &vector) {
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