#include "BVH.h"
#include <iostream>

using namespace lh2core;

constexpr int numberOfBins = 8;

void Swap(int* a, int* b) {
	int t = *a;
	*a = *b;
	*b = t;
}

void BVH::Update(AnimationType animationType) {
	int nodeIndex = 0;
	int first = 0;
	int last = vcount / 3 - 1;
	int poolPtr = 2;

	switch (animationType) {
	case StaticAnimation: 
		// model has no animation
		if (updateId == 0) {
			clock_t begin = clock();
			Subdivide(Binning, nodeIndex, first, last, poolPtr);
			clock_t end = clock();
			double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
			cout << "constructed bvh of " << vcount << " triangles in " << elapsed_secs << "s" << endl;
		}

		break;

	case ModestAnimation:
		if (updateId == 0) {
			Subdivide(Binning, nodeIndex, first, last, poolPtr);
		}
		else {
			GrowBounds();
		}

		break;

	case DynamicAnimation:
		// only reconstruct BVH every 10 frames
		if (updateId % 10 == 0) {
			Subdivide(MedianSplit, nodeIndex, first, last, poolPtr);
		}
		// refit bvh every other time
		else {
			RefitBounds();
		}

		break;
	}


	updateId += 1;

}

void BVH::RefitBounds(const int nodeIndex) {
	BVHNode&node = pool[nodeIndex];

	if (node.IsLeaf()) {
		CalculateBounds(node.GetLeft(), node.GetLeft() + node.GetCount() - 1, node.bounds);
	}
	else {
		int left = node.GetLeft();
		int right = node.GetRight();

		RefitBounds(left);
		RefitBounds(right);

		node.bounds = pool[left].bounds.Union(pool[right].bounds);
	}
}

void BVH::GrowBounds(const int nodeIndex) {
	BVHNode&node = pool[nodeIndex];

	if (node.IsLeaf()) {
		aabb bounds;
		CalculateBounds(node.GetLeft(), node.GetLeft() + node.GetCount() - 1, bounds);
		node.bounds.Grow(bounds);
	}
	else {
		int left = node.GetLeft();
		int right = node.GetRight();

		RefitBounds(left);
		RefitBounds(right);

		node.bounds = pool[left].bounds.Union(pool[right].bounds);
	}
}


void BVH::Subdivide(const SubdivideHeuristic subdivideHeuristc, const int nodeIndex, const int first, const int last, int &poolPtr) {
	BVHNode &node = pool[nodeIndex];
	CalculateBounds(first, last, node.bounds);
	int splitIndex;
	switch (subdivideHeuristc) {
	case Binning:
		splitIndex = Median(node, first, last);
		break;
	case MedianSplit:
		splitIndex = BinningSurfaceAreaHeuristic(node, first, last);
		break;
	case AllSplits:
		splitIndex = SurfaceAreaHeuristic(node, first, last);
		break;
	}
	

	if (splitIndex == -1) {
		node.count = last - first + 1;
		node.leftFirst = first;
	}
	else {
		node.leftFirst = poolPtr;
		poolPtr += 2;
		node.count = 0;

		Subdivide(subdivideHeuristc, node.GetLeft(), first, splitIndex, poolPtr);
		Subdivide(subdivideHeuristc, node.GetRight(), splitIndex + 1, last, poolPtr);
	}
}

void BVH::CalculateBounds(const int first, const int last, aabb&bounds) {
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
			if (get_axis(axis, centroids[indices[j]]) <= pivot) {
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
			if (get_axis(longestAxis, centroids[indices[j]]) < splitPlane) {
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

BVHTopNode* FindBestMatch(BVHTopNode* a, const vector<BVHTopNode*>&topNodes) {
	float3 centerA = (a->bounds.bmin3 + a->bounds.bmax3) * 0.5f;

	BVHTopNode* bestNode = a;
	float bestDistance = 1e34f;

	for (BVHTopNode*b : topNodes) {
		if (b == a) continue;

		float3 centerB = (b->bounds.bmin3 + b->bounds.bmax3) * 0.5f;
		float distance = length(centerB - centerA);
		if (distance < bestDistance) {
			bestDistance = distance;
			bestNode = b;
		}
	}

	return bestNode;
}

void BVHTop::UpdateTopLevel(vector<BVHTopNode*> instances) {
	if (instances.size() == 0) return;

	vector<BVHTopNode*> topNodes(instances);
	int topLevelBVHsPtr = 0;

	BVHTopNode*a = topNodes.front();
	BVHTopNode*b = FindBestMatch(a, topNodes);

	while (topNodes.size() > 1) {
		BVHTopNode* c = FindBestMatch(b, topNodes);

		if (a == c) {
			BVHTopNode*topNode = &pool[topLevelBVHsPtr++];
			topNode->bvh = nullptr;
			topNode->left = a;
			topNode->right = b;
			topNode->bounds = a->bounds.Union(b->bounds);

			topNodes.erase(find(topNodes.begin(), topNodes.end(), a));
			topNodes.erase(find(topNodes.begin(), topNodes.end(), b));
			topNodes.push_back(topNode);

			a = topNode;
			b = FindBestMatch(a, topNodes);
		}
		else {
			a = b;
			b = c;
		}
	}

	root = topNodes.front();
}
