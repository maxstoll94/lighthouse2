#pragma once
#include "rendersystem.h"
#include "classes.h"

namespace lh2core
{
	enum SubdivideHeuristic { Binning, MedianSplit, AllSplits };

	//  +-----------------------------------------------------------------------------+
	//  |  BVHNode                                                                    |
	//  |  1: bounds contain 2 float3's to describe an axis aligned bounding box.	  |	
	//	|  2: isLeaf describes if the node contains Primitives or more BVHNode's	  |
	//	|  3: left the index of the left BVH child(only applicable if !isLeaf)		  | 	
	//	|  4: right the index of the right BVH child(only applicable if !isLeaf and	  |	
	//  |     is always equal to left + 1)											  |					
	// 	|  5: first the index of the first primitive(only applicable if isLeaf)		  |
	//	|  6: count the number of primitives(only applicable if isLeaf)         LH2'19|
	//  +-----------------------------------------------------------------------------+
	struct BVHNode {
	public:
		aabb bounds;
		int leftFirst;
		int count;

		__inline aabb GetBounds() { return bounds; }
		__inline bool IsLeaf() { return count != 0; }
		__inline int GetLeft() { return leftFirst; }
		__inline int GetRight() { return leftFirst + 1; }

		__inline int GetCount() { return count; }
		__inline int GetFirst() { return leftFirst; }
	};

	class BVH {
	public:
		BVHNode* pool;
		int* indices;

		float4* vertices = 0;		// vertex data received via SetGeometry
		float4* centroids = 0;
		int vcount = 0;				// vertex count
		CoreTri* triangles = 0;		// 'fat' triangle data
		int updateId = 0;

		void RefitBounds(const int nodeIndex = 0);
		void GrowBounds(const int nodeIndex = 0);
		void Subdivide(const SubdivideHeuristic subdivideHeuristc, const int nodeIndex, const int first, const int last, int&poolPtr);
		void CalculateBounds(const int first, const int last, aabb&aabb);
		void QuickSortPrimitives(const int axis, const int first, const int last);
		int Median(BVHNode&node, const int first, const int last);
		int BinningSurfaceAreaHeuristic(BVHNode&node, const int first, const int last);
		int SurfaceAreaHeuristic(BVHNode&node, const int first, const int last);
		void Update(AnimationType animationType);
	};

	class BVHTopNode {
	public:
		aabb bounds; // 24
		
		// leaf
		BVH* bvh; // 64
		mat4 transform; //64

		// tree
		BVHTopNode*left; //64
		BVHTopNode*right; //64

		int instanceIdx;

		__inline bool IsLeaf() const { return bvh != nullptr; }
	};
	
	class BVHTop {
	public:
		BVHTopNode*root;
		BVHTopNode*pool;
		int bvhCount = 0;
		void UpdateTopLevel(vector<BVHTopNode*> instances);
	};
}
