#pragma once
#include "rendersystem.h"
#include "classes.h"

namespace lh2core
{
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
		Mesh* mesh;
		int* indices;

		void ConstructBVH(Mesh *mesh);
		void Subdivide(const int nodeIndex, const int first, const int last, int &poolPtr);
		void CalculateBounds(const int first, const int last, aabb &aabb);
		void QuickSortPrimitives(const int axis, const int first, const int last);
		int Median(BVHNode &node, const int first, const int last);
		int BinningSurfaceAreaHeuristic(BVHNode &node, const int first, const int last);
	};
}