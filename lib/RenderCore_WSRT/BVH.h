#pragma once

#include "rendersystem.h"

namespace lh2core
{
	struct Primitive {
		// TODO
	};

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
		aabb bounds = aabb();
		int left, right;
		int first, count;

		// methods
		bool IsLeaf();
		void Subdivide();
		void Partition();

		__inline bool IsLeaf() { return count != 0; }
	};

	class BVH {
	public:
		BVHNode* pool;
		BVHNode root;
		int poolPtr;

		// methods
		void ConstructBVH(Primitive* primitive);
		aabb CalculateBounds(Primitive* primitive, int first, int count);
	};
}