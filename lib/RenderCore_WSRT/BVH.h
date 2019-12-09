#pragma once
#include "rendersystem.h"

namespace lh2core
{
	enum Axis {
		Xaxis = 0, Yaxis = 1, Zaxis = 2
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
	public:
		aabb bounds;
		uint leftFirst;
		uint count;

		__inline bool IsLeaf() { return count != 0; }
		__inline uint GetLeft() { return leftFirst; }
		__inline uint GetRight() { return leftFirst + 1; }
	};

	class BVH {
	public:
		BVHNode* pool;
		CoreTri* primitives;
		uint* indices;

		// methods
		void ConstructBVH(CoreTri* primitive, uint primitiveCount);
		void Subdivide(uint nodeIndex, uint first, uint last, uint &poolPtr);
		bool Partition(aabb bounds, uint &splitIndex, uint first, uint last);
		aabb CalculateBounds(uint first, uint last);
		void QuickSortPrimitives(Axis axis, uint first, uint last);
		float GetAxis(Axis axis, float3 vector);
		void Swap(uint *a, uint *b);
	};
}