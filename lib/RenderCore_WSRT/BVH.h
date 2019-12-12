#pragma once
#include "rendersystem.h"
#include "classes.h"

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

		__inline aabb GetBounds() { return bounds; }
		__inline bool IsLeaf() { return count != 0; }
		__inline uint GetLeft() { return leftFirst; }
		__inline uint GetRight() { return leftFirst + 1; }

		__inline uint GetCount() { return count; }
		__inline uint GetFirst() { return leftFirst; }

	};

	class BVH {
	public:
		BVHNode* pool;
		Mesh* mesh;
		uint* indices;

		void ConstructBVH(Mesh *mesh);
		void Subdivide(const uint nodeIndex, const uint first, const uint last, uint &poolPtr);
		void CalculateBounds(const uint first, const uint last, aabb &aabb);
		void QuickSortPrimitives(const Axis axis, const uint first, const uint last);
		//int PartitionSAH(BVHNode &node, uint first, uint last);
		int SurfaceAreaHeuristic(BVHNode & node, uint first, uint last);
		uint * CalculateSplitIndices(BVHNode & node, uint first, uint last);
		int PartitionNever(BVHNode & node, uint first, uint last);
	};
}