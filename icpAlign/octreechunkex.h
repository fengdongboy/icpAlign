#pragma once
#include "Vec.h"
#include "OctreeIndex.h"
#include <vector>
#include "EsslamBaseExport.h"

namespace esslam
{
	class ESSLAM_API OctreeChunkEx
	{
	public:
		OctreeChunkEx();
		~OctreeChunkEx();

		int AddPoint(int depth_max, int& current, int current_point_index, IndexKey& offset,
			std::vector<OctreeIndex>& indexes);
	private:
		int m_children[8];
	};
}