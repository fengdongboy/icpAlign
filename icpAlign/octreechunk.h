#pragma once
#include "Vec.h"
#include "OctreeIndex.h"
#include <vector>
#include "EsslamBaseExport.h"

#define ESSLAM_API

namespace esslam
{
	class ESSLAM_API OctreeChunk
	{
	public:
		OctreeChunk();
		~OctreeChunk();

		inline bool Valid() { return m_valid; }
		inline void SetValid() { m_valid = true; }
		void SetMin(const trimesh::vec3& bmin);

		int AddPoint(int depth_max, int& current, int current_point_index, IndexKey& offset,
			std::vector<OctreeIndex>& indexes);
		int AddPointPersist(int depth_max, int& current, int current_point_index, IndexKey& offset,
			std::vector<OctreeIndex>& indexes, float resolution, const trimesh::point& p, const std::vector<trimesh::point>& points);
		int AddPoint(int depth_max, int& current, int current_point_index, std::vector<OctreeIndex>& indexes,
			const std::vector<char>& cell_indexes);

		void Clear(int depth_max, std::vector<OctreeIndex>& indexes);
	private:
		bool m_valid;
		int m_children[8];

	public:
		trimesh::vec3 m_bmin;
	};
}