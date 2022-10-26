#pragma once
//#include "EsslamBaseExport.h"
#include "Vec.h"
#include "octreechunk.h"
#include <vector>
#include "OctreeIndex.h"
#include "Xform.h"
#include "TriMesh.h"

#define  ESSLAM_API

namespace esslam
{
	class ESSLAM_API Octree
	{
	public:
		Octree(int cell_depth = 5, float cell_resolution = 0.2f, int point_size = 8000000);
		~Octree();

		void Initialize(const trimesh::vec3& center);
		void Clear();
		void Insert(const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals);
		void Insert(const std::vector<trimesh::vec3>& points,
			const std::vector<trimesh::vec3>& normals, const std::vector<trimesh::Color>& colors,
			const trimesh::xform& xf, std::vector<int>& indexes);

		void QuickInsert(const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals);
	protected:

	public:
		bool m_initialized;
		const int m_cell_depth;
		const float m_cell_resolution;
		const float m_cell_resolution2;
		const float m_cell_resolution_inv;
		const float m_len;
		const int m_chunk_size;
		const int m_chunk_depth;
		const float m_chunk_resolution;
		const float m_chunk_resolution_inv;
		const int m_chunk_size2;

		trimesh::vec3 m_center;
		trimesh::vec3 m_min;

		std::vector<OctreeChunk> m_chunks;

		std::vector<OctreeIndex> m_indexes;

		trimesh::TriMesh m_trimesh;

		//std::vector<trimesh::vec3> m_points;
		//std::vector<trimesh::vec3> m_normals;

		int m_current_index;
		int m_last_point_index;
		int m_current_point_index;

		std::vector<QuickIndex> m_quick_index;

	public:
		bool m_use_cube_center;
		bool m_use_persist_fusion;
	};
}