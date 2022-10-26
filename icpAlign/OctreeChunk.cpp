#include "OctreeChunk.h"
#include <assert.h>

namespace esslam
{

	OctreeChunk::OctreeChunk()
		:m_valid(false)
	{
		m_children[0] = -1;
		m_children[1] = -1;
		m_children[2] = -1;
		m_children[3] = -1;
		m_children[4] = -1;
		m_children[5] = -1;
		m_children[6] = -1;
		m_children[7] = -1;
	}

	OctreeChunk::~OctreeChunk()
	{

	}

	void OctreeChunk::SetMin(const trimesh::vec3& bmin)
	{
		m_bmin = bmin;
	}

	int OctreeChunk::AddPoint(int depth_max, int& current, int current_point_index, IndexKey& key,
		std::vector<OctreeIndex>& indexes)
	{
		int key_max = 1 << depth_max;
		if (key.x < 0 || key.x >= key_max ||
			key.y < 0 || key.y >= key_max ||
			key.z < 0 || key.z >= key_max)
			return -1;

		key_max /= 2;

		short ix = key.x / key_max;
		short iy = key.y / key_max;
		short iz = key.z / key_max;

		int& index = m_children[ix * 4 + iy * 2 + iz];
		if (index < 0) index = current++;

		OctreeIndex& octree_index = indexes.at(index);
		key.x = key.x % key_max;
		key.y = key.y % key_max;
		key.z = key.z % key_max;

		OctreeIndex* current_oc = &octree_index;
		int depth = 0;
		while (depth < depth_max - 2)
		{
			key_max /= 2;
			short iix = key.x / key_max;
			short iiy = key.y / key_max;
			short iiz = key.z / key_max;

			int& iindex = current_oc->m_children[iix * 4 + iiy * 2 + iiz];
			if (iindex < 0) iindex = current++;

			current_oc = &indexes.at(iindex);


			key.x = key.x % key_max;
			key.y = key.y % key_max;
			key.z = key.z % key_max;
			++depth;
		}

		int& point_index = current_oc->m_children[key.x * 4 + key.y * 2 + key.z];
		if (point_index < 0)
			point_index = current_point_index;
		return point_index;
	}

	int OctreeChunk::AddPointPersist(int depth_max, int& current, int current_point_index, IndexKey& key,
		std::vector<OctreeIndex>& indexes, float resolution, const trimesh::point& p, const std::vector<trimesh::point>& points)
	{
		int key_max = 1 << depth_max;
		if (key.x < 0 || key.x >= key_max ||
			key.y < 0 || key.y >= key_max ||
			key.z < 0 || key.z >= key_max)
			return -1;

		key_max /= 2;

		short ix = key.x / key_max;
		short iy = key.y / key_max;
		short iz = key.z / key_max;

		int& index = m_children[ix * 4 + iy * 2 + iz];
		if (index < 0) index = current++;

		OctreeIndex& octree_index = indexes.at(index);
		key.x = key.x % key_max;
		key.y = key.y % key_max;
		key.z = key.z % key_max;

		OctreeIndex* current_oc = &octree_index;
		int depth = 0;
		while (depth < depth_max - 2)
		{
			key_max /= 2;
			short iix = key.x / key_max;
			short iiy = key.y / key_max;
			short iiz = key.z / key_max;

			int& iindex = current_oc->m_children[iix * 4 + iiy * 2 + iiz];
			if (iindex < 0) iindex = current++;

			current_oc = &indexes.at(iindex);


			key.x = key.x % key_max;
			key.y = key.y % key_max;
			key.z = key.z % key_max;
			++depth;
		}

		int& point_index = current_oc->m_children[key.x * 4 + key.y * 2 + key.z];
		if (point_index < 0)
		{
			bool insert = true;
			//test nearest
			for (int i = 0; i < 8; ++i)
			{
				if (current_oc->m_children[i] >= 0)
				{
					const trimesh::point& np = points.at(current_oc->m_children[i]);
					float d = trimesh::len2(np - p);
					if (d < resolution)
					{
						insert = false;
						break;
					}
				}
			}
			if(insert) point_index = current_point_index;
		}
		return point_index;
	}

	int OctreeChunk::AddPoint(int depth_max, int& current, int current_point_index, std::vector<OctreeIndex>& indexes,
		const std::vector<char>& cell_indexes)
	{
		int depth_size = (int)cell_indexes.size();
		assert(depth_size >= 2);
		int& index = m_children[cell_indexes.at(0)];
		//if (index < 0) index = current++;
		if (index < 0)
		{
			index = current;
			++current;
		}
		OctreeIndex* current_oc = &indexes.at(index);

		int depth = 0;
		for (int i = 1; i <= depth_size - 2; ++i)
		{
			int& iindex = current_oc->m_children[cell_indexes[i]];
			if (iindex < 0)
			{
				iindex = current;
				++current;
			}
			current_oc = &indexes[iindex];
		}

		int& point_index = current_oc->m_children[cell_indexes.back()];
		if (point_index < 0)
			point_index = current_point_index;
		return point_index;
	}

	void OctreeChunk::Clear(int depth_max, std::vector<OctreeIndex>& indexes)
	{
		/*if (depth_max > 1)
		{
			for (int i = 0; i < 8; ++i)
			{
				int index = m_children[i];
				if (index > 0)
				{
					OctreeIndex* current_oc = &indexes.at(index);
					current_oc->
				}
			}
		}
		

		int depth = 0;
		for (int i = 1; i <= depth_size - 2; ++i)
		{
			int& iindex = current_oc->m_children[cell_indexes[i]];
			if (iindex < 0)
			{
				current_oc = NULL;
				break;
			}
			current_oc = &indexes[iindex];
		}

		if (current_oc)
		{

		}
		int& point_index = current_oc->m_children[cell_indexes.back()];
		if (point_index < 0)
			point_index = current_point_index;*/
	}
}