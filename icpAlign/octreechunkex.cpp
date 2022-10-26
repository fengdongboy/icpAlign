#include "OctreeChunkEx.h"
#include <assert.h>

namespace esslam
{

	OctreeChunkEx::OctreeChunkEx()
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

	OctreeChunkEx::~OctreeChunkEx()
	{

	}

	int OctreeChunkEx::AddPoint(int depth_max, int& current, int current_point_index, IndexKey& key,
		std::vector<OctreeIndex>& indexes)
	{
		int key_max = 1 << depth_max;
		if (key.x < 0 || key.x >= key_max ||
			key.y < 0 || key.y >= key_max ||
			key.z < 0 || key.z >= key_max)
			return -1;

		int* leaf_data = &m_children[0];
		if (depth_max > 1)
		{
			while (depth_max > 1)
			{
				short ix = key.x / key_max;
				short iy = key.y / key_max;
				short iz = key.z / key_max;

				int& index = leaf_data[ix * 4 + iy * 2 + iz];
				if (index < 0) index = current++;
				leaf_data = &indexes.at(index).m_children[0];

				key.x = key.x % key_max;
				key.y = key.y % key_max;
				key.z = key.z % key_max;

				--depth_max;
				key_max /= 2;
			}
		}

		int& point_index = leaf_data[key.x * 4 + key.y * 2 + key.z];
		if (point_index < 0)
			point_index = current_point_index;
		return point_index;
	}
}