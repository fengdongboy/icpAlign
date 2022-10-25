#include <functional>   // std::greater
#include <algorithm>    // std::sort
#include <numeric>
#include "projectionicpmapping.h"
#include <omp.h>
#include "get_boundingbox_corner.h"
#include "compute_boundingbox.h"

namespace trimesh
{
	ProjectionICPMapping::ProjectionICPMapping(float fx, float fy, float cx, float cy)
		:m_fx(fx), m_fy(fy), m_cx(cx), m_cy(cy), m_source(NULL)
		, m_target(NULL), m_tracer(NULL), m_sample_count(1000)
		, m_full(false)
	{
		m_k1 = 0.0f;
		m_k2 = 0.0f;
		m_k3 = 0.0f;
		m_k4 = 0.0f;
		m_k5 = 0.0f;
	}

	ProjectionICPMapping::~ProjectionICPMapping()
	{

	}

	void ProjectionICPMapping::SetSource(trimesh::TriMesh* source)
	{
		m_source = source;
	}

	void ProjectionICPMapping::SetTarget(trimesh::TriMesh* target)
	{
		m_target = target;
	}

	void ProjectionICPMapping::SetSampleCount(int sample_count)
	{
		m_sample_count = sample_count;
	}

	void ProjectionICPMapping::SetFull()
	{
		m_full = true;
	}

	int ProjectionICPMapping::GetSourcePointSize()
	{
		return m_source->vertices.size();
	}

	int  ProjectionICPMapping::GetTargetPointSize()
	{
		return m_target->vertices.size();
	}

	bool ProjectionICPMapping::Valid()
	{
		return m_source && m_target;
	}


	bool ProjectionICPMapping::SetSourcePointKeyData(int KeyIdx, int PIdx)
	{
		if (PIdx < 0 || PIdx >  m_source->vertices.size())
			return false;

		//已经是关键帧
		if (m_source->KeyFrameIndex[PIdx] > -1)
			return false;

		m_source->KeyFrameIndex[PIdx] = KeyIdx;
		//m_source->bKeyFrame[PIdx] = KeyIdx;
		//m_source->bKeyFrame[idx] 

		return true;
	}

	bool ProjectionICPMapping::SetTargetPointKeyData(int KeyIdx, int PIdx)
	{
		if (PIdx < 0 || PIdx >  m_target->vertices.size())
			return false;

		//已经是关键帧
		if (m_target->KeyFrameIndex[PIdx] > -1)
			return false;

		//m_target->bKeyFrame[PIdx] = true;
		m_target->KeyFrameIndex[PIdx] = KeyIdx;
		//m_source->bKeyFrame[idx] 

		return true;
	}

	void ProjectionICPMapping::Setup()
	{
		const trimesh::box3& source_box = m_source->bbox;
		const trimesh::box3& target_box = m_target->bbox;

		//获取源点云包围盒角点
		GetBoundingboxCorner(source_box, m_source_corner);
	}

	void ProjectionICPMapping::SetTracer(ProjectionICPTracer* tracer)
	{
		m_tracer = tracer;
	}

	void ProjectionICPMapping::BeforeFMMapping(const trimesh::xform& source_form)
	{
		m_target_indexes.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		int nv = 0;
		int sz = 100;
		if (m_full)
		{
			nv = m_target->vertices.size();
			sz = 1;
		}
		else
		{
			nv = (int)m_target->vertices.size() / sz;
			if (nv < 1000)
			{
				nv = 1000;
				sz = (int)m_target->vertices.size() / nv;
			}
		}

		std::vector<int> idx(nv, -1);
		int num_omp = omp_get_num_procs();
//		//#pragma omp parallel for num_threads(num_omp)
		{
#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = i * sz;
				if (index >= m_target->vertices.size() ) continue;
				const trimesh::point& target_point = m_target->vertices.at(index);
				{
					trimesh::point p = xf12 * target_point;

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((m_fx * p[0] + m_cx * p[2]) / p[2]);
					int v = (int)roundf((m_fy * p[1] + m_cy * p[2]) / p[2]);

					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6 && trimesh::len2(target_point - p2) < 1.0f)
							{
								idx[i] = index;
							}
						}
					}
				}
			}
		}

		for (int i = 0; i < nv; i++)
		{
			if (idx[i] >= 0)
			{
				m_target_indexes.push_back(idx[i]);
			}
		}
	}

	void ProjectionICPMapping::FMMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs)
	{
		pairs.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		//float xmin, xmax, ymin, ymax, zmin, zmax;
		//TransformBoundingbox(m_source_corner, source_form,
		//	xmin, xmax, ymin, ymax, zmin, zmax);

		int nv = (int)m_target_indexes.size();
		std::vector<PtPair> 	pairs1(nv);
		std::vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();
		//#pragma omp parallel for num_threads(num_omp)
		{
#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = m_target_indexes.at(i);
				const trimesh::point& target_point = m_target->vertices.at(index);
				//if (target_point.x > xmin && target_point.x < xmax &&
				//	target_point.y > ymin && target_point.y < ymax &&
				//	target_point.z > zmin && target_point.z < zmax)
				{
					trimesh::point p = xf12 * target_point;

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((m_fx * p[0] + m_cx * p[2]) / p[2]);
					int v = (int)roundf((m_fy * p[1] + m_cy * p[2]) / p[2]);

					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(target_point, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}

		if (n < 100)
		{
			if ((float)n / (float)nv < 0.2)
				pairs.resize(0);
		}
	}

	void ProjectionICPMapping::Mapping(const trimesh::xform &source_form, std::vector<PtPair> &pairs)
	{
		pairs.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		float xmin, xmax, ymin, ymax, zmin, zmax;
		TransformBoundingbox(m_source_corner, source_form,
			xmin, xmax, ymin, ymax, zmin, zmax);

		const int sz = floor(m_source->vertices.size() / m_sample_count);
		int nv = (int)m_target->vertices.size() / sz;
		std::vector<PtPair> 	pairs1(nv);
		std::vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();

		//#pragma omp parallel for num_threads(num_omp)
		{
//#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = i * sz;

				const trimesh::point& target_point = m_target->vertices.at(index);
				if (target_point.x > xmin && target_point.x < xmax &&
					target_point.y > ymin && target_point.y < ymax &&
					target_point.z > zmin && target_point.z < zmax)
				{
					trimesh::point p = xf12 * target_point;

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((m_fx * p[0] + m_cx * p[2]) / p[2]);
					int v = (int)roundf((m_fy * p[1] + m_cy * p[2]) / p[2]);

					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						int n = m_target->grid[j];
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(target_point, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}

		if (n < 100)
		{
			if ((float)n / (float)nv < 0.2)
				pairs.resize(0);
		}
	}

	void ProjectionICPMapping::FullMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs)
	{
		pairs.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		float xmin, xmax, ymin, ymax, zmin, zmax;
		TransformBoundingbox(m_source_corner, source_form,
			xmin, xmax, ymin, ymax, zmin, zmax);

		int nv = (int)m_target->vertices.size();
		std::vector<PtPair> 	pairs1(nv);
		std::vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();
		//#pragma omp parallel for num_threads(num_omp)
		{
#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = i;
				const trimesh::point& target_point = m_target->vertices.at(index);
				if (target_point.x > xmin && target_point.x < xmax &&
					target_point.y > ymin && target_point.y < ymax &&
					target_point.z > zmin && target_point.z < zmax)
				{
					trimesh::point p = xf12 * target_point;

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((m_fx * p[0] + m_cx * p[2]) / p[2]);
					int v = (int)roundf((m_fy * p[1] + m_cy * p[2]) / p[2]);

					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(target_point, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}
	}

	void ProjectionICPMapping::FullFMMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs)
	{
		pairs.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		//float xmin, xmax, ymin, ymax, zmin, zmax;
		//TransformBoundingbox(m_source_corner, source_form,
		//	xmin, xmax, ymin, ymax, zmin, zmax);

		int nv = (int)m_target_indexes.size();
		std::vector<PtPair> 	pairs1(nv);
		std::vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();
		//#pragma omp parallel for num_threads(num_omp)
		{
#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = m_target_indexes.at(i);
				const trimesh::point& target_point = m_target->vertices.at(index);
				//if (target_point.x > xmin && target_point.x < xmax &&
				//	target_point.y > ymin && target_point.y < ymax &&
				//	target_point.z > zmin && target_point.z < zmax)
				{
					trimesh::point p = xf12 * target_point;

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((m_fx * p[0] + m_cx * p[2]) / p[2]);
					int v = (int)roundf((m_fy * p[1] + m_cy * p[2]) / p[2]);

					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(target_point, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}
	}

	void ProjectionICPMapping::SetToration(float* k2)
	{
		m_k1 = k2[0];
		m_k2 = k2[1];
		m_k3 = k2[2];
		m_k4 = k2[3];
		m_k5 = k2[4];
	}

	void ProjectionICPMapping::FixMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs)
	{
		pairs.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		float xmin, xmax, ymin, ymax, zmin, zmax;
		TransformBoundingbox(m_source_corner, source_form,
			xmin, xmax, ymin, ymax, zmin, zmax);

		const int sz = floor(m_source->vertices.size() / m_sample_count);
		int nv = (int)m_target->vertices.size() / sz;
		std::vector<PtPair> 	pairs1(nv);
		std::vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();
		//#pragma omp parallel for num_threads(num_omp)
		{
			//#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = i * sz;
				const trimesh::point& target_point = m_target->vertices.at(index);
				if (target_point.x > xmin && target_point.x < xmax &&
					target_point.y > ymin && target_point.y < ymax &&
					target_point.z > zmin && target_point.z < zmax)
				{
					trimesh::point p = xf12 * target_point;

					double xn = (double)(p.x/p.z);
					double yn = (double)(p.y/p.z);

					// add distortion
					double r2 = xn*xn + yn*yn;
					double r4 = r2*r2;
					double r6 = r2*r2*r2;

					//Radial distortion:
					double cdist = 1 + m_k1 * r2 + m_k2 * r4 + m_k5 * r6;

					//tangential distortion:
					double a1 = 2 * xn*yn;
					double a2 = r2 + 2 * xn * xn;
					double a3 = r2 + 2 * yn * yn;

					double delta_x = m_k3 * a1 + m_k4 * a2;
					double delta_y = m_k3 * a3 + m_k4 * a1;

					double xd = xn * cdist + delta_x;
					double yd = yn * cdist + delta_y;

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((m_fx * xd + m_cx ));
					int v = (int)roundf((m_fy * yd + m_cy ));

					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(target_point, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}

		std::cout << (int)pairs.size() << std::endl;
	}

	void ProjectionICPMapping::FixMappingWithoutAddDistortion(const trimesh::xform& source_form, std::vector<PtPair> &pairs)
	{
		pairs.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		float xmin, xmax, ymin, ymax, zmin, zmax;
		TransformBoundingbox(m_source_corner, source_form,
			xmin, xmax, ymin, ymax, zmin, zmax);

		const int sz = floor(m_source->vertices.size() / m_sample_count);//wzk m_sample_count ???
		int nv = (int)m_target->vertices.size() / sz;
		std::vector<PtPair> 	pairs1(nv);
		std::vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();
		//#pragma omp parallel for num_threads(num_omp)
		{
			//#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = i * sz;
				const trimesh::point& target_point = m_target->vertices.at(index);
				if (target_point.x > xmin && target_point.x < xmax &&
					target_point.y > ymin && target_point.y < ymax &&
					target_point.z > zmin && target_point.z < zmax)
				{
					trimesh::point p = xf12 * target_point;
					int u = (int)roundf((m_fx * p[0] + m_cx * p[2]) / p[2]);
					int v = (int)roundf((m_fy * p[1] + m_cy * p[2]) / p[2]);
					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(target_point, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}

		std::cout << (int)pairs.size() << std::endl;
	}
} // namespace test_trimesh
