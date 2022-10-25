#ifndef TEST_ICP_H
#define TEST_ICP_H

#include "projectionicpheader.h"
#include<sstream>

namespace trimesh 
{
	class ProjectionICPMapping
	{
	public:
		ProjectionICPMapping(float fx, float fy, float cx, float cy);
		~ProjectionICPMapping();

		void SetSource(trimesh::TriMesh* source);
		void SetTarget(trimesh::TriMesh* target);
		bool Valid();
		void Setup();

		void SetToration(float* k2);

		void Mapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs);
		void BeforeFMMapping(const trimesh::xform& source_form);
		void FMMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs);
		void FixMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs);
		void FixMappingWithoutAddDistortion(const trimesh::xform& source_form, std::vector<PtPair> &pairs);
		void FullMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs);
		void FullFMMapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs);

		void SetTracer(ProjectionICPTracer* tracer);
		void SetSampleCount(int sample_count);
		void SetFull();

		int GetSourcePointSize();
		int GetTargetPointSize();

		bool SetSourcePointKeyData(int KeyIdx, int PIdx);
		bool SetTargetPointKeyData(int KeyIdx, int PIdx);
	private:
		const float m_fx;
		const float m_fy;
		const float m_cx;
		const float m_cy;

		trimesh::TriMesh* m_source;
		trimesh::TriMesh* m_target;

		float m_maxdist;
		std::vector<trimesh::point> m_source_corner;
		ProjectionICPTracer* m_tracer;

		std::vector<int> m_target_indexes;

		float m_k1;
		float m_k2;
		float m_k3;
		float m_k4;
		float m_k5;

		int m_sample_count;
		bool m_full;
	};
}

#endif
