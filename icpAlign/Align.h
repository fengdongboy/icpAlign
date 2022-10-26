#pragma once
#ifndef __ALIGN_H__
#define __ALIGN_H__

/*!
 * \class Align
 *
 * \brief 
 *
 * \author admin
 * \date 10 25 2022
 */

#include "TriMesh.h"
#include "projectionicp.h"
#include "Octree.h"

typedef std::shared_ptr<trimesh::TriMesh> TriMeshPtr;
class Align
{
public:
	Align();
	~Align();

	void setParams(float fx, float fy, float cx, float cy);

	void ProcessOneFrame(TriMeshPtr& mesh);

	void setup();

	void LocateOneFrame(TriMeshPtr& mesh);

	void FusionFrame(TriMeshPtr& mesh);

	bool Frame2Frame(TriMeshPtr& mesh);

	bool Frame2Model(TriMeshPtr& mesh, float& e, bool relocate);

	bool Relocate(TriMeshPtr& mesh);

	float RelocateOnce(trimesh::TriMesh* source, trimesh::TriMesh* target, trimesh::xform& xf);

	void SetLastMesh(TriMeshPtr& mesh, bool use_as_keyframe);

	void SetProjectionICPTracer(trimesh::ProjectionICPTracer* tracer);

	void Clear();

	const trimesh::TriMesh& getFushMesh(void) const;
	trimesh::TriMesh& getFushMesh(void) ;

private:
	TriMeshPtr m_last_mesh;
	std::unique_ptr<trimesh::ProjectionICP> m_icp;

	std::vector<TriMeshPtr> m_key_frames;

	float m_fx;
	float m_fy;
	float m_cx;
	float m_cy;

	std::vector< trimesh::xform> m_xforms;

	//size_t m_first_frame_count = 3000;
	bool mFirstFrame = true;
	float m_least_overlap_ratio = 0.5;
	int m_cell_depth = 6;   /// 要限制在2到6间
	float m_cell_resolution = 0.0f;
	int m_max_octree_count = 8000000;
	int m_use_cube_center = 0;
	int m_max_frames = 3000;
	std::vector<int> m_layers;
	std::unique_ptr<esslam::Octree> m_octree;
	int m_use_persist_fusion = 0;
	float m_ff_icp_rms = 0.3f;
	float m_fm_icp_rms = 0.25f;
	float m_key_frame_ratio = 0.5f;
	float m_keyframe_rms = 0.1f;
	int m_first_frame_count = 500;
	int m_keyframe_count = 5000;
	bool m_use_fast_icp = true;
};

#endif