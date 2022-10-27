#include "Align.h"
#include<ppl.h>
#include<concurrent_vector.h>

Align::Align()
{
	
}

Align::~Align()
{

}

void Align::setParams(float fx, float fy, float cx, float cy)
{
	m_fx = fx;
	m_fy = fy;
	m_cx = cx;
	m_cy = cy;
	setup();
}

bool Align::ProcessOneFrame(TriMeshPtr& mesh)
{
	m_xforms.push_back(trimesh::xform());

	bool r = LocateOneFrame(mesh);
	if (r)
	FusionFrame(mesh);

	return r;
}

void Align::setup()
{
	//trimesh::CameraData camera_data;

	//readCalibParam(camera_data);

	//m_fx = camera_data.m_fx;
	//m_fy = camera_data.m_fy;
	//m_cx = camera_data.m_cx;
	//m_cy = camera_data.m_cy;

	std::cout << "fx fy cx cy " << m_fx << " " << m_fy << " " << m_cx << " " << m_cy << std::endl;
	m_icp.reset(new trimesh::ProjectionICP(m_fx, m_fy, m_cx, m_cy));

	m_icp->SetLeastOverlapRatio(m_least_overlap_ratio);
	m_icp->SetFull();



	m_cell_resolution = 0.0f;
	if (m_cell_depth == 2) m_cell_resolution = 1.6f;
	if (m_cell_depth == 3) m_cell_resolution = 0.8f;
	if (m_cell_depth == 4) m_cell_resolution = 0.4f;
	if (m_cell_depth == 5) m_cell_resolution = 0.2f;
	if (m_cell_depth == 6) m_cell_resolution = 0.1f;

	if (m_max_octree_count < 8000000) m_max_octree_count = 8000000;
	if (m_max_octree_count > 20000000) m_max_octree_count = 20000000;
	m_layers.resize(m_max_octree_count, -1);

	m_octree.reset(new esslam::Octree(m_cell_depth, m_cell_resolution, m_max_octree_count));
	m_octree->m_use_cube_center = m_use_cube_center;
	m_octree->m_use_persist_fusion = m_use_persist_fusion;

	m_xforms.clear();
	m_xforms.resize(m_max_frames);
}

bool Align::LocateOneFrame(TriMeshPtr& mesh)
{
	if (mFirstFrame)
	{
		if (mesh->vertices.size() >= m_first_frame_count)
		{
			mFirstFrame = false;
		}
		return true;
	}
	else
	{
		int failed_reason = 0;
		bool result = Frame2Frame(mesh);
		if (result)
		{
			//locate_data.locate_type = 0;
		}
		else
		{
			//locate_data.locate_type = 1;
			result = Relocate(mesh);
		}

		if (!result) failed_reason = 1;


		if (result)
		{
			float error = 0.0f;

			result = Frame2Model(mesh, error);
			if (!result) failed_reason = 2;
		}
		return result;
	}
}

void Align::FusionFrame(TriMeshPtr& mesh)
{
	m_xforms.at(mesh->frame) = mesh->global;

	if (!m_octree->m_initialized)
		m_octree->Initialize(mesh->bbox.center());

	std::vector<int> indexes(mesh->vertices.size(), -1);
	m_octree->Insert(mesh->vertices, mesh->normals, mesh->colors, mesh->global, indexes);

	int new_count = 0;
	size_t vsize = mesh->vertices.size();
	for (size_t i = 0; i < vsize; ++i)
	{
		int index = indexes.at(i);
		if (index >= 0 && (m_layers.at(index) > 0 || m_layers.at(index) == -1))
		{
			++new_count;
		}
	}

	bool use_as_keyframe = false;
	if (((float)new_count / (float)vsize > m_key_frame_ratio) &&
		(vsize >= m_keyframe_count) )
	{
		use_as_keyframe = true;
	}

	int flag = use_as_keyframe ? 0 : mesh->frame;
	for (size_t i = 0; i < vsize; ++i)
	{
		int index = indexes.at(i);
		if (index >= 0 && (m_layers.at(index) != 0))
			m_layers.at(index) = flag;
	}

	SetLastMesh(mesh, use_as_keyframe);
	//if (use_as_keyframe) locate_data.use_as_key = true;

	int new_num = m_octree->m_current_point_index - m_octree->m_last_point_index;
	if (use_as_keyframe) std::cout << "Use as keyframe. " << new_count << " " << new_num << std::endl;
	//std::cout << "Octree nodes " << m_octree->m_current_index <<
	//	" points " << m_octree->m_current_point_index << std::endl;

	/// 每次新加的点 由new_data收集发给渲染
	//if (m_visual_processor && new_num > 0)
	//{
	//	NewAppendData* new_data = new NewAppendData();
	//	new_data->chunk = -1;
	//	new_data->position.resize(new_num);
	//	new_data->normals.resize(new_num);
	//	new_data->colors.resize(new_num);
	//	for (int i = m_octree->m_last_point_index, j = 0; i < m_octree->m_current_point_index; ++i, ++j)
	//	{
	//		const trimesh::vec3& v = m_octree->m_trimesh.vertices.at(i);
	//		const trimesh::vec3& n = m_octree->m_trimesh.normals.at(i);
	//		const trimesh::Color& c = m_octree->m_trimesh.colors.at(i);
	//		new_data->position.at(j) = v;
	//		new_data->normals.at(j) = n;
	//		new_data->colors.at(j) = c;
	//	}
	//	m_visual_processor->OnAppendNewPoints(new_data);
	//}
}

bool Align::Frame2Frame(TriMeshPtr& mesh)
{
	if (m_last_mesh == nullptr) return false;

	trimesh::xform xf;
	m_icp->SetSource(mesh.get());
	m_icp->SetTarget(m_last_mesh.get());
	float err_p = 0.0f;
	if (m_use_fast_icp) err_p = m_icp->FastDo(xf);
	else err_p = m_icp->Do(xf);

	if ((m_use_cube_center == 1) && (err_p > m_ff_icp_rms || err_p < 0.0f))
	{
		trimesh::vec3 sc = mesh->bbox.center();
		trimesh::vec3 tc = m_last_mesh->bbox.center();

		xf = trimesh::xform::trans(tc - sc);
		if (m_use_fast_icp) err_p = m_icp->FastDo(xf);
		else err_p = m_icp->Do(xf);
		printf("err_p = %f\n", err_p);
	}

	//if (m_locate_tracer) m_locate_tracer->OnAfterF2F();
	if (err_p > m_ff_icp_rms || err_p < 0.0f)
		return false;

	mesh->global = m_last_mesh->global * xf;

	return true;
}

bool Align::Frame2Model(TriMeshPtr& mesh, float& e)
{
	trimesh::xform xf = mesh->global;
	m_icp->SetSource(mesh.get());
	m_icp->SetTarget(&m_octree->m_trimesh);

	//if (m_locate_tracer) m_locate_tracer->OnBeforeF2M();

	float error = m_icp->FMQuickDo(xf, 7);

	//if (m_locate_tracer) m_locate_tracer->OnAfterF2M();

	e = error;
	if (error > m_fm_icp_rms || error < 0.0f)
		return false;

	mesh->global = xf;
	return true;
}

bool Align::Relocate(TriMeshPtr& mesh)
{
	TriMeshPtr dest_mesh;
	trimesh::xform xf;
	size_t size = m_key_frames.size();

	if (size > 0)
	{
		//if (m_locate_tracer) m_locate_tracer->OnBeforeRelocate();

		std::vector<float> errors(size, -1.0f);
		std::vector<trimesh::xform> matrixes(size);
		Concurrency::parallel_for<size_t>(0, size, [this, &mesh, &matrixes, &errors](size_t i) {
			errors.at(i) = RelocateOnce(mesh.get(), m_key_frames.at(i).get(), matrixes.at(i));
			});

		float min_error = FLT_MAX;
		int index = -1;
		for (size_t i = 0; i < size; ++i)
		{
			float e = errors.at(i);
			if (e <= m_ff_icp_rms && e >= 0.0f && e < min_error)
			{
				min_error = e;
				index = (int)i;
			}
		}

		if (index >= 0 && index < (int)size)
		{
			dest_mesh = m_key_frames.at(index);
			xf = matrixes.at(index);
		}
	}

	if (dest_mesh)
	{
		mesh->global = dest_mesh->global * xf;

		return true;
	}
	else
	{
		return false;
	}
}

float Align::RelocateOnce(trimesh::TriMesh* source, trimesh::TriMesh* target, trimesh::xform& xf)
{
	trimesh::ProjectionICP icp(m_fx, m_fy, m_cx, m_cy);
	//if (m_icp_parameters.fullsample == 1) icp.SetFull();

	icp.SetSource(source);
	icp.SetTarget(target);
	icp.SetLeastOverlapRatio(m_least_overlap_ratio);

	trimesh::xform init_xf;
	float error = -1.0f;
	if (m_use_fast_icp) error = icp.FastDo(init_xf);
	else error = icp.Do(init_xf);

	if (error > m_fm_icp_rms || error < 0.0f)
	{
		trimesh::vec3 center = source->bbox.center();
		for (int i = 1; i < 4; ++i)
		{
			trimesh::xform ixf;
			double rad = (double)i * 90.0f * M_PIf / 180.0f;
			if (i % 2 == 0)
				ixf = trimesh::xform::trans(0.0f, -60.0f, 0.0f) * trimesh::xform::trans(center) * trimesh::xform::rot(rad, 0.0, 0.0, 1.0) * trimesh::xform::trans(-center);
			else
				ixf = trimesh::xform::trans(-60.0f, 0.0f, 0.0f) * trimesh::xform::trans(center) * trimesh::xform::rot(rad, 0.0, 0.0, 1.0) * trimesh::xform::trans(-center);

			init_xf = ixf;
			if (m_use_fast_icp) error = icp.FastDo(init_xf);
			else error = icp.Do(init_xf);
		}
	}

	//if (m_locate_tracer && m_locate_tracer->NeedSave())
	//{
	//	trimesh::vec3 center = source->bbox.center();
	//	for (int i = 1; i < 4; ++i)
	//	{
	//		trimesh::xform ixf;
	//		double rad = (double)i * 90.0f * M_PIf / 180.0f;
	//		if (i % 2 == 0)
	//			ixf = trimesh::xform::trans(0.0f, -60.0f, 0.0f) * trimesh::xform::trans(center) * trimesh::xform::rot(rad, 0.0, 0.0, 1.0) * trimesh::xform::trans(-center);
	//		else
	//			ixf = trimesh::xform::trans(-60.0f, 0.0f, 0.0f) * trimesh::xform::trans(center) * trimesh::xform::rot(rad, 0.0, 0.0, 1.0) * trimesh::xform::trans(-center);

	//		//m_locate_tracer->OnRelocateFailed(source, target, ixf, i, m_fx, m_fy, m_cx, m_cy);
	//	}
	//}
	xf = init_xf;
	return error;
}

void Align::SetLastMesh(TriMeshPtr& mesh, bool use_as_keyframe)
{
	m_last_mesh = mesh;

	if (use_as_keyframe) m_key_frames.push_back(mesh);
}


void Align::Clear()
{
	m_key_frames.clear();
	m_last_mesh = NULL;
	if (m_octree) m_octree->Clear();
	m_layers.clear();
}

const trimesh::TriMesh& Align::getFushMesh(void) const
{
	return m_octree->m_trimesh;
}

trimesh::TriMesh& Align::getFushMesh(void)
{
	return m_octree->m_trimesh;
}
