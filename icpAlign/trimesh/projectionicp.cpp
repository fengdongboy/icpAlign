#include "projectionicp.h"

namespace trimesh
{
	ProjectionICP::ProjectionICP(float fx, float fy, float cx, float cy)
		:m_mapping(fx, fy, cx, cy), m_tracer(NULL), m_step(0), m_mapping_type(0), m_fix_type(false)
		, m_full_type(false)
	{
		m_initial_iters = 2;
		m_max_iters = 80;
		m_termination_iter_thresh = 20;
		m_final_iters = 2;

		m_dist_thresh_mult_start = 5.0f;
		m_normdot_thresh_start = 0.6f;

		m_dist_thresh_mult_final = 2.0f;
		m_normdot_thresh_final = 0.9f;

		m_thresh_rate_of_change = 0.002f;

		m_step_init_iter = 0;
		m_step_middle_iter = 0;
		m_step_final_iter = 0;

		m_last_error = -1.0f;
		m_cinteration = 0;
		m_least_overlap_ratio = 0.1f;
		m_sample_count = 1000;
		m_pairs_num = 0;
		m_source_num = 0;
		m_target_num = 0;
	}

	ProjectionICP::~ProjectionICP()
	{

	}

	void ProjectionICP::SetLeastOverlapRatio(float least_overlap_ratio)
	{
		m_least_overlap_ratio = least_overlap_ratio;
		//std::cout << "least_overlap_ratio:" << least_overlap_ratio << std::endl;
	}

	void ProjectionICP::SetSource(trimesh::TriMesh* source)
	{
		m_mapping.SetSource(source);
		m_source_num = source->vertices.size();
	}

	void ProjectionICP::SetTarget(trimesh::TriMesh* target)
	{
		m_mapping.SetTarget(target);
		m_target_num = target->vertices.size();
	}

	void ProjectionICP::SetMappingType(int mapping_type)
	{
		m_mapping_type = mapping_type;
		if (m_mapping_type != 0 && m_mapping_type != 1)
			m_mapping_type = 0;
	}

	void ProjectionICP::SetFixType(float* k2)
	{
		m_fix_type = true;
		m_mapping.SetToration(k2);
	}

	void ProjectionICP::SetSampleCount(int sample_count)
	{
		m_mapping.SetSampleCount(sample_count);
		m_sample_count = sample_count;
	}

	bool ProjectionICP::FastStep()
	{
		if (!m_mapping.Valid())
			return false;

		if (m_step == 0)
		{
			m_step_dist_thresh_mult = m_dist_thresh_mult_start;
			m_step_normdot_thresh = m_normdot_thresh_start;
			m_mapping.Setup();
			++m_step;
			return true;
		}

		if (m_step == 1)
		{
			float err = 0.0f;
			StepMiddleInteration(m_init_step_xf, err);

			if (err < 0) {
				return false;
			}

			if (abs(err - m_last_error) <= 0.001f)
			{
				++m_cinteration;
			}
			else
				m_cinteration = 0;

			m_last_error = err;

			if (m_cinteration >= 3)
			{
				++m_step;
				return true;
			}

			return true;
		}
		return false;
	}

	bool ProjectionICP::Step()
	{
		if (!m_mapping.Valid())
			return false;

		if (m_step == 0)
		{
			m_mapping.Setup();
			++m_step;
			return true;
		}

		if (m_step == 1)
		{
			++m_step_init_iter;
			if (m_step_init_iter >= m_initial_iters)
			{
				m_step_dist_thresh_mult = m_dist_thresh_mult_start;
				m_step_normdot_thresh = m_normdot_thresh_start;
				m_step_min_err = 0.0f;
				m_step_iter_of_min_err = -1;
				++m_step;
			}
			return StepInitialInteration(m_init_step_xf);
		}

		if (m_step == 2)
		{
			float err = 0.0f;
			StepMiddleInteration(m_init_step_xf, err);

			if (err < 0.0)
				return false;

			if (err < m_step_min_err || m_step_iter_of_min_err < 0)
			{
				m_step_min_err = err;
				m_step_iter_of_min_err = m_step_middle_iter;
			}

			if (m_step_middle_iter - m_step_iter_of_min_err >= m_termination_iter_thresh &&
				m_step_iter_of_min_err >= 0)
			{
				++m_step;
				return true;
			}

			if (err <= 0.02)
			{
				++m_step;
				return true;
			}
			

			++m_step_middle_iter;
			if (m_step_middle_iter >= m_max_iters)
				++m_step;

			return true;
		}

		if (m_step == 3)
		{
			++m_step_final_iter;
			if (m_step_final_iter >= m_final_iters)
				++m_step;
			return StepFinalInteration(m_init_step_xf);
		}

		return false;
	}

	bool ProjectionICP::StepFinalInteration(trimesh::xform &source_form)
	{
		m_estimation.SetDistThreshMult(m_dist_thresh_mult_final);
		m_estimation.SetNormDotThresh(m_normdot_thresh_final);
		float err = InnerDo(source_form);

		if (err < 0.0f)
			return false;
		return true;
	}

	void ProjectionICP::StepMiddleInteration(trimesh::xform& source_form, float& error)
	{
		m_estimation.SetIterType(ICP_POINT_TO_PLANE);

		m_step_dist_thresh_mult = trimesh::mix(m_step_dist_thresh_mult, m_dist_thresh_mult_final,
			m_thresh_rate_of_change);
		m_step_normdot_thresh = trimesh::mix(m_step_normdot_thresh, m_normdot_thresh_final,
			m_thresh_rate_of_change);

		// Do an iteration
		m_estimation.SetDistThreshMult(m_step_dist_thresh_mult);
		m_estimation.SetNormDotThresh(m_step_normdot_thresh);
		error = InnerDo(source_form);
	}

	bool ProjectionICP::StepInitialInteration(trimesh::xform &source_form)
	{
		m_estimation.SetIterType(ICP_POINT_TO_POINT);
		m_estimation.SetDistThreshMult(m_dist_thresh_mult_start);
		m_estimation.SetNormDotThresh(m_normdot_thresh_start);
		float err = InnerDo(source_form);

		if (err < 0.0f)
			return false;
		return true;
	}

	void ProjectionICP::ResetStep()
	{
		m_step = 0;
		m_step_init_iter = 0;
		m_step_middle_iter = 0;
		m_step_final_iter = 0;

		m_last_error = -1.0f;
		m_cinteration = 0;
	}

	void ProjectionICP::SetStepInitialMatrix(const trimesh::xform& xf)
	{
		m_init_step_xf = xf;
	}

	float ProjectionICP::FastDo(trimesh::xform & source_form)
	{
		if (!m_mapping.Valid())
			return -1.0f;

		m_mapping.Setup();

		return FastInteration_F2F(source_form);
	}

	void ProjectionICP::SetFull()
	{
		m_full_type = true;
		m_mapping.SetFull();
	}

	float ProjectionICP::FMQuickDo(trimesh::xform & source_form, int max_times)
	{
		if (!m_mapping.Valid())
			return -1.0f;

		SetMappingType(1);

		m_mapping.Setup();
		m_mapping.BeforeFMMapping(source_form);

		// First, do a few point-to-point iterations for stability in case the
		// initial misalignment is big.
		float err = InitialInteration(source_form);

		if(err >= 0.0f)
			err = FastInteration(source_form);

		if(err >= 0.0f)
			err = FinalInteration(source_form);

		SetMappingType(0);
		return err;
	}

	float ProjectionICP::DefaultTimesDo(trimesh::xform & source_form, int times)
	{
		if (!m_mapping.Valid())
			return -1.0f;

		m_mapping.Setup();

		float err = 0.0f;
		float dist_thresh_mult = m_dist_thresh_mult_start;
		float normdot_thresh = m_normdot_thresh_start;
		m_estimation.SetIterType(ICP_POINT_TO_PLANE);
		for (int iter = 0; iter < times; iter++) {
			// Update thresholds
			dist_thresh_mult = trimesh::mix(dist_thresh_mult, m_dist_thresh_mult_final,
				m_thresh_rate_of_change);
			normdot_thresh = trimesh::mix(normdot_thresh, m_normdot_thresh_final,
				m_thresh_rate_of_change);

			// Do an iteration
			m_estimation.SetDistThreshMult(dist_thresh_mult);
			m_estimation.SetNormDotThresh(normdot_thresh);
			err = InnerDo(source_form);

			if (err < 0) {
				break;
			}
		}

		return err;
	}

	float ProjectionICP::FastInteration_F2F(trimesh::xform &source_form)
	{
		int cinteration = 0;
		float err = 0.0f;
		float dist_thresh_mult = m_dist_thresh_mult_start;
		float normdot_thresh = m_normdot_thresh_start;
		// Now the real (point-to-plane) iterations
		float min_err = -1.0f;
		m_estimation.SetIterType(ICP_POINT_TO_PLANE);
		for (int iter = 0; iter < m_max_iters; iter++) {
			// Update thresholds
			dist_thresh_mult = trimesh::mix(dist_thresh_mult, m_dist_thresh_mult_final,
				m_thresh_rate_of_change);
			normdot_thresh = trimesh::mix(normdot_thresh, m_normdot_thresh_final,
				m_thresh_rate_of_change);

			// Do an iteration
			m_estimation.SetDistThreshMult(dist_thresh_mult);
			m_estimation.SetNormDotThresh(normdot_thresh);
			err = InnerDo(source_form);

			if (err < 0) {
				return err;
			}

			if (abs(err - min_err) <= 0.001f)
			{
				++cinteration;
			}
			else
				cinteration = 0;

			min_err = err;

			if (cinteration >= 3)
				break;
		}

		int sz = floor(m_source_num / m_sample_count);
		if (m_full_type)
			sz = 1;
		float overlap_ratio = float(m_pairs_num*sz) / float(m_source_num);
		float overlap_ratio_target = float(m_pairs_num*sz) / float(m_target_num);//add?by?chl20191029
		if (overlap_ratio < m_least_overlap_ratio && overlap_ratio_target < m_least_overlap_ratio)//add?by?chl20191029
		{
			std::cout << "overlap ratio too low:" << overlap_ratio << std::endl;
			err = -1.0f;
		}

		return err;
	}

	float ProjectionICP::FastInteration(trimesh::xform &source_form)
	{
		int cinteration = 0;
		float err = 0.0f;
		float dist_thresh_mult = m_dist_thresh_mult_start;
		float normdot_thresh = m_normdot_thresh_start;
		// Now the real (point-to-plane) iterations
		float min_err = -1.0f;
		m_estimation.SetIterType(ICP_POINT_TO_PLANE);
		for (int iter = 0; iter < m_max_iters; iter++) {
			// Update thresholds
			dist_thresh_mult = trimesh::mix(dist_thresh_mult, m_dist_thresh_mult_final,
				m_thresh_rate_of_change);
			normdot_thresh = trimesh::mix(normdot_thresh, m_normdot_thresh_final,
				m_thresh_rate_of_change);

			// Do an iteration
			m_estimation.SetDistThreshMult(dist_thresh_mult);
			m_estimation.SetNormDotThresh(normdot_thresh);
			err = InnerDo(source_form);

			if (err < 0) {
				return err;
			}

			if (abs(err - min_err) <= 0.001f) 
			{
				++cinteration;
			}
			else
				cinteration = 0;

			min_err = err;

			if (cinteration >= 3)
				break;
		}

		int sz = floor(m_source_num / m_sample_count);
		if (m_full_type)
			sz = 1;
		float overlap_ratio = float(m_pairs_num*sz) / float(m_source_num);
		if(overlap_ratio < m_least_overlap_ratio )
		{
			std::cout << "overlap ratio too low:" << overlap_ratio << std::endl;
			err = -1.0f;
		}

		return err;
	}

	float ProjectionICP::Do(trimesh::xform &source_form)
	{
		if (!m_mapping.Valid())
			return -1.0f;

		m_mapping.Setup();

		// First, do a few point-to-point iterations for stability in case the
		// initial misalignment is big.
		float err = InitialInteration(source_form);
		if (err < 0.0f) return err;

		err = MiddleInteration(source_form);
		if (err < 0.0f) return err;

		err = FinalInteration(source_form);

		return err;
	}

	float ProjectionICP::MiddleInteration(trimesh::xform& source_form)
	{
		float err = 0.0f;
		float dist_thresh_mult = m_dist_thresh_mult_start;
		float normdot_thresh = m_normdot_thresh_start;
		// Now the real (point-to-plane) iterations
		float min_err = 0.0f;
		int iter_of_min_err = -1;
		m_estimation.SetIterType(ICP_POINT_TO_PLANE);
		for (int iter = 0; iter < m_max_iters; iter++) {
			// Update thresholds
			dist_thresh_mult = trimesh::mix(dist_thresh_mult, m_dist_thresh_mult_final,
				m_thresh_rate_of_change);
			normdot_thresh = trimesh::mix(normdot_thresh, m_normdot_thresh_final,
				m_thresh_rate_of_change);

			// Do an iteration
			m_estimation.SetDistThreshMult(dist_thresh_mult);
			m_estimation.SetNormDotThresh(normdot_thresh);
			err = InnerDo(source_form);

			if (err < 0) {
				return err;
			}

			if (err < min_err || iter_of_min_err < 0) {
				min_err = err;
				iter_of_min_err = iter;
			}

			// Stop if we've gone at least TERMINATION_ITER_THRESH
			// iterations without seeing a new minimum error
			if (iter - iter_of_min_err >= m_termination_iter_thresh &&
				iter_of_min_err >= 0) {
				iter++; // Get #-of-iters printf correct
				break;
			}

			if (err <= 0.02)
				break;
		}
		return err;
	}

	float ProjectionICP::FinalInteration(trimesh::xform &source_form)
	{
		float err = 0.0f;
		m_estimation.SetDistThreshMult(m_dist_thresh_mult_final);
		m_estimation.SetNormDotThresh(m_normdot_thresh_final);
		for (int iter = 0; iter < m_final_iters; iter++)
		{
			err = InnerDo(source_form);
			if (err < 0.0f) {
				return err;
			}
		}

		int sz = floor(m_source_num / m_sample_count);
		if (m_full_type)
			sz = 1;
		float overlap_ratio = float(m_pairs_num*sz) / float(m_source_num);
		if (overlap_ratio < m_least_overlap_ratio)
		{
			std::cout << "overlap ratio too low:" << overlap_ratio << std::endl;
			err = -1.0f;
		}

		return err;
	}

	float ProjectionICP::InitialInteration(trimesh::xform &source_form)
	{
		float err = 0.0f;
		m_estimation.SetIterType(ICP_POINT_TO_POINT);
		m_estimation.SetDistThreshMult(m_dist_thresh_mult_start);
		m_estimation.SetNormDotThresh(m_normdot_thresh_start);
		for (int iter = 0; iter < m_initial_iters; iter++)
		{
			err = InnerDo(source_form);

			if (err < 0.0f) {
				return err;
			}
		}

		return err;
	}

	float ProjectionICP::InnerDo(trimesh::xform& source_form)
	{
		std::vector<PtPair> pairs;
		m_pairs_num = 0;
		//std::cout << "m_mapping_type: " << m_mapping_type << std::endl;
		if (m_fix_type)
		{
			//m_mapping.FixMapping(source_form, pairs);
			m_mapping.FixMappingWithoutAddDistortion(source_form, pairs);
		}
		else if (m_full_type)
		{
			if (m_mapping_type == 0)
				m_mapping.FullMapping(source_form, pairs);
			else if (m_mapping_type == 1)
				m_mapping.FullFMMapping(source_form, pairs);
		}
		else
		{
			if (m_mapping_type == 0)
				m_mapping.Mapping(source_form, pairs);
			else if (m_mapping_type == 1)
				m_mapping.FMMapping(source_form, pairs);
		}

		int npairs = (int)pairs.size();
		if (npairs == 0)
		{
			return -1.0f;
		}

		m_pairs_num = npairs;

		//wzk
		//if ((float)m_pairs_num / m_mapping.GetSourcePointSize() < 0.8)
		//if ((float)m_pairs_num / m_mapping.GetTargetPointSize() < 0.8 && m_pairs_num > 0)
		//{
		//	for (int iter = 0; iter < m_pairs_num; iter++)
		//	{
		//		int KeyIdx = 1;
		//		SetSourcePointKeyData(KeyIdx, int PIdx)
	
		//		SetTargetPointKeyData(KeyIdx, int PIdx)
		//	}
		//}

		if (m_tracer) m_tracer->OnPreStepCorrespondences(pairs);
		float err = m_estimation.Estimate(pairs, source_form);

		if (m_tracer)
		{
			m_tracer->OnError(err);
			m_tracer->OnStepCorrespondences(pairs);
			m_tracer->OnMatrix(source_form);
		}
		return err;
	}

	void ProjectionICP::SetTracer(ProjectionICPTracer* tracer)
	{
		m_tracer = tracer;
		m_mapping.SetTracer(m_tracer);
		m_estimation.SetTracer(m_tracer);
	}
}