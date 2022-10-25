#include "SlamVo.h"
#include <base\bind.h>
#include "timestamp.h"
#include "Profiler.h"
#include <opencv2/opencv.hpp>

namespace esslam
{

	SlamVO::SlamVO()
		:base::Thread("SlamVO"), m_profiler(NULL)
		, m_visual_processor(NULL), m_fix_mode(false)
		, m_io(0)
	{

	}

	SlamVO::~SlamVO()
	{

	}

	void SlamVO::StartVO(const SlamParameters& parameters)
	{
		m_parameters = parameters;
		m_vo_type = m_parameters.debug_param.vo_type;

		if (m_vo_type == 0)
			m_vo_impl.Setup(m_parameters);
		else if (m_vo_type == 1)
			m_chunk_impl.Setup(m_parameters);
		else if (m_vo_type == 2 || m_vo_type == 3)
			m_nooctree_vo_impl.Setup(m_parameters);
		if (m_vo_type == 4)
		{
			m_sift_vo_impl.Setup(m_parameters);
		}
		//else if (m_vo_type == 3)
		//	m_pointmatch_impl.Setup(m_parameters);

		m_vo_locator.Setup(m_parameters);
		m_vo_fusion.Setup(m_parameters);  
		m_vo_fusion.StartFusion();
		m_vo_fusion.SetKeyFrameAdder(this);

		bool start = Start();
	}

	void SlamVO::StopVO()
	{
		Stop();
		m_vo_fusion.StopFusion();
	}

	void SlamVO::SetVOProfiler(VOProfiler* profiler)
	{
		m_profiler = profiler;
	}

	void SlamVO::OnFrame(trimesh::TriMesh* mesh)
	{
		std::cout << "VO....." << std::endl;

		if (mesh)
			task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessFrame, base::Unretained(this), mesh));
	}

	//目前使用这个接口(原本使用OnFrame接口)
	void SlamVO::OnFrameWithIndex(trimesh::TriMesh* mesh, unsigned int index)
	{
		if (mesh)
			task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessFrameWithIndex, base::Unretained(this), mesh, index));
	}

	void SlamVO::OnFrameWithPXYZ(trimesh::TriMesh* mesh, trimesh::TriMesh* PXYZ)
	{
		std::cout << "VO....." << std::endl;
		if (mesh && PXYZ)
			task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessFrameWithPXYZ, base::Unretained(this), mesh,PXYZ));
	}

	//处理匹配帧的接口
	void SlamVO::ProcessFrame(trimesh::TriMesh* mesh)
	{
		
		LC_TICK

		if (mesh->frame == 55)
		{
			//std::cout << "error" << std::endl;
		}
		if (m_profiler)
		{
			m_profiler->OnMesh(*mesh);
			m_profiler->OnBeforeLocate();
		}

		TriMeshPtr mesh_ptr(mesh);//|ì?à?????
		if (m_io) m_io->SaveCloud(mesh_ptr, mesh_ptr->frame);
		
		LocateData locate_data;
		locate_data.lost = true;
		locate_data.locate_type = 0;
		locate_data.use_as_key = false;
		locate_data.rms = 9999.0f;

		if (m_fix_mode)
		{
			ProcessFixMode(mesh_ptr);
		}
		else
		{
			if (m_parameters.icp_param.only_show_frame == 1)
			{
				LC_TICK
				if (m_visual_processor)
				{
					CurrentFrameData* data = new CurrentFrameData();
					data->fix = false;
					data->lost = false;
					data->color = false;
					data->mesh = mesh_ptr;
					m_visual_processor->OnCurrentFrame(data);
				}
				LC_TICK
				//std::cout << "Only Show Frame." << std::endl;
			}
			else if(m_parameters.icp_param.only_show_frame == 2)
			{
				LC_TICK
				if (m_visual_processor)
				{
					CurrentFrameData* data = new CurrentFrameData();
					data->fix = false;
					data->lost = false;
					data->color = true;
					data->mesh = mesh_ptr;
					m_visual_processor->OnCurrentFrame(data);
				}
				LC_TICK
				//std::cout << "Only Show Color Frame." << std::endl;
			}
			else if (m_parameters.reader_param.use_t_vo)
			{
					//std::cout << "Locate " << std::endl;
				//使用视觉里程计（匹配和重定位）
					m_vo_locator.Locate(mesh_ptr, locate_data);
					
				if (m_visual_processor)
				{
					CurrentFrameData* data = new CurrentFrameData();
					data->lost = locate_data.lost;
					data->fix = false;
					data->color = false;
					data->mesh = mesh_ptr;
					m_visual_processor->OnCurrentFrame(data);
				}

					if (!locate_data.lost)
						m_vo_fusion.Fusion(mesh_ptr, locate_data.locate_type == 1);
			}
			else
			{
				
				m_data_lock.Acquire();
				if (m_vo_type == 0)
					m_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
				else if (m_vo_type == 1)
					m_chunk_impl.ProcessOneFrame(mesh_ptr, locate_data);//wzk1
				else if (m_vo_type == 2 || m_vo_type == 3)//wzk
					m_nooctree_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
				//else if (m_vo_type == 3)
				//	m_pointmatch_impl.ProcessOneFrame(mesh_ptr, locate_data);
				m_data_lock.Release();

				if (!locate_data.lost && locate_data.use_as_key)
				{

				}
			}
		}

		if (m_profiler)
		{
			m_profiler->OnLocateResult(locate_data);
			m_profiler->OnAfterLocate();
			//if(!locate_data.lost) m_profiler->OnResult(mesh_ptr->frame, mesh_ptr->global);
			if (!locate_data.lost)
			{
				m_io->SaveXF(mesh_ptr->global, mesh_ptr->frame);
				if (locate_data.use_as_key)
					m_io->SaveKeyFrame(mesh_ptr->frame);
			}
		}

		LC_TICK
	}

	void SlamVO::ProcessFrameWithIndex(trimesh::TriMesh* mesh, unsigned int index)
	{
		LC_TICK

			if (mesh->frame == 55)
			{
				//std::cout << "error" << std::endl;
			}
		if (m_profiler)
		{
			m_profiler->OnMesh(*mesh); //记录mesh的点数
			m_profiler->OnBeforeLocate(); //记录此刻的时间
		}

		TriMeshPtr mesh_ptr(mesh);
		if (m_io) m_io->SaveCloud(mesh_ptr, mesh_ptr->frame);  //可通过配置文件配置是否需要保存点云

		LocateData locate_data;
		locate_data.lost = true;
		locate_data.locate_type = 0;
		locate_data.use_as_key = false;
		locate_data.rms = 9999.0f;
		locate_data.data_index = index;

		//固定扫描
		if (m_fix_mode)
		{
			ProcessFixMode(mesh_ptr);
		}
		else //手持扫描
		{
			if (m_parameters.icp_param.only_show_frame == 1)
			{
				LC_TICK
					if (m_visual_processor)
					{
						CurrentFrameData* data = new CurrentFrameData();
						data->fix = false; //是否为固定扫描，这里为手持模式，所以设定为false
						data->lost = false;
						data->color = true;
						data->mesh = mesh_ptr;
						m_visual_processor->OnCurrentFrame(data);
					}
				LC_TICK
					//std::cout << "Only Show Frame." << std::endl;
			}
			else if (m_parameters.icp_param.only_show_frame == 2)
			{
				LC_TICK
					if (m_visual_processor)
					{
						CurrentFrameData* data = new CurrentFrameData();
						data->fix = false;
						data->lost = false;
						data->color = true;
						data->mesh = mesh_ptr;
						m_visual_processor->OnCurrentFrame(data);
					}
				LC_TICK
					//std::cout << "Only Show Color Frame." << std::endl;
			}
			else if (m_parameters.reader_param.use_t_vo)
			{
				//std::cout << "Locate " << std::endl;
				m_vo_locator.Locate(mesh_ptr, locate_data);

				if (m_visual_processor)
				{
					CurrentFrameData* data = new CurrentFrameData();
					data->lost = locate_data.lost;
					data->fix = false;
					data->color = false;
					data->mesh = mesh_ptr;
					m_visual_processor->OnCurrentFrame(data);
				}

				if (!locate_data.lost)
					m_vo_fusion.Fusion(mesh_ptr, locate_data.locate_type == 1);
			}
			else
			{
				m_data_lock.Acquire();
				if (m_vo_type == 0)
					m_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
				else if (m_vo_type == 1)
					m_chunk_impl.ProcessOneFrame(mesh_ptr, locate_data);
				else if (m_vo_type == 2 || m_vo_type == 3)
				{
					PROFILING_BEGIN;
					m_nooctree_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
					PROFILING_END(time:);
				}
					
				//else if (m_vo_type == 3)
				//	m_pointmatch_impl.ProcessOneFrame(mesh_ptr, locate_data);
				m_data_lock.Release();

			}
		}

		if (m_profiler)
		{
			m_profiler->OnLocateResult(locate_data);
			m_profiler->OnAfterLocate();
			//if(!locate_data.lost) m_profiler->OnResult(mesh_ptr->frame, mesh_ptr->global);
			if (!locate_data.lost)
			{
				m_io->SaveXF(mesh_ptr->global, mesh_ptr->frame);
				if (locate_data.use_as_key)
					m_io->SaveKeyFrame(mesh_ptr->frame);
			}
		}

		LC_TICK
	}

	void SlamVO::ProcessFrameWithPXYZ(trimesh::TriMesh* mesh, trimesh::TriMesh* PXYZ)
	{
		LC_TICK

			if (mesh->frame == 55)
			{
				//std::cout << "error" << std::endl;
			}
		if (m_profiler)
		{
			m_profiler->OnMesh(*mesh);
			m_profiler->OnBeforeLocate();
		}

		TriMeshPtr mesh_ptr(mesh);
		if (m_io) m_io->SaveCloud(mesh_ptr, mesh_ptr->frame);

		TriMeshPtr PXYZ_ptr(PXYZ);

		LocateData locate_data;
		locate_data.lost = true;
		locate_data.locate_type = 0;
		locate_data.use_as_key = false;
		locate_data.rms = 9999.0f;

		if (m_fix_mode)
		{
			ProcessFixMode(mesh_ptr);
		}
		else
		{
			if (m_parameters.icp_param.only_show_frame == 1)
			{
				LC_TICK
					if (m_visual_processor)
					{
						CurrentFrameData* data = new CurrentFrameData();
						data->fix = false;
						data->lost = false;
						data->color = true;
						data->mesh = mesh_ptr;
						m_visual_processor->OnCurrentFrame(data);
					}
				LC_TICK
					//std::cout << "Only Show Frame." << std::endl;
			}
			else if (m_parameters.icp_param.only_show_frame == 2)
			{
				LC_TICK
					if (m_visual_processor)
					{
						CurrentFrameData* data = new CurrentFrameData();
						data->fix = false;
						data->lost = false;
						data->color = true;
						data->mesh = mesh_ptr;
						m_visual_processor->OnCurrentFrame(data);
					}
				LC_TICK
					//std::cout << "Only Show Color Frame." << std::endl;
			}
			else if (m_parameters.reader_param.use_t_vo)
			{
				//std::cout << "Locate " << std::endl;
				m_vo_locator.Locate(mesh_ptr, locate_data);

				if (m_visual_processor)
				{
					CurrentFrameData* data = new CurrentFrameData();
					data->lost = locate_data.lost;
					data->fix = false;
					data->color = false;
					data->mesh = mesh_ptr;
					m_visual_processor->OnCurrentFrame(data);
				}

				if (!locate_data.lost)
					m_vo_fusion.Fusion(mesh_ptr, locate_data.locate_type == 1);
			}
			else
			{
				m_data_lock.Acquire();
				if (m_vo_type == 0)
					m_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
				else if (m_vo_type == 1)
					m_chunk_impl.ProcessOneFrame(mesh_ptr, locate_data);//wzk1
				else if (m_vo_type == 2 || m_vo_type == 3)
					m_nooctree_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
				//else if (m_vo_type == 3)
				//	m_pointmatch_impl.ProcessOneFrame(mesh_ptr, locate_data);
				m_data_lock.Release();

				if (!locate_data.lost && locate_data.use_as_key)
				{
					m_io->SavePXYZ(PXYZ_ptr, mesh_ptr->frame);
				}
			}
		}

		if (m_profiler)
		{
			m_profiler->OnLocateResult(locate_data);
			m_profiler->OnAfterLocate();
			//if(!locate_data.lost) m_profiler->OnResult(mesh_ptr->frame, mesh_ptr->global);
			if (!locate_data.lost)
			{
				m_io->SaveXF(mesh_ptr->global, mesh_ptr->frame);
				if (locate_data.use_as_key)
					m_io->SaveKeyFrame(mesh_ptr->frame);
			}
		}

		LC_TICK
	}


	void SlamVO::SetVisualProcessor(VisualProcessor* processor)
	{
		m_visual_processor = processor;
		m_vo_impl.SetVisualProcessor(processor);
		m_chunk_impl.SetVisualProcessor(processor);
		m_nooctree_vo_impl.SetVisualProcessor(processor);
		m_sift_vo_impl.SetVisualProcessor(processor);
		m_pointmatch_impl.SetVisualProcessor(processor);
		m_vo_fusion.SetVisualProcessor(processor);
	}
	
	void SlamVO::SetLocateTracer(LocateTracer* tracer)
	{
		m_vo_impl.SetLocateTracer(tracer);
	}
	
	void SlamVO::SetICPTracer(trimesh::ProjectionICPTracer* tracer)
	{
		m_vo_impl.SetProjectionICPTracer(tracer);
	}

	void SlamVO::SetIO(DataIO* io)
	{
		m_io = io;
	}

	void SlamVO::Resume()
	{
		if (m_vo_type == 0)
			m_vo_impl.Resume();
		if (m_vo_type == 2 || m_vo_type==3)
			m_nooctree_vo_impl.Resume();
		//if (m_vo_type == 3)
		//	m_pointmatch_impl.clear();
	}

	void SlamVO::Clear()
	{
		if (m_parameters.reader_param.use_t_vo)
		{
			m_vo_locator.Clear();
			m_vo_fusion.Clear();
		}
		else
		{
			if (m_vo_type == 0)
				m_vo_impl.Clear();
			else if (m_vo_type == 2 || m_vo_type == 3)
				m_nooctree_vo_impl.Clear();
			//else if (m_vo_type == 3)
			//	m_pointmatch_impl.clear();
		}

		m_fix_mesh.clear();
	}

	void SlamVO::Build(IBuildTracer& tracer)
	{
		if (m_fix_mode)
		{
			size_t total_size = 0;
			std::vector<trimesh::vec3> position;
			std::vector<trimesh::vec3> normals;
			std::vector<trimesh::Color> colors;
			for (size_t i = 0; i < m_fix_mesh.size(); ++i)
				total_size += m_fix_mesh.at(i)->vertices.size();
			
			if (total_size > 0)
			{
				position.resize(total_size);
				normals.resize(total_size);
				colors.resize(total_size);
				size_t curr = 0;
				for (size_t i = 0; i < m_fix_mesh.size(); ++i)
				{
					TriMeshPtr mesh = m_fix_mesh.at(i);
					size_t size = mesh->vertices.size();
					memcpy(&position.at(curr), &mesh->vertices.at(0), 3 * size * sizeof(float));
					memcpy(&normals.at(curr), mesh->normals.at(0), 3 * size * sizeof(float));
					//memcpy(&normals.at(curr), mesh->normals.at(0), 3 * size * sizeof(float));
					curr += size;
				}
			}
			if (total_size > 0)
				;//tracer.OnPoints((int)total_size, (float*)&position.at(0), (float*)&normals.at(0), (unsigned char*)&colors.at(0));
			else
				tracer.OnPoints(0, 0, 0, 0);
		}
		else
		{
			if (m_parameters.reader_param.use_t_vo)
			{
				std::map<int, int> nooverlap;
				m_vo_fusion.Build(tracer, nooverlap);
				m_vo_locator.Build(tracer, nooverlap);
			}
			else
			{
				m_data_lock.Acquire();
				if (m_vo_type == 0)
					m_vo_impl.Build(tracer);
				else if (m_vo_type == 2 || m_vo_type == 3)
					m_nooctree_vo_impl.Build(tracer);
				//else if (m_vo_type == 3)
				//	m_pointmatch_impl.Build(tracer);
				m_data_lock.Release();
			}
		}
	}

	void SlamVO::SetFixMode()
	{
		m_vo_locator.SetFixMode();
		m_vo_fusion.SetFixMode();
		m_vo_impl.SetFixMode();
		m_fix_mode = true;
	}

	void SlamVO::ProcessAddKeyFrame(TriMeshPtr mesh)
	{
		m_vo_locator.AddKeyFrame(mesh);
	}

	void SlamVO::AddKeyFrame(TriMeshPtr mesh)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessAddKeyFrame, base::Unretained(this), mesh));
	}

	void SlamVO::ProcessFixMode(TriMeshPtr mesh)
	{
		m_fix_mesh.push_back(mesh);

#if 0
		static int i = 0;
		char name[32];
		sprintf(name, "%d.ply", i);
		mesh->write(name);
		++i;
#endif 
		if (m_visual_processor)
		{
			CurrentFrameData* data = new CurrentFrameData();
			data->lost = false;
			data->fix = true;
			data->color = false;
			data->mesh = mesh;
			m_visual_processor->OnCurrentFrame(data);
		}
	}

	bool SlamVO::GetXf(int index, trimesh::xform& xf)
	{
		if (m_parameters.reader_param.use_t_vo)
			return m_vo_fusion.GetXf(index, xf);
		else
		{
			if (m_vo_type == 0)
				return m_vo_impl.GetXf(index, xf);
			else if (m_vo_type == 2 || m_vo_type == 3)
				return m_nooctree_vo_impl.GetXf(index, xf);
			//else if (m_vo_type == 3)
			//	return m_pointmatch_impl.GetXf(index, xf);
			return false;
		}
	}

	bool SlamVO::GetChuncks(std::vector<trimesh::TriMesh*>& chuncks, std::vector<trimesh::xform>& xfs)
	{
		if (m_vo_type==2 ||m_vo_type==3)
			return m_nooctree_vo_impl.GetChuncks(chuncks,xfs);
		//else if (m_vo_type==3)
		//	return m_pointmatch_impl.GetChuncks(chuncks,xfs);
		
		else
			std::cout << "vo_type!=2, no implete yet!" << std::endl;

		return false;
	}

	bool SlamVO::GetOneChunckData(int index, std::vector<trimesh::vec3>& position, std::vector<trimesh::vec3>& normals, trimesh::xform& xf)
	{
		if ( m_vo_type == 2 ||m_vo_type == 3)
			return m_nooctree_vo_impl.GetOneChunckData(index, position, normals, xf);
		//else if (m_vo_type == 3)
		//	return m_pointmatch_impl.GetOneChunckData(index, position, normals, xf);		
		else
			std::cout << "vo_type!=2, no implete yet!" << std::endl;

		return false;
	}

	void SlamVO::EditScanData(const std::vector<bool>& edit_result)
	{
		if (m_vo_type == 2 ||m_vo_type == 3)
			m_nooctree_vo_impl.EditScanData(edit_result);
		//else if (m_vo_type == 3)
		//{
		//	m_pointmatch_impl.EditScanData(edit_result);
		//}
	}

	void SlamVO::PreLastScanData(void)
	{
		if (m_vo_type == 2 || m_vo_type == 3)
			m_nooctree_vo_impl.PreLastScanData();
		//else if (m_vo_type == 3)		
		//	m_pointmatch_impl.PreLastScanData();
		
	}

	void SlamVO::DeleteLastScanData(void)
	{
		if (m_vo_type == 2 ||m_vo_type == 3)
			m_nooctree_vo_impl.DeleteLastScanData();
		//else if(m_vo_type == 3)
		//	m_pointmatch_impl.DeleteLastScanData();
	}

	void SlamVO::ProcessFrameWithImg(trimesh::TriMesh* mesh, unsigned char* img_buffer, unsigned int index)
	{
		LC_TICK

		if (m_profiler)
		{
			m_profiler->OnMesh(*mesh); //记录mesh的点数
			m_profiler->OnBeforeLocate(); //记录此刻的时间
		}

		TriMeshPtr mesh_ptr(mesh);
		if (m_io) m_io->SaveCloud(mesh_ptr, mesh_ptr->frame);  //可通过配置文件配置是否需要保存点云

		LocateData locate_data;
		locate_data.lost = true;
		locate_data.locate_type = 0;
		locate_data.use_as_key = false;
		locate_data.rms = 9999.0f;
		locate_data.data_index = index;

		//固定扫描
		if (m_fix_mode)
		{
			ProcessFixMode(mesh_ptr);
		}
		else //手持扫描
		{
			if (m_parameters.icp_param.only_show_frame == 1)
			{
				LC_TICK
					if (m_visual_processor)
					{
						CurrentFrameData* data = new CurrentFrameData();
						data->fix = false; //是否为固定扫描，这里为手持模式，所以设定为false
						data->lost = false;
						data->color = true;
						data->mesh = mesh_ptr;
						m_visual_processor->OnCurrentFrame(data);
					}
				LC_TICK
					//std::cout << "Only Show Frame." << std::endl;
			}
			else if (m_parameters.icp_param.only_show_frame == 2)
			{
				LC_TICK
					if (m_visual_processor)
					{
						CurrentFrameData* data = new CurrentFrameData();
						data->fix = false;
						data->lost = false;
						data->color = true;
						data->mesh = mesh_ptr;
						m_visual_processor->OnCurrentFrame(data);
					}
				LC_TICK
					//std::cout << "Only Show Color Frame." << std::endl;
			}
			else if (m_parameters.reader_param.use_t_vo)
			{
				//std::cout << "Locate " << std::endl;
				m_vo_locator.Locate(mesh_ptr, locate_data);

				if (m_visual_processor)
				{
					CurrentFrameData* data = new CurrentFrameData();
					data->lost = locate_data.lost;
					data->fix = false;
					data->color = false;
					data->mesh = mesh_ptr;
					m_visual_processor->OnCurrentFrame(data);
				}

				if (!locate_data.lost)
					m_vo_fusion.Fusion(mesh_ptr, locate_data.locate_type == 1);
			}
			else
			{
				m_data_lock.Acquire();
				if (m_vo_type == 0)
					m_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
				else if (m_vo_type == 1)
					m_chunk_impl.ProcessOneFrame(mesh_ptr, locate_data);
				else if (m_vo_type == 2 || m_vo_type == 3)
					m_nooctree_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
				else if (m_vo_type == 4)
				{
					PROFILING_BEGIN;
					//////////////////////////////对图像做极线矫正////////////////////////////////////
					cv::Mat l_img(640, 1280, CV_8UC1, (void*)img_buffer);
					cv::Mat l_result;
					cv::remap(l_img, l_result, m_map1, m_map2, cv::INTER_CUBIC);
					m_sift_vo_impl.ProcessOneFrame(mesh_ptr, l_result ,locate_data);
					PROFILING_END(time sift : );
				}
				//else if (m_vo_type == 3)
				//	m_pointmatch_impl.ProcessOneFrame(mesh_ptr, locate_data);
				m_data_lock.Release();

			}
		}

		if (m_profiler)
		{
			m_profiler->OnLocateResult(locate_data);
			m_profiler->OnAfterLocate();
			//if(!locate_data.lost) m_profiler->OnResult(mesh_ptr->frame, mesh_ptr->global);
			if (!locate_data.lost)
			{
				m_io->SaveXF(mesh_ptr->global, mesh_ptr->frame);
				if (locate_data.use_as_key)
					m_io->SaveKeyFrame(mesh_ptr->frame);
			}
		}

		LC_TICK
	}

	void SlamVO::setMap(const cv::Mat& map1, const cv::Mat& map2)
	{
		m_map1 = map1.clone();
		m_map2 = map2.clone();
	}

	void SlamVO::OnFrameWithImg(trimesh::TriMesh* mesh, unsigned char* img_buffer, unsigned int index)
	{
		if (mesh)
			task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessFrameWithImg, base::Unretained(this), mesh, img_buffer ,index));
	}

}