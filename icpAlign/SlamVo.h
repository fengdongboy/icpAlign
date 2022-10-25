#pragma once
#include "../interface/slam_interface.h"
#include <base\threading\thread.h>
#include "InnerInterface.h"
#include "SlamParameters.h"
#include "VoImpl.h"
#include "VOLocator.h"
#include "VOFusion.h"
#include "chunkvoimpl.h"
#include "nooctreevoimpl.h"
#include "PointMatchImpl.h"
#include "siftVoImpl.h"

#include <opencv2/opencv.hpp>

static uint64_t _PERFORMACNE_PROFILING_TIME__;
#define PROFILING_BEGIN _PERFORMACNE_PROFILING_TIME__ = cv::getTickCount();
#define PROFILING_END(x) std::cout << (#x) << ": " << double(cv::getTickCount() - _PERFORMACNE_PROFILING_TIME__) / cv::getTickFrequency()*1000.0 << "ms" << std::endl;

#define USE_CHUNK 1
namespace esslam
{
	class SlamVO : public base::Thread, public VO, public KeyFrameAdder
	{
	public:
		SlamVO();
		virtual ~SlamVO();

		void StartVO(const SlamParameters& parameters);
		void StopVO();

		void OnFrame(trimesh::TriMesh* mesh);
		void OnFrameWithIndex(trimesh::TriMesh* mesh, unsigned int index);
		void OnFrameWithImg(trimesh::TriMesh* mesh, unsigned char* img_buffer, unsigned int index);


		void OnFrameWithPXYZ(trimesh::TriMesh* mesh, trimesh::TriMesh* PXYZ);
		void SetVisualProcessor(VisualProcessor* processor);
		void SetVOProfiler(VOProfiler* profiler);
		void SetLocateTracer(LocateTracer* tracer);
		void SetICPTracer(trimesh::ProjectionICPTracer* tracer);
		void SetIO(DataIO* io);
		void setMap(const cv::Mat& map1, const cv::Mat& map2); //设置极线矫正参数
		void Clear();
		void Resume();

		void Build(IBuildTracer& tracer);
		void EditScanData(const std::vector<bool>& edit_result);
		void PreLastScanData(void);
		void DeleteLastScanData(void);
		void SetFixMode();

		void AddKeyFrame(TriMeshPtr mesh);

		bool GetXf(int index, trimesh::xform& xf);
		bool GetChuncks(std::vector<trimesh::TriMesh*>& chuncks, std::vector<trimesh::xform>& xfs);
		bool GetOneChunckData(int index, std::vector<trimesh::vec3>& position, std::vector<trimesh::vec3>& normals, trimesh::xform& xf);
	protected:
		void ProcessFrame(trimesh::TriMesh* mesh);
		void ProcessFrameWithIndex(trimesh::TriMesh* mesh,unsigned int index);
		void ProcessFrameWithPXYZ(trimesh::TriMesh* mesh, trimesh::TriMesh* PXYZ);

		//添加使用sift特征点粗匹配接口
		void ProcessFrameWithImg(trimesh::TriMesh* mesh, unsigned char* img_buffer, unsigned int index);

		void ProcessAddKeyFrame(TriMeshPtr mesh);
		void ProcessFixMode(TriMeshPtr mesh);
	private:
		SlamParameters m_parameters;
		VOImpl m_vo_impl;
		ChunkVOImpl m_chunk_impl;
		NooctreeVOImpl m_nooctree_vo_impl;
		PointMatchImpl m_pointmatch_impl;
		SiftVoImpl m_sift_vo_impl;

		VOLocator m_vo_locator;
		VOFusion m_vo_fusion;
		VOProfiler* m_profiler;

		VisualProcessor* m_visual_processor;

		bool m_fix_mode;
		std::vector<TriMeshPtr> m_fix_mesh;

		DataIO* m_io;

		base::Lock m_data_lock;
		int m_vo_type;


		/////////////////////////////////////sift特征点粗匹配相关//////////////////////////////////
		cv::Mat m_map1;
		cv::Mat m_map2;
	};
}