
/// 通过opencv读取相机参数代码，需要opencv库支持

void readCalibParam(trimesh::CameraData& data)
{
	cv::Mat R, t, l_distort, r_distort, l_A, r_A;
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect rect1, rect2;



	readValMatBin<double>("F:/work/F1_Scanner/Fox Scan/calib/lr_r", R);
	readValMatBin<double>("F:/work/F1_Scanner/Fox Scan/calib/lr_t", t);
	readValMatBin<double>("F:/work/F1_Scanner/Fox Scan/calib/l_distort", l_distort);
	readValMatBin<double>("F:/work/F1_Scanner/Fox Scan/calib/r_distort", r_distort);
	readValMatBin<double>("F:/work/F1_Scanner/Fox Scan/calib/l_internal", l_A);
	readValMatBin<double>("F:/work/F1_Scanner/Fox Scan/calib/r_internal", r_A);

	cv::Size img_size(512, 640);
	stereoRectify(l_A, l_distort, r_A, r_distort, img_size, R, t, R1, R2, P1, P2, Q, 0, -1, cv::Size(), &rect1, &rect2);

	data.m_fx = P1.at<double>(0, 0);
	data.m_fy = P1.at<double>(1, 1);

	data.m_cx = P1.at<double>(0, 2);
	data.m_cy = P1.at<double>(1, 2);
}