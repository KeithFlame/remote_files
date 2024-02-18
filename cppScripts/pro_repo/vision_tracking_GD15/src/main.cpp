//#define _CRT_SECURE_NO_WARNINGS
#define USE_RECTANGLE
#define USE_DRAW_RESULT

#ifdef USE_RECTANGLE

#include "marker_identifier_new.h"
#include "ippe.h"
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <ceres/ceres.h>
#include <time.h>
#include <iostream>
#include <string>

//rectify
Eigen::Matrix3d left_camera_intrinsic;
Eigen::Vector4d left_camera_distortion;
Eigen::Matrix3d right_camera_intrinsic;
Eigen::Vector4d right_camera_distortion;

Eigen::Matrix3d rect_left_camera;
Eigen::Matrix3d rect_right_camera;
Eigen::Matrix3d A_cam;
double b_dis;
int global_img_width = 1920, global_img_height = 1080;

bool loadConfig()
{
	Eigen::Matrix3d rotation_right_left_T;
	Eigen::Vector3d translation_right_left;


	std::ifstream camFile1;
	camFile1.open("./data_x/camParas.raw", std::ios::in);
	double cam_datas[7][4] = { 0 };
	for (int i = 0; i < 7; i++)
	{
		camFile1 >> cam_datas[i][0] >> cam_datas[i][1] >> cam_datas[i][2] >> cam_datas[i][3];
	}
	camFile1 >> global_img_width >> global_img_height;

	/*sc1*/
	left_camera_intrinsic << cam_datas[0][0], 0, cam_datas[0][1],
		0, cam_datas[0][2], cam_datas[0][3],
		0, 0, 1;
	left_camera_distortion << cam_datas[1][0], cam_datas[1][1], cam_datas[1][2], cam_datas[1][3];

	right_camera_intrinsic << cam_datas[2][0], 0, cam_datas[2][1],
		0, cam_datas[2][2], cam_datas[2][3],
		0, 0, 1;
	right_camera_distortion << cam_datas[3][0], cam_datas[3][1], cam_datas[3][2], cam_datas[3][3];

	rotation_right_left_T << cam_datas[4][0], cam_datas[4][1], cam_datas[4][2],
		cam_datas[4][3], cam_datas[5][0], cam_datas[5][1],
		cam_datas[5][2], cam_datas[5][3], cam_datas[6][0];

	translation_right_left << cam_datas[6][1], cam_datas[6][2], cam_datas[6][3];

	auto rotation_right_left = rotation_right_left_T.transpose();
	Eigen::AngleAxisd r_axang;
	r_axang.fromRotationMatrix(rotation_right_left);
	Eigen::AngleAxisd r_axang_n1 = r_axang, r_axang_n2 = r_axang;
	r_axang_n1.angle() = 0.5 * r_axang.angle();
	r_axang_n2.angle() = -0.5 * r_axang.angle();
	auto rect_left_1 = r_axang_n1.toRotationMatrix();
	auto rect_right_1 = r_axang_n2.toRotationMatrix();

	Eigen::Vector3d x_before = rect_right_1 * translation_right_left;
	Eigen::Vector3d x_after = (x_before(0) > 0 ? 1 : -1) * Eigen::Vector3d(1, 0, 0);

	Eigen::AngleAxisd r_axang2;
	auto rot_axis2 = x_before.cross(x_after);

	auto rot_angle2 = acos(x_before.dot(x_after) / (x_before.norm() * x_after.norm()));
	r_axang2.axis() = rot_axis2;
	r_axang2.angle() = rot_angle2;

	auto r_align = r_axang2.toRotationMatrix();

	rect_left_camera = r_align * rect_left_1;
	rect_right_camera = r_align * rect_right_1;

	Eigen::Vector4d f_vec;
	f_vec << left_camera_intrinsic(0, 0), left_camera_intrinsic(1, 1),
		right_camera_intrinsic(0, 0), right_camera_intrinsic(1, 1);

	float f_comm = f_vec.maxCoeff() * 1.05;
	float cx_comm = 0.5 * (left_camera_intrinsic(0, 2) + right_camera_intrinsic(0, 2));
	float cy_comm = 0.5 * (left_camera_intrinsic(1, 2) + right_camera_intrinsic(1, 2));

	A_cam << f_comm, 0, cx_comm,
		0, f_comm, cy_comm,
		0, 0, 1;//offline before year
	b_dis = translation_right_left.norm();

	std::cout << A_cam << std::endl;
	std::cout << b_dis << std::endl;

	return true;
}

void showResult(const cv::Mat& A, double t, cv::Mat& leftSrc, cv::Mat& rightSrc, 
	const std::vector<Eigen::Matrix4f>& Tcam_marker, const std::vector<std::vector<cv::Point2f>>& leftMarkerPoints, 
	const std::vector<std::vector<cv::Point2f>>& rightMarkerPoints, const double cost_gripper) {
	/*camera parameters*/
	const cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
		1., 0., 0., 0.,
		0., 1., 0., 0.,
		0., 0., 1., 0.);

	const cv::Mat cameraMatrixl = A;
	const cv::Mat cameraMatrixr = A;
	const cv::Mat distCoeffsl = (cv::Mat_<float>(1, 4) <<
		0.0, 0.0, 0.0, 0.0);
	const cv::Mat distCoeffsr = (cv::Mat_<float>(1, 4) <<
		0.0, 0.0, 0.0, 0.0);
	const cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
		1.0, 0.0, 0.0, -t,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0);
	int lineThickness = 8;

	cv::Mat proMl(3, 4, CV_32F), proMr(3, 4, CV_32F);
	proMl = cameraMatrixl * T1;
	proMr = cameraMatrixr * T2;
	for (int i = 0; i < Tcam_marker.size(); i++) {
		// 3D points
		float l0 = 20;  // length from point0 to pointx
		cv::Point3f point0(Tcam_marker[i](0, 3), Tcam_marker[i](1, 3), Tcam_marker[i](2, 3));
		cv::Point3f pointx = point0 + l0 * cv::Point3f(Tcam_marker[i](0, 0), Tcam_marker[i](1, 0), Tcam_marker[i](2, 0));
		cv::Point3f pointy = point0 + l0 * cv::Point3f(Tcam_marker[i](0, 1), Tcam_marker[i](1, 1), Tcam_marker[i](2, 1));
		cv::Point3f pointz = point0 + l0 * cv::Point3f(Tcam_marker[i](0, 2), Tcam_marker[i](1, 2), Tcam_marker[i](2, 2));

		// project to image plane
		cv::Mat point0_l(3, 1, CV_32F), point0_r(3, 1, CV_32F);
		point0_l = proMl * (cv::Mat_<float>(4, 1) << point0.x, point0.y, point0.z, 1.0);
		point0_r = proMr * (cv::Mat_<float>(4, 1) << point0.x, point0.y, point0.z, 1.0);
		Corner point0_l_2d = Corner(point0_l.at<float>(0, 0) / point0_l.at<float>(2, 0), point0_l.at<float>(1, 0) / point0_l.at<float>(2, 0));
		Corner point0_r_2d = Corner(point0_r.at<float>(0, 0) / point0_r.at<float>(2, 0), point0_r.at<float>(1, 0) / point0_r.at<float>(2, 0));
		cv::Mat pointx_l(3, 1, CV_32F), pointx_r(3, 1, CV_32F);
		pointx_l = proMl * (cv::Mat_<float>(4, 1) << pointx.x, pointx.y, pointx.z, 1.0);
		pointx_r = proMr * (cv::Mat_<float>(4, 1) << pointx.x, pointx.y, pointx.z, 1.0);
		Corner pointx_l_2d = Corner(pointx_l.at<float>(0, 0) / pointx_l.at<float>(2, 0), pointx_l.at<float>(1, 0) / pointx_l.at<float>(2, 0));
		Corner pointx_r_2d = Corner(pointx_r.at<float>(0, 0) / pointx_r.at<float>(2, 0), pointx_r.at<float>(1, 0) / pointx_r.at<float>(2, 0));
		cv::Mat pointy_l(3, 1, CV_32F), pointy_r(3, 1, CV_32F);
		pointy_l = proMl * (cv::Mat_<float>(4, 1) << pointy.x, pointy.y, pointy.z, 1.0);
		pointy_r = proMr * (cv::Mat_<float>(4, 1) << pointy.x, pointy.y, pointy.z, 1.0);
		Corner pointy_l_2d = Corner(pointy_l.at<float>(0, 0) / pointy_l.at<float>(2, 0), pointy_l.at<float>(1, 0) / pointy_l.at<float>(2, 0));
		Corner pointy_r_2d = Corner(pointy_r.at<float>(0, 0) / pointy_r.at<float>(2, 0), pointy_r.at<float>(1, 0) / pointy_r.at<float>(2, 0));
		cv::Mat pointz_l(3, 1, CV_32F), pointz_r(3, 1, CV_32F);
		pointz_l = proMl * (cv::Mat_<float>(4, 1) << pointz.x, pointz.y, pointz.z, 1.0);
		pointz_r = proMr * (cv::Mat_<float>(4, 1) << pointz.x, pointz.y, pointz.z, 1.0);
		Corner pointz_l_2d = Corner(pointz_l.at<float>(0, 0) / pointz_l.at<float>(2, 0), pointz_l.at<float>(1, 0) / pointz_l.at<float>(2, 0));
		Corner pointz_r_2d = Corner(pointz_r.at<float>(0, 0) / pointz_r.at<float>(2, 0), pointz_r.at<float>(1, 0) / pointz_r.at<float>(2, 0));


		//disrectify
		point0_l_2d = disRectifyOneCorner(point0_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);
		pointx_l_2d = disRectifyOneCorner(pointx_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);
		pointy_l_2d = disRectifyOneCorner(pointy_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);
		pointz_l_2d = disRectifyOneCorner(pointz_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);

		point0_r_2d = disRectifyOneCorner(point0_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);
		pointx_r_2d = disRectifyOneCorner(pointx_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);
		pointy_r_2d = disRectifyOneCorner(pointy_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);
		pointz_r_2d = disRectifyOneCorner(pointz_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);

		cv::line(leftSrc, point0_l_2d, pointx_l_2d, cv::Scalar(0, 0, 255), lineThickness);
		cv::line(leftSrc, point0_l_2d, pointy_l_2d, cv::Scalar(0, 255, 0), lineThickness);
		cv::line(leftSrc, point0_l_2d, pointz_l_2d, cv::Scalar(255, 0, 0), lineThickness);
		cv::line(rightSrc, point0_r_2d, pointx_r_2d, cv::Scalar(0, 0, 255), lineThickness);
		cv::line(rightSrc, point0_r_2d, pointy_r_2d, cv::Scalar(0, 255, 0), lineThickness);
		cv::line(rightSrc, point0_r_2d, pointz_r_2d, cv::Scalar(255, 0, 0), lineThickness);
	}

	double resize_scale = 0.2;
	cv::Mat img_left_resized, img_right_resized;
	cv::resize(leftSrc, img_left_resized, cv::Size(), resize_scale, resize_scale);
	cv::resize(rightSrc, img_right_resized, cv::Size(), resize_scale, resize_scale);

	/*joint image*/
	cv::Mat joint_image;
	hconcat(img_left_resized, img_right_resized, joint_image);

	if (Tcam_marker.size() > 0)
	{
		char* chCode;
		std::string str2;
		Eigen::AngleAxisd rot_axang;
		Eigen::Matrix3d Rcam_marker;
		Rcam_marker = Tcam_marker[0].topLeftCorner(3, 3).cast<double>();
		rot_axang.fromRotationMatrix(Rcam_marker);
		Eigen::Vector3d rot_axis;
		rot_axis = rot_axang.axis() * rot_axang.angle();
		chCode = new(std::nothrow)char[200];
		std::sprintf(chCode, "position [%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf, %.2lf]", 
			Tcam_marker[0](0, 3), Tcam_marker[0](1, 3), Tcam_marker[0](2, 3)
		, rot_axis(0), rot_axis(1), rot_axis(2), cost_gripper);
		str2 = std::string(chCode);
		delete[]chCode;

		cv::putText(joint_image, str2, cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(64, 187, 208), 2);
	}

	cv::imshow("imgRes", joint_image);
}

std::vector<cv::Mat> matrixALL;
std::vector<int> idALL{468, 357, 159, 789, 456, 123, 369, 258, 147};
//std::vector<int> idALL{ 123};
std::vector<cv::Mat> pts4d_5comm;
std::vector<std::vector<cv::Mat>> pts4d_all_3others;
std::vector<std::vector<cv::Point2f>> registered_mks;
bool is_inited = false;

void initMarker3DCoodinates()
{
	bool trocar_gripper_readed = false, marker_8points_readed = false;
	// read file: matrixALL.
	std::ifstream fileStream;
	std::string tmp;
	fileStream.open("./data_x/trocargripper.raw", std::ios::in);
	if (fileStream.fail()) {
		printf("fail to open file\n");
	}
	else {
		matrixALL.clear();
		for (int i = 0; i < (idALL.size()); i++) {
			cv::Mat tmpV = cv::Mat(4, 4, CV_32F);
			for (int j = 0; j < 4; j++) {  // four rows.
				std::getline(fileStream, tmp, '\n');
				std::istringstream is(tmp);
				for (int k = 0; k < 4; k++) {  // four colums.
					std::string str_tmp;
					is >> str_tmp;
					float ssss = std::stof(str_tmp);
					tmpV.at<float>(j, k) = ssss;
				}
			}
			std::cout << tmpV << std::endl;
			matrixALL.push_back(tmpV);
		}
		fileStream.close();

		trocar_gripper_readed = true;
	}

	std::ifstream fileStream1;
	fileStream1.open("./data_x/marker8points.raw", std::ios::in);
	if (fileStream1.fail()) {
		printf("fail to open marker8points file\n");
	}
	else {
		pts4d_5comm.clear();
		pts4d_all_3others.clear();
		for (size_t i = 0; i < 5; i++)
		{
			float data_xy[2] = { 0 };
			std::getline(fileStream1, tmp, '\n');
			std::istringstream is(tmp);
			for (int k = 0; k < 2; k++) {  // two colums.
				std::string str_tmp;
				is >> str_tmp;
				data_xy[k] = std::stof(str_tmp);
			}

			pts4d_5comm.push_back((cv::Mat_<float>(4, 1) << data_xy[0], data_xy[1], 0.0, 1.0));
		}
		for (size_t i = 0; i < idALL.size(); i++)
		{
			std::vector<cv::Mat> pts4d_3others_tmp;
			for (size_t j = 0; j < 3; j++) // three rows
			{
				float data_xy[2] = { 0 };
				std::getline(fileStream1, tmp, '\n');
				std::istringstream is(tmp);
				for (int k = 0; k < 2; k++) {  // two colums.
					std::string str_tmp;
					is >> str_tmp;
					data_xy[k] = std::stof(str_tmp);
				}
				pts4d_3others_tmp.push_back((cv::Mat_<float>(4, 1) << data_xy[0], data_xy[1], 0.0, 1.0));
			}
			pts4d_all_3others.push_back(pts4d_3others_tmp);
		}

		fileStream1.close();

		marker_8points_readed = true;
	}
	is_inited = trocar_gripper_readed && marker_8points_readed;
	
	if (is_inited)
	{
		registered_mks.clear();
		float unit_len = pts4d_5comm.at(1).at<float>(0, 0);
		for (size_t i = 0; i < pts4d_all_3others.size(); i++)
		{
			std::vector<cv::Point2f> one_mk_pcs;
			for (size_t j = 0; j < pts4d_all_3others.at(i).size(); j++)
			{
				one_mk_pcs.push_back(cv::Point2f(pts4d_all_3others.at(i).at(j).at<float>(0, 0) / unit_len,
					pts4d_all_3others.at(i).at(j).at<float>(1, 0) / unit_len));
			}
			registered_mks.push_back(one_mk_pcs);
		}
	}
}

void calcAnalyticPosebyIPPE(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D, 
	Eigen::Matrix3d& matrixTemp1, Eigen::Vector3d& vectTemp1, Eigen::Matrix3d& matrixTemp2, Eigen::Vector3d& vectTemp2)
{
	std::vector<cv::Point3f> object3DPoints;//存储四个点的世界坐标
	std::vector<cv::Point2f> image2DPoints;//存储四个点的图像坐标
	cv::Mat camera_matrix;//内参数矩阵
	cv::Mat distortion_coefficients;//畸变系数
	camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
	camera_matrix.ptr<double>(0)[0] = A_cam(0, 0);
	camera_matrix.ptr<double>(0)[2] = A_cam(0, 2);
	camera_matrix.ptr<double>(1)[1] = A_cam(1, 1);
	camera_matrix.ptr<double>(1)[2] = A_cam(1, 2);
	camera_matrix.ptr<double>(2)[2] = 1.0f;
	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

	//设置特征点的世界坐标,图像坐标
	for (int i = 0; i < points_3D.size(); i++)
	{
		//三维坐标的单位是毫米
		object3DPoints.push_back(cv::Point3f(points_3D.at(i)(0), points_3D.at(i)(1), 0));
		image2DPoints.push_back(corners_2D.at(i));
	}

	IPPE::PoseSolver planePoseSolver;
	cv::Mat rvec1, tvec1; //first pose
	cv::Mat rvec2, tvec2; //second pose
	float err1, err2; //RMS reprojection errors of first and second poses

	planePoseSolver.solveGeneric(object3DPoints, image2DPoints, camera_matrix, distortion_coefficients, rvec1, tvec1, err1, rvec2, tvec2, err2);

	/*******************提取旋转矩阵*********************/
	cv::Mat RoteM1, TransM1;
	double rm1[9];
	RoteM1 = cv::Mat(3, 3, CV_64FC1, rm1);
	cv::Rodrigues(rvec1, RoteM1);
	TransM1 = tvec1;
	cv::cv2eigen(RoteM1, matrixTemp1);
	cv::cv2eigen(TransM1, vectTemp1);

	//another
	cv::Mat RoteM2, TransM2;
	double rm2[9];
	RoteM2 = cv::Mat(3, 3, CV_64FC1, rm2);
	cv::Rodrigues(rvec2, RoteM2);
	TransM2 = tvec2;
	cv::cv2eigen(RoteM2, matrixTemp2);
	cv::cv2eigen(TransM2, vectTemp2);
	//std::cout << "err1: " << err1 << "\terr2: " << err2 << std::endl;

	return ;
}

int main()
{

	if (!loadConfig())
	{
		return false;
	}

	cv::Size size(global_img_width, global_img_height);
	cv::Rect gp_rect_l = cv::Rect(0, 0, size.width, size.height);
	cv::Rect gp_rect_r = cv::Rect(0, 0, size.width, size.height);
	int waitTime = 1;
	double r_k2 = 12.0, amp_rate = 1.05;
	Eigen::Matrix4f Tcam_gripper = Eigen::Matrix4f::Identity();
	bool use_gp_roi = false;

	// Doctor.Chen
	cv::Mat A = (cv::Mat_<float>(3, 3) <<
		A_cam(0, 0), A_cam(0, 1), A_cam(0, 2),
		A_cam(1, 0), A_cam(1, 1), A_cam(1, 2),
		A_cam(2, 0), A_cam(2, 1), A_cam(2, 2));
	double t = b_dis;// 3.989100;
	if (A.empty()) {
		return false;
	}

	cv::Mat leftSrc;// = cv::imread("./figs/25_l.png");  // CV_8UC1
	cv::Mat rightSrc;// = cv::imread("./figs/25_r.png");;  // CV_8UC1


	int loop_count = 0, write_count = 0;
	//cv::VideoCapture capture_left, capture_right;
	//std::cout << "cap1 loading ..." << std::endl;
	//capture_left.open(1);
	//std::cout << "cap1 loaded,cap2 loading ..." << capture_left.isOpened() << std::endl;
	//capture_right.open(0);
	//std::cout << "cap2 loaded." << capture_right.isOpened() << std::endl;

	//capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
	//capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
	//capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
	////capture_left.set(cv::CAP_PROP_FRAME_COUNT, 19);
	//capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);

	//std::cout << "over21" << std::endl;
	//capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
	//capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
	//capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
	//capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);



	std::ofstream record_pose_data;
	record_pose_data.open("./data_x/sm_pose_data.raw");

	//cv::namedWindow("imgRes", cv::WINDOW_NORMAL);
	char wr_name[100];
	//while (1/*capture_left.read(leftSrc) && capture_right.read(rightSrc)*/)

	//if (1)
		for (size_t image_num1 = 1; image_num1 < 65; image_num1++)
	{
		sprintf(wr_name, "F:/code_git/cppScripts/pro_repo/open_camera_try2/1234/left_%03d.jpg", image_num1);
		leftSrc = cv::imread(wr_name);
		sprintf(wr_name, "F:/code_git/cppScripts/pro_repo/open_camera_try2/1234//right_%03d.jpg", image_num1);
		rightSrc = cv::imread(wr_name);


		cv::Mat leftOrg, rightOrg;
		leftOrg = leftSrc.clone();
		rightOrg = rightSrc.clone();

		if (!is_inited)
		{
			initMarker3DCoodinates();
		}



		Eigen::Matrix4d	mat4;
		for (size_t i = 0; i < 9; i++)
		{
			cv::cv2eigen(matrixALL[i], mat4);
			std::cout << idALL[i] << std::endl;
			std::cout << mat4 << std::endl;
		}







		/*******************************************************************/
		std::vector<int> leftMarkerId, rightMarkerId;
		std::vector<std::vector<cv::Point2f>> leftMarkerPoints, rightMarkerPoints;
		markerIdentify(leftSrc, rightSrc, gp_rect_l, gp_rect_r, false, registered_mks,
			leftMarkerId, rightMarkerId, leftMarkerPoints, rightMarkerPoints);

		// draw
		cv::Scalar color_arr[5] = { cv::Scalar(0, 255, 255), cv::Scalar(0, 0, 255),
			cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 0) };

		for (size_t i = 0; i < leftMarkerPoints.size(); i++) {
			int clc_id = 0;

			cv::circle(leftSrc, leftMarkerPoints[i][0], 4, color_arr[4], -1);
			for (int j = 1; j < leftMarkerPoints.at(i).size(); j++) {
				cv::circle(leftSrc, leftMarkerPoints[i][j], 5, color_arr[1], -1);
			}
		}
		cv::rectangle(leftSrc, gp_rect_l, cv::Scalar(0, 255, 255), 2);
		for (size_t i = 0; i < rightMarkerPoints.size(); i++) {
			int clc_id = 0;

			cv::circle(rightSrc, rightMarkerPoints[i][0], 4, color_arr[4], -1);
			for (int j = 1; j < rightMarkerPoints.at(i).size(); j++) {
				cv::circle(rightSrc, rightMarkerPoints[i][j], 5, color_arr[1], -1);
			}
		}
		cv::rectangle(rightSrc, gp_rect_r, cv::Scalar(0, 255, 255), 2);
		
		//cv::Rect rect_tem1 = cv::Rect(2000,1500,700,500);
		//cv::Rect rect_tem2 = cv::Rect(2800, 1550, 700, 500);
		//cv::Mat joint_image;
		//hconcat(leftSrc(rect_tem2), rightSrc(rect_tem1), joint_image);
		//imshow("", joint_image);
		//cv::waitKey();

		//std::cout << "leftGripperPoints2d_before: \n" << std::endl;
		//for (size_t i = 0; i < leftMarkerPoints[0].size(); i++)
		//	std::cout << leftMarkerPoints[1][i] << std::endl;
		//for (size_t i = 0; i < rightMarkerPoints[0].size(); i++)
		//	std::cout << rightMarkerPoints[1][i] << std::endl;

		//rectify
		if (leftMarkerPoints.size() > 0)
		{
			for (size_t i = 0; i < leftMarkerPoints.size(); i++)
			{
				for (size_t j = 0; j < leftMarkerPoints.at(i).size(); j++)
				{
					leftMarkerPoints.at(i).at(j) = rectifyOneCorner(leftMarkerPoints.at(i).at(j), A_cam,
						rect_left_camera, left_camera_intrinsic, left_camera_distortion);
				}
			}
		}
		if (rightMarkerPoints.size() > 0)
		{
			for (size_t i = 0; i < rightMarkerPoints.size(); i++)
			{
				for (size_t j = 0; j < rightMarkerPoints.at(i).size(); j++)
				{
					rightMarkerPoints.at(i).at(j) = rectifyOneCorner(rightMarkerPoints.at(i).at(j), A_cam,
						rect_right_camera, right_camera_intrinsic, right_camera_distortion);
				}
			}
		}

		Corners leftGripperPoints2d, rightGripperPoints2d;
		std::vector<Eigen::Vector3d> leftGripperPoints3d, rightGripperPoints3d;
		std::vector<cv::Vec3f> leftGripperPoints3d_cv, rightGripperPoints3d_cv;
		bool find_gripper = true;

#if 1

		for (size_t i = 0; i < leftMarkerId.size(); i++) {
			//std::cout << "leftMarkerId[" << i << "] " << leftMarkerId[i] << std::endl;
			for (size_t j = 0; j < idALL.size(); j++) {
				if (leftMarkerId[i] == idALL[j]) {  // 3-5: gripper id  && j > 2 && j < 6
					for (size_t k = 0; k < leftMarkerPoints.at(i).size()/*pts4d_5comm.size()*/; k++)
					{
						cv::Mat pts4d;
						if (k < 5)
							pts4d = pts4d_5comm.at(k).clone();  // p_comm
						else
							pts4d = pts4d_all_3others.at(j).at(k - 5).clone();  // p_3others

						leftGripperPoints2d.emplace_back(leftMarkerPoints[i][k]);
						pts4d = matrixALL[j] * pts4d;
						leftGripperPoints3d.emplace_back(Eigen::Vector3d(pts4d.at<float>(0, 0), pts4d.at<float>(1, 0), pts4d.at<float>(2, 0)));
						leftGripperPoints3d_cv.emplace_back(cv::Vec3d(pts4d.at<float>(0, 0), pts4d.at<float>(1, 0), pts4d.at<float>(2, 0)));
					}
					find_gripper = true;
					//break;
				}
			}
		}

		// right image
		for (size_t i = 0; i < rightMarkerId.size(); i++) {
			for (size_t j = 0; j < idALL.size(); j++) {
				if (rightMarkerId[i] == idALL[j]) {  // 3-5: gripper id   && j > 2 && j < 6
					for (size_t k = 0; k < rightMarkerPoints.at(i).size()/*pts4d_5comm.size()*/; k++)
					{
						cv::Mat pts4d;
						if (k < 5)
							pts4d = pts4d_5comm.at(k).clone();  // p_comm
						else
							pts4d = pts4d_all_3others.at(j).at(k - 5).clone();  // p_3others

						rightGripperPoints2d.emplace_back(rightMarkerPoints[i][k]);
						pts4d = matrixALL[j] * pts4d;
						rightGripperPoints3d.emplace_back(Eigen::Vector3d(pts4d.at<float>(0, 0), pts4d.at<float>(1, 0), pts4d.at<float>(2, 0)));
						rightGripperPoints3d_cv.emplace_back(cv::Vec3d(pts4d.at<float>(0, 0), pts4d.at<float>(1, 0), pts4d.at<float>(2, 0)));
					}
					find_gripper = find_gripper && true;
					//break;
				}
			}
		}
#endif
		// extract gp roi
		if (use_gp_roi)
		{
			std::vector<cv::Point2f> rcr_t;
			if (leftGripperPoints2d.size() > 0)
			{
				int uid = 0;
				if (leftGripperPoints2d.size() > 8)
				{
					float test_len1 = sqrtf(powf(leftGripperPoints2d.at(1).x - leftGripperPoints2d.at(0).x, 2.0)
						+ powf(leftGripperPoints2d.at(1).y - leftGripperPoints2d.at(0).y, 2.0)) +
						sqrtf(powf(leftGripperPoints2d.at(2).x - leftGripperPoints2d.at(0).x, 2.0)
							+ powf(leftGripperPoints2d.at(2).y - leftGripperPoints2d.at(0).y, 2.0));
					float test_len2 = sqrtf(powf(leftGripperPoints2d.at(9).x - leftGripperPoints2d.at(8).x, 2.0)
						+ powf(leftGripperPoints2d.at(9).y - leftGripperPoints2d.at(8).y, 2.0)) +
						sqrtf(powf(leftGripperPoints2d.at(10).x - leftGripperPoints2d.at(8).x, 2.0)
							+ powf(leftGripperPoints2d.at(10).y - leftGripperPoints2d.at(8).y, 2.0));
					if (test_len2 > test_len1)
					{
						uid = 1;
					}
				}

				std::vector<Eigen::Vector2f> rps_t;
				for (size_t rei = 0; rei < 4; rei++)
				{
					rps_t.push_back(Eigen::Vector2f(leftGripperPoints2d.at(8 * uid + rei).x,
						leftGripperPoints2d.at(8 * uid + rei).y));
				}
				Eigen::Vector2f rp_ctr = 0.5 * (rps_t.at(0) + rps_t.at(3));
				float sl_ = r_k2 * std::max((float)(rps_t.at(1) - rps_t.at(0)).norm(),
					(float)(rps_t.at(2) - rps_t.at(0)).norm());

				rcr_t.push_back(cv::Point2f(rp_ctr.x() - 0.5 * sl_, rp_ctr.y() - 0.6 * sl_));
				rcr_t.push_back(cv::Point2f(rp_ctr.x() - 0.5 * sl_, rp_ctr.y() + 0.6 * sl_));
				rcr_t.push_back(cv::Point2f(rp_ctr.x() + 0.5 * sl_, rp_ctr.y() + 0.6 * sl_));
				rcr_t.push_back(cv::Point2f(rp_ctr.x() + 0.5 * sl_, rp_ctr.y() - 0.6 * sl_));
			}
			else
			{
				rcr_t.push_back(cv::Point2f(gp_rect_l.x - 0.5 * (amp_rate - 1.0) * gp_rect_l.width,
					gp_rect_l.y - 0.5 * (amp_rate - 1.0) * gp_rect_l.height));
				rcr_t.push_back(cv::Point2f(gp_rect_l.x + amp_rate * gp_rect_l.width,
					gp_rect_l.y - 0.5 * (amp_rate - 1.0) * gp_rect_l.height));
				rcr_t.push_back(cv::Point2f(gp_rect_l.x + amp_rate * gp_rect_l.width,
					gp_rect_l.y + amp_rate * gp_rect_l.height));
				rcr_t.push_back(cv::Point2f(gp_rect_l.x - 0.5 * (amp_rate - 1.0) * gp_rect_l.width,
					gp_rect_l.y + amp_rate * gp_rect_l.height));
			}
			int temp_minx = std::max((int)std::min({ rcr_t[0].x, rcr_t[1].x, rcr_t[2].x, rcr_t[3].x }), (int)0);
			int temp_miny = std::max((int)std::min({ rcr_t[0].y, rcr_t[1].y, rcr_t[2].y, rcr_t[3].y }), (int)0);
			int temp_maxx = std::min((int)std::max({ rcr_t[0].x, rcr_t[1].x, rcr_t[2].x, rcr_t[3].x }), (int)leftSrc.cols - 1);
			int temp_maxy = std::min((int)std::max({ rcr_t[0].y, rcr_t[1].y, rcr_t[2].y, rcr_t[3].y }), (int)leftSrc.rows - 1);
			gp_rect_l = cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx, temp_maxy - temp_miny);
		}
		if (use_gp_roi)
		{
			std::vector<cv::Point2f> rcr_t;
			if (rightGripperPoints2d.size() > 0)
			{
				int uid = 0;
				if (rightGripperPoints2d.size() > 8)
				{
					float test_ren1 = sqrtf(powf(rightGripperPoints2d.at(1).x - rightGripperPoints2d.at(0).x, 2.0)
						+ powf(rightGripperPoints2d.at(1).y - rightGripperPoints2d.at(0).y, 2.0)) +
						sqrtf(powf(rightGripperPoints2d.at(2).x - rightGripperPoints2d.at(0).x, 2.0)
							+ powf(rightGripperPoints2d.at(2).y - rightGripperPoints2d.at(0).y, 2.0));
					float test_ren2 = sqrtf(powf(rightGripperPoints2d.at(9).x - rightGripperPoints2d.at(8).x, 2.0)
						+ powf(rightGripperPoints2d.at(9).y - rightGripperPoints2d.at(8).y, 2.0)) +
						sqrtf(powf(rightGripperPoints2d.at(10).x - rightGripperPoints2d.at(8).x, 2.0)
							+ powf(rightGripperPoints2d.at(10).y - rightGripperPoints2d.at(8).y, 2.0));
					if (test_ren2 > test_ren1)
					{
						uid = 1;
					}
				}

				std::vector<Eigen::Vector2f> rps_t;
				for (size_t rei = 0; rei < 4; rei++)
				{
					rps_t.push_back(Eigen::Vector2f(rightGripperPoints2d.at(8 * uid + rei).x,
						rightGripperPoints2d.at(8 * uid + rei).y));
				}
				Eigen::Vector2f rp_ctr = 0.5 * (rps_t.at(0) + rps_t.at(3));
				float sl_ = r_k2 * std::max((float)(rps_t.at(1) - rps_t.at(0)).norm(),
					(float)(rps_t.at(2) - rps_t.at(0)).norm());

				rcr_t.push_back(cv::Point2f(rp_ctr.x() - 0.5 * sl_, rp_ctr.y() - 0.6 * sl_));
				rcr_t.push_back(cv::Point2f(rp_ctr.x() - 0.5 * sl_, rp_ctr.y() + 0.6 * sl_));
				rcr_t.push_back(cv::Point2f(rp_ctr.x() + 0.5 * sl_, rp_ctr.y() + 0.6 * sl_));
				rcr_t.push_back(cv::Point2f(rp_ctr.x() + 0.5 * sl_, rp_ctr.y() - 0.6 * sl_));
			}
			else
			{
				rcr_t.push_back(cv::Point2f(gp_rect_r.x - 0.5 * (amp_rate - 1.0) * gp_rect_r.width,
					gp_rect_r.y - 0.5 * (amp_rate - 1.0) * gp_rect_r.height));
				rcr_t.push_back(cv::Point2f(gp_rect_r.x + amp_rate * gp_rect_r.width,
					gp_rect_r.y - 0.5 * (amp_rate - 1.0) * gp_rect_r.height));
				rcr_t.push_back(cv::Point2f(gp_rect_r.x + amp_rate * gp_rect_r.width,
					gp_rect_r.y + amp_rate * gp_rect_r.height));
				rcr_t.push_back(cv::Point2f(gp_rect_r.x - 0.5 * (amp_rate - 1.0) * gp_rect_r.width,
					gp_rect_r.y + amp_rate * gp_rect_r.height));
			}
			int temp_minx = std::max((int)std::min({ rcr_t[0].x, rcr_t[1].x, rcr_t[2].x, rcr_t[3].x }), (int)0);
			int temp_miny = std::max((int)std::min({ rcr_t[0].y, rcr_t[1].y, rcr_t[2].y, rcr_t[3].y }), (int)0);
			int temp_maxx = std::min((int)std::max({ rcr_t[0].x, rcr_t[1].x, rcr_t[2].x, rcr_t[3].x }), (int)rightSrc.cols - 1);
			int temp_maxy = std::min((int)std::max({ rcr_t[0].y, rcr_t[1].y, rcr_t[2].y, rcr_t[3].y }), (int)rightSrc.rows - 1);
			gp_rect_r = cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx, temp_maxy - temp_miny);
		}

		// calculate T initial.
		Eigen::Matrix3d A_eig;
		A_eig << A.at<float>(0, 0), A.at<float>(0, 1), A.at<float>(0, 2),
			A.at<float>(1, 0), A.at<float>(1, 1), A.at<float>(1, 2),
			A.at<float>(2, 0), A.at<float>(2, 1), A.at<float>(2, 2);

		Eigen::Matrix3d R_res_gripper = Eigen::Matrix3d::Identity();
		Eigen::Vector3d t_res_gripper = Eigen::Vector3d::Zero();
		double cost_gripper = 0.0;
		//gripper
		if (find_gripper == true && leftMarkerId.size()>0) {
			std::vector<Eigen::Matrix3d> rot_inits_gripper;
			std::vector<Eigen::Vector3d> trans_inits_gripper;
			if (leftGripperPoints2d.size() > 0)
			{  // use left camera points to solve pnp.
				std::vector<Eigen::Vector3d>::const_iterator first3d = leftGripperPoints3d.begin();
				std::vector<Eigen::Vector3d>::const_iterator last3d = leftGripperPoints3d.begin() + 7;
				std::vector<Eigen::Vector3d> p3d_onemk(first3d, last3d);
				Corners::const_iterator first2d = leftGripperPoints2d.begin();
				Corners::const_iterator last2d = leftGripperPoints2d.begin() + 7;
				Corners p2d_onemk(first2d, last2d);

				Eigen::Matrix3d rotm_1, rotm_2;
				Eigen::Vector3d trans_1, trans_2;
				calcAnalyticPosebyIPPE(p2d_onemk, p3d_onemk, rotm_1, trans_1, rotm_2, trans_2);

				/*LM Optimization*/
				rot_inits_gripper.push_back(rotm_1);
				trans_inits_gripper.push_back(trans_1);
				rot_inits_gripper.push_back(rotm_2);
				trans_inits_gripper.push_back(trans_2);
			}
			optInTwoImagesNonePlane(leftGripperPoints2d, leftGripperPoints3d, rightGripperPoints2d, rightGripperPoints3d,
				rot_inits_gripper, trans_inits_gripper, A_eig, t, R_res_gripper, t_res_gripper, cost_gripper);

			Tcam_gripper << R_res_gripper(0, 0), R_res_gripper(0, 1), R_res_gripper(0, 2), t_res_gripper(0),
				R_res_gripper(1, 0), R_res_gripper(1, 1), R_res_gripper(1, 2), t_res_gripper(1),
				R_res_gripper(2, 0), R_res_gripper(2, 1), R_res_gripper(2, 2), t_res_gripper(2),
				0.0, 0.0, 0.0, 1.0;

			if (cost_gripper > 15.0)
			{
				find_gripper = false;
			}

			std::vector<Eigen::Matrix4f> T_marker;
			if (find_gripper)
			{
				T_marker.emplace_back(Tcam_gripper);
			}
			showResult(A, t, leftSrc, rightSrc, T_marker, leftMarkerPoints, rightMarkerPoints, cost_gripper);

			std::cout << "T_marker: \n" << image_num1 << '\t' << 0 << '\t' <<0
				<<'\t' << cost_gripper << "\n" << T_marker[0] << std::endl;
			
			loop_count++;
			//cv::waitKey();
			auto key = cv::waitKey(waitTime);
			if (key == 'q' || key == 'Q')
			{
				record_pose_data.close();
				exit(0);// break;
			}
			if (key == 'w' || key == 'W')
			{
				char wr_name[100];
				sprintf(wr_name, "./test_pics/record_l%04d.jpg", write_count);
				cv::imwrite(wr_name, leftSrc);
				sprintf(wr_name, "./test_pics/record_r%04d.jpg", write_count);
				cv::imwrite(wr_name, rightSrc);
				write_count++;
			}
			if (key == 's' || key == 'S')
			{
				if (waitTime > 0)
				{
					waitTime = 0;
					std::cout << "stoped" << std::endl;
				}
				else
				{
					waitTime = 1;
				}
			}
			if (key == 'r' || key == 'R')
			{
				Eigen::Quaterniond R_quat(R_res_gripper);
				record_pose_data << Tcam_gripper(0, 3) << "\t" << Tcam_gripper(1, 3) << "\t" << Tcam_gripper(2, 3) << "\t" << R_quat.w()
					<< "\t" << R_quat.x() << "\t" << R_quat.y() << "\t" << R_quat.z() << "\t" << cost_gripper << std::endl;
			}

		}
		else
		{
			std::cout << "T_marker: \n" << image_num1<<'\t'<<0 << '\t' << 0 << '\t'
				<< cost_gripper << "\n" << Eigen::Matrix4f::Identity() << std::endl;
		}
	}
	return 0;
}
#endif

