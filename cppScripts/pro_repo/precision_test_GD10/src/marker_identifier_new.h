#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <ceres/ceres.h>
//#define DEBUG_OUTPUT

using Corner = cv::Point_<float>;
using Corners = std::vector<Corner>;
struct gripperRois
{
	gripperRois()
	{
		memset(verts, 0, 8 * sizeof(float));
		try_ = 0;
	}
	float verts[4][2];
	int try_;
};


template <typename T>
inline T DotProduct(const T x[3], const T y[3])
{
	return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}

template <typename T>
inline void CrossProduct(const T x[3], const T y[3], T result[3])
{
	result[0] = x[1] * y[2] - x[2] * y[1];
	result[1] = x[2] * y[0] - x[0] * y[2];
	result[2] = x[0] * y[1] - x[1] * y[0];
}

template <typename T>
inline void RotPRY(const T* pose, T mat[9])
{
	const T phi = pose[0];
	const T theta = pose[1];
	const T psi = pose[2];

	mat[0] = cos(phi) * cos(theta);
	mat[1] = -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi);
	mat[2] = sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);
	mat[3] = sin(phi) * cos(theta);
	mat[4] = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
	mat[5] = -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi);
	mat[6] = -sin(theta);
	mat[7] = cos(theta) * sin(psi);
	mat[8] = cos(theta) * cos(psi);
}

template <typename T>
inline void Pose2RT(const T* pose, T R[9], T t[3])
{
	RotPRY(pose, R);
	t[0] = pose[3];
	t[1] = pose[4];
	t[2] = pose[5];
}

template <typename T>
inline void MatMulMat(const T A[9], const T B[9], T result[9])
{
	T A_row_1[3] = { A[0], A[1], A[2] };
	T A_row_2[3] = { A[3], A[4], A[5] };
	T A_row_3[3] = { A[6], A[7], A[8] };

	T B_col_1[3] = { B[0], B[3], B[6] };
	T B_col_2[3] = { B[1], B[4], B[7] };
	T B_col_3[3] = { B[2], B[5], B[8] };

	result[0] = DotProduct(A_row_1, B_col_1);
	result[1] = DotProduct(A_row_1, B_col_2);
	result[2] = DotProduct(A_row_1, B_col_3);

	result[3] = DotProduct(A_row_2, B_col_1);
	result[4] = DotProduct(A_row_2, B_col_2);
	result[5] = DotProduct(A_row_2, B_col_3);

	result[6] = DotProduct(A_row_3, B_col_1);
	result[7] = DotProduct(A_row_3, B_col_2);
	result[8] = DotProduct(A_row_3, B_col_3);
}

template <typename T>
inline void MatMulVec(const T A[9], const T B[3], T result[3])
{
	T A_row_1[3] = { A[0], A[1], A[2] };
	T A_row_2[3] = { A[3], A[4], A[5] };
	T A_row_3[3] = { A[6], A[7], A[8] };

	result[0] = DotProduct(A_row_1, B);
	result[1] = DotProduct(A_row_2, B);
	result[2] = DotProduct(A_row_3, B);
}

template <typename T>
inline void VecAddVec(const T A[3], const T B[3], T result[3])
{
	result[0] = A[0] + B[0];
	result[1] = A[1] + B[1];
	result[2] = A[2] + B[2];
}

//ceres
void optInTwoImagesNonePlane(const Corners& left_corners_2D, const std::vector<Eigen::Vector3d>& left_points_3D, const Corners& right_corners_2D,
	const std::vector<Eigen::Vector3d>& right_points_3D, const std::vector<Eigen::Matrix3d>& R_inits, const std::vector<Eigen::Vector3d>& t_inits, const Eigen::Matrix3d& A_cam,
	const double& t_cam, Eigen::Matrix3d& R_res, Eigen::Vector3d& t_res, double& cost);

Corner rectifyOneCorner(const Corner& cs, const Eigen::Matrix3d& A_cam, const Eigen::Matrix3d& rect_rot, 
	const Eigen::Matrix3d& cam_intrins, Eigen::Vector4d& cam_distor);

Corner disRectifyOneCorner(const Corner& corner_, const Eigen::Matrix3d& A_cam,
	const Eigen::Matrix3d& camera_intrinsic, const Eigen::Matrix3d& rect_camera, const Eigen::Vector4d& camera_distortion);

struct PatternContainer {
	PatternContainer()
		: p1(-1, -1),
		p2(-1, -1),
		p3(-1, -1),
		p4(-1, -1),
		p5(-1, -1),
		p6(-1, -1),
		p7(-1, -1),
		p8(-1, -1)
	{
	}

	cv::Point2f p1;
	cv::Point2f p2;
	cv::Point2f p3;
	cv::Point2f p4;
	cv::Point2f p5;
	cv::Point2f p6;
	cv::Point2f p7;
	cv::Point2f p8;
};

int getId(const PatternContainer& sorted_pcs, const std::vector<cv::Point2f>& registrated_pcs);
void findSquares(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares);
void findSquares_adapt(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares);
void detectRectangles(const cv::Mat& thresImg, std::vector<std::vector<cv::Point>>& OutMarkerContours);
bool findCircles(const std::vector < std::vector<cv::Point>>& contoursRect, const cv::Mat& image, 
	std::vector<std::vector<cv::Point2f>>& result, std::vector<std::vector<cv::Point>>& resRect);
bool findCircles_new(const std::vector < std::vector<cv::Point>>& contoursRect, const cv::Mat& image, 
	std::vector<std::vector<cv::Point2f>>& result, std::vector<std::vector<cv::Point>>& resRect);
bool findCircles_comb(const std::vector < std::vector<cv::Point>>& contoursRect, const cv::Mat& image,
	std::vector<std::vector<cv::Point2f>>& result, std::vector<std::vector<cv::Point>>& resRect);

bool find8Points(const cv::Mat& image, std::vector<std::vector<cv::Point2f>>& result,
	std::vector<std::vector<cv::Point>>& contoursRect);

bool distinguish8Points(const std::vector<cv::Point2f>& pointsIn, PatternContainer& pointsOut);

std::vector<int> crossCheck(std::vector<PatternContainer>& leftPoints, std::vector<PatternContainer>& rightPoints,
	const std::vector<std::vector<cv::Point2f>>& registrated_pcs);

std::vector<cv::Point3f> uv2xyz(const std::vector<cv::Point2f>& lPts, const std::vector<cv::Point2f>& rPts,
	const cv::Mat& cameraMatrix, double t);

cv::Mat Tinit(const cv::Point3f& pts0, const cv::Point3f& pts1, const cv::Point3f& pts2);

void markerIdentify(const cv::Mat& leftSrc, const cv::Mat& rightSrc,
	const cv::Rect& gb_rect_l, const cv::Rect& gb_rect_r, const bool& is_global,
	const std::vector<std::vector<cv::Point2f>>& registrated_pcs,
	std::vector<int>& leftMarkerId, std::vector<int>& rightMarkerId,
	std::vector<std::vector<cv::Point2f>>& leftMarkerPoints, 
	std::vector<std::vector<cv::Point2f>>& rightMarkerPoints);