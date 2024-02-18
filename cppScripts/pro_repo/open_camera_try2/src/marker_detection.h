#pragma once
#include "measurement_system_params.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "ippe.h"

using Corners = std::vector<Corner>;

class MarkerDetection{
public:
	MarkerDetection();
	MarkerDetection(MEASUREMENT_SYSTEM_PARAMS);
	bool getPoseFromFrames(const bool is_global, std::vector<PATTERN_CONTAINER>& left_marker_decal,
		std::vector<PATTERN_CONTAINER>& right_marker_decal, cv::Rect& left_roi, cv::Rect& right_roi,
		Eigen::Matrix4d& pose, float& residue);
	Corner disRectifyOneCorner(const Corner& corner_, const Eigen::Matrix3d& A_cam,
		const Eigen::Matrix3d& camera_intrinsic, const Eigen::Matrix3d& rect_camera, const Eigen::Vector4d& camera_distortion);
	bool uploadConfig(MEASUREMENT_SYSTEM_PARAMS& msp);

private:
	bool checkRepetitivePattern(std::vector<PATTERN_CONTAINER>& left_marker_decal,
		std::vector<PATTERN_CONTAINER>& right_marker_decal);
	void calcAnalyticPosebyIPPE(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D,
		Eigen::Matrix3d& matrixTemp1, Eigen::Vector3d& vectTemp1, Eigen::Matrix3d& matrixTemp2, Eigen::Vector3d& vectTemp2);
	void processOneFrame(const bool is_left, const bool is_global, cv::Rect& rect,std::vector<PATTERN_CONTAINER>& marker_decal, 
		Corners& Points2d, std::vector<Eigen::Vector3d>& Points3d);
	Corner rectifyOneCorner(const Corner& cs, const Eigen::Matrix3d& A_cam, const Eigen::Matrix3d& rect_rot,
		const Eigen::Matrix3d& cam_intrins, Eigen::Vector4d& cam_distor);
	void setROI(cv::Rect& rect, double ratio = 1.05);
	void optInTwoImagesNonePlane(const Corners& left_corners_2D, const std::vector<Eigen::Vector3d>& left_points_3D, const Corners& right_corners_2D,
		const std::vector<Eigen::Vector3d>& right_points_3D, const std::vector<Eigen::Matrix3d>& R_inits, const std::vector<Eigen::Vector3d>& t_inits, const Eigen::Matrix3d& A_cam,
		const double& t_cam, Eigen::Matrix3d& R_res, Eigen::Vector3d& t_res, double& cost);
	Corner rectifySinglePixel(const Corner pixel, const Eigen::Vector4d distortion_coeff);
public:
	MEASUREMENT_SYSTEM_PARAMS measurement_params;
};