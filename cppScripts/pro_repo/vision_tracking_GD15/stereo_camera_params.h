#pragma once
#include <Eigen/Core>

typedef struct _STEREO_CAMERA_PARAMS_
{
	Eigen::Matrix3d left_camera_intrinsic;
	Eigen::Matrix3d right_camera_intrinsic;
	Eigen::Matrix3d rotation_right_left_T;
	Eigen::Vector4d left_camera_intrinsic;
	Eigen::Vector4d right_camera_intrinsic;
	Eigen::Vector3d translation_right_left;

	Eigen::Matrix3d rect_left_camera;
	Eigen::Matrix3d rect_right_camera;
	Eigen::Matrix3d A_cam;
	double b_dis;
	int global_img_width;
	int global_img_height;
}STEREO_CAMERA_PARAMS;