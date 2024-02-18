#pragma once
#pragma once
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#define MAX_MARKER_SIZE            8
using Corner = cv::Point_<float>;

//双目相机原始内外参数
typedef struct _STEREO_CAMERA_INITIAL_PARAMS_
{
	Eigen::Matrix3d left_camera_intrinsic;										// 左相机内参
	Eigen::Matrix3d right_camera_intrinsic;										// 右相机内参
	Eigen::Vector4d left_camera_distortion;										// 左相机畸变参数
	Eigen::Vector4d right_camera_distortion;									// 右相机畸变参数
	Eigen::Matrix3d rotation_right2left;										// 左相机坐标系下，右相机的姿态
	Eigen::Vector3d translation_right_left;										// 左相机坐标系下，右相机的位置
	unsigned int image_width;													// 图像的size
	unsigned int image_height;													// 图像的size
}STEREO_CAMERA_INITIAL_PARAMS;

//双目相机对齐后内外参数
typedef struct _RECTIFIED_STEREO_CAMERA_PARAMS_
{
	Eigen::Matrix3d rectified_left_camera_rotation;								// 对齐后，左相机的旋转
	Eigen::Matrix3d rectified_right_camera_rotation;							// 对齐后，右相机的旋转
	Eigen::Matrix3d A_cam;														// 相机的共用内参
	double b_dis;																// 对齐后，在相机坐标系下，右相机与左相机的距离，一般为负数
	unsigned int image_width;													// 图像的size
	unsigned int image_height;													// 图像的size
}RECTIFIED_STEREO_CAMERA_PARAMS;

// 相机参数
typedef struct _STEREO_CAMERA_PARAMS_
{
	STEREO_CAMERA_INITIAL_PARAMS initial_camera_params;							// 相机原始参数
	RECTIFIED_STEREO_CAMERA_PARAMS rectified_camera_params;						// 相机处理后参数
}STEREO_CAMERA_PARAMS;

//marker贴纸八个点坐标定义
typedef struct _PHISICAL_MARKER_DECAL_POSE_
{
	Corner position[MAX_MARKER_SIZE];											// 在贴纸表面，八个点的位置
	unsigned short marker_decal_id;												// 这个贴纸的代号
}PHISICAL_MARKER_DECAL_POSE;

//marker贴纸八个点坐标定义
typedef struct _PATTERN_CONTAINER_
{
	cv::Point2f figure_coordinations[MAX_MARKER_SIZE];							// 在贴纸表面，八个点的位置
	unsigned short marker_pixel_id;												// 这个贴纸的代号
}PATTERN_CONTAINER;

//marker的定义
typedef struct _PHISICAL_MARKER_POSE_
{
	PHISICAL_MARKER_DECAL_POSE pose[MAX_MARKER_SIZE];							// 在marker表面，八个贴纸；在贴纸表面，八个点的位置
	Eigen::Matrix4d matrixALL[MAX_MARKER_SIZE];									// 在marker坐标系下，每个贴纸的坐标
	unsigned short marker_decal_id[MAX_MARKER_SIZE];							// 所有贴纸的代号
	unsigned short marker_id;													// marker的ID
}PHISICAL_MARKER_POSE;

// 测量ROI增长速度
typedef struct _MEASUREMENT_ROI_INCREMENTAL_
{
	double founded_ratio;														// 发现marker时的ROI增长率
	double unfounded_ratio;														// 未发现marker时ROI增长率
}MEASUREMENT_ROI_INCREMENTAL;

// 图像缩放
typedef struct _FIGURE_SCALE_RATIO_
{
	double regular_ratio;														// 正常缩放
	double angle_test_ratio;													// 张开角度标定时的缩放
}FIGURE_SCALE_RATIO;

// 其他辅助变量
typedef struct _OTHER_ACCESSERY_
{
	MEASUREMENT_ROI_INCREMENTAL roi_incremental_ratio;							// roi增长率
	FIGURE_SCALE_RATIO figure_scale;											// 图像显示缩放
}OTHER_ACCESSERY;

// 测量系统参数
typedef struct _MEASUREMENT_SYSTEM_PARAMS_
{
	STEREO_CAMERA_PARAMS camera;												// 相机参数
	PHISICAL_MARKER_POSE marker;												// marker参数
	OTHER_ACCESSERY other_params;												// 其他辅助变量
}MEASUREMENT_SYSTEM_PARAMS;