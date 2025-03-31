#pragma once
#pragma once
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#define MAX_MARKER_SIZE            8
using Corner = cv::Point_<float>;

//˫Ŀ���ԭʼ�������
typedef struct _STEREO_CAMERA_INITIAL_PARAMS_
{
	Eigen::Matrix3d left_camera_intrinsic;										// ������ڲ�
	Eigen::Matrix3d right_camera_intrinsic;										// ������ڲ�
	Eigen::Vector4d left_camera_distortion;										// ������������
	Eigen::Vector4d right_camera_distortion;									// ������������
	Eigen::Matrix3d rotation_right2left;										// ���������ϵ�£����������̬
	Eigen::Vector3d translation_right_left;										// ���������ϵ�£��������λ��
	unsigned int image_width;													// ͼ���size
	unsigned int image_height;													// ͼ���size
}STEREO_CAMERA_INITIAL_PARAMS;

//˫Ŀ���������������
typedef struct _RECTIFIED_STEREO_CAMERA_PARAMS_
{
	Eigen::Matrix3d rectified_left_camera_rotation;								// ��������������ת
	Eigen::Matrix3d rectified_right_camera_rotation;							// ��������������ת
	Eigen::Matrix3d A_cam;														// ����Ĺ����ڲ�
	double b_dis;																// ��������������ϵ�£��������������ľ��룬һ��Ϊ����
	unsigned int image_width;													// ͼ���size
	unsigned int image_height;													// ͼ���size
}RECTIFIED_STEREO_CAMERA_PARAMS;

// �������
typedef struct _STEREO_CAMERA_PARAMS_
{
	STEREO_CAMERA_INITIAL_PARAMS initial_camera_params;							// ���ԭʼ����
	RECTIFIED_STEREO_CAMERA_PARAMS rectified_camera_params;						// �����������
}STEREO_CAMERA_PARAMS;

//marker��ֽ�˸������궨��
typedef struct _PHISICAL_MARKER_DECAL_POSE_
{
	Corner position[MAX_MARKER_SIZE];											// ����ֽ���棬�˸����λ��
	unsigned short marker_decal_id;												// �����ֽ�Ĵ���
}PHISICAL_MARKER_DECAL_POSE;

//marker��ֽ�˸������궨��
typedef struct _PATTERN_CONTAINER_
{
	cv::Point2f figure_coordinations[MAX_MARKER_SIZE];							// ����ֽ���棬�˸����λ��
	unsigned short marker_pixel_id;												// �����ֽ�Ĵ���
}PATTERN_CONTAINER;

//marker�Ķ���
typedef struct _PHISICAL_MARKER_POSE_
{
	PHISICAL_MARKER_DECAL_POSE pose[MAX_MARKER_SIZE];							// ��marker���棬�˸���ֽ������ֽ���棬�˸����λ��
	Eigen::Matrix4d matrixALL[MAX_MARKER_SIZE];									// ��marker����ϵ�£�ÿ����ֽ������
	unsigned short marker_decal_id[MAX_MARKER_SIZE];							// ������ֽ�Ĵ���
	unsigned short marker_id;													// marker��ID
}PHISICAL_MARKER_POSE;

// ����ROI�����ٶ�
typedef struct _MEASUREMENT_ROI_INCREMENTAL_
{
	double founded_ratio;														// ����markerʱ��ROI������
	double unfounded_ratio;														// δ����markerʱROI������
}MEASUREMENT_ROI_INCREMENTAL;

// ͼ������
typedef struct _FIGURE_SCALE_RATIO_
{
	double regular_ratio;														// ��������
	double angle_test_ratio;													// �ſ��Ƕȱ궨ʱ������
}FIGURE_SCALE_RATIO;

// ������������
typedef struct _OTHER_ACCESSERY_
{
	MEASUREMENT_ROI_INCREMENTAL roi_incremental_ratio;							// roi������
	FIGURE_SCALE_RATIO figure_scale;											// ͼ����ʾ����
}OTHER_ACCESSERY;

// ����ϵͳ����
typedef struct _MEASUREMENT_SYSTEM_PARAMS_
{
	STEREO_CAMERA_PARAMS camera;												// �������
	PHISICAL_MARKER_POSE marker;												// marker����
	OTHER_ACCESSERY other_params;												// ������������
}MEASUREMENT_SYSTEM_PARAMS;