#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_open_camera_try2.h"
#include <QtWidgets/QWidget>
#include <QMessageBox>
#include <QCloseEvent>
#include <QSettings>
#include <QDebug>
#include <QTimer>

#include<opencv2\opencv.hpp>
#include<opencv2\dnn.hpp>
#include<iostream>
#include<string>

#include "MvCamera.h"
#include "camera_thread.h"
#include "marker_detection.h"
#include "tinyxml2.h"

using namespace cv::dnn;

#define MAX_DEVICE_NUM   2
#define MAX_TOOL_TYPE_NUM   256

class CameraManipulation : public QMainWindow
{
	Q_OBJECT

public:
	CameraManipulation(QWidget* parent = Q_NULLPTR);
	//~PcbDetectv3();

private:
	Ui::open_camera_try2Class ui;
	QTimer* display_timer;
	tinyxml2::XMLDocument conf_read;

	CMvCamera* m_stereocamera[MAX_DEVICE_NUM];         // 相机指针对象
	MV_CC_DEVICE_INFO_LIST m_stDevList;              // 设备信息列表结构体变量，用来存储设备列表
	cv::Mat* image_L = new cv::Mat();              //用于保存左相机图像的图像指针对象
	cv::Mat* image_R = new cv::Mat();              //用于保存右相机有图像的图像指针对象
	int devices_num;//设备数量
	MarkerDetection* marker_detection;
	MEASUREMENT_SYSTEM_PARAMS measurement_system_params;
	std::string all_tool_type[MAX_TOOL_TYPE_NUM] = {""};

	/*ch:状态 | en:Status*/
	bool  m_bOpenDevice;                        // ch:是否打开设备 | en:Whether to open device
	bool  m_bStartGrabbing;                     // ch:是否开始抓图 | en:Whether to start grabbing
	int   m_nTriggerMode;                       // ch:触发模式 | en:Trigger Mode
	int   m_bContinueStarted;                      // 开启过连续采集图像
	MV_SAVE_IAMGE_TYPE   m_nSaveImageType;      // ch:保存图像格式 | en:Save Image Type

	// 第二只臂需要初始化的变量
	unsigned short marker_id = 0;
	bool is_detection = false;
	bool is_recongnization = false;
	bool is_global_detection = true;
	std::string current_tool_type = "";

/* 槽函数声明*/
private slots:

	void OnBnClickedEnumButton();              // ch:按下查找设备按钮:枚举  
	void OnBnClickedOpenButton();               // ch:打开设备 | en:Open device
	void OnBnClickedCloseButton();              // ch:关闭设备 | en:Close Devices

	void displayImage();                        // ch:Qlable 显示图像
	void displayLeftImage(const cv::Mat* imagePrt, int cameraIndex);
	void displayRightImage(const cv::Mat* imagePrt, int cameraIndex);


	/*ch:图像采集 | en:Image Acquisition*/
	void OnBnClickedContinusModeRadio();        // ch:连续模式 | en:Continus Mode
	void OnBnClickedTriggerModeRadio();         // ch:触发模式 | en:Trigger Mode
	void OnBnClickedStartGrabbingButton();      // ch:开始采集 | en:Start Grabbing
	void OnBnClickedStopGrabbingButton();       // ch:结束采集 | en:Stop Grabbing
	void OnBnClickedSoftwareOnceButton();       // ch:软触发一次 | en:Software Trigger Execute Once


	void OnBnClickedStartCornerRecongnization();
	void OnBnClickedStartPoseRecongnization();
	void OnBnClickedStartGlobalRecongnization();

	/*ch:图像保存 | en:Image Save*/
	void OnBnClickedSaveJpgButton();            // ch:保存jpg | en:Save jpg
	/*ch:图片保存 | en:Save Image*/
	void SaveImage();                     // ch:保存图片 | en:Save Image

private:
	void OpenDevices();                    // ch:打开设备 | en:Open device
	void CloseDevices();                   // ch:关闭设备 | en:Close Device
	bool isExists(std::string);
	void showResult(cv::Mat& leftSrc, cv::Mat& rightSrc,
		const Eigen::Matrix4d& Tcam_marker, const std::vector<PATTERN_CONTAINER>& left_marker_decal,
		const std::vector<PATTERN_CONTAINER>& right_marker_decal, const float residue);
	
	bool alignCameras(STEREO_CAMERA_INITIAL_PARAMS initial_camera);

	Eigen::Matrix3d getRotation(double,double,double);
	// 配置文件读取
	bool readConnectionInfo();
	bool readAllToolType();
	bool readToolParams();
	bool readCameraParams();
	bool readMarkerParams();
	bool readMovementParams(std::string func_path);
	bool readOtherinfo();

public:
	//MyThread* myThread; //线程对象
	CameraThread* m_thread_leftcamera = NULL;  //左相机线程对象
	CameraThread* m_thread_rightcamera = NULL; //右相机线程对象
};

