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

	CMvCamera* m_stereocamera[MAX_DEVICE_NUM];         // ���ָ�����
	MV_CC_DEVICE_INFO_LIST m_stDevList;              // �豸��Ϣ�б�ṹ������������洢�豸�б�
	cv::Mat* image_L = new cv::Mat();              //���ڱ��������ͼ���ͼ��ָ�����
	cv::Mat* image_R = new cv::Mat();              //���ڱ����������ͼ���ͼ��ָ�����
	int devices_num;//�豸����
	MarkerDetection* marker_detection;
	MEASUREMENT_SYSTEM_PARAMS measurement_system_params;
	std::string all_tool_type[MAX_TOOL_TYPE_NUM] = {""};

	/*ch:״̬ | en:Status*/
	bool  m_bOpenDevice;                        // ch:�Ƿ���豸 | en:Whether to open device
	bool  m_bStartGrabbing;                     // ch:�Ƿ�ʼץͼ | en:Whether to start grabbing
	int   m_nTriggerMode;                       // ch:����ģʽ | en:Trigger Mode
	int   m_bContinueStarted;                      // ����������ɼ�ͼ��
	MV_SAVE_IAMGE_TYPE   m_nSaveImageType;      // ch:����ͼ���ʽ | en:Save Image Type

	// �ڶ�ֻ����Ҫ��ʼ���ı���
	unsigned short marker_id = 0;
	bool is_detection = false;
	bool is_recongnization = false;
	bool is_global_detection = true;
	std::string current_tool_type = "";

/* �ۺ�������*/
private slots:

	void OnBnClickedEnumButton();              // ch:���²����豸��ť:ö��  
	void OnBnClickedOpenButton();               // ch:���豸 | en:Open device
	void OnBnClickedCloseButton();              // ch:�ر��豸 | en:Close Devices

	void displayImage();                        // ch:Qlable ��ʾͼ��
	void displayLeftImage(const cv::Mat* imagePrt, int cameraIndex);
	void displayRightImage(const cv::Mat* imagePrt, int cameraIndex);


	/*ch:ͼ��ɼ� | en:Image Acquisition*/
	void OnBnClickedContinusModeRadio();        // ch:����ģʽ | en:Continus Mode
	void OnBnClickedTriggerModeRadio();         // ch:����ģʽ | en:Trigger Mode
	void OnBnClickedStartGrabbingButton();      // ch:��ʼ�ɼ� | en:Start Grabbing
	void OnBnClickedStopGrabbingButton();       // ch:�����ɼ� | en:Stop Grabbing
	void OnBnClickedSoftwareOnceButton();       // ch:�����һ�� | en:Software Trigger Execute Once


	void OnBnClickedStartCornerRecongnization();
	void OnBnClickedStartPoseRecongnization();
	void OnBnClickedStartGlobalRecongnization();

	/*ch:ͼ�񱣴� | en:Image Save*/
	void OnBnClickedSaveJpgButton();            // ch:����jpg | en:Save jpg
	/*ch:ͼƬ���� | en:Save Image*/
	void SaveImage();                     // ch:����ͼƬ | en:Save Image

private:
	void OpenDevices();                    // ch:���豸 | en:Open device
	void CloseDevices();                   // ch:�ر��豸 | en:Close Device
	bool isExists(std::string);
	void showResult(cv::Mat& leftSrc, cv::Mat& rightSrc,
		const Eigen::Matrix4d& Tcam_marker, const std::vector<PATTERN_CONTAINER>& left_marker_decal,
		const std::vector<PATTERN_CONTAINER>& right_marker_decal, const float residue);
	
	bool alignCameras(STEREO_CAMERA_INITIAL_PARAMS initial_camera);

	Eigen::Matrix3d getRotation(double,double,double);
	// �����ļ���ȡ
	bool readConnectionInfo();
	bool readAllToolType();
	bool readToolParams();
	bool readCameraParams();
	bool readMarkerParams();
	bool readMovementParams(std::string func_path);
	bool readOtherinfo();

public:
	//MyThread* myThread; //�̶߳���
	CameraThread* m_thread_leftcamera = NULL;  //������̶߳���
	CameraThread* m_thread_rightcamera = NULL; //������̶߳���
};

