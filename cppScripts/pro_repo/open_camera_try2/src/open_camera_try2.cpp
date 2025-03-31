#include <QWidget>
#include <QValidator>

#include <algorithm>
#include <direct.h>
#include <io.h>

#include "open_camera_try2.h"
#include "timer.h"

#define TRIGGER_SOURCE  7
#define TRIGGER_ON      1
#define TRIGGER_OFF     0
#define START_GRABBING_ON   1
#define START_GRABBING_OFF  0
#define IMAGE_NAME_LEN          256

CameraManipulation::CameraManipulation(QWidget* parent) : QMainWindow(parent),
devices_num(0){
	ui.setupUi(this);

	/*����ʹ�ܳ�ʼ��*/
	ui.bntEnumDevices->setEnabled(true);
	ui.bntCloseDevices->setEnabled(false);
	ui.bntOpenDevices->setEnabled(false);
	ui.rbnt_Continue_Mode->setEnabled(false);
	ui.rbnt_SoftTigger_Mode->setEnabled(false);
	ui.bntStartGrabbing->setEnabled(false);
	ui.bntStopGrabbing->setEnabled(false);
	ui.bntSoftwareOnce->setEnabled(false);
	ui.bntSave_JPG->setEnabled(false);

	// �̶߳���ʵ����
	m_thread_leftcamera = new CameraThread;  //������̶߳���
	m_thread_rightcamera = new CameraThread; //������̶߳���

	// ͼ��ָ��ʵ����
	image_L = new cv::Mat();    // ͼ��ָ��ʵ����
	image_R = new cv::Mat();    // ͼ��ָ��ʵ���� 

	// ʱ��
	display_timer = new QTimer(this);

	// ����ʼ��
	marker_detection = new MarkerDetection();
	// ��ʼ������
	devices_num = 0;
	m_nTriggerMode = TRIGGER_ON;
	m_bStartGrabbing = START_GRABBING_ON;
	m_bContinueStarted = 0;
	m_nSaveImageType = MV_Image_Bmp;

	for (unsigned int i = 0; i < MAX_DEVICE_NUM; i++)
	{
		m_stereocamera[i] = new CMvCamera;
		// ��������ʼ��
		m_stereocamera[i]->m_pBufForDriver = NULL;
		m_stereocamera[i]->m_nBufSizeForDriver = 0;
		m_stereocamera[i]->m_nBufSizeForSaveImage = 0;
	}

	// �����ʼ��
	connect(ui.bntEnumDevices, SIGNAL(clicked()), this, SLOT(OnBnClickedEnumButton()));
	connect(ui.bntOpenDevices, SIGNAL(clicked()), this, SLOT(OnBnClickedOpenButton()));
	connect(ui.bntCloseDevices, SIGNAL(clicked()), this, SLOT(OnBnClickedCloseButton()));
	// ͼ��ɼ�
	connect(ui.rbnt_Continue_Mode, SIGNAL(clicked()), this, SLOT(OnBnClickedContinusModeRadio()));
	connect(ui.rbnt_SoftTigger_Mode, SIGNAL(clicked()), this, SLOT(OnBnClickedTriggerModeRadio()));
	connect(ui.bntStartGrabbing, SIGNAL(clicked()), this, SLOT(OnBnClickedStartGrabbingButton()));
	connect(ui.bntStopGrabbing, SIGNAL(clicked()), this, SLOT(OnBnClickedStopGrabbingButton()));
	connect(ui.bntSoftwareOnce, SIGNAL(clicked()), this, SLOT(OnBnClickedSoftwareOnceButton()));

	connect(ui.bntSave_JPG, SIGNAL(clicked()), this, SLOT(OnBnClickedSaveJpgButton()));

	// ͼ����ʾ
	connect(display_timer, SIGNAL(timeout()), this, SLOT(displayImage()));


	// ʶ��ؼ�
	connect(ui.bntCornerRecongnization,SIGNAL(clicked()),this,SLOT(OnBnClickedStartCornerRecongnization()));
	connect(ui.bntPoseRecongnization, SIGNAL(clicked()), this, SLOT(OnBnClickedStartPoseRecongnization()));
	connect(ui.bntGlobalRecongnization, SIGNAL(clicked()), this, SLOT(OnBnClickedStartGlobalRecongnization()));

	// ���ݳ�ʼ��
	std::fill(all_tool_type, all_tool_type + MAX_TOOL_TYPE_NUM, "0");
	readAllToolType();
	readCameraParams();
	readMarkerParams();
	readOtherinfo();
	marker_detection->measurement_params = measurement_system_params;
	for (size_t i = 0; i < 8; i++)
	{
		std::cout << measurement_system_params.marker.marker_decal_id[i] << std::endl;
		std::cout << measurement_system_params.marker.matrixALL[i] << std::endl;

	}
	//memset((char*)&measurement_system_params, 0, sizeof(measurement_system_params));

}


/*************************************************** ����ۺ��� *************************************************** */
// ch:���²����豸��ť:ö�� | en:Click Find Device button:Enumeration 
void CameraManipulation::OnBnClickedEnumButton(){
	memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));    // ch:��ʼ���豸��Ϣ�б�  
	int nRet = MV_OK;
	nRet = CMvCamera::EnumDevices(MV_USB_DEVICE, &m_stDevList);             // ch:ö�������������豸,����豸����

	devices_num = m_stDevList.nDeviceNum;
	if (devices_num > 0)
	{
		ui.bntOpenDevices->setEnabled(true);// �豸������0��ʹ�ܴ��豸����
	}
}

// �����,�������
void CameraManipulation::OpenDevices(){
	int nRet = MV_OK;
	std::string left_or_right = (char *)m_stDevList.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chUserDefinedName;
	if ("0" == left_or_right)
	{
		m_stereocamera[0]->m_nTLayerType = m_stDevList.pDeviceInfo[0]->nTLayerType;
		nRet = m_stereocamera[0]->Open(m_stDevList.pDeviceInfo[0]);
		m_stereocamera[1]->m_nTLayerType = m_stDevList.pDeviceInfo[1]->nTLayerType;
		nRet = m_stereocamera[1]->Open(m_stDevList.pDeviceInfo[1]);
	}
	else
	{
		m_stereocamera[0]->m_nTLayerType = m_stDevList.pDeviceInfo[1]->nTLayerType;
		nRet = m_stereocamera[0]->Open(m_stDevList.pDeviceInfo[1]);
		m_stereocamera[1]->m_nTLayerType = m_stDevList.pDeviceInfo[0]->nTLayerType;
		nRet = m_stereocamera[1]->Open(m_stDevList.pDeviceInfo[0]);
	}
	m_stereocamera[0]->setTriggerMode(TRIGGER_ON);
	m_stereocamera[0]->setTriggerSource(TRIGGER_SOURCE);
	m_stereocamera[1]->setTriggerMode(TRIGGER_ON);
	m_stereocamera[1]->setTriggerSource(TRIGGER_SOURCE);
	std::cout << m_stDevList.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.nbcdUSB << std::endl;

}

void CameraManipulation::OnBnClickedOpenButton(){
	// ʹ�� "��ʼ�ɼ�" ����
	ui.bntOpenDevices->setEnabled(false);
	ui.bntCloseDevices->setEnabled(true);
	ui.rbnt_Continue_Mode->setEnabled(true);
	ui.rbnt_SoftTigger_Mode->setEnabled(true);

	ui.rbnt_Continue_Mode->setCheckable(true);
	OpenDevices();
}

// ch:�ر��豸 | en:Close Device
void CameraManipulation::CloseDevices(){
	// �ر��̡߳����
	if (m_thread_leftcamera->isRunning())
	{
		m_thread_leftcamera->requestInterruption();
		m_thread_leftcamera->wait();
		m_stereocamera[0]->StopGrabbing();

	}

	if (m_thread_rightcamera->isRunning())
	{
		m_thread_rightcamera->requestInterruption();
		m_thread_rightcamera->wait();
		m_stereocamera[1]->StopGrabbing();
	}
	m_stereocamera[0]->Close();
	m_stereocamera[1]->Close();
	

	// ch:�ر�֮����ö��һ�� | en:Enumerate after close
	memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));    // ch:��ʼ���豸��Ϣ�б�  
	int devices_num = MV_OK;
	devices_num = CMvCamera::EnumDevices(MV_USB_DEVICE, &m_stDevList);           // ch:ö�������������豸,����豸����
}

// ch:���¹ر��豸��ť���ر��豸 ,�������پ��| en:Click Close button: Close Device
void CameraManipulation::OnBnClickedCloseButton(){
	ui.bntOpenDevices->setEnabled(true);
	ui.bntCloseDevices->setEnabled(false);
	// ͼ��ɼ��ؼ�
	ui.rbnt_Continue_Mode->setEnabled(false);
	ui.rbnt_SoftTigger_Mode->setEnabled(false);
	ui.bntStartGrabbing->setEnabled(false);
	ui.bntStopGrabbing->setEnabled(false);
	// ����ͼ��ؼ�
	ui.bntSave_JPG->setEnabled(false);
	// �ر��豸�������߳�
	CloseDevices();
}

// ��ʼ�����ɼ�ͼ��
void CameraManipulation::OnBnClickedStartGrabbingButton(){
	m_bContinueStarted = 1; // Ϊ����ģʽ���һ�£��л�����ģʽʱ��ִ��ֹͣ�ɼ�ͼ����

	// ͼ��ɼ��ؼ�
	ui.bntStartGrabbing->setEnabled(false);
	ui.bntStopGrabbing->setEnabled(true);
	// ����ͼ��ؼ�
	ui.bntSave_JPG->setEnabled(true);

	int camera_Index = 0;

	// ���ж�ʲôģʽ�����ж��Ƿ����ڲɼ�
	if (m_nTriggerMode == TRIGGER_ON){
		// ch:��ʼ�ɼ�֮��Ŵ���workthread�߳� | en:Create workthread after start grabbing
		for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++){
			//��������ɼ�
			m_stereocamera[i]->StartGrabbing();

			camera_Index = i;
			if (camera_Index == 0){
				m_thread_leftcamera->getCameraPtr(m_stereocamera[0]); //�̻߳�ȡ�����ָ��
				m_thread_leftcamera->getImagePtr(image_L);  //�̻߳�ȡ��ͼ��ָ��
				m_thread_leftcamera->setCameraSymbol(0); //�����ǵ� ==0

				if (!m_thread_leftcamera->isRunning()){
					m_thread_leftcamera->start();
					m_stereocamera[0]->softTrigger();
					m_stereocamera[0]->ReadBuffer(*image_L);//��ȡMat��ʽ��ͼ��
				}

			}

			if (camera_Index == 1){
				m_thread_rightcamera->getCameraPtr(m_stereocamera[1]); //�̻߳�ȡ�����ָ��
				m_thread_rightcamera->getImagePtr(image_R);   //�̻߳�ȡ��ͼ��ָ��
				m_thread_rightcamera->setCameraSymbol(0); //����� Index==1

				if (!m_thread_rightcamera->isRunning()){
					m_thread_rightcamera->start();
					m_stereocamera[1]->softTrigger();
					m_stereocamera[1]->ReadBuffer(*image_R);//��ȡMat��ʽ��ͼ��
				}
			}
		}
		display_timer->start(30);
	}
}

// ch:���½����ɼ���ť | en:Click Stop button
void CameraManipulation::OnBnClickedStopGrabbingButton()
{
	ui.bntStartGrabbing->setEnabled(true);
	ui.bntStopGrabbing->setEnabled(false);
	for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++){
		//�ر����
		if (m_thread_leftcamera->isRunning()){
			m_stereocamera[0]->StopGrabbing();
			m_thread_leftcamera->requestInterruption();
			m_thread_leftcamera->wait();
		}
		if (m_thread_rightcamera->isRunning()){
			m_stereocamera[1]->StopGrabbing();
			m_thread_rightcamera->requestInterruption();
			m_thread_rightcamera->wait();
		}
	}
}

void CameraManipulation::displayImage(){
	std::vector<PATTERN_CONTAINER> left_marker_decal, right_marker_decal;
	Eigen::Matrix4d Tmarker;
	cv::Rect rect = cv::Rect(0, 0, measurement_system_params.camera.initial_camera_params.image_width,
		measurement_system_params.camera.initial_camera_params.image_height);
	static cv::Rect left_roi = rect;
	static cv::Rect	right_roi = rect;
	float residue = 0.0;
	float ratio = (float)measurement_system_params.other_params.figure_scale.regular_ratio;
	m_thread_leftcamera->setCameraSymbol(is_detection);
	m_thread_rightcamera->setCameraSymbol(is_detection);
	//auto tic = mmath::timer::getCurrentTimePoint();
	cv::Mat* left_image_port;
	cv::Mat* right_image_port;
	int iter = 0;
	int flag;
	while (left_marker_decal.size() == 0 || right_marker_decal.size() == 0)
	{
		left_image_port = m_thread_leftcamera->getFigure(left_marker_decal);
		right_image_port = m_thread_rightcamera->getFigure(right_marker_decal);
		if (!is_detection) {
			flag = 1;
			break;
		}
			
		if (iter++ > 3){
			flag = 2;
			break;
		}
		QThread::msleep(1);
		flag = 0;
	}
	//float t1 = mmath::timer::getDurationSince(tic);
	//std::cout << "duration: " << flag << "\t" <<t1<< std::endl;
	if (is_recongnization){
		bool rc = marker_detection->getPoseFromFrames(is_global_detection, left_marker_decal, right_marker_decal,
			left_roi, right_roi, Tmarker, residue);
		if (!rc){
			Tmarker = Eigen::Matrix4d::Identity();
			Tmarker(2, 3) = 300.0;
			residue = 100.0;
		}
		else {
			showResult(*left_image_port, *right_image_port,
				Tmarker, left_marker_decal, right_marker_decal, residue);
		}
		if (is_global_detection){
			left_roi = rect;
			right_roi = rect;
		}

		m_thread_leftcamera->setDetectionROI(left_roi);
		m_thread_rightcamera->setDetectionROI(right_roi);
		std::cout << left_roi << "\t" << right_roi << std::endl;

		
	}
	displayLeftImage(left_image_port, 0);
	displayRightImage(right_image_port, 0);
}

void CameraManipulation::displayLeftImage(const Mat * imagePrt, int cameraIndex){
	cv::Mat rgb;
	cv::cvtColor(*imagePrt, rgb, CV_BGR2RGB);

	//�ж��Ǻڰס���ɫͼ��
	QImage QmyImage_L = QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, QImage::Format_RGB888);
	QmyImage_L = (QmyImage_L).scaled(ui.label_camera_display->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	//��ʾͼ��
	ui.label_camera_display->setPixmap(QPixmap::fromImage(QmyImage_L));
}


void CameraManipulation::displayRightImage(const Mat * imagePrt, int cameraIndex){
	cv::Mat rgb;
	cv::cvtColor(*imagePrt, rgb, CV_BGR2RGB);

	QImage 	QmyImage_R = QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, QImage::Format_RGB888);
	QmyImage_R = (QmyImage_R).scaled(ui.label_camera_display_right->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	//��ʾͼ��
	ui.label_camera_display_right->setPixmap(QPixmap::fromImage(QmyImage_R));
}


// ch:��������ģʽ��ť | en:Click Continues button
void CameraManipulation::OnBnClickedContinusModeRadio(){
	ui.bntStartGrabbing->setEnabled(true);
	m_nTriggerMode = TRIGGER_ON;
}

// ch:���´���ģʽ��ť | en:Click Trigger Mode button
void CameraManipulation::OnBnClickedTriggerModeRadio(){
	// �������ɼ�ģʽ�Ѿ����ڲɼ���״̬�л�����
	if (m_bContinueStarted == 1) {
		OnBnClickedStopGrabbingButton();//��ִ��ֹͣ�ɼ�
	}

	ui.bntStartGrabbing->setEnabled(false);
	ui.bntSoftwareOnce->setEnabled(true);

	m_nTriggerMode = TRIGGER_OFF;
	for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++){
		m_stereocamera[i]->setTriggerMode(m_nTriggerMode);
	}
}

// ch:���������һ�ΰ�ť | en:Click Execute button
void CameraManipulation::OnBnClickedSoftwareOnceButton(){
	// ����ͼ��ؼ�
	ui.bntSave_JPG->setEnabled(true);

	if (m_nTriggerMode == TRIGGER_OFF){
		int nRet = MV_OK;
		for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++){
			//��������ɼ�
			m_stereocamera[i]->StartGrabbing();

			if (i == 0){
				nRet = m_stereocamera[i]->CommandExecute("TriggerSoftware");
				m_stereocamera[i]->ReadBuffer(*image_L);
				displayLeftImage(image_L, i);//�����ͼ��
			}
			if (i == 1){			//��������ɼ�
				nRet = m_stereocamera[i]->CommandExecute("TriggerSoftware");
				m_stereocamera[i]->ReadBuffer(*image_R);
				displayRightImage(image_R, i);
			}
		}
	}
}

void CameraManipulation::OnBnClickedStartCornerRecongnization(){
	is_detection = !is_detection;
}

void CameraManipulation::OnBnClickedStartPoseRecongnization()
{
	is_recongnization = !is_recongnization;
}

void CameraManipulation::OnBnClickedStartGlobalRecongnization(){
	is_global_detection = !is_global_detection;
}

// ch:���±���jpgͼƬ��ť | en:Click Save JPG button
void CameraManipulation::OnBnClickedSaveJpgButton(){
	m_nSaveImageType = MV_Image_Jpeg;
	std::string file_path = ui.pathEdit->text().toStdString();
	isExists(file_path);
	SaveImage();
}

// ch:����ͼƬ | en:Save Image
void CameraManipulation::SaveImage(){
	// ch:��ȡ1��ͼ | en:Get one frame
	static unsigned int frame_num = 1;
	MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	unsigned int nDataLen = 0;
	int nRet = MV_OK;
	for (int i = 0; i < devices_num; i++){
		// ch:���ڵ�һ�α���ͼ��ʱ���뻺�棬�� CloseDevice ʱ�ͷ�
		// en:Request buffer first time save image, release after CloseDevice
		if (NULL == m_stereocamera[i]->m_pBufForDriver){
			unsigned int nRecvBufSize = 0;
			unsigned int nRet = m_stereocamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);

			m_stereocamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // һ֡���ݴ�С
			m_stereocamera[i]->m_pBufForDriver = (unsigned char*)malloc(m_stereocamera[i]->m_nBufSizeForDriver);
		}

		nRet = m_stereocamera[i]->GetOneFrameTimeout(m_stereocamera[i]->m_pBufForDriver, &nDataLen, m_stereocamera[i]->m_nBufSizeForDriver, &stImageInfo, 1000);
		if (MV_OK == nRet){
			// ch:���ڵ�һ�α���ͼ��ʱ���뻺�棬�� CloseDevice ʱ�ͷ�
			// en:Request buffer first time save image, release after CloseDevice
			if (NULL == m_stereocamera[i]->m_pBufForSaveImage){
				// ch:BMPͼƬ��С��width * height * 3 + 2048(Ԥ��BMPͷ��С)
				// en:BMP image size: width * height * 3 + 2048 (Reserved BMP header size)
				m_stereocamera[i]->m_nBufSizeForSaveImage = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
			}
			// ch:���ö�Ӧ��������� | en:Set camera parameter
			MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
			stParam.enImageType = m_nSaveImageType; // ch:��Ҫ�����ͼ������ | en:Image format to save;
			stParam.enPixelType = stImageInfo.enPixelType;  // �����Ӧ�����ظ�ʽ | en:Pixel format
			stParam.nBufferSize = m_stereocamera[i]->m_nBufSizeForSaveImage;  // �洢�ڵ�Ĵ�С | en:Buffer node size
			stParam.nWidth = stImageInfo.nWidth;         // �����Ӧ�Ŀ� | en:Width
			stParam.nHeight = stImageInfo.nHeight;          // �����Ӧ�ĸ� | en:Height
			stParam.nDataLen = stImageInfo.nFrameLen;
			stParam.pData = m_stereocamera[i]->m_pBufForDriver;
			stParam.pImageBuffer = m_stereocamera[i]->m_pBufForSaveImage;
			stParam.nJpgQuality = 90;       // ch:jpg���룬���ڱ���Jpgͼ��ʱ��Ч������BMPʱSDK�ں��Ըò���

			nRet = m_stereocamera[i]->SaveImage(&stParam);

			char chImageName[IMAGE_NAME_LEN] = { 0 };
			if (MV_Image_Jpeg == stParam.enImageType){
				if (i == 0)
					sprintf_s(chImageName, IMAGE_NAME_LEN, "./%s/left_%03d.jpg", ui.pathEdit->text().toStdString().c_str(),frame_num);
				if (i == 1)
					sprintf_s(chImageName, IMAGE_NAME_LEN, "./%s/right_%03d.jpg", ui.pathEdit->text().toStdString().c_str(), frame_num);			
			}

			FILE* fp = fopen(chImageName, "wb");
			fwrite(m_stereocamera[i]->m_pBufForSaveImage, 1, stParam.nImageLen, fp);
			fclose(fp);
		}
	}
	frame_num++;
}

bool CameraManipulation::isExists(std::string file_path){
	std::string dir = "./"+ file_path; //�ļ���·��
	if (access(dir.c_str(), 0) == -1) { //�жϸ��ļ����Ƿ����
#ifdef WIN32
		int flag = mkdir(dir.c_str());  //Windows�����ļ���
#else
		int flag = mkdir(dir.c_str(), S_IRWXU);  //Linux�����ļ���
#endif
		if (flag == 0) {  //�����ɹ�
			//std::cout << "Create directory successfully." << std::endl;
			return true;
		}
		else { //����ʧ��
			//std::cout << "Fail to create directory." << std::endl;
			//throw std::exception();
			return false;
		}
	}
	else {
		return true;
	}
}

void CameraManipulation::showResult(cv::Mat& leftSrc, cv::Mat& rightSrc,
	const Eigen::Matrix4d& Tcam_marker, const std::vector<PATTERN_CONTAINER>& left_marker_decal,
	const std::vector<PATTERN_CONTAINER>& right_marker_decal, const float residue) {
	/*camera parameters*/
	Eigen::Matrix3d A_cam = measurement_system_params.camera.rectified_camera_params.A_cam;
	Eigen::Matrix3d left_camera_intrinsic = measurement_system_params.camera.initial_camera_params.left_camera_intrinsic;
	Eigen::Matrix3d right_camera_intrinsic = measurement_system_params.camera.initial_camera_params.right_camera_intrinsic;
	Eigen::Matrix3d rect_left_camera = measurement_system_params.camera.rectified_camera_params.rectified_left_camera_rotation;
	Eigen::Matrix3d rect_right_camera = measurement_system_params.camera.rectified_camera_params.rectified_right_camera_rotation;
	Eigen::Vector4d left_camera_distortion = measurement_system_params.camera.initial_camera_params.left_camera_distortion;
	Eigen::Vector4d right_camera_distortion = measurement_system_params.camera.initial_camera_params.right_camera_distortion;

	const cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
		1., 0., 0., 0.,
		0., 1., 0., 0.,
		0., 0., 1., 0.);
	const cv::Mat cameraMatrixl = (cv::Mat_<float>(3, 3) <<
		A_cam(0, 0), A_cam(0, 1), A_cam(0, 2),
		A_cam(1, 0), A_cam(1, 1), A_cam(1, 2),
		A_cam(2, 0), A_cam(2, 1), A_cam(2, 2));;
	const cv::Mat cameraMatrixr = cameraMatrixl;
	const cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
		1.0, 0.0, 0.0, -measurement_system_params.camera.rectified_camera_params.b_dis,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0);
	int lineThickness = 15;

	cv::Mat proMl(3, 4, CV_32F), proMr(3, 4, CV_32F);
	proMl = cameraMatrixl * T1;
	proMr = cameraMatrixr * T2;
		// 3D points
	float l0 = 30;  // length from point0 to pointx
	cv::Point3f point0(Tcam_marker(0, 3), Tcam_marker(1, 3), Tcam_marker(2, 3));
	cv::Point3f pointx = point0 + l0 * cv::Point3f(Tcam_marker(0, 0), Tcam_marker(1, 0), Tcam_marker(2, 0));
	cv::Point3f pointy = point0 + l0 * cv::Point3f(Tcam_marker(0, 1), Tcam_marker(1, 1), Tcam_marker(2, 1));
	cv::Point3f pointz = point0 + l0 * cv::Point3f(Tcam_marker(0, 2), Tcam_marker(1, 2), Tcam_marker(2, 2));

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
	float ratio = 1.f;// (float)measurement_system_params.other_params.figure_scale.regular_ratio;
	point0_l_2d = ratio * marker_detection->disRectifyOneCorner(point0_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);
	pointx_l_2d = ratio * marker_detection->disRectifyOneCorner(pointx_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);
	pointy_l_2d = ratio * marker_detection->disRectifyOneCorner(pointy_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);
	pointz_l_2d = ratio * marker_detection->disRectifyOneCorner(pointz_l_2d, A_cam, left_camera_intrinsic, rect_left_camera, left_camera_distortion);

	point0_r_2d = ratio * marker_detection->disRectifyOneCorner(point0_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);
	pointx_r_2d = ratio * marker_detection->disRectifyOneCorner(pointx_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);
	pointy_r_2d = ratio * marker_detection->disRectifyOneCorner(pointy_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);
	pointz_r_2d = ratio * marker_detection->disRectifyOneCorner(pointz_r_2d, A_cam, right_camera_intrinsic, rect_right_camera, right_camera_distortion);

	cv::line(leftSrc, point0_l_2d, pointx_l_2d, cv::Scalar(0, 0, 255), lineThickness);
	cv::line(leftSrc, point0_l_2d, pointy_l_2d, cv::Scalar(0, 255, 0), lineThickness);
	cv::line(leftSrc, point0_l_2d, pointz_l_2d, cv::Scalar(255, 0, 0), lineThickness);
	cv::line(rightSrc, point0_r_2d, pointx_r_2d, cv::Scalar(0, 0, 255), lineThickness);
	cv::line(rightSrc, point0_r_2d, pointy_r_2d, cv::Scalar(0, 255, 0), lineThickness);
	cv::line(rightSrc, point0_r_2d, pointz_r_2d, cv::Scalar(255, 0, 0), lineThickness);
	cv::Scalar color_arr[5] = { cv::Scalar(0, 0, 255),
	cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),cv::Scalar(0, 255, 255), cv::Scalar(255, 255, 0)};

	for (size_t i = 0; i < left_marker_decal.size(); i++) {
		cv::circle(leftSrc, left_marker_decal[i].figure_coordinations[0], 4, color_arr[0], -1);
		cv::circle(leftSrc, left_marker_decal[i].figure_coordinations[1], 4, color_arr[1], -1);
		cv::circle(leftSrc, left_marker_decal[i].figure_coordinations[2], 4, color_arr[2], -1);
		cv::circle(leftSrc, left_marker_decal[i].figure_coordinations[3], 4, color_arr[3], -1);
		cv::circle(leftSrc, left_marker_decal[i].figure_coordinations[4], 4, color_arr[4], -1);
		cv::circle(rightSrc, right_marker_decal[i].figure_coordinations[0], 4, color_arr[0], -1);
		cv::circle(rightSrc, right_marker_decal[i].figure_coordinations[1], 4, color_arr[1], -1);
		cv::circle(rightSrc, right_marker_decal[i].figure_coordinations[2], 4, color_arr[2], -1);
		cv::circle(rightSrc, right_marker_decal[i].figure_coordinations[3], 4, color_arr[3], -1);
		cv::circle(rightSrc, right_marker_decal[i].figure_coordinations[4], 4, color_arr[4], -1);
		for (int j = 5; j < MAX_MARKER_SIZE; j++) {
			cv::circle(leftSrc, ratio * left_marker_decal[i].figure_coordinations[i], 5, color_arr[4], -1);
			cv::circle(rightSrc, ratio * right_marker_decal[i].figure_coordinations[i], 5, color_arr[4], -1);
		}
	}


	char* chCode;
	std::string str2;
	Eigen::AngleAxisd rot_axang;
	Eigen::Matrix3d Rcam_marker;
	Rcam_marker = Tcam_marker.topLeftCorner(3, 3).cast<double>();
	rot_axang.fromRotationMatrix(Rcam_marker);
	Eigen::Vector3d rot_axis;
	rot_axis = rot_axang.axis() * rot_axang.angle();
	chCode = new(std::nothrow)char[200];
	std::sprintf(chCode, "position [%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf, %.2lf]",
		Tcam_marker(0, 3), Tcam_marker(1, 3), Tcam_marker(2, 3)
		, rot_axis(0), rot_axis(1), rot_axis(2), residue);
	str2 = std::string(chCode);
	delete[]chCode;

	cv::putText(leftSrc, str2, cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 4.2, cv::Scalar(64, 187, 208), 2);
	cv::putText(leftSrc, str2, cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 4.2, cv::Scalar(64, 187, 208), 2);
}

bool CameraManipulation::alignCameras(STEREO_CAMERA_INITIAL_PARAMS initial_camera)
{
	auto rotation_right_left = initial_camera.rotation_right2left.transpose();
	Eigen::AngleAxisd r_axang;
	r_axang.fromRotationMatrix(rotation_right_left);
	Eigen::AngleAxisd r_axang_n1 = r_axang, r_axang_n2 = r_axang;
	r_axang_n1.angle() = 0.5 * r_axang.angle();
	r_axang_n2.angle() = -0.5 * r_axang.angle();
	auto rect_left_1 = r_axang_n1.toRotationMatrix();
	auto rect_right_1 = r_axang_n2.toRotationMatrix();

	Eigen::Vector3d x_before = rect_right_1 * initial_camera.translation_right_left;
	Eigen::Vector3d x_after = (x_before(0) > 0 ? 1 : -1) * Eigen::Vector3d(1, 0, 0);

	Eigen::AngleAxisd r_axang2;
	auto rot_axis2 = x_before.cross(x_after);

	auto rot_angle2 = acos(x_before.dot(x_after) / (x_before.norm() * x_after.norm()));
	r_axang2.axis() = rot_axis2;
	r_axang2.angle() = rot_angle2;

	auto r_align = r_axang2.toRotationMatrix();



	Eigen::Vector4d f_vec;
	f_vec << initial_camera.left_camera_intrinsic(0, 0), initial_camera.left_camera_intrinsic(1, 1),
		initial_camera.right_camera_intrinsic(0, 0), initial_camera.right_camera_intrinsic(1, 1);

	float f_comm = f_vec.maxCoeff() * 1.05;
	float cx_comm = 0.5 * (initial_camera.left_camera_intrinsic(0, 2) + initial_camera.right_camera_intrinsic(0, 2));
	float cy_comm = 0.5 * (initial_camera.left_camera_intrinsic(1, 2) + initial_camera.right_camera_intrinsic(1, 2));

	RECTIFIED_STEREO_CAMERA_PARAMS rectified_camera_params;
	rectified_camera_params.rectified_left_camera_rotation = r_align * rect_left_1;
	rectified_camera_params.rectified_right_camera_rotation = r_align * rect_right_1;
	rectified_camera_params.A_cam << f_comm, 0, cx_comm,
		0, f_comm, cy_comm,
		0, 0, 1;//offline before year
	rectified_camera_params.b_dis = initial_camera.translation_right_left.norm();
	rectified_camera_params.image_width = initial_camera.image_width;
	rectified_camera_params.image_height = initial_camera.image_height;
	measurement_system_params.camera.rectified_camera_params = rectified_camera_params;
	return true;
}

Eigen::Matrix3d CameraManipulation::getRotation(double a, double b, double c)
{
	Eigen::Vector3d vec(a,b,c);
	Eigen::AngleAxisd axg;
	double vecn= vec.norm();
	axg.axis() = vec / vecn;
	axg.angle() = CV_PI * vecn / 180;
	return axg.toRotationMatrix();
}

bool CameraManipulation::readConnectionInfo(){
	return false;
}

bool CameraManipulation::readAllToolType(){
	int i = 0;
	std::string tool_list = "./conf/public/tool_list.xml";
	conf_read.LoadFile(tool_list.c_str());
	if (conf_read.Error()){
		std::string str = "ERROR::FAILED_TO_LOAD_FILE "+tool_list;
		return false;
	}

	tinyxml2::XMLElement* root = conf_read.RootElement();
	root = root->FirstChildElement();
	while (root != NULL){
		all_tool_type[i] = root->GetText();
		root = root->NextSiblingElement();
		i++;
	}
	return true;
}

bool CameraManipulation::readToolParams(){
	return false;
}

bool CameraManipulation::readCameraParams(){
	int i = 0;
	std::string tool_list = "./conf/private/camera_params.xml";
	conf_read.LoadFile(tool_list.c_str());
	if (conf_read.Error()) {
		std::string str = "ERROR::FAILED_TO_LOAD_FILE " + tool_list;
		return false;
	}
	tinyxml2::XMLElement* root = conf_read.RootElement();
	std::vector<std::vector<double>> res;
	while (root != NULL){
		tinyxml2::XMLElement* data = root->FirstChildElement();
		std::vector<double> camera;
		while (data != NULL) {
			camera.emplace_back(std::atof(data->GetText()));
			data = data->NextSiblingElement();
			i++;
		}
		root = root->NextSiblingElement();
		res.emplace_back(camera);
	}
	if (4 == res.size() && 8 == res[0].size() && 8 == res[1].size() && 6 == res[2].size() && 2 == res[3].size()){
		Eigen::Matrix3d  r1, r2, r;
		r2 = r1 = Eigen::Matrix3d::Identity();
		Eigen::Vector4d d1, d2;
		Eigen::Vector3d t;
		r1(0, 0) = res[0][0]; r1(0, 2) = res[0][1]; r1(1, 1) = res[0][2]; r1(1, 2) = res[0][3];
		r2(0, 0) = res[1][0]; r2(0, 2) = res[1][1]; r2(1, 1) = res[1][2]; r2(1, 2) = res[1][3];
		d1 << res[0][4], res[0][5], res[0][6], res[0][7];
		d2 << res[1][4], res[1][5], res[1][6], res[1][7];
		t << res[2][0], res[2][1], res[2][2];
		r = getRotation(res[2][3], res[2][4], res[2][5]);

		STEREO_CAMERA_INITIAL_PARAMS initial_camera;
		initial_camera.left_camera_intrinsic = r1;
		initial_camera.left_camera_distortion = d1;
		initial_camera.right_camera_intrinsic = r2;
		initial_camera.right_camera_distortion = d2;
		initial_camera.rotation_right2left = r;
		initial_camera.translation_right_left = t;
		initial_camera.image_width = res[3][0];
		initial_camera.image_height = res[3][1];
		measurement_system_params.camera.initial_camera_params = initial_camera;
		alignCameras(initial_camera);
		return true;
	}
	else{
		std::string str = "ERROR::CAMERA_PARAMS_ARE_ABNORMAL " + tool_list;
		return false;
	}
}

bool CameraManipulation::readMarkerParams()
{
	int i = 0;
	std::string marker_list = "./conf/public/marker_with_8decals.xml";
	conf_read.LoadFile(marker_list.c_str());
	if (conf_read.Error()) {
		std::string str = "ERROR::FAILED_TO_LOAD_FILE " + marker_list;
		return false;
	}

	tinyxml2::XMLElement* root = conf_read.RootElement();
	std::string res;
	PHISICAL_MARKER_POSE one_marker;
	std::vector<PHISICAL_MARKER_POSE> all_marker;
	Eigen::Matrix4d mat= Eigen::Matrix4d::Identity();
	Eigen::Matrix3d rot;
	std::vector<double> pose;
	while (root != NULL) {
		res = root->Attribute("id");
		one_marker.marker_id = (unsigned short)std::atof(res.c_str());
		tinyxml2::XMLElement* decal = root->FirstChildElement();
		while (decal != NULL)
		{
			res = decal->Attribute("id");
			one_marker.marker_decal_id[i] = (unsigned short)std::atof(res.c_str());
			one_marker.pose[i].marker_decal_id = one_marker.marker_decal_id[i];
			tinyxml2::XMLElement* data = decal->FirstChildElement();
			while (data != NULL)
			{
				pose.emplace_back(std::atof(data->GetText()));
				data = data->NextSiblingElement();
			}
			if (pose.size() != 6)
			{
				std::string str = "ERROR::FAILED_TO_LOAD_MARKER_PARAMS!!!";
				return false;
			}
			mat(0, 3) = pose[0]; mat(1, 3) = pose[1]; mat(2, 3) = pose[2];
			rot = getRotation(pose[3], pose[4], pose[5]);
			mat.topLeftCorner(3, 3) = rot;
			one_marker.matrixALL[i] = mat;
			i++;
			pose.clear();
			decal = decal->NextSiblingElement();
		}
		root = root->NextSiblingElement();
		all_marker.emplace_back(one_marker);
		i = 0;
	}

	bool flag = false;
	for (size_t i = 0; i < all_marker.size(); i++)
	{
		if (marker_id == all_marker[i].marker_id)
		{
			measurement_system_params.marker = all_marker[i];
			flag = true;
			break;
		}
	}
	if (!flag)
	{
		std::string str = "ERROR::FAILED_TO_FIND_MARKER!!!";
		return false;
	}

	std::string marker_decal_list = "./conf/public/marker_decal.xml";
	conf_read.LoadFile(marker_decal_list.c_str());
	if (conf_read.Error()) {
		std::string str = "ERROR::FAILED_TO_LOAD_FILE " + marker_list;
		return false;
	}
	i = 0;

	std::vector<int> ID;
	ID.resize(3,0);
	tinyxml2::XMLElement* data;
	for (i = 0; i < MAX_MARKER_SIZE; i++) {
		root = conf_read.RootElement();
		data = root->FirstChildElement();
		for (size_t j = 0; j < 5; j++) {
			measurement_system_params.marker.pose[i].position[j].x = std::atof(data->GetText());
			data = data->NextSiblingElement();
			measurement_system_params.marker.pose[i].position[j].y = std::atof(data->GetText());
			root = root->NextSiblingElement();
			data = root->FirstChildElement();
		}
		unsigned short marker_decal_id = measurement_system_params.marker.marker_decal_id[i];
		int k = 0, cid = 0;
		ID[2] = marker_decal_id % 10;
		marker_decal_id = marker_decal_id / 10;
		ID[1] = marker_decal_id % 10;
		marker_decal_id = marker_decal_id / 10;
		ID[0] = marker_decal_id;
		while (root!=NULL) {
			cid = (int)std::atoi(root->Attribute("id"));
			if (cid == ID[k]){
				data = root->FirstChildElement();
				measurement_system_params.marker.pose[i].position[k+5].x = std::atof(data->GetText());
				data = data->NextSiblingElement();
				measurement_system_params.marker.pose[i].position[k+5].y = std::atof(data->GetText());
				k = k < ID.size()-1 ? k + 1 : k;
			}
			root = root->NextSiblingElement();
		}
	}
	return true;
}

bool CameraManipulation::readMovementParams(std::string func_path)
{
	return false;
}

bool CameraManipulation::readOtherinfo()
{
	int i = 0;
	std::string other_info = "./conf/public/other_params.xml";
	conf_read.LoadFile(other_info.c_str());
	if (conf_read.Error()) {
		std::string str = "ERROR::FAILED_TO_LOAD_FILE " + other_info;
		return false;
	}
	tinyxml2::XMLElement* root = conf_read.RootElement();
	std::vector<std::vector<double>> res;
	while (root != NULL) {
		tinyxml2::XMLElement* data = root->FirstChildElement();
		std::vector<double> other;
		while (data != NULL) {
			other.emplace_back(std::atof(data->GetText()));
			data = data->NextSiblingElement();
			i++;
		}
		root = root->NextSiblingElement();
		res.emplace_back(other);
	}

	if (2 == res.size() && 2 == res[0].size() && 2 == res[1].size()) {
		measurement_system_params.other_params.roi_incremental_ratio.founded_ratio = res[0][0];
		measurement_system_params.other_params.roi_incremental_ratio.unfounded_ratio = res[0][1];
		measurement_system_params.other_params.figure_scale.regular_ratio = res[1][0];
		measurement_system_params.other_params.figure_scale.angle_test_ratio = res[1][1];
	}
	else
		return false;


	return true;
}
