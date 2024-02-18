//
//#include <opencv2\videoio.hpp>
//#include <opencv2\highgui.hpp>
//#include <iostream>
//#include "MvCamera.h"
//int main(int argc, char* argv[])
//{
//
//	CMvCamera mc;
//	MV_CC_DEVICE_INFO_LIST  m_stDevList;
//	MV_FRAME_OUT* pf= new MV_FRAME_OUT;
//	MV_SAVE_IMAGE_PARAM_EX3* frame_save = new MV_SAVE_IMAGE_PARAM_EX3;
//	int nRet = mc.EnumDevices(MV_USB_DEVICE, &m_stDevList);
//	
//	int mRet = mc.Open(m_stDevList.pDeviceInfo[2]);
//	std::cout<< m_stDevList.nDeviceNum <<std::endl;
//	bool rc = mc.IsDeviceConnected();
//	mc.StartGrabbing();
//	mc.GetImageBuffer(pf, 30);
//	mc.StopGrabbing();
//
//	mc.SaveImage(frame_save);
//	
//	//mc.DisplayOneFrame(dfi);
//	//Convert2Mat(frame_out, unsigned char* pData)
//	//mc.SaveImage(frame_save);
//	//{
//	//	return MV_CC_SaveImageEx3(m_hDevHandle, pstParam);
//	//}
//	return 0;
//
//}
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

// OpenCV includes
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
using namespace cv;

int main(int argc, const char** argv)
{
	// 1.������Ƶ�ɼ�����;
	VideoCapture cap;

	// 2.��Ĭ�����;
	cap.open(0);

	// 3.�ж�����Ƿ�򿪳ɹ�;
	if (!cap.isOpened())
		return -1;

	// 4.��ʾ��������;
	namedWindow("Video", 1);
	for (;;)
	{
		// ��ȡ�µ�һ֡;
		Mat frame;
		cap >> frame;
		if (frame.empty())
			return 0;

		// ��ʾ�µ�֡;
		imshow("Video", frame);

		// �����˳���ʾ;
		if (waitKey(30) >= 0) break;
	}

	// 5.�ͷ���Ƶ�ɼ�����;
	cap.release();

	return 0;
}
