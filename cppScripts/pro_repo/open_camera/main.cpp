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
	// 1.创建视频采集对象;
	VideoCapture cap;

	// 2.打开默认相机;
	cap.open(0);

	// 3.判断相机是否打开成功;
	if (!cap.isOpened())
		return -1;

	// 4.显示窗口命名;
	namedWindow("Video", 1);
	for (;;)
	{
		// 获取新的一帧;
		Mat frame;
		cap >> frame;
		if (frame.empty())
			return 0;

		// 显示新的帧;
		imshow("Video", frame);

		// 按键退出显示;
		if (waitKey(30) >= 0) break;
	}

	// 5.释放视频采集对象;
	cap.release();

	return 0;
}
