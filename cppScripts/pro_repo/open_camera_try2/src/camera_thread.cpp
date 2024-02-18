#include "camera_thread.h"

#define f_width 5472
#define f_height 3648

CameraThread::CameraThread()
{
	rect = cv::Rect(0,0, f_width, f_height);
	img_detection = new ImageDetection();
	PATTERN_CONTAINER md;
	memset(&md,0,sizeof(md));
	marker_decal.emplace_back(md);
	//marker_decal.clear();
	//std::vector<PATTERN_CONTAINER> marker_decal2;
	//marker_decal = marker_decal2;
}

CameraThread::~CameraThread()
{
	terminate();
	if (cameraPtr != NULL)
	{
		delete cameraPtr;
	}
	if (imagePtr != NULL)
	{
		delete imagePtr;
	}
	if (img_detection != NULL)
	{
		delete img_detection;
	}
}

void CameraThread::getCameraPtr(CMvCamera* camera)
{
	cameraPtr = camera;
}

void CameraThread::getImagePtr(cv::Mat* image)
{
	imagePtr = image;
}

void CameraThread::setCameraSymbol(int flag, bool is_shown)
{
	if (flag == 0)
		is_detected = false;
	else
		is_detected = true;
}

void CameraThread::setDetectionROI(const cv::Rect rect)
{
	this->rect = rect;
}

cv::Mat* CameraThread::getFigure(std::vector<PATTERN_CONTAINER>& marker_decal)
{
	marker_decal = this->marker_decal;
	return imagePtr;
}


void CameraThread::run()
{
	if (cameraPtr == NULL) {
		return;
	}

	if (imagePtr == NULL) {
		return;
	}

	while (!isInterruptionRequested()){
		cameraPtr->softTrigger();
		int x = cameraPtr->ReadBuffer(*imagePtr);
		if (x == 0 && is_detected) {
			marker_decal.clear();
			img_detection->markerIdentify(*imagePtr, rect, marker_decal);
			is_detected = false;
		}

		//msleep(1);
	}

}
