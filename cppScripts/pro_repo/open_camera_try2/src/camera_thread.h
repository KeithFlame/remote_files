#pragma once
#ifndef MYTHREAD_H
#define MYTHREAD_H
#include "QThread"
#include "MvCamera.h"
#include "opencv2/opencv.hpp"
#include <iostream> 
#include "image_detection.h"

using namespace std;

class CameraThread :public QThread
{
	Q_OBJECT

public:
	CameraThread();
	~CameraThread();

	void run();
	void getCameraPtr(CMvCamera* camera);
	void getImagePtr(cv::Mat* image);
	void setCameraSymbol(int flag, bool is_shown = true);
	void setDetectionROI(const cv::Rect rect);
	cv::Mat* getFigure(std::vector<PATTERN_CONTAINER>& marker_decal);

private:
	CMvCamera* cameraPtr = NULL;
	cv::Mat* imagePtr = NULL;
	cv::Rect rect;
	ImageDetection* img_detection;

	std::vector<PATTERN_CONTAINER> marker_decal;
	bool is_detected = false;
	bool is_shown = false;
	int TriggerMode = 0;
};

#endif // MYTHREAD_H
