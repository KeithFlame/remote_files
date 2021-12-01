#pragma once

#ifndef TRACKER__

#define TRACKER__



#include <opencv2/highgui/highgui.hpp>  

#include <opencv2/imgproc/imgproc.hpp>  

#include <opencv2/core/core.hpp>  

#include <opencv2/opencv.hpp>



#include <opencv2\highgui\highgui.hpp>

#include <opencv2\features2d\features2d.hpp>



//#include "rectifier.h"



#include<iostream>

#include<math.h>

#include <string> 

#include<fstream>



class Tracker

{

public:

	Tracker(int i = 0);

	~Tracker();

	void getCurrentFrame();

	cv::Mat curFrame;

	std::vector<cv::Point2f> frameCorners;

private:

	//std::vector<cv::Point2f> frameCorners;

	cv::VideoCapture cap;

};

#endif // !TRACKER__