#include "tracker.h"

Tracker::Tracker(int i)
{

	cap.open(i);
	cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 2592);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1944);
	cap.set(cv::CAP_PROP_BUFFERSIZE, 3);
	cap.set(cv::CAP_PROP_SHARPNESS, 10);
	
	getCurrentFrame();
}

Tracker::~Tracker()
{
	cap.release();
}

void Tracker::getCurrentFrame()
{
	cv::Mat frame;
	if (cap.isOpened())
	{

		if (cap.read(frame))
		{
			curFrame = frame;
		}


	}
	else
	{
		std::cout << "ERROR::TRACKER::LEFT_CAMERA_NOT_SUCCESFULLY_OPEN" << std::endl;
	}

}