#include <iostream>
#include <opencv.hpp>

#include<opencv.hpp>

int main()
{
	cv::VideoCapture cap;
	cv::Mat frame;
	cap.open(0, cv::CAP_MSMF);
	if (!cap.isOpened())
	{
		std::cout << "CAMERA OPEN ERROR!!" << std::endl;
		return 0;
	}
	cap >> frame;
	imwrite("asd.jpg", frame);

	return 0;
}

//int main()
//{
//	char data[4] = { 1070,41,42 };
//	for (size_t i = 0; i < sizeof(data) / sizeof(char); i++)
//		std::cout <<"data: " << int(data[i]) << std::endl;
//	for (auto i:data)
//		std::cout << "data: " << int(i) << std::endl;
//	return 0;
//}