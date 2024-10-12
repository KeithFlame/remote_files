#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
//工程目录下的temp文件夹里，与.sln同级的temp文件
string writePath_left = "./left/";
string writePath_right = "./right/";
int main(int argc, char** argv) {
	//return 0;

	 // 指定文件路径
	std::string filePath = "./resolution.log";

	// 创建 ifstream 对象并打开文件
	std::ifstream inputFile(filePath);
	int number1,number2;
	if (inputFile.is_open()) {
		// 读取第一行数据
		
		inputFile >> number1;
		std::cout << "Number 1: " << number1 << std::endl;

		// 读取第二行数据
		inputFile >> number2;
		std::cout << "Number 2: " << number2 << std::endl;

		// 关闭文件
		inputFile.close();
	}
	else {
		std::cout << "Failed to open file: " << filePath << std::endl;
	}


	VideoCapture capture_left(0);
	VideoCapture capture_right(1);
	std::cout << "cap1 loaded,cap2 loading ..." << capture_left.isOpened() << std::endl;
	std::cout << "cap2 loaded." << capture_right.isOpened() << std::endl;

	cv::Size size(number1, number2);
	capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
	capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
	capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
	capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);

	capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
	capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
	capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
	capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);
	string name_left,name_right;
	namedWindow("hello");
	int i = 0;
	std::string str;
	Mat frame_left;
	Mat frame_right;
	bool rc_left, rc_right;
	while (1) {
		
		capture_left >> frame_left;
		capture_right >> frame_right;
		if (32 == waitKey(2)) {			//空格拍照
			if (i < 10)
				str = "M000" + to_string(i);
			else if(i<100)
				str = "M00" + to_string(i);
			else if(i<1000)
				str = "M0" + to_string(i);
			name_left = writePath_left + str + ".jpg";
			name_right = writePath_right + str + ".jpg";
			rc_left = imwrite(name_left, frame_left);
			rc_right = imwrite(name_right, frame_right);
			cout <<  rc_left<<'\t'<< rc_right << endl;
			i++;
		}
		if ('q' == waitKey(1)) {			//'q'退出
			break;
		}
		cv::Size dsiz(1200, 450);
		cv::Mat result;
		cv::hconcat(frame_left, frame_right, result);
		cv::resize(result, result,dsiz);
		imshow("hello", result);

	}
	//waitKey(0);
	return 0;
}