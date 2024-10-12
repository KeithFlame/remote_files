#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
//����Ŀ¼�µ�temp�ļ������.slnͬ����temp�ļ�
string writePath_left = "./left/";
string writePath_right = "./right/";
int main(int argc, char** argv) {
	//return 0;

	 // ָ���ļ�·��
	std::string filePath = "./resolution.log";

	// ���� ifstream ���󲢴��ļ�
	std::ifstream inputFile(filePath);
	int number1,number2;
	if (inputFile.is_open()) {
		// ��ȡ��һ������
		
		inputFile >> number1;
		std::cout << "Number 1: " << number1 << std::endl;

		// ��ȡ�ڶ�������
		inputFile >> number2;
		std::cout << "Number 2: " << number2 << std::endl;

		// �ر��ļ�
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
		if (32 == waitKey(2)) {			//�ո�����
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
		if ('q' == waitKey(1)) {			//'q'�˳�
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