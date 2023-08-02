#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;
//工程目录下的temp文件夹里，与.sln同级的temp文件
string writePath = "./pic1/";
int main(int argc, char** argv) {
	VideoCapture capture(0);
	std::cout << "cap1 loaded,cap2 loading ..." << capture.isOpened() << std::endl;
	cv::Size size(2592, 1944);
	capture.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
	capture.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
	capture.set(cv::CAP_PROP_BUFFERSIZE, 3);
	string name;
	namedWindow("hello");
	int i = 0;
	std::string str;
	while (1) {
		Mat frame;
		capture >> frame;
		if (32 == waitKey(20)) {			//空格拍照
			if (i < 10)
				str = "M000" + to_string(i);
			else if(i<100)
				str = "M00" + to_string(i);
			else if(i<1000)
				str = "M0" + to_string(i);
			name = writePath + str + ".jpg";
			bool rc = imwrite(name, frame);

			cout << name <<'\t'<< rc << endl;
			i++;
		}
		if (97 == waitKey(10)) {			//'a'退出
			break;
		}
		cv::Size dsiz(600, 450);
		cv::resize(frame, frame ,dsiz);
		imshow("hello", frame);

	}
	//waitKey(0);
}