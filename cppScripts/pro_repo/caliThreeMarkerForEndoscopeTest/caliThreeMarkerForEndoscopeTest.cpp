#include<iostream>
//#include "camera_calibration.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tracker.h"
#include "map_calculator.h"

//using namespace std;
//using namespace cv;

//#define BEFORE_CALIBRATION 

//#define LEFT_HALF 
void getConfigV2(std::string leftPath, std::string rightPath, float edgeLength, int width, int height);
void getCaliedPicture(std::string leftPath, std::string rightPath,std::string savePath);
int main() {

#ifdef BEFORE_CALIBRATION
	int numFrame(0);

	std::string saveAddress1, saveAddress2;

	cv::Mat frame1, frame2;
	if (1)
	{

		getCaliedPicture("F:/pictures/endoscope_bending_test/systemTest/left/", "F:/pictures/endoscope_bending_test/systemTest/right/",
			"F:/pictures/endoscope_bending_test/systemTest/");
		//cv::Size frame_size = cv::Size(2592, 1944);
		//
		//MapCalculator MapCalculator1(".", 0);
		//MapCalculator MapCalculator2(".", 1);
		//std::cout << "123" << std::endl;
		//frame1 = cv::imread("F:\\code_git\\matlabScripts\\stereoCalibration\\D1125_1\\left\\left06.jpg");
		//frame2 = cv::imread("F:\\code_git\\matlabScripts\\stereoCalibration\\D1125_1\\right\\right06.jpg");
		//
		//frame1 = MapCalculator1.rectify(frame1);
		//frame2 = MapCalculator2.rectify(frame2);
		//std::cout << frame1.size() << std::endl;
		//cv::resize(frame1, frame1, cv::Size(2592 /2, 1944 / 2));
		//cv::resize(frame2, frame2, cv::Size(2592 / 2, 1944 / 2));
		//std::cout << "123" << std::endl;
		//cv::waitKey(1);
		//cv::imshow("left", frame1);
		//cv::imshow("right", frame2);
		//cv::waitKey();
		return 0;
	}

	Tracker tracker1(0);
	Tracker tracker2(1);
	while (1)

	{



		tracker1.getCurrentFrame();
		tracker1.getCurrentFrame();

		frame1 = tracker1.curFrame;

		frame2 = tracker2.curFrame;

		

		cv::resize(frame1, frame1, cv::Size(tracker1.curFrame.cols / 2, tracker1.curFrame.rows / 2));

		cv::resize(frame2, frame2, cv::Size(tracker1.curFrame.cols / 2, tracker1.curFrame.rows / 2));

		if (frame1.data)

		{

			cv::imshow("left", frame1);

			cv::imshow("right", frame2);

		}

		if (cv::waitKey(60) == 'c')

		{



			numFrame++;

			saveAddress1 = "W:/video/stereo_calibration/calibration_of_keith/NO18/left/calibration_" + std::to_string(numFrame) + ".jpg";

			saveAddress2 = "W:/video/stereo_calibration/calibration_of_keith/NO18/right/calibration_" + std::to_string(numFrame) + ".jpg";

			std::cout << saveAddress1 << std::endl;

			if (cv::imwrite(saveAddress1, tracker1.curFrame) && cv::imwrite(saveAddress2, tracker2.curFrame))

				std::cout << std::endl << "-> Shot it, " << numFrame << " times." << std::endl;

		}



		if (cv::waitKey(30) == 27)

			break;

	}

#else
	std::cout << "123 Begin!" << std::endl;
	getConfigV2("F:/pictures/endoscope_bending_test/systemTest/left_rect/", "F:/pictures/endoscope_bending_test/systemTest/right_rect/", 5.f, 3, 3);
	std::cout << "123 End!" << std::endl;
#endif // BEFORE_CALIBRATION

	return 0;

}

void getConfigV2(std::string leftPath, std::string rightPath, float edgeLength, int width, int height)
{
	std::vector<cv::String> image_file_L;
	std::vector<cv::String> image_file_R;
	std::vector<std::vector<cv::Point2f> > l_image_points, r_image_points;
	std::vector<std::vector<cv::Point3f> > object_points;
	std::vector<cv::Mat> l_images, r_images;
	cv::glob(leftPath, image_file_L);             //��Ŀ¼�µ��ļ���ȡ��������
	cv::glob(rightPath, image_file_R);

	for (int i = 0; i < image_file_L.size(); i++)
	{
		l_images.push_back(cv::imread(image_file_L[i]));
	}

	for (int ii = 0; ii < image_file_R.size(); ii++)
	{
		r_images.push_back(cv::imread(image_file_R[ii]));
	}

	std::vector<cv::Point3f> ob_p;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			ob_p.push_back(cv::Point3f(j * edgeLength, i * edgeLength, 0.f));
		}
	}

	int count = 0;
	for (int i = 0; i < l_images.size(); i++)
	{
		count += 1;
		cv::Mat lim = l_images[i];
		cv::Mat rim = r_images[i];
		cv::Size im_size = lim.size();
#ifdef LEFT_HALF
		
		
		cv::Point p_start = cv::Point(im_size.width / 2,0);
		cv::Point p_end = cv::Point(im_size.width, im_size.height);
#else
		cv::Point p_start = cv::Point(0, 0);
		cv::Point p_end = cv::Point(im_size.width / 2, im_size.height);
#endif
		cv::rectangle(lim, p_start, p_end,cv::Scalar(0,0,0),-1);
		cv::rectangle(rim, p_start, p_end, cv::Scalar(0, 0, 0), -1);

		std::vector<cv::Point2f> l_im_p;
		std::vector<cv::Point2f> r_im_p;
		bool l_pattern_found = findChessboardCorners(lim, cv::Size(width, height), l_im_p, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK); //
		bool r_pattern_found = findChessboardCorners(rim, cv::Size(width, height), r_im_p, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK); //
		if (l_pattern_found && r_pattern_found)
		{
			object_points.push_back(ob_p);

			cv::Mat gray;
			cvtColor(lim, gray, cv::COLOR_RGB2GRAY);
			cornerSubPix(gray, l_im_p, cv::Size(15, 15), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

			cvtColor(rim, gray, cv::COLOR_RGB2GRAY);
			cornerSubPix(gray, r_im_p, cv::Size(15, 15), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

			if (l_im_p[0].x > l_im_p[1].x)
			{

				std::vector<cv::Point2f> reversePoints;
				reversePoints.reserve(l_im_p.size());
				std::vector<cv::Point2f>::reverse_iterator riter;
				for (riter = l_im_p.rbegin(); riter != l_im_p.rend(); riter++)
				{
					reversePoints.push_back(*riter);
				}
				l_im_p = reversePoints;
				std::cout << "L [" << i << "]    " << l_im_p[0].x << "  " << l_im_p[0].y << "  " << l_im_p[1].x << "  " << l_im_p[1].y << "  ";
			}
			if (r_im_p[0].x > r_im_p[1].x)
			{

				std::vector<cv::Point2f> reversePoints;
				reversePoints.reserve(r_im_p.size());
				std::vector<cv::Point2f>::reverse_iterator riter;
				for (riter = r_im_p.rbegin(); riter != r_im_p.rend(); riter++)
				{
					reversePoints.push_back(*riter);
				}
				r_im_p = reversePoints;
				std::cout << "R [" << i << "]    " << r_im_p[0].x << "  " << r_im_p[0].y << "  " << r_im_p[1].x << "  " << r_im_p[1].y << "  " << std::endl;
			}

			l_image_points.push_back(l_im_p);
			r_image_points.push_back(r_im_p);

			std::cout << "->done " << count << " pairs of pictures" << std::endl;
			if (1)
			{

				cv::Mat im_show = lim.clone();
				drawChessboardCorners(im_show, cv::Size(width, height), l_im_p, true);
				cv::resize(im_show, im_show, cv::Size(960, 540), 0, 0, cv::INTER_AREA);
				cv::imshow("Left Chessboard corners", im_show);
				im_show = rim.clone();
				drawChessboardCorners(im_show, cv::Size(width, height), r_im_p, true);
				cv::resize(im_show, im_show, cv::Size(960, 540), 0, 0, cv::INTER_AREA);
				cv::imshow("Right Chessboard corners", im_show);
				while (char(cv::waitKey(1)) != '\r')
				{

				}
			}
		}
		else
		{
			std::cout << "->!!!not done " << count << " pairs" << " Name is: " << image_file_L[i] << std::endl << "\t " << image_file_R[i] << std::endl;

		}
	}

	std::vector<std::vector<cv::Point2f> > rl_image_points, rr_image_points;
	rl_image_points.resize(l_image_points.size());

	rr_image_points.resize(r_image_points.size());

	std::cout << l_image_points.size() << "    " << l_image_points[0].size() << std::endl;

	for (int i = 0; i < rl_image_points.size(); i++)
	{
		rl_image_points[i].emplace_back(l_image_points[i][0]);
		rl_image_points[i].emplace_back(l_image_points[i][1]);
		rl_image_points[i].emplace_back(l_image_points[i][2]);
		rl_image_points[i].emplace_back(l_image_points[i][3]);
		rl_image_points[i].emplace_back(l_image_points[i][4]);
		rl_image_points[i].emplace_back(l_image_points[i][5]);
		rl_image_points[i].emplace_back(l_image_points[i][6]);
		rl_image_points[i].emplace_back(l_image_points[i][7]);
		rl_image_points[i].emplace_back(l_image_points[i][8]);

		rr_image_points[i].emplace_back(r_image_points[i][0]);
		rr_image_points[i].emplace_back(r_image_points[i][1]);
		rr_image_points[i].emplace_back(r_image_points[i][2]);
		rr_image_points[i].emplace_back(r_image_points[i][3]);
		rr_image_points[i].emplace_back(r_image_points[i][4]);
		rr_image_points[i].emplace_back(r_image_points[i][5]);
		rr_image_points[i].emplace_back(r_image_points[i][6]);
		rr_image_points[i].emplace_back(r_image_points[i][7]);
		rr_image_points[i].emplace_back(r_image_points[i][8]);

		//std::cout << rl_image_points[i].size() << "    " << rr_image_points[i].size() << std::endl;
	}
	std::ofstream fwrite;
	fwrite.open("verify_pic_pixel_out_right.log");
	if (fwrite.is_open())
	{
		for (int i = 0; i < rl_image_points.size(); i++)
		{
			for (int j = 0; j < rl_image_points[i].size(); j++)
			{
				float x = rl_image_points[i][j].x;
				float y = rl_image_points[i][j].y;
				fwrite << x << "    " << y << "    ";

			}
			for (int j = 0; j < rl_image_points[i].size(); j++)
			{
				float x = rr_image_points[i][j].x;
				float y = rr_image_points[i][j].y;
				fwrite << x << "    " << y << "    ";

			}
			fwrite << '\n';
		}
	}
	else
	{
		std::cout << "can not open 'MPixel.log'" << std::endl;
	}
	fwrite.close();
}

void getCaliedPicture(std::string leftPath, std::string rightPath, std::string savePath)
{
	std::vector<cv::String> image_file_L;
	std::vector<cv::String> image_file_R;
	std::vector<std::vector<cv::Point2f> > l_image_points, r_image_points;
	std::vector<std::vector<cv::Point3f> > object_points;
	std::vector<cv::Mat> l_images, r_images;
	cv::glob(leftPath, image_file_L);             //��Ŀ¼�µ��ļ���ȡ��������
	cv::glob(rightPath, image_file_R);

	for (int i = 0; i < image_file_L.size(); i++)
	{
		l_images.push_back(cv::imread(image_file_L[i]));
	}

	for (int ii = 0; ii < image_file_R.size(); ii++)
	{
		r_images.push_back(cv::imread(image_file_R[ii]));
	}
	MapCalculator MapCalculator1(".", 0);
	MapCalculator MapCalculator2(".", 1);
	int count = 0;
	std::string saveAddress1, saveAddress2;
	saveAddress1 = savePath + "left_rect/";
	saveAddress2 = savePath + "right_rect/";
	for (int i = 0; i < l_images.size(); i++)
	{
		count += 1;
		cv::Mat lim = l_images[i];
		cv::Mat rim = r_images[i];
		lim = MapCalculator1.rectify(lim);
		rim = MapCalculator2.rectify(rim);

		if (1)
		{
			std::string save_address_left, save_address_right;
			cv::Mat im_show = lim.clone();
			cv::resize(im_show, im_show, cv::Size(960, 720), 0, 0, cv::INTER_AREA);
			cv::imshow("Left Chessboard corners", im_show);
			im_show = rim.clone();
			cv::resize(im_show, im_show, cv::Size(960, 720), 0, 0, cv::INTER_AREA);
			cv::imshow("Right Chessboard corners", im_show);
			while (char(cv::waitKey(1)) != '\r')
			{

			}
			if (count < 10)
			{
				save_address_left = saveAddress1 + "left_0" + std::to_string(count) + ".jpg";
				save_address_right = saveAddress2 + "right_0" + std::to_string(count) + ".jpg";
			}
			else
			{
				save_address_left = saveAddress1 + "left_" + std::to_string(count) + ".jpg";
				save_address_right = saveAddress2 + "right_" + std::to_string(count) + ".jpg";
			}
			if(cv::imwrite(save_address_left, lim) && cv::imwrite(save_address_right, rim))
				std::cout << std::endl << "-> Shot it, " << count << " times." << std::endl;

		}

	}
}
