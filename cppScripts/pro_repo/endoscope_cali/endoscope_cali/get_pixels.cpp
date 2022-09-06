#include<iostream>
//#include "camera_calibration.h"
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;


void getConfigV2(string leftPath, string rightPath, float edgeLength, int width, int height);
void getConfigV3(string leftPath, string rightPath, float edgeLength, int width, int height, int pic_type);
int main(int argc, char** argv) {

	// pic_type:0-calib, 1-verify, 2-fat
	int pic_type = 0;

	if (argc > 1)
	{
		pic_type = atoi(argv[1]);
	}
	std::cout << "123 Begin!" << std::endl;
	std::fstream fread;
	fread.open("./arm_id.log");
	char armbuf[500];
	int proc_id(0);
	if (fread.is_open())
	{
		fread.getline(armbuf, sizeof(armbuf));
		proc_id = std::atoi(armbuf);
		//std::cout << "arm_id: " << proc_id << std::endl;
	}
	else
	{
		std::cout << "cannot open the arm_id file! " << std::endl;
	}

	std::string Zpath;
	if (1 == pic_type)
		//1-verify
		Zpath = "./picture_factory_verify/" + std::to_string(proc_id);
	else if (2 == pic_type)
		//2-fat
		Zpath = "./picture_factory_test/" + std::to_string(proc_id);
	else
		//0-calib
		Zpath = "./picture_factory/" + std::to_string(proc_id);

	std::cout << Zpath << std::endl;
	getConfigV3(Zpath + "/left/", Zpath + "/right/", 5.f, 3, 3, pic_type);
	std::cout << "arm_id: " << proc_id << std::endl;
	system("pause");
	return 0;

}

void getConfigV2(string leftPath, string rightPath, float edgeLength, int width, int height)
{
	std::vector<cv::String> image_file_L;
	std::vector<cv::String> image_file_R;
	vector<vector<Point2f> > l_image_points, r_image_points;
	vector<vector<Point3f> > object_points;
	vector<Mat> l_images, r_images;
	glob(leftPath, image_file_L);             //��Ŀ¼�µ��ļ���ȡ��������
	glob(rightPath, image_file_R);

	for (int i = 0; i < image_file_L.size(); i++)
	{
		l_images.push_back(imread(image_file_L[i]));
	}

	for (int ii = 0; ii < image_file_R.size(); ii++)
	{
		r_images.push_back(imread(image_file_R[ii]));
	}

	vector<Point3f> ob_p;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			ob_p.push_back(Point3f(j * edgeLength, i * edgeLength, 0.f));
		}
	}

	int count = 0;
	for (int i = 0; i < l_images.size(); i++)
	{
		count += 1;
		Mat lim = l_images[i];
		Mat rim = r_images[i];

		vector<Point2f> l_im_p;
		vector<Point2f> r_im_p;
		bool l_pattern_found = findChessboardCorners(lim, Size(width, height), l_im_p, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK); //
		bool r_pattern_found = findChessboardCorners(rim, Size(width, height), r_im_p, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK); //
		if (l_pattern_found && r_pattern_found)
		{
			object_points.push_back(ob_p);

			Mat gray;
			cvtColor(lim, gray, COLOR_RGB2GRAY);
			cornerSubPix(gray, l_im_p, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

			cvtColor(rim, gray, COLOR_RGB2GRAY);
			cornerSubPix(gray, r_im_p, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
			std::cout << l_im_p << std::endl;
			if (l_im_p[0].x > l_im_p[1].x)
			{

				std::vector<cv::Point2f> reversePoints;
				reversePoints.reserve(l_im_p.size());
				vector<cv::Point2f>::reverse_iterator riter;
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
				vector<cv::Point2f>::reverse_iterator riter;
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
			if (0)
			{

				Mat im_show = lim.clone();
				drawChessboardCorners(im_show, Size(width, height), l_im_p, true);
				resize(im_show, im_show, Size(960, 540), 0, 0, INTER_AREA);
				imshow("Left Chessboard corners", im_show);
				im_show = rim.clone();
				drawChessboardCorners(im_show, Size(width, height), r_im_p, true);
				resize(im_show, im_show, Size(960, 540), 0, 0, INTER_AREA);
				imshow("Right Chessboard corners", im_show);
				while (char(waitKey(1)) != '\r')
				{

				}
			}
		}
		else
		{
			std::cout << "->!!!not done " << count << " pairs" << " Name is: " << image_file_L[i] << std::endl << "\t " << image_file_R[i] << std::endl;

		}
	}

	vector<vector<Point2f> > rl_image_points, rr_image_points;
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

	//for(int i = 0;i< rl_image_points.size();i++)
	//{
	//	Mat im_show = l_images[i].clone();
	//	drawChessboardCorners(im_show, Size(width, height), rl_image_points[i], true);
	//	imshow("Left Chessboard corners", im_show);
	//	im_show = r_images[i].clone();
	//	drawChessboardCorners(im_show, Size(width, height), rr_image_points[i], true);
	//	imshow("Right Chessboard corners", im_show);
	//	while (char(waitKey(1)) != '\r')
	//	{

	//	}
	//}



	//Mat T = Mat(1, 3, CV_32FC1, Scalar::all(0));
	//Mat l_cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	//FileStorage fs(filename11, FileStorage::READ);
	//fs["l_cameraMatrix"] >> l_cameraMatrix;
	//fs["T"] >> T;
	//fs.release();

	vector<vector < Point3f>> stereoPoints;
	stereoPoints.resize(rl_image_points.size());
	for (int i = 0; i < rl_image_points.size(); i++)
		for (int j = 0; j < rl_image_points[0].size(); j++)
		{
			Point3f tem;
			//////57
			////float t = 4.0105; 
			////float f = 1175.217f;
			////float P = 1.0833f;

			//59
			float t = 4.0972f;
			float f = 1116.735f;
			float P = 1.0f;// 247f;
			std::cout << "delta_v:[" << i << "] " << rl_image_points[i][j].y << "    " << rr_image_points[i][j].y << "    " << rl_image_points[i][j].y - rr_image_points[i][j].y << "    "
				<< "delta_u:[" << i << "] " << rl_image_points[i][j].x << "    " << rr_image_points[i][j].x << "    " << rl_image_points[i][j].x - rr_image_points[i][j].x << std::endl;

			tem.z = t * f / (rl_image_points[i][j].x - rr_image_points[i][j].x);
			tem.x = tem.z * (960.f - rl_image_points[i][j].x) / f / P;
			tem.y = tem.z * (540.f - rl_image_points[i][j].y) / f / P;
			stereoPoints[i].push_back(tem);
			//std::cout << stereoPoints[i][j].x << "    "<< stereoPoints[i][j].y <<"    "<< stereoPoints[i][j].z <<std::endl;

		}
	for (int i = 0; i < stereoPoints.size(); i++)
	{
		Point3f A, B, C, D, E, F, G, H, I;
		A = stereoPoints[i][0];
		B = stereoPoints[i][1];
		C = stereoPoints[i][2];
		D = stereoPoints[i][3];
		E = stereoPoints[i][4];
		F = stereoPoints[i][5];
		G = stereoPoints[i][6];
		H = stereoPoints[i][7];
		I = stereoPoints[i][8];

		float disY = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2));
		float disX = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2) + pow(A.z - C.z, 2));
		std::cout << "disX,disY: " << disX << "    " << disY << "        "
			<< i << "    " << A.x << " " << A.y << " " << A.z << "    " << B.x << " " << B.y << " " << B.z << "    " << C.x << " " << C.y << " " << C.z << "    " << D.x << " " << D.y << " " << D.z << "    " << E.x << " " << E.y << " " << E.z << "    "
			<< F.x << " " << F.y << " " << F.z << "    " << G.x << " " << G.y << " " << G.z << "    " << H.x << " " << H.y << " " << H.z << "    " << I.x << " " << I.y << " " << I.z << "    " << std::endl;
	}

}
void getConfigV3(string leftPath, string rightPath, float edgeLength, int width, int height, int pic_type)
{
	std::vector<cv::String> image_file_L;
	std::vector<cv::String> image_file_R;
	vector<vector<Point2f> > l_image_points, r_image_points;
	vector<vector<Point3f> > object_points;
	vector<Mat> l_images, r_images;
	glob(leftPath, image_file_L);             //��Ŀ¼�µ��ļ���ȡ��������
	glob(rightPath, image_file_R);

	for (int i = 0; i < image_file_L.size(); i++)
	{
		l_images.push_back(imread(image_file_L[i]));
	}

	for (int ii = 0; ii < image_file_R.size(); ii++)
	{
		r_images.push_back(imread(image_file_R[ii]));
	}

	vector<Point3f> ob_p;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			ob_p.push_back(Point3f(j * edgeLength, i * edgeLength, 0.f));
		}
	}

	int count = 0;
	for (int i = 0; i < l_images.size(); i++)
	{
		count += 1;
		Mat lim = l_images[i];
		//rotate(lim,lim,ROTATE_180);
		Mat rim = r_images[i];
		//rotate(rim, rim, ROTATE_180);

		vector<Point2f> l_im_p;
		vector<Point2f> r_im_p;
		bool l_pattern_found = findChessboardCorners(lim, Size(width, height), l_im_p, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK); //
		bool r_pattern_found = findChessboardCorners(rim, Size(width, height), r_im_p, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK); //
		if (l_pattern_found && r_pattern_found)
		{
			object_points.push_back(ob_p);
			Mat gray;
			cvtColor(lim, gray, COLOR_RGB2GRAY);
			cornerSubPix(gray, l_im_p, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

			cvtColor(rim, gray, COLOR_RGB2GRAY);
			cornerSubPix(gray, r_im_p, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

			if (l_im_p[0].x > l_im_p[1].x)
			{

				std::vector<cv::Point2f> reversePoints;
				reversePoints.reserve(l_im_p.size());
				vector<cv::Point2f>::reverse_iterator riter;
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
				vector<cv::Point2f>::reverse_iterator riter;
				for (riter = r_im_p.rbegin(); riter != r_im_p.rend(); riter++)
				{
					reversePoints.push_back(*riter);
				}
				r_im_p = reversePoints;
				std::cout << "R [" << i << "]    " << r_im_p[0].x << "  " << r_im_p[0].y << "  " << r_im_p[1].x << "  " << r_im_p[1].y << "  " << std::endl;
			}

			float l_d19 = sqrtf((l_im_p[0].x - l_im_p[8].x) * (l_im_p[0].x - l_im_p[8].x) + (l_im_p[0].y - l_im_p[8].y) * (l_im_p[0].y - l_im_p[8].y));
			float l_d37 = sqrtf((l_im_p[2].x - l_im_p[6].x) * (l_im_p[2].x - l_im_p[6].x) + (l_im_p[2].y - l_im_p[6].y) * (l_im_p[2].y - l_im_p[6].y));
			float r_d19 = sqrtf((r_im_p[0].x - r_im_p[8].x) * (r_im_p[0].x - r_im_p[8].x) + (r_im_p[0].y - r_im_p[8].y) * (r_im_p[0].y - r_im_p[8].y));
			float r_d37 = sqrtf((r_im_p[2].x - r_im_p[6].x) * (r_im_p[2].x - r_im_p[6].x) + (r_im_p[2].y - r_im_p[6].y) * (r_im_p[2].y - r_im_p[6].y));
			float s_l = l_d19 * l_d37;
			float s_r = r_d19 * r_d37;
			if (s_l / s_r < 0.6)
			{
				// use Right ROI to find corners in Left pic

				int roi_x_min = std::max(0, int(r_im_p[6].x - 100));
				int roi_y_min = 0;
				int width_roi = (int(r_im_p[2].x - r_im_p[6].x) + 200) > 1920 ? 1920 : (int(r_im_p[2].x - r_im_p[6].x) + 200);
				int height_roi = 1080;
				cv::Rect im_roi = cv::Rect(roi_x_min, roi_y_min, width_roi, height_roi);
				std::cout << im_roi << std::endl;
				Mat m = Mat(1080, 1920, CV_8UC3, Scalar(0, 0, 0));
				Mat roi_mm = lim(im_roi);
				roi_mm.copyTo(m(im_roi));
				//imshow("", lim(lim_roi));
				l_pattern_found = findChessboardCorners(m, Size(width, height), l_im_p, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

				if (l_pattern_found)
				{
					cvtColor(lim, gray, COLOR_RGB2GRAY);
					cornerSubPix(gray, l_im_p, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
					if (l_im_p[0].x > l_im_p[1].x)
					{

						std::vector<cv::Point2f> reversePoints;
						reversePoints.reserve(l_im_p.size());
						vector<cv::Point2f>::reverse_iterator riter;
						for (riter = l_im_p.rbegin(); riter != l_im_p.rend(); riter++)
						{
							reversePoints.push_back(*riter);
						}
						l_im_p = reversePoints;
						std::cout << "---->Sec.   L [" << i << "]    " << l_im_p[0].x << "  " << l_im_p[0].y << "  " << l_im_p[1].x << "  " << l_im_p[1].y << "  ";
					}
					l_d19 = sqrtf((l_im_p[0].x - l_im_p[8].x) * (l_im_p[0].x - l_im_p[8].x) + (l_im_p[0].y - l_im_p[8].y) * (l_im_p[0].y - l_im_p[8].y));
					l_d37 = sqrtf((l_im_p[2].x - l_im_p[6].x) * (l_im_p[2].x - l_im_p[6].x) + (l_im_p[2].y - l_im_p[6].y) * (l_im_p[2].y - l_im_p[6].y));
					s_l = l_d19 * l_d37;
					if (s_l / s_r < 0.6)
					{
						std::cout << "->!!!not done " << count << " pairs" << " Name is: " << image_file_L[i] << std::endl << "\t " << image_file_R[i] << std::endl;
					}
				}
				else
				{
					std::cout << "->!!!not done " << count << " pairs" << " Name is: " << image_file_L[i] << std::endl << "\t " << image_file_R[i] << std::endl;

				}
			}
			else if (s_r / s_l < 0.6)
			{
				// use Left ROI to find corners in Right pic
				int roi_x_min = std::max(0, int(l_im_p[6].x - 100));
				int roi_y_min = 0;
				int width_roi = (int(l_im_p[2].x - l_im_p[6].x) + 200) > 1920 ? 1920 : (int(l_im_p[2].x - l_im_p[6].x) + 200);
				int height_roi = 1080;
				cv::Rect im_roi = cv::Rect(roi_x_min, roi_y_min, width_roi, height_roi);
				std::cout << im_roi << std::endl;
				Mat m = Mat(1080, 1920, CV_8UC3, Scalar(0, 0, 0));
				Mat roi_mm = rim(im_roi);
				roi_mm.copyTo(m(im_roi));
				//imshow("", lim(lim_roi));
				r_pattern_found = findChessboardCorners(m, Size(width, height), r_im_p, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

				if (r_pattern_found)
				{
					cvtColor(rim, gray, COLOR_RGB2GRAY);
					cornerSubPix(gray, r_im_p, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
					if (r_im_p[0].x > r_im_p[1].x)
					{

						std::vector<cv::Point2f> reversePoints;
						reversePoints.reserve(r_im_p.size());
						vector<cv::Point2f>::reverse_iterator riter;
						for (riter = r_im_p.rbegin(); riter != r_im_p.rend(); riter++)
						{
							reversePoints.push_back(*riter);
						}
						r_im_p = reversePoints;
						std::cout << "---->Sec.   R [" << i << "]    " << r_im_p[0].x << "  " << r_im_p[0].y << "  " << r_im_p[1].x << "  " << r_im_p[1].y << "  " << std::endl;
					}
					r_d19 = sqrtf((r_im_p[0].x - r_im_p[8].x) * (r_im_p[0].x - r_im_p[8].x) + (r_im_p[0].y - r_im_p[8].y) * (r_im_p[0].y - r_im_p[8].y));
					r_d37 = sqrtf((r_im_p[2].x - r_im_p[6].x) * (r_im_p[2].x - r_im_p[6].x) + (r_im_p[2].y - r_im_p[6].y) * (r_im_p[2].y - r_im_p[6].y));
					s_r = r_d19 * r_d37;
					if (s_r / s_l < 0.6)
					{
						std::cout << "->!!!not done " << count << " pairs" << " Name is: " << image_file_L[i] << std::endl << "\t " << image_file_R[i] << std::endl;
					}
				}
				else
				{
					std::cout << "->!!!not done " << count << " pairs" << " Name is: " << image_file_L[i] << std::endl << "\t " << image_file_R[i] << std::endl;

				}

			}

			l_image_points.push_back(l_im_p);
			r_image_points.push_back(r_im_p);

			std::cout << "->done " << count << " pairs of pictures" << std::endl;
			if (0)
			{

				Mat im_show = lim.clone();
				drawChessboardCorners(im_show, Size(width, height), l_im_p, true);
				resize(im_show, im_show, Size(960, 540), 0, 0, INTER_AREA);
				imshow("Left Chessboard corners", im_show);
				im_show = rim.clone();
				drawChessboardCorners(im_show, Size(width, height), r_im_p, true);
				resize(im_show, im_show, Size(960, 540), 0, 0, INTER_AREA);
				imshow("Right Chessboard corners", im_show);


				cv::waitKey(0);
			}
		}
		else
		{
			std::cout << "->!!!not done " << count << " pairs" << " Name is: " << image_file_L[i] << std::endl << "\t " << image_file_R[i] << std::endl;

		}
	}
	vector<vector<Point2f> > rl_image_points, rr_image_points;
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

	std::string outlog_name;
	if (1 == pic_type)
		outlog_name = "verify_pic_pixel_out.log";
	else if (2 == pic_type)
		outlog_name = "fat_pic_pixel_out.log";
	else
		outlog_name = "calib_pic_pixel_out.log";

	if (1 == pic_type)
	{
		// only process 1 pic when verify
		std::ofstream fwrite;
		fwrite.open(outlog_name);
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
			std::cout << "can not open " + outlog_name << std::endl;
		}
		fwrite.close();
	}
	else
	{
		// process 3 pics

		std::ofstream fwrite;
		fwrite.open(outlog_name);
		if (fwrite.is_open())
		{
			for (int i = 0; i < rl_image_points.size(); i += 3)
			{
				for (int j = 0; j < rl_image_points[i].size(); j++)
				{
					float x = rl_image_points[i][j].x + rl_image_points[i + 1][j].x + rl_image_points[i + 2][j].x;
					float y = rl_image_points[i][j].y + rl_image_points[i + 1][j].y + rl_image_points[i + 2][j].y;
					fwrite << x / 3 << "    " << y / 3 << "    ";

				}
				for (int j = 0; j < rl_image_points[i].size(); j++)
				{
					float x = rr_image_points[i][j].x + rr_image_points[i + 1][j].x + rr_image_points[i + 2][j].x;
					float y = rr_image_points[i][j].y + rr_image_points[i + 1][j].y + rr_image_points[i + 2][j].y;
					fwrite << x / 3 << "    " << y / 3 << "    ";

				}
				fwrite << '\n';
			}
		}
		else
		{
			std::cout << "can not open " + outlog_name << std::endl;
		}
		fwrite.close();

	}
}
