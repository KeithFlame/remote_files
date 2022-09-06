#include<iostream>
#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#define filename11 "./cam_calib/stereo_calib.xml"
#include <fstream>
#include <numeric>
#include <vector>

using namespace std;
using namespace cv;


int getConfigV2(string leftPath, string rightPath, float edgeLength, int width, int height);

int main(int argc, char** argv) {

	int arm_id;
	ifstream in("arm_id.log");
	in >> arm_id;
	in.close();

	string str_arm_id = to_string(arm_id);
	std::cout << "arm id: " << str_arm_id << std::endl;


	string leftPath;
	string rightPath;
	string str1 = "picture_factory\\";
	string str2 = str_arm_id + "\\left\\";
	string str3 = str_arm_id + "\\right\\";
	leftPath = str1 + str2;
	rightPath = str1 + str3;

	std::cout << "right pic path: " << rightPath << std::endl;
	std::cout << "left pic path: " << leftPath << std::endl;

	// exe2
	// getConfigV2(rightPath, leftPath, 5.f, 3, 3);

	// exe1
	// getConfigV2(leftPath, rightPath, 5.f, 3, 3);
	int RightLeftErr;
	RightLeftErr = getConfigV2(leftPath, rightPath, 5.f, 3, 3);
	if (RightLeftErr)
	{
		std::cout << "�û���������ϸ񣬲���������궨�������½û��䣡����" << std::endl;
	}
	else
	{
		std::cout << "�ɽ�����һ������" << std::endl;
	}
	system("pause");
	return 0;

}

int getConfigV2(string leftPath, string rightPath, float edgeLength, int width, int height)
{
	fstream outfile;
	outfile.open("calib_pic_coord_out.log", ios::out);

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

	int arm_id;
	ifstream in("arm_id.log");
	in >> arm_id;
	in.close();
	string str_arm_id = to_string(arm_id);
	string camaraPPath;
	camaraPPath = "map\\" + str_arm_id + "\\camaraP.log";
	fstream infile(camaraPPath);
	float t = 0.0, f = 0.0, P = 0.0;
	if (!infile)
	{
		cout << "Unable to open the file !";
	}
	infile >> t;
	infile >> f;
	infile >> P;
	infile.close();
	P = 1.0f;
	cout << "camara parameter:" << endl;
	cout << "t=" << t << endl;
	cout << "f=" << f << endl;
	cout << "P=" << P << endl;

	vector<float> deltav;
	deltav.clear();
	vector<vector < Point3f>> stereoPoints;
	stereoPoints.resize(rl_image_points.size());
	for (int i = 0; i < rl_image_points.size(); i++)
		for (int j = 0; j < rl_image_points[0].size(); j++)
		{
			Point3f tem;

			//57
			//float t = 4.0105f; //T=[-4.078214909837174;-0.02371211643894806;0.04762036211935505]
			//float f = 1175.217f;
			//float P = 1.0833f;

			//59
			//float t = 4.0972f; //T=[-4.078214909837174;-0.02371211643894806;0.04762036211935505]
			//float f = 1116.735f;
			//float P = 1.0247f;  //			float P = 1.0444f;
			deltav.emplace_back(fabs(rl_image_points[i][j].y - rr_image_points[i][j].y));
			std::cout << "delta_v:[" << i << "] " << rl_image_points[i][j].y << "    " << rr_image_points[i][j].y << "    " << rl_image_points[i][j].y - rr_image_points[i][j].y << "    "
				<< "delta_u:[" << i << "] " << rl_image_points[i][j].x << "    " << rr_image_points[i][j].x << "    " << rl_image_points[i][j].x - rr_image_points[i][j].x << std::endl;

			tem.z = t * f / (rl_image_points[i][j].x - rr_image_points[i][j].x);
			tem.x = -t / 2 + tem.z * (-960.f + rl_image_points[i][j].x) / f / P;
			tem.y = tem.z * (-540.f + rl_image_points[i][j].y) / f / P;
			stereoPoints[i].push_back(tem);
			std::cout << stereoPoints[i][j].x << "    " << stereoPoints[i][j].y << "    " << stereoPoints[i][j].z << std::endl;
			//if (tem.z < 0)
			//{
			//	std::cout << "��ͷ�ӷ�����ɾ����Ƭ���ر��������ѡ<������ͷ>��ť�����´�������գ�" << std::endl;
			//	return 1;
			//}

		}

	float sum_deltav = accumulate(begin(deltav), end(deltav), 0.0);   // accumulate����������vector�͵ĺ�����
	float mean_deltav = sum_deltav / deltav.size();                   // ���ֵ
	std::cout << "mean deltaV: " << mean_deltav << std::endl;
	fstream rectify_result_file;
	rectify_result_file.open("result\\" + str_arm_id + "\\rectify_result.log", ios::out);
	rectify_result_file << mean_deltav << std::endl;
	rectify_result_file.close();

	if (mean_deltav > 2)
	{
		std::cout << "�û���������ϸ񣬲���������궨�������½û��䣡����" << std::endl;
		return 1;
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
		float dis1 = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2));
		float dis2 = sqrt(pow(A.x - D.x, 2) + pow(A.y - D.y, 2) + pow(A.z - D.z, 2));
		float dis3 = sqrt(pow(D.x - E.x, 2) + pow(D.y - E.y, 2) + pow(D.z - E.z, 2));
		float dis4 = sqrt(pow(B.x - E.x, 2) + pow(B.y - E.y, 2) + pow(B.z - E.z, 2));
		float dis5 = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2) + pow(A.z - C.z, 2));
		float dis6 = sqrt(pow(B.x - C.x, 2) + pow(B.y - C.y, 2) + pow(B.z - C.z, 2));
		float dis7 = sqrt(pow(C.x - D.x, 2) + pow(C.y - D.y, 2) + pow(C.z - D.z, 2));
		float dis8 = sqrt(pow(E.x - C.x, 2) + pow(E.y - C.y, 2) + pow(E.z - C.z, 2));
		std::cout << "disX,disY: " << dis1 << "    " << dis2 << "        " << dis3 << "        " << dis4 << "        " << dis5 << "        "
			<< dis6 << "        " << dis7 << "        " << dis8 << "        " << i << "    " << A << " " << "    " << B << "    " << C << "    " << D << "    " << E << "    " << F << "    " << G << "    " << H << "    " << I << "    " << std::endl;
		outfile << A << '\t' << B << '\t' << C << '\t' << D << '\t' << E << '\t' << F << '\t' << G << '\t' << H << '\t' << I << std::endl;


	}
	outfile.close();

	//if (stereoPoints[0][0].z < 0.0)
	//{
		// exe2
		//std::cout << "��ͷ�÷�����˫��image_to_coordinate.exe�������¼��㣡" << std::endl;
		// exe
		//std::cout << "��ͷ�÷�����˫��image_to_coordinate2.exe�������¼��㣡" << std::endl;
	//}

	return 0;
}

