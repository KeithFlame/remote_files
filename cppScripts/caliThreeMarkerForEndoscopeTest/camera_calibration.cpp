#include "camera_calibration.h"
#include "assert.h"


using namespace cv;
using namespace std;

int single_camera_calib::getImagesPath(string Path, int Num) {
	//string t("/left_");
	for (int i = 1; i < Num; i++) {
		stringstream str;
		str << Path << i << ".png";
		filelist.push_back(str.str());
	}
	if (filelist.size() > 1)
		return 1;
	else
		return 0;
}

int double_camera_calib::getImagesPath(string Path_left, int Num_left, string Path_right, int Num_right) {
	//string t("/left_");
	//for (int i = 1; i < Num_left; i++) {
	//	stringstream str;
	//	str << Path_left << i << ".png";
	//	filelist_left.push_back(str.str());
	//}

	//for (int i = 1; i < Num_right; i++) {
	//	stringstream str;
	//	str << Path_right << i << ".png";
	//	filelist_right.push_back(str.str());
	//}

	glob(Path_left, filelist_left);             //将目录下的文件读取到容器中
	glob(Path_right, filelist_right);

	if (filelist_right.size() > 1 || filelist_left.size() > 1)
		return 1;
	else
		return 0;
}

vector<Point2f> single_camera_calib::getOneCorners(int count, const vector<string> file) {
	image = imread(file[count]);
	if (findChessboardCorners(image, board_size, image_points_buf) == 0) {
		cout << "!!" << file[count] << "  cannot detect chessboard corners!!" << endl;

	}
	else {
		Mat view_gray;
		//imshow("test", image);
		//waitKey(500);
		cvtColor(image, view_gray, COLOR_BGR2GRAY);
		//亚像素精细化
		cornerSubPix(view_gray, image_points_buf, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
		if (image_points_buf[0].x > image_points_buf[1].x)
		{
			std::vector<cv::Point2f> reversePoints;
			reversePoints.reserve(image_points_buf.size());
			vector<cv::Point2f>::reverse_iterator riter;
			for (riter = image_points_buf.rbegin(); riter != image_points_buf.rend(); riter++)
			{
				reversePoints.push_back(*riter);
			}
			image_points_buf = reversePoints;
		}
		drawChessboardCorners(view_gray, board_size, image_points_buf, false);
		//imshow("corner_detection", view_gray);
		//waitKey(50);
	}

	return image_points_buf;
}

vector<vector<Point2f>> single_camera_calib::getCorners() {

	for (image_count = 0; image_count < filelist.size(); image_count++) {
		getOneCorners(image_count, filelist);
		image_points_seq.push_back(image_points_buf);

	}

	return image_points_seq;
}

vector<vector<Point2f>> double_camera_calib::getCorners() {
	for (image_count = 0; image_count < filelist_left.size(); image_count++) {
		getOneCorners(image_count, filelist_left);
		image_points_seq_left.push_back(image_points_buf);

		getOneCorners(image_count, filelist_right);
		image_points_seq_right.push_back(image_points_buf);

	}
	return image_points_seq_left;
}

int single_camera_calib::getChessboardPoints() {
	int i, j, t;
	for (t = 0; t < image_count; t++) {
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++) {
			for (j = 0; j < board_size.width; j++) {
				Point3f realPoint;
				//假设棋盘格放在射界坐标系z=0的平面上
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	if (object_points.size() > 1)
		return 1;
	else
		return 0;
}

int single_camera_calib::calibrate() {
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	saveCalibration(image_count, rvecsMat, tvecsMat, cameraMatrix, distCoeffs, object_points, image_points_seq, fout, board_size);
	//double total_err = 0.0;        //所有图像的平均误差总和
	//double err = 0.0;              //当前图像的平均误差
	//vector<Point2f> image_points2; //保存重新计算得到的投影点

	//for (int i = 0; i < image_count; i++) {
	//	vector<Point3f> tempPointSet = object_points[i];
	//	
	//	//根据相机内外参数，对空间的三维点进行重新投影
	//	projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
	//	//计算重投影点与旧投影点（图像中的点）之间的误差，并保存标定结果
	//	//int saveCalibration(int image_count, vector<Mat> & rvecsMat, vector<Mat> & tvecsMat, Mat & cameraMatrix, Mat & distCoeffs, vector<vector<Point3f>> object_points, vector<vector<Point2f>> image_points_seq, ofstream & fout, Size board_size);

	//	//计算重投影点与旧投影点（图像中的点）之间的误差
	//	vector<Point2f> tempImagePoint = image_points_seq[i];
	//	Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
	//	Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);

	//	for (int j = 0; j < tempImagePoint.size(); j++)
	//	{
	//		image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
	//		tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);

	//	}
	//	err = norm(image_points2Mat, tempImagePoint, NORM_L2);
	//	total_err += err /= (board_size.width * board_size.height);
	//	fout << "第" << i + 1 << "幅图像的平均误差为：" << err << "像素" << endl;
	//}
	//fout << "总体平均误差为：" << total_err / image_count << "像素" << endl<<endl;
	////***********误差计算完成***************//

	////***********保存标定结果***************//
	//Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	//fout << "相机内参矩阵：" << endl;
	//fout << cameraMatrix << endl << endl;
	//fout << "畸变系数：" << endl;
	//fout << distCoeffs << endl << endl << endl;
	//for (int i = 0; i < image_count; i++)
	//{
	//	fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
	//	fout << tvecsMat[i] << endl;

	//	Rodrigues(tvecsMat[i], rotation_matrix);
	//	fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
	//	fout << rotation_matrix << endl;
	//	fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
	//	fout << rvecsMat[i] << endl << endl;

	//}
	//fout << endl;
	////***********标定结果保存结束******************//

	//if (total_err != 0)
	return 1;
	//else
	//	return 0;
}

int double_camera_calib::calibrate() {

	cameraMatrix_left = Mat(3, 3, CV_32FC1, Scalar::all(0));
	distCoeffs_left = Mat(1, 5, CV_32FC1, Scalar::all(0));
	cameraMatrix_right = Mat(3, 3, CV_32FC1, Scalar::all(0));
	distCoeffs_right = Mat(1, 5, CV_32FC1, Scalar::all(0));


	calibrateCamera(object_points, image_points_seq_left, image_size, cameraMatrix_left, distCoeffs_left, rvecsMat_left, tvecsMat_left, 0);
	std::cout << cameraMatrix_left << std::endl;
	std::cout << distCoeffs_left << std::endl << std::endl << std::endl;
	saveCalibration(image_count, rvecsMat_left, tvecsMat_left, cameraMatrix_left, distCoeffs_left, object_points, image_points_seq_left, fout_left, board_size);
	//不知道 cameraMatrix, distCoeffs, rvecsMat, tvecsMat 是覆盖还是push back,需要检测
	//以上数据为vector型，每次push_back新值
	calibrateCamera(object_points, image_points_seq_right, image_size, cameraMatrix_right, distCoeffs_right, rvecsMat_right, tvecsMat_right, 0);
	cout << cameraMatrix_right << endl;
	cout << distCoeffs_right << endl << endl << endl;
	saveCalibration(image_count, rvecsMat_right, tvecsMat_right, cameraMatrix_right, distCoeffs_right, object_points, image_points_seq_right, fout_right, board_size);


	//cv::Mat iMl = (cv::Mat_<int>(3, 3) << 1059.811f,0,968.89f,
	//	0,1065.4f,549.58f,
	//	0,0,1.0f );
	//cv::Mat iMr = (cv::Mat_<int>(3, 3) << 1047.7f, 0, 976.9f,
	//	0, 1054.3f, 555.3f,
	//	0, 0, 1.0f);
	//cv::Mat dCl = (cv::Mat_<int>(1, 5) << 0.0678f, - 0.0720f,0.0f, -0.0008439f, - 0.0004717f);
	//cv::Mat dCr = (cv::Mat_<int>(1, 5) << 0.0814f - 0.0811f, 0.0f, -0.0060f,    0.0032f);
	double rms = stereoCalibrate(object_points, image_points_seq_left, image_points_seq_right,
		cameraMatrix_left, distCoeffs_left,
		cameraMatrix_right, distCoeffs_right,
		image_size, R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "cameraMatrix_left" << endl << cameraMatrix_left << endl << endl;
	cout << "cameraMatrix_right" << endl << cameraMatrix_right << endl << endl;
	cout << "R" << endl << R << endl << endl;
	cout << "T" << endl << T << endl;
	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, image_size, R, T, R_left,
		R_right, P_left, P_right, Q, CALIB_ZERO_DISPARITY, -1, image_size, &validROIL, &validROIR);
	Mat e1 = -T / norm(T);
	cout << "T=" << endl << " " << T << endl << endl;
	cout << "cameraMatrix_left[2]: " << endl << cameraMatrix_left << endl << endl;
	cout << "cameraMatrix_right[2]: " << endl << cameraMatrix_right << endl << endl;
	cout << "R_right = " << endl << " " << R_right << endl << endl;
	cout << "R_left=" << endl << "" << R_left << endl << endl;
	cout << "P_left = " << endl << " " << P_left << endl << endl;
	cout << "P_right=" << endl << "" << P_right << endl << endl;
	//Mat a=(Mat_<double>(3,1)<<0,0,1);
	//Mat e2 = e1.cross(a);
	//Mat e3 = e2.cross(e1);
	//R_left.col(0) = e1;
	//R_left.col(1) = e2;
	//R_left.col(2) = e3;
	//R_right = R * R_left;
	//cout << "===================After correction==================" << endl;
	//cout << "R_right = " << endl << " " << R_right << endl << endl;
	//cout << "R_left=" << endl << "" << R_left << endl << endl;

	saveStereoParam(cameraMatrix_left, cameraMatrix_right, distCoeffs_left, distCoeffs_right, R, T, R_left, R_right, P_left, P_right, Q);
	return 1;
}

int single_camera_calib::showCalibratedPictures() {
	Mat R = Mat::eye(3, 3, CV_32F);
	cv::calibratePicture(image_size, image_count, cameraMatrix, distCoeffs, filelist, R, cameraMatrix);
	return 1;
	//Mat mapx = Mat(image_size, CV_32FC1);
	//Mat mapy = Mat(image_size, CV_32FC1);
	//Mat R = Mat::eye(3, 3, CV_32F);
	//string imageFlieName;
	//std::stringstream str;
	//for (int i = 0; i != image_count; i++) {
	//	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
	//	Mat imageSource = imread(filelist[i]);
	//	Mat newImage = imageSource.clone();
	//	remap(imageSource, newImage, mapx, mapy, INTER_LINEAR);
	//	str.clear();
	//	imageFlieName.clear();
	//	str << i + 1;
	//	str >> imageFlieName;
	//	imageFlieName += "_calib.png";
	//	imwrite(imageFlieName, newImage);
	//}
	//return 1;
}

int double_camera_calib::showCalibratedPictures() {
	string left = "./rectifiedPictures/left/";
	string right = "./rectifiedPictures/right/";




	calibratePicture(image_size, image_count, cameraMatrix_left, distCoeffs_left, filelist_left, R_left, P_left, left);
	calibratePicture(image_size, image_count, cameraMatrix_right, distCoeffs_right, filelist_right, R_right, P_right, right);

	/*
	//显示校正结果
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(image_size.width, image_size.height);
	w = cvRound(image_size.width * sf);
	h = cvRound(image_size.height * sf);
	canvas.create(h, w * 2, CV_8UC3);

	Mat rectifyImageL2 = imread("./NO65XX50/left/left01.jpg");
	Mat rectifyImageR2 = imread("./NO65XX50/right/right01.jpg");
	Mat canvasPart = canvas(Rect(0, 0, w, h));
	resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),
		cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);

	cout << "Painted ImageL" << endl;

	//右图像画到画布上
	canvasPart = canvas(Rect(w, 0, w, h));
	resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	cout << "Painted ImageR" << endl;

	//画上对应的线条
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

	imshow("rectified", canvas);

	cout << "wait key" << endl;
	waitKey(0);

*/
	return 1;
}

int double_camera_calib::calibratePicturesPairs(string Path_left, string Path_right)
{
	vector<string>          filelist_l;
	vector<string>          filelist_r;
	string left = "./rectifiedPictures/left/";
	string right = "./rectifiedPictures/right/";

	glob(Path_left, filelist_l);             //将目录下的文件读取到容器中
	glob(Path_right, filelist_r);
	int imageCount = (filelist_l.size() + filelist_r.size()) / 2;


	calibratePicture(image_size, imageCount, cameraMatrix_left, distCoeffs_left, filelist_l, R_left, P_left, left);
	calibratePicture(image_size, imageCount, cameraMatrix_right, distCoeffs_right, filelist_r, R_right, P_right, right);

	return 0;
}


