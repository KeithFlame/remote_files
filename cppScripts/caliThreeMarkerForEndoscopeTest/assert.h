#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
using namespace std;
using namespace cv;

int calibratePicture(Size image_size, int image_count, Mat cameraMatrix, Mat distCoeffs, vector<string> filelist, Mat& R, Mat& P, string file = "calibrated/") {
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	//.Mat R = Mat::eye(3, 3, CV_32F);
	string imageFlieName;
	std::stringstream str;
	for (int i = 0; i != image_count; i++) {
		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, P, image_size, CV_32FC1, mapx, mapy);
		Mat imageSource = imread(filelist[i]);
		Mat newImage = imageSource.clone();
		remap(imageSource, newImage, mapx, mapy, INTER_LINEAR);
		str.clear();
		imageFlieName.clear();
		str << file << i + 1;
		str >> imageFlieName;
		imageFlieName = filelist[i] + "_calib.jpg";
		imwrite(imageFlieName, newImage);
	}
	return 1;
}

int saveCalibration(int image_count, vector<Mat>& rvecsMat, vector<Mat>& tvecsMat, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point3f>> object_points, vector<vector<Point2f>> image_points_seq, ofstream& fout, Size board_size) {
	double total_err = 0.0;        //所有图像的平均误差总和
	double err = 0.0;              //当前图像的平均误差
	vector<Point2f> image_points2; //保存重新计算得到的投影点

	for (int i = 0; i < image_count; i++) {
		vector<Point3f> tempPointSet = object_points[i];

		//根据相机内外参数，对空间的三维点进行重新投影
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);

		//计算重投影点与旧投影点（图像中的点）之间的误差
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);

		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);

		}
		err = norm(image_points2Mat, tempImagePoint, NORM_L2);
		total_err += err /= (board_size.width * board_size.height);
		fout << "第" << i + 1 << "幅图像的平均误差为：" << err << "像素" << endl;
	}
	fout << "总体平均误差为：" << total_err / image_count << "像素" << endl << endl;
	//***********误差计算完成***************//

	//***********保存标定结果***************//
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	fout << "相机内参矩阵：" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "畸变系数：" << endl;
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << tvecsMat[i] << endl;

		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << rvecsMat[i] << endl << endl;

	}
	fout << endl;
	return 1;
}


void saveStereoParam(Mat cameraMatrix_left, Mat cameraMatrix_right, Mat distCoeffs_left, Mat distCoeffs_right, Mat R, Mat T, Mat R_left, Mat R_right, Mat P_left, Mat P_right, Mat Q)
{
	/*保存数据*/
	/*输出数据*/
	FileStorage fs("intrisics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "cameraMatrixL" << cameraMatrix_left << "cameraDistcoeffL" << distCoeffs_left << "cameraMatrixR" << cameraMatrix_right << "cameraDistcoeffR" << distCoeffs_right;
		fs.release();
		cout << "cameraMatrixL=:" << cameraMatrix_left << endl << "cameraDistcoeffL=:" << distCoeffs_left << endl << "cameraMatrixR=:" << cameraMatrix_right << endl << "cameraDistcoeffR=:" << distCoeffs_right << endl;
	}
	else
	{
		cout << "Error: can not save the intrinsics!!!!" << endl;
	}

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "Rl" << R_left << "Rr" << R_right << "Pl" << P_left << "Pr" << P_right << "Q" << Q;
		cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << R_left << endl << "Rr" << R_right << endl << "Pl" << P_left << endl << "Pr" << P_right << endl << "Q" << Q << endl;
		fs.release();
	}
	else
	{
		cout << "Error: can not save the extrinsic parameters\n";
	}

}
