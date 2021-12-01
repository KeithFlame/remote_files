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
	double total_err = 0.0;        //����ͼ���ƽ������ܺ�
	double err = 0.0;              //��ǰͼ���ƽ�����
	vector<Point2f> image_points2; //�������¼���õ���ͶӰ��

	for (int i = 0; i < image_count; i++) {
		vector<Point3f> tempPointSet = object_points[i];

		//�����������������Կռ����ά���������ͶӰ
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);

		//������ͶӰ�����ͶӰ�㣨ͼ���еĵ㣩֮������
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
		fout << "��" << i + 1 << "��ͼ���ƽ�����Ϊ��" << err << "����" << endl;
	}
	fout << "����ƽ�����Ϊ��" << total_err / image_count << "����" << endl << endl;
	//***********���������***************//

	//***********����궨���***************//
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	fout << "����ڲξ���" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "����ϵ����" << endl;
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << tvecsMat[i] << endl;

		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << rvecsMat[i] << endl << endl;

	}
	fout << endl;
	return 1;
}


void saveStereoParam(Mat cameraMatrix_left, Mat cameraMatrix_right, Mat distCoeffs_left, Mat distCoeffs_right, Mat R, Mat T, Mat R_left, Mat R_right, Mat P_left, Mat P_right, Mat Q)
{
	/*��������*/
	/*�������*/
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
