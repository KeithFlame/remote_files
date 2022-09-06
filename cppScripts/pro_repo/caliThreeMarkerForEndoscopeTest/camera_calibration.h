#pragma once
#include <opencv2/core.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

class single_camera_calib {
public:
	virtual int getImagesPath(string Path, int Num);//图像路径与数量
	vector<Point2f> getOneCorners(int count, const vector<string> file);//标定一个图像
	virtual vector<vector<Point2f>> getCorners();//得到所有图像的角点坐标
	int getChessboardPoints();//得到棋盘格角点三维坐标（人为假设）
	virtual int calibrate();//标定
	virtual int showCalibratedPictures();
	single_camera_calib(const Size& image, const Size& board, const string& Save) : image_size(image), board_size(board), square_size(Size(5, 5)), fout(Save) {};
	single_camera_calib(const Size& image, const Size& board) : image_size(image), board_size(board), square_size(Size(5, 5)) {};
	/*与继承类共用变量*/
	/*图像信息*/
	Mat                     image;//保存当前读取的图像
	Size                    board_size;//保存棋盘格大小（内角点，长X宽）
	Size                    image_size;//保存当前图像大小——需显示初始化
	int                     image_count;//计数
	string                  filename;//当前图像位置
	vector<Point2f>         image_points_buf;//保存当前图像识别的点

	/*棋盘格信息*/
	vector<vector<Point3f>> object_points;//棋盘格上角点三维坐标
private:
	/*图像信息*/
	vector<vector<Point2f>> image_points_seq;//保存所有图像识别的点
	vector<string>          filelist;//所有图像位置

	/*内外参数*/
	Mat                     cameraMatrix;//= Mat(3, 3, CV_32FC1, Scalar::all(0));//内参矩阵
	Mat                     distCoeffs;//= Mat(1, 5, CV_32FC1, Scalar::all(0));//摄像机畸变系数
	vector<Mat>             tvecsMat;//外参-每幅图像旋转量的数组
	vector<Mat>             rvecsMat;//外参-每幅图像平移量的数组
	//int                     Num;//图像数量

	/*棋盘格信息*/
	Size                    square_size;//棋盘格实测大小——需显示初始化

	//Size                    square_size;


	/*保存结果文件*/
	ofstream                fout;//保存标定结果文件——显示初始化

};



class double_camera_calib :public single_camera_calib {
public:
	vector<vector<Point2f>> getCorners();//左右相机分别得到各自角点坐标
	int getImagesPath(string Path_left, int Num_left, string Path_right, int Num_right);//
	int calibrate();
	int showCalibratedPictures();
	double_camera_calib(const Size& image, const Size& board, const string SaveName_left, const string SaveName_right) : single_camera_calib(image, board), fout_left(SaveName_left), fout_right(SaveName_right) {};
	//Size image_size;
	int calibratePicturesPairs(string Path_left, string Path_right);
private:
	/*图像信息更新*/
	vector<string>          filelist_left;
	vector<string>          filelist_right;
	vector<vector<Point2f>> image_points_seq_left;//保存左所有图像识别的点
	vector<vector<Point2f>> image_points_seq_right;//保存右所有图像识别的点

	/*内外参数*/
	Mat                     cameraMatrix_left;//= Mat(3, 3, CV_32FC1, Scalar::all(0));//内参矩阵
	Mat                     distCoeffs_left;//= Mat(1, 5, CV_32FC1, Scalar::all(0));//摄像机畸变系数
	vector<Mat>             tvecsMat_left;//外参-每幅图像旋转量的数组
	vector<Mat>             rvecsMat_left;//外参-每幅图像平移量的数组
	Mat                     cameraMatrix_right;//= Mat(3, 3, CV_32FC1, Scalar::all(0));//内参矩阵
	Mat                     distCoeffs_right;//= Mat(1, 5, CV_32FC1, Scalar::all(0));//摄像机畸变系数
	vector<Mat>             tvecsMat_right;//外参-每幅图像旋转量的数组
	vector<Mat>             rvecsMat_right;//外参-每幅图像平移量的数组
	Mat                     R_left;//校正旋转矩阵
	Mat                     R_right;//校正旋转矩阵
	Mat                     P_left;//投影矩阵
	Mat                     P_right;//投影矩阵
	Mat                     Q;//重投影矩阵

	/*双目之间的旋转、平移（以左相机为起始）*/
	Mat                     R;//旋转矢量
	Mat                     T;//平移矢量
	Mat                     E;//本征矩阵
	Mat                     F;//基础矩阵

	/*映射表*/

	Mat mapLx, mapLy, mapRx, mapRy;
	Rect validROIL, validROIR;

	/*保存结果文件更新*/
	ofstream                fout_left;
	ofstream                fout_right;


};