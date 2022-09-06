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
	virtual int getImagesPath(string Path, int Num);//ͼ��·��������
	vector<Point2f> getOneCorners(int count, const vector<string> file);//�궨һ��ͼ��
	virtual vector<vector<Point2f>> getCorners();//�õ�����ͼ��Ľǵ�����
	int getChessboardPoints();//�õ����̸�ǵ���ά���꣨��Ϊ���裩
	virtual int calibrate();//�궨
	virtual int showCalibratedPictures();
	single_camera_calib(const Size& image, const Size& board, const string& Save) : image_size(image), board_size(board), square_size(Size(5, 5)), fout(Save) {};
	single_camera_calib(const Size& image, const Size& board) : image_size(image), board_size(board), square_size(Size(5, 5)) {};
	/*��̳��๲�ñ���*/
	/*ͼ����Ϣ*/
	Mat                     image;//���浱ǰ��ȡ��ͼ��
	Size                    board_size;//�������̸��С���ڽǵ㣬��X��
	Size                    image_size;//���浱ǰͼ���С��������ʾ��ʼ��
	int                     image_count;//����
	string                  filename;//��ǰͼ��λ��
	vector<Point2f>         image_points_buf;//���浱ǰͼ��ʶ��ĵ�

	/*���̸���Ϣ*/
	vector<vector<Point3f>> object_points;//���̸��Ͻǵ���ά����
private:
	/*ͼ����Ϣ*/
	vector<vector<Point2f>> image_points_seq;//��������ͼ��ʶ��ĵ�
	vector<string>          filelist;//����ͼ��λ��

	/*�������*/
	Mat                     cameraMatrix;//= Mat(3, 3, CV_32FC1, Scalar::all(0));//�ڲξ���
	Mat                     distCoeffs;//= Mat(1, 5, CV_32FC1, Scalar::all(0));//���������ϵ��
	vector<Mat>             tvecsMat;//���-ÿ��ͼ����ת��������
	vector<Mat>             rvecsMat;//���-ÿ��ͼ��ƽ����������
	//int                     Num;//ͼ������

	/*���̸���Ϣ*/
	Size                    square_size;//���̸�ʵ���С��������ʾ��ʼ��

	//Size                    square_size;


	/*�������ļ�*/
	ofstream                fout;//����궨����ļ�������ʾ��ʼ��

};



class double_camera_calib :public single_camera_calib {
public:
	vector<vector<Point2f>> getCorners();//��������ֱ�õ����Խǵ�����
	int getImagesPath(string Path_left, int Num_left, string Path_right, int Num_right);//
	int calibrate();
	int showCalibratedPictures();
	double_camera_calib(const Size& image, const Size& board, const string SaveName_left, const string SaveName_right) : single_camera_calib(image, board), fout_left(SaveName_left), fout_right(SaveName_right) {};
	//Size image_size;
	int calibratePicturesPairs(string Path_left, string Path_right);
private:
	/*ͼ����Ϣ����*/
	vector<string>          filelist_left;
	vector<string>          filelist_right;
	vector<vector<Point2f>> image_points_seq_left;//����������ͼ��ʶ��ĵ�
	vector<vector<Point2f>> image_points_seq_right;//����������ͼ��ʶ��ĵ�

	/*�������*/
	Mat                     cameraMatrix_left;//= Mat(3, 3, CV_32FC1, Scalar::all(0));//�ڲξ���
	Mat                     distCoeffs_left;//= Mat(1, 5, CV_32FC1, Scalar::all(0));//���������ϵ��
	vector<Mat>             tvecsMat_left;//���-ÿ��ͼ����ת��������
	vector<Mat>             rvecsMat_left;//���-ÿ��ͼ��ƽ����������
	Mat                     cameraMatrix_right;//= Mat(3, 3, CV_32FC1, Scalar::all(0));//�ڲξ���
	Mat                     distCoeffs_right;//= Mat(1, 5, CV_32FC1, Scalar::all(0));//���������ϵ��
	vector<Mat>             tvecsMat_right;//���-ÿ��ͼ����ת��������
	vector<Mat>             rvecsMat_right;//���-ÿ��ͼ��ƽ����������
	Mat                     R_left;//У����ת����
	Mat                     R_right;//У����ת����
	Mat                     P_left;//ͶӰ����
	Mat                     P_right;//ͶӰ����
	Mat                     Q;//��ͶӰ����

	/*˫Ŀ֮�����ת��ƽ�ƣ��������Ϊ��ʼ��*/
	Mat                     R;//��תʸ��
	Mat                     T;//ƽ��ʸ��
	Mat                     E;//��������
	Mat                     F;//��������

	/*ӳ���*/

	Mat mapLx, mapLy, mapRx, mapRy;
	Rect validROIL, validROIR;

	/*�������ļ�����*/
	ofstream                fout_left;
	ofstream                fout_right;


};