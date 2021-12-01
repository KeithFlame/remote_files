#pragma once

#include <opencv2\ml\ml.hpp>

#include <opencv2\opencv.hpp>





class Rectifier

{

public:

	Rectifier(const cv::Size& img_size)

	{

		assert(img_size.width == 2592 && img_size.height == 1944);

		init_rectify_para();

	}



	enum ImgIdx

	{

		LEFT,

		RIGHT,



		INVAILD

	};



	cv::Mat rectify(const cv::Mat& img, const ImgIdx& id = INVAILD) const

	{

		cv::Rect lRect(1296, 972, 2592, 1944);

		cv::Rect rRect(1296, 972, 2592, 1944);

		cv::Mat lmap1, lmap2, rmap1, rmap2;

		cv::Mat res;

		switch (id)

		{

		case ImgIdx::LEFT:

			lmap1 = left_map1(lRect);

			lmap2 = left_map2(lRect);

			cv::remap(img, res, lmap1, lmap2, cv::INTER_LINEAR);

			//cv::remap(res, res, fov_ymap, fov_xmap, cv::INTER_LINEAR);

			break;

		case ImgIdx::RIGHT:

			rmap1 = right_map1(rRect);

			rmap2 = right_map2(rRect);

			cv::remap(img, res, rmap1, rmap2, cv::INTER_LINEAR);

			//cv::remap(res, res, fov_ymap, fov_xmap, cv::INTER_LINEAR);

			break;

		default:

			return img;

		}



		return res;

	}



private:

	void init_rectify_para()

	{

		cv::Ptr<cv::ml::TrainData> train_data;



		train_data = cv::ml::TrainData::loadFromCSV("./map/mapx1.csv", 0); //xmap1_d1012.csv

		left_map1 = train_data->getTrainSamples();

		train_data = cv::ml::TrainData::loadFromCSV("./map/mapy1.csv", 0);  //ymap1_d1012.csv

		left_map2 = train_data->getTrainSamples();

		train_data = cv::ml::TrainData::loadFromCSV("./map/mapx2.csv", 0);  //xmap2_d1012.csv

		right_map1 = train_data->getTrainSamples();

		train_data = cv::ml::TrainData::loadFromCSV("./map/mapy2.csv", 0);  //ymap2_d1012.csv

		right_map2 = train_data->getTrainSamples();



		////xmap1_for_fov.csv

		//train_data = cv::ml::TrainData::loadFromCSV("map/xmap1_for_fov.csv", 0);  //xmap1_for_fov.csv

		//fov_xmap = train_data->getTrainSamples();

		//train_data = cv::ml::TrainData::loadFromCSV("map/ymap1_for_fov.csv", 0);  //ymap1_for_fov.csv

		//fov_ymap = train_data->getTrainSamples();

	}



private:

	cv::Mat left_map1;

	cv::Mat left_map2;

	cv::Mat right_map1;

	cv::Mat right_map2;

	//cv::Mat fov_xmap;

	//cv::Mat fov_ymap;

};

