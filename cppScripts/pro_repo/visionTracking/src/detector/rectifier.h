#pragma once
#include "opencv2/opencv.hpp"
#include <opencv2/ml/ml.hpp>

class Rectifier
{
public:
	Rectifier(const cv::Size& img_size)
	{
		init_rectify_para(img_size);
		//init_rectify_para_offline();
	}

	enum ImgIdx
	{
		LEFT,
		RIGHT
	};
	cv::Mat rectify(const cv::Mat& img, const ImgIdx& id) const
	{
		cv::Mat res;
		switch (id)
		{
		case ImgIdx::LEFT:
			cv::remap(img, res, left_map1, left_map2, cv::INTER_LINEAR);
			break;
		case ImgIdx::RIGHT:
			cv::remap(img, res, right_map1, right_map2, cv::INTER_LINEAR);
			break;
		}

		return res;
	}

private:
	void init_rectify_para(const cv::Size& img_size)//
	{

		cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 2581.34315054188, 0, 1344.46210385682,
			0, 2582.09162968203, 841.136063439237,
			0, 0, 1);
		cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
			<< -0.134691513585362, 0.130592337587337,0, -7.60794452605355e-05, 8.10921732322988e-05);

		cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 2551.54901397146, 0, 1278.22111026208,
			0, 2551.25374595165, 762.718182817149,
			0, 0, 1);
		cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
			<< -0.141803742942858, 0.162131222993819,0, 0.000318546745358037, 0.000369389151228364);

		const cv::Mat RR = (cv::Mat_<double>(3, 3)
			<< 0.999992446350254, -0.00382237708291059, 0.000704752347185453,
			0.00381656249749037, 0.999960112469338, 0.00807510501552604,
			-0.000735590332708053, -0.00807235428763280, 0.999967147461914);

		cv::Mat t = (cv::Mat_<double>(3, 1)
			<< -50.2422591120947, -0.696494931298555, -0.0719422764092836);	//before year


		//cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
		//	<< 1096.6, 0, 1004.6,
		//	0, 1101.2, 547.8316,
		//	0, 0, 1);
		//cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
		//	<< 0.0757, -0.0860, -2.2134e-4, 2.4925e-4, 0.00000);

		//cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
		//	<< 1095.2, 0, 996.3653,
		//	0, 1100.7, 572.7941,
		//	0, 0, 1);
		//cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
		//	<< 0.0775, -0.0932, -9.2589e-4, 1.5443e-4, 0.00000);

		//const cv::Mat RR = (cv::Mat_<double>(3, 3)
		//	<< 0.999991605633247, -0.000345422533180, 0.004082811079910,
		//	0.000333318919503, 0.999995549306528, 0.002964838213577,
		//	-0.004083817030495, -0.002963452447460, 0.999987270113001);

		//cv::Mat t = (cv::Mat_<double>(3, 1)
		//	<< -4.049079591541234, -0.004234103893078, -0.065436975906188);	//before year

		/*cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1104.2, -2.4, 1008.5,
			0, 1109.4, 546.4,
			0, 0, 1);
		cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0814, -0.0882, 2.399e-4, 2.69e-4, 0.00000);

		cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1102.8, -0.8, 999.7,
			0, 1109.4, 571.6,
			0, 0, 1);
		cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0847, -0.0969, -7.128e-4, -5.977e-4, 0.00000);

		const cv::Mat RR = (cv::Mat_<double>(3, 3)
			<< 1, 0.0007, 0.0033,
			-0.0008, 1, 0.004,
			-0.0033, -0.004, 1);

		//after year
		cv::Mat t = (cv::Mat_<double>(3, 1)
			<< -4.0176, 0.0395, -0.0046);*/

		/* 20200513_8#_1*/
		/*cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1075.8, -1.1, 1007.2,
			0, 1082.4, 529.7,
			0, 0, 1);
		cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0751, -0.0808, -7.5e-6, 8.826e-4, 0.00000);

		cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1079.8, -1.5, 1011.0,
			0, 1086.4, 542.9,
			0, 0, 1);
		cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0824, -0.0883, 3.0e-4, 1.9e-3, 0.00000);

		const cv::Mat RR = (cv::Mat_<double>(3, 3)
			<< 1, 0.0007, 0.0006,
			-0.0007, 1, 0.0065,
			-0.0006, -0.0065, 1);

		cv::Mat t = (cv::Mat_<double>(3, 1)
			<< -4.0950, -0.0419, 0.2869);	*/

		/* 20200513_8#_1_1*/
		//cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
		//	<< 1071.7168, -0.815, 1008.1466 - 1,
		//	0, 1078.37136, 530.394 - 1,
		//	0, 0, 1);
		//cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
		//	<< 0.074562686357436, - 0.079519468581189, -8.213e-5, 7.2867e-4, 0.00000);

		//cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
		//	<< 1074.85162, -1.59856, 1011.6617 - 1,
		//	0, 1081.3193, 543.29855 - 1,
		//	0, 0, 1);
		//cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
		//	<< 0.081559896790157, - 0.085768449858552, 0.000237424530315, 0.001930937925259, 0.00000);

		//const cv::Mat RR = (cv::Mat_<double>(3, 3)
		//	<< 0.999999187264726,   0.000547667464618,   0.001151316740009,
		//	- 0.000555541146737,   0.999976386279779,   0.006849690275411,
		//	- 0.001147538200631, - 0.006850324312248,   0.999975877815508);

		//cv::Mat t = (cv::Mat_<double>(3, 1)
		//	<< -4.034538343537860,   0.009984433312119,   0.232970426330152);

		/* 20200513_8#_2*/
		//cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
		//	<< 1079.77186982183, -0.367982924631244,          1006.51262023875,
		//	0 ,         1086.21427976457,          526.487983456705,
		//	0,                         0 ,                        1);
		//cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
		//	<< 0.0725093385656308, - 0.075006188030748, -0.00133055640917461,      0.000366745997333382, 0.00000);

		//cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
		//	<< 1078.34532744803, -1.1552181105112,          1007.97058450723,
		//	0,          1085.02361142418,          539.730330943253,
		//	0,                         0,                         1);
		//cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
		//	<< 0.0818196555579786, -0.0864852804829531, -0.000858985678981031,       0.00138137275726706, 0.00000);

		//const cv::Mat RR = (cv::Mat_<double>(3, 3)
		//	<< 0.99999956167026,      0.000432925364390533,     0.000830201732700478,
		//	- 0.00043884606822172,          0.99997437948551,       0.00714477338320185,
		//	- 0.000827087308884551, - 0.00714513458220142,         0.999974131154594);

		//cv::Mat t = (cv::Mat_<double>(3, 1)
		//	<< -3.91594686220194 - 0.00114289829821496 - 0.0266335836048761);

		/* 20200513_8#_1_3*/
		/*cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1073.33371214815, - 0.401536598346964,          1011.03105727049 - 1,
			0,          1079.56809055395,          530.497472624825 - 1,
			0,                         0,                         1);
		cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0763098008888325, - 0.0818023605012234, -0.000195325357216444,      0.000786056632674668, 0.00000);

		cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1080.70703773029, - 1.34216488729895,          1011.98500664377 - 1,
			0,          1087.10722912051,          542.410940770911 - 1,
			0,                         0,                         1);
		cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0830129208433482, - 0.0877301361376609, 7.3600765273296e-06,        0.0019409087679065, 0.00000);

		const cv::Mat RR = (cv::Mat_<double>(3, 3)
			<< 0.999999749704242,      0.000365459631198382,      0.000605830596418752,
			- 0.000368964649990157,          0.99998313790287,       0.00579549609748529,
			- 0.00060370236097799, - 0.00579571817697117,         0.999983022453018);

		cv::Mat t = (cv::Mat_<double>(3, 1)
			<< -3.90017350836936,        0.0106857527184255,         0.543034940084207);*/

		/* 20200513_8#_1_4*/
		/*cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1085.64341578531, - 0.687245116022428,          1006.50609258889 - 1,
			0,          1092.07961888344,          526.889877921805 - 1,
			0,                         0,                         1);
		cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0753898814526862, - 0.0815048758618605, -0.000959483850198064,      0.000366744345212011, 0.00000);

		cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
			<< 1086.60162553224, - 1.44980966395366,          1009.08792650481 - 1,
			0,          1093.04732567911,          540.107936389197 - 1,
			0,                         0,                         1);
		
		cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
			<< 0.0830681592673981, - 0.088806857730219, -0.000629994711320965,       0.00146491491996792, 0.00000);

		const cv::Mat RR = (cv::Mat_<double>(3, 3)
			<< 0.999999581448362,      0.000461457203930522,      0.000790038195700223,
			- 0.000466802916353034,         0.999976909039004,       0.00677963744137547,
			- 0.000786891440421702, - 0.00678000339588089,         0.999976705906599);

		cv::Mat t = (cv::Mat_<double>(3, 1)
			<< -4.00168253900523, - 0.0160669249225035,         0.116345470508562);*/

			/* 20200524_8#_1_1*/
	//cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
	//	<< 1089.75412596249, - 1.03252218627915,          1001.42260300135 - 1,
	//	0,          1096.49579528382,          526.239058004513 - 1,
	//	0,                         0,                         1);
	//cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
	//	<< 0.0776956475722123, - 0.0837677817100938, -0.0015961509607403, - 9.64582721053099e-05, 0.00000);

	//cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
	//	<< 1088.9416711783, - 1.89137579721679,          1008.40740961482 - 1, 
	//	0,          1094.83165439802,          541.438876193291 - 1,
	//	0,                         0,                         1);

	//cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
	//	<< 0.0827329663875497, - 0.0884353610165518, -0.00106293230765931,       0.00132225768562438, 0.00000);

	//const cv::Mat RR = (cv::Mat_<double>(3, 3)
	//	<< 0.999998554795346,       0.00058333949680443,        0.0015969102200989,
	//	- 0.00059421290331237,         0.999976587144849,       0.00681704284612838,
	//	- 0.00159289618152767, - 0.00681798189876455,         0.999975488601887);

	//cv::Mat t = (cv::Mat_<double>(3, 1)
	//	<< -4.26048345806762, - 0.132320270401973, - 0.0410620081857698);

	/*20200607 #11_1*/
	//cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3)
	//	<< 1105.51781978229, - 0.377865749189934,          981.722162948607,
	//	0,          1111.76479178759,          583.614591540024,
	//	0,                         0,                         1);
	//cv::Mat left_distortion = (cv::Mat_<double>(1, 5)
	//	<< 0.0866352440010647, - 0.0968047709333446, 0.000418803789960815,      0.000580510730540805, 0.00000);

	//cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3)
	//	<< 1106.62997521243,        0.0946826152839248,          964.898995665339,
	//	0,          1111.99932755666,          580.187563236892,
	//	0,                         0,                         1);

	//cv::Mat right_distortion = (cv::Mat_<double>(1, 5)
	//	<< 0.0820513858570241, - 0.0921664547816551, 0.000224065046895253, - 2.6508133001942e-05, 0.00000);

	//const cv::Mat RR = (cv::Mat_<double>(3, 3)
	//	<< 0.999982495361019, - 0.000529546639632001, - 0.00589309357686899,
	//	0.000548668376467724,          0.99999458890849,       0.00324362709837338,
	//	0.0058913440369702, - 0.00324680367393723,         0.999977374909723);

	//cv::Mat t = (cv::Mat_<double>(3, 1)
	//	<< -4.1499722636852, - 0.25168590920162,         0.322670269646619);

		cv::Mat R;
		cv::transpose(RR, R);

		cv::Mat R1, R2, P1, P2, Q;
		cv::stereoRectify(
			left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, img_size, R, t,
			R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY);

		cv::initUndistortRectifyMap(
			left_camera_matrix, left_distortion, R1, P1, img_size, CV_16SC2,
			left_map1, left_map2);

		cv::initUndistortRectifyMap(
			right_camera_matrix, right_distortion, R2, P2, img_size, CV_16SC2,
			right_map1, right_map2);

		cv::Mat left_map1_temp;
		cv::Mat left_map2_temp;
		cv::Mat right_map1_temp;
		cv::Mat right_map2_temp;
		left_map1_temp = left_map1;
		left_map2_temp = left_map2;
		right_map1_temp = right_map1;
		right_map2_temp = right_map2;

		std::cout << "P1:" << P1 << std::endl;
		std::cout << "P2:" << P2 << std::endl;
	}

	void init_rectify_para_offline()
	{
		cv::Ptr<cv::ml::TrainData> train_data;

		train_data = cv::ml::TrainData::loadFromCSV("../../map/xmap1.csv", 0);
		left_map1 = train_data->getTrainSamples();
		train_data = cv::ml::TrainData::loadFromCSV("../../map/ymap1.csv", 0);
		left_map2 = train_data->getTrainSamples();
		train_data = cv::ml::TrainData::loadFromCSV("../../map/xmap2.csv", 0);
		right_map1 = train_data->getTrainSamples();
		train_data = cv::ml::TrainData::loadFromCSV("../../map/ymap2.csv", 0);
		right_map2 = train_data->getTrainSamples();


		cv::Mat left_map1_temp;
		cv::Mat left_map2_temp;
		cv::Mat right_map1_temp;
		cv::Mat right_map2_temp;
		left_map1_temp = left_map1;
		left_map2_temp = left_map2;
		right_map1_temp = right_map1;
		right_map2_temp = right_map2;

		//train_data = cv::ml::TrainData::loadFromCSV("../../map/ymap2.csv", 0);
	}

private:
	cv::Mat left_map1;
	cv::Mat left_map2;
	cv::Mat right_map1;
	cv::Mat right_map2;
};