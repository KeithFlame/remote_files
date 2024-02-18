#include "marker_identifier_new.h"
#include <valarray>  
#include <time.h>

static float minDistancePointRect(const cv::Point2f& px, const std::vector<cv::Point>& rect)
{
	float min_dis = 1e10;
	for (size_t i = 0; i < 4; i++)
	{
		cv::Point2f pA = cv::Point2f(rect.at(i).x, rect.at(i).y);
		cv::Point2f pB = cv::Point2f(rect.at((i + 1) % 4).x, rect.at((i + 1) % 4).y);

		float ab = sqrtf(powf((pB.x - pA.x), 2) + powf((pB.y - pA.y), 2)); // |AB|
		float ap = sqrtf(powf((px.x - pA.x), 2) + powf((px.y - pA.y), 2)); // |AP|
		float bp = sqrtf(powf((px.x - pB.x), 2) + powf((px.y - pB.y), 2)); // |BP|
		float r = ((px.x - pA.x) * (pB.x - pA.x) + (px.y - pA.y) * (pB.y - pA.y)) / powf(ab, 2);

		float distance = 0;
		if (ab > 0)
		{
			if (r >= 1)
				distance = bp;
			else if (r > 0 && r < 1)
				distance = sqrt(pow(ap, 2) - r * r * pow(ab, 2));
			else
				distance = ap;
		}

		if (distance < min_dis)
		{
			min_dis = distance;
		}
	}

	return min_dis;
}

static bool findCircles_sub(const std::vector<cv::Point>& cR, const cv::Mat& image,
	std::vector<cv::Point2f>& temp_result)
{
	std::vector<int> minx, miny, maxx, maxy;
	const float areaPortion = 2;
	bool circle_found = false;

	std::vector<std::vector<cv::Point>> contoursUpdated2;
	cv::Mat roi;
	int temp_minx = std::max(std::min({ cR[0].x, cR[1].x, cR[2].x, cR[3].x }), 0);
	int temp_miny = std::max(std::min({ cR[0].y, cR[1].y, cR[2].y, cR[3].y }), 0);
	int temp_maxx = std::min(std::max({ cR[0].x, cR[1].x, cR[2].x, cR[3].x }), image.cols - 1);
	int temp_maxy = std::min(std::max({ cR[0].y, cR[1].y, cR[2].y, cR[3].y }), image.rows - 1);
	cv::Mat temp = image(cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx, temp_maxy - temp_miny));

	// if roi is similar to those already exist.
	bool unique_roi = true;
	for (size_t i = 0; i < minx.size(); i++) {
		if (fabs(temp_minx + temp_maxx - minx[i] - maxx[i]) < (temp_maxx - temp_minx) * 0.2 &&
			fabs(temp_miny + temp_maxy - miny[i] - maxy[i]) < (temp_maxy - temp_miny) * 0.2) {
			unique_roi = false;
			break;
		}
	}
	if (unique_roi == false) {
		return false;
	}
	//printf("#####find8Point #2\n");

	// method 2
	cv::Mat temp2;
	cv::boxFilter(temp, temp2, -1, cv::Size(21, 21));
	temp2 = temp - temp2;
	roi = temp2 > 10;  // 0.04;

	//printf("#####find8Point #2.1\n");
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::morphologyEx(roi, roi, cv::MORPH_OPEN, element);  // eliminate while noise in black region.
	cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, element);  // eliminate black noise in white region.
	cv::Canny(roi, roi, 5, 50.0 * 2, 5);

	//printf("#####find8Point #2.2\n");
	std::vector<std::vector<cv::Point>> contoursInRect;
	cv::findContours(roi, contoursInRect, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	//printf("#####find8Point #3\n");
		// 1. overlap contour erasing.
		// if two contours' area is similar, and distance is small, then erase one of them.
		// 2. too little and too big contour erasing.
		// 3. verify poins are in the rect.
	std::vector<std::vector<cv::Point>> contoursUpdated0;
	for (size_t i = 0; i < contoursInRect.size(); i++) {
		auto getCross = [](const cv::Point& p1, const cv::Point& p2, const cv::Point& p) {
			return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
		};
		auto sign = [](const double x) {
			if (x > 0) {
				return 1;
			}
			else if (x < 0) {
				return -1;
			}
			else {
				return 0;
			}
		};

		if (contoursInRect[i].size() < 10) {  // too little points
			continue;
		}
		cv::RotatedRect boxi = cv::fitEllipse(contoursInRect[i]);
		double areai = cv::contourArea(contoursInRect[i]);
		if (areai < 0.001 * roi.cols * roi.rows || areai > 0.1 * roi.cols * roi.rows) {
			continue;
		}
		bool unique = 1;
		for (const auto& cUj : contoursUpdated0) {
			cv::RotatedRect boxj = cv::fitEllipse(cUj);
			if ((pow(boxi.center.x - boxj.center.x, 2) + pow(boxi.center.y - boxj.center.y, 2)) * CV_PI < areai &&
				abs(cv::contourArea(cUj) - areai) < areai * 0.4) {
				unique = 0;
				break;
			}
		}

		if (unique == 1 && sign(getCross(cR[0], cR[1], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) * sign(getCross(cR[2], cR[3], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) >= 0 &&
			sign(getCross(cR[1], cR[2], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) * sign(getCross(cR[3], cR[0], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) >= 0) {
			contoursUpdated0.emplace_back(contoursInRect[i]);
		}
	}

	//printf("#####find8Point #4\n");
	if (contoursUpdated0.size() < 8) {
		return false;
	}

	// concentric area erasing
	std::vector<std::vector<cv::Point>> contoursUpdated_0_5;  // 0.5
	for (size_t i = 0; i < contoursUpdated0.size(); i++) {
		double area_i = cv::contourArea(contoursUpdated0[i]);
		cv::RotatedRect box_i = cv::fitEllipse(contoursUpdated0[i]);

		bool inner = false;
		for (size_t j = 0; j < contoursUpdated0.size(); j++) {
			double area_j = cv::contourArea(contoursUpdated0[j]);
			cv::RotatedRect box_j = cv::fitEllipse(contoursUpdated0[j]);

			// concentric area. the smaller one is erased.
			if ((pow(box_i.center.x - box_j.center.x, 2) + pow(box_i.center.y - box_j.center.y, 2)) * CV_PI < area_i &&
				area_i < area_j) {
				inner = true;
				break;
			}
		}
		if (inner == false) {
			contoursUpdated_0_5.emplace_back(contoursUpdated0[i]);
		}
	}


	//printf("#####find8Point #5\n");
		// 1. contour area voting.
	int candidate = -1;
	for (size_t i = 0; i < contoursUpdated_0_5.size(); i++) {
		double area_i = cv::contourArea(contoursUpdated_0_5[i]);
		int pixel_i = contoursUpdated_0_5[i].size();
		cv::RotatedRect box_i = cv::fitEllipse(contoursUpdated_0_5[i]);

		int voteNum = 0;
		for (size_t j = 0; j < contoursUpdated_0_5.size(); j++) {
			double area_j = cv::contourArea(contoursUpdated_0_5[j]);
			int pixel_j = contoursUpdated_0_5[j].size();
			cv::RotatedRect box_j = cv::fitEllipse(contoursUpdated_0_5[j]);

			// concentric area. the smaller one is erased.
			if ((pow(box_i.center.x - box_j.center.x, 2) +
				pow(box_i.center.y - box_j.center.y, 2)) * CV_PI < area_i &&
				area_i < area_j) {
				voteNum = 0;
				break;
			}

			// vote
			if (area_j / area_i < areaPortion && area_i / area_j < areaPortion &&
				pixel_i / pixel_j < areaPortion && pixel_j / pixel_i < areaPortion) {
				voteNum++;
			}
		}

		if (voteNum >= 8) {
			candidate = i;
			break;
		}
	}
	if (candidate == -1) {
		return false;
	}

	//printf("#####find8Point #6\n");
		// update contours1.
	std::vector<std::vector<cv::Point>> contoursUpdated1;
	float areaCan = cv::contourArea(contoursUpdated_0_5[candidate]);
	int pixelCan = contoursUpdated_0_5[candidate].size();
	for (size_t i = 0; i < contoursUpdated_0_5.size(); i++) {
		float area_i = cv::contourArea(contoursUpdated_0_5[i]);
		int pixel_i = contoursUpdated_0_5[i].size();
		if (areaCan / area_i < areaPortion && area_i / areaCan < areaPortion &&
			pixel_i / pixelCan < areaPortion && pixelCan / pixel_i < areaPortion) {
			contoursUpdated1.emplace_back(contoursUpdated_0_5.at(i));
		}
	}

	// distance voting. points which are too close to the edge are erased.
	candidate = -1;
	//float ppDistance = sqrt(areaCan / CV_PI) * 8;  // r*8
	float temp_pR = sqrt(areaCan / CV_PI);
	for (size_t i = 0; i < contoursUpdated1.size(); i++) {
		auto getCross = [](const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p) {
			return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
		};

		cv::RotatedRect boxi = cv::fitEllipse(contoursUpdated1[i]);
		bool validate = true;
		for (int j = 0; j < 4; j++) {  // point j (0-3)
			cv::Point2f corner0 = cv::Point2f(cR[(j + 1) % 4].x - temp_minx, cR[(j + 1) % 4].y - temp_miny);
			cv::Point2f corner1 = cv::Point2f(cR[(j) % 4].x - temp_minx, cR[(j) % 4].y - temp_miny);
			float dis = fabs(getCross(corner0, corner1, boxi.center)) / sqrt(pow(corner0.x - corner1.x, 2) + pow(corner0.y - corner1.y, 2));
			if (dis < temp_pR) {
				validate = false;
				break;
			}
		}
		if (validate == true) {
			contoursUpdated2.emplace_back(contoursUpdated1.at(i));
		}
	}

	//printf("#####find8Point #7\n");
		// update output
	if (contoursUpdated2.size() == 8) {
		circle_found = true;
		//pR.emplace_back(temp_pR);
		minx.emplace_back(temp_minx);
		maxx.emplace_back(temp_maxx);
		miny.emplace_back(temp_miny);
		maxy.emplace_back(temp_maxy);
		//roi = cv::Scalar::all(0);
		for (int i = 0; i < 8; i++)
		{
			cv::RotatedRect box = cv::fitEllipse(contoursUpdated2[i]);

			// draw the ellipse
			//cv::drawContours(out, contoursUpdated2, i, cv::Scalar::all(100));
			//cv::ellipse(roi, box, cv::Scalar::all(255));

			temp_result.emplace_back(cv::Point2f(box.center.x, box.center.y) +
				cv::Point2f(temp_minx, temp_miny));
		}
	}

	return circle_found;
}

static bool findCircles_new_sub(const std::vector<cv::Point>& cR, const cv::Mat& image,
	std::vector<cv::Point2f>& temp_result)
{
	const float AREA_THRED = 8.0;
	const float SCALE_THRED = 1.8;
	const int NEAR_BORDER_THRED = 2;
	bool circle_found = false;

	cv::Mat roi;
	int temp_minx = std::max(std::min({ cR[0].x, cR[1].x, cR[2].x, cR[3].x }), 0);
	int temp_miny = std::max(std::min({ cR[0].y, cR[1].y, cR[2].y, cR[3].y }), 0);
	int temp_maxx = std::min(std::max({ cR[0].x, cR[1].x, cR[2].x, cR[3].x }), image.cols - 1);
	int temp_maxy = std::min(std::max({ cR[0].y, cR[1].y, cR[2].y, cR[3].y }), image.rows - 1);
	//int scale_rect = std::min(temp_maxx - temp_minx, temp_maxy - temp_miny)*0;
	//temp_maxx = std::min(temp_maxx + scale_rect, image.cols - 1);
	//temp_maxy = std::min(temp_maxy + scale_rect, image.rows - 1);
	//temp_minx = std::max(temp_minx - scale_rect,0);
	//temp_miny = std::max(temp_miny - scale_rect,0);
	cv::Mat img_roi = image(cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx,
		temp_maxy - temp_miny));// .clone();

	//cv::Mat imageSrc;
	//cv::cvtColor(img_roi, imageSrc, cv::COLOR_GRAY2BGR);
	//cv::imshow("",img_roi);
	//cv::waitKey();

	cv::Mat mask = cv::Mat::zeros(img_roi.size(), CV_8UC1);
	cv::Point start = cv::Point(temp_minx, temp_miny);
	std::vector<std::vector<cv::Point>> temp_cts;
	std::vector<cv::Point> temp_pnts;
	for (size_t ddi = 0; ddi < 4; ddi++)
	{
		temp_pnts.push_back(cR.at(ddi) - start);
	}
	temp_cts.push_back(temp_pnts);
	cv::drawContours(mask, temp_cts, 0, cv::Scalar::all(255), -1);

	cv::Mat img_dst;
	img_roi.copyTo(img_dst, mask);
	img_roi = img_dst.clone();
	//cv::imshow("img_roi", img_roi);
	//cv::waitKey();

	//float thresh1 = cv::threshold(img_roi, img_dst, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY_INV);
	//float thresh2 = cv::mean(img_roi)[0];
	//float refined_thresh = MIN(thresh1, 0.8 * thresh2);
	//img_roi = img_roi < refined_thresh;

	//find circles
	cv::threshold(img_roi, img_roi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	std::vector<cv::Point2f> clc_centers;
	std::vector<float> clc_areas;
	std::vector<float> clc_scales;


	//cv::Mat imageSrc1 = imageSrc.clone();
	// method 2
	cv::Mat labels, stats, centroids;
	int num = cv::connectedComponentsWithStats(img_roi, labels, stats, centroids);
	for (uint16_t i = 1; i < num; i++) {
		float area = (float)stats.at<int>(i, 4);
		if (area > AREA_THRED && area < 10000) {
			float width = (float)stats.at<int>(i, 2);
			float height = (float)stats.at<int>(i, 3);
			float scale = (float)width / (float)height;
			if (scale < SCALE_THRED && scale > 1.0 / SCALE_THRED)
			{
				float xc = (float)centroids.at<double>(i, 0);
				float yc = (float)centroids.at<double>(i, 1);
				cv::Point2f pc = cv::Point2f(xc, yc);

				//not at the border
				if (minDistancePointRect(pc, temp_pnts) > NEAR_BORDER_THRED)
				{
					clc_centers.push_back(pc + cv::Point2f(temp_minx, temp_miny));
					if (scale > 1.0)
						clc_scales.push_back(scale);
					else
						clc_scales.push_back(1.0 / scale);

					clc_areas.push_back(area);
					//circle(imageSrc, clc_centers.back(), 6, cv::Scalar(0, 0, 255), 1);
				}
			}
		}
	}
	//cv::imshow("imageSrc", img_roi);

	//去除过小区域，初始化颜色表
	//std::vector<cv::Vec3b> colors(num);
	//colors[0] = cv::Vec3b(0, 0, 0); // background pixels remain black.
	//for (int i = 1; i < num; i++) {
	//	colors[i] = cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
	//	//去除面积小于100的连通域
	//	if (stats.at<int>(i, cv::CC_STAT_AREA) < 100)
	//		colors[i] = cv::Vec3b(0, 0, 0); // small regions are painted with black too.
	//}
	////按照label值，对不同的连通域进行着色
	//cv::Mat img_color = cv::Mat::zeros(img_roi.size(), CV_8UC3);
	//for (int y = 0; y < img_color.rows; y++)
	//	for (int x = 0; x < img_color.cols; x++)
	//	{
	//		int label = labels.at<int>(y, x);
	//		//cv::CV_Assert(0 <= label && label <= n);
	//		img_color.at<cv::Vec3b>(y, x) = colors[label];
	//	}

	////统计降噪后的连通区域
	////cv::cvtColor(img_color, img_gray, cv::COLOR_BGR2GRAY);
	////threshold(img_gray, img_gray, 1, 255, cv::THRESH_BINARY);
	////n = cv::connectedComponentsWithStats(img_gray, labels, stats, centroids);
	////sprintf_s(title, "过滤小目标后的连通区域数量：%d\n", n);
	////num_connect = title;
	//imshow("", img_color);

	//cv::waitKey();

	if (clc_centers.size() >= 8)
	{
		circle_found = true;
		if (clc_centers.size() == 8)
		{
			temp_result = clc_centers;
		}
		else
		{
			float area_sum = std::accumulate(std::begin(clc_areas), std::end(clc_areas), 0.0);
			float area_avg = area_sum / ((float)clc_areas.size());
			struct clcSorted
			{
				clcSorted() : area(0.0), cntr(0, 0) {}
				float area;
				cv::Point2f cntr;
			};

			std::vector<clcSorted> clcsSorted;
			for (size_t cci = 0; cci < clc_centers.size(); cci++)
			{
				clcSorted temp_clcs;
				temp_clcs.area = fabs(clc_areas.at(cci) - area_avg)
					+ (clc_scales.at(cci) - 1.0) * 1 * area_avg;
				temp_clcs.cntr = clc_centers.at(cci);
				clcsSorted.push_back(temp_clcs);
			}
			std::sort(clcsSorted.begin(), clcsSorted.end(),
				[](const auto& lhs, const auto& rhs) { return lhs.area < rhs.area; });

			std::vector<cv::Point2f> clc_8centers;
			for (size_t eti = 0; eti < 8; eti++)
			{
				clc_8centers.push_back(clcsSorted.at(eti).cntr);
				//circle(imageSrc1, clc_8centers.back(), 6, cv::Scalar(0, 0, 255), 1);
			}
			temp_result = clc_8centers;
		}
	}

	//cv::Mat img_color = cv::Mat::zeros(image.size(), CV_8UC3);
	//for (int i = 0; i < temp_result.size(); i++) {
	//	cv::circle(img_color, temp_result[i], 15, cv::Scalar(0, 255, 120), -1);//画点，其实就是实心圆
	//}
	//cv::resize(img_color, img_color, cv::Size(), 0.125,0.125);
	//cv::imshow("imageSrc1", img_color(cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx,
	//	temp_maxy - temp_miny)));
	//cv::waitKey();

	return circle_found;
}


// rectify
using PixelType = float;
const PixelType PI = CV_PI;
struct CostFunctorDistortion {
	CostFunctorDistortion(const PixelType& x_d, const PixelType& y_d,
		const double& k1, const double& k2, const double& p1, const double& p2)
		:x_d(x_d), y_d(y_d), k1(k1), k2(k2), p1(p1), p2(p2)
	{
	}

	template <typename T>
	bool operator()(const T* const x, T* residual) const {

		T r_xy = T(x[0] * x[0] + x[1] * x[1]);
		residual[0] = T(x_d) - (x[0] * (T(1) + T(k1) * r_xy + T(k2) * r_xy * r_xy) + T(2) * T(p1) * x[0] * x[1] + T(p2) * (r_xy + T(2) * x[0] * x[0]));
		residual[1] = T(y_d) - (x[1] * (T(1) + T(k1) * r_xy + T(k2) * r_xy * r_xy) + T(2) * T(p2) * x[0] * x[1] + T(p1) * (r_xy + T(2) * x[1] * x[1]));

		return true;
	}

	PixelType x_d;
	PixelType y_d;
	double k1;
	double k2;
	double p1;
	double p2;
};

Corner rectifySinglePixel(const Corner pixel, const Eigen::Vector4d distortion_coeff)
{
	// The variable to solve for with its initial value.
	double x_ud[2] = { pixel.x, pixel.y };

	// Build the problem.
	ceres::Problem problem;

	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	problem.AddResidualBlock(
		new ceres::AutoDiffCostFunction<CostFunctorDistortion, 2, 2>(
			new CostFunctorDistortion(pixel.x, pixel.y, distortion_coeff(0), distortion_coeff(1), distortion_coeff(2), distortion_coeff(3))),
		nullptr,
		x_ud);

	// Run the solver!
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	//options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);


	return Corner(x_ud[0], x_ud[1]);
}

Corner rectifyOneCorner(const Corner& cs, const Eigen::Matrix3d& A_cam,
	const Eigen::Matrix3d& rect_rot, const Eigen::Matrix3d& cam_intrins, Eigen::Vector4d& cam_distor)
{
	Corner cr_res(0, 0);

	PixelType x_d = (cs.x - cam_intrins(0, 2)) / cam_intrins(0, 0);
	PixelType y_d = (cs.y - cam_intrins(1, 2)) / cam_intrins(1, 1);
	auto pixel_ud = rectifySinglePixel(Corner(x_d, y_d), cam_distor);
	Eigen::Vector3d p_ud; p_ud << pixel_ud.x, pixel_ud.y, 1;
	auto pos = rect_rot * p_ud;
	PixelType l_x_ud_uv = pos(0) / pos(2);
	PixelType l_y_ud_uv = pos(1) / pos(2);

	cr_res.x = A_cam(0, 0) * l_x_ud_uv + A_cam(0, 2);
	cr_res.y = A_cam(0, 0) * l_y_ud_uv + A_cam(1, 2);

	return cr_res;
}


Corner disRectifyOneCorner(const Corner& corner_, const Eigen::Matrix3d& A_cam,
	const Eigen::Matrix3d& camera_intrinsic, const Eigen::Matrix3d& rect_camera, const Eigen::Vector4d& camera_distortion)
{
	Corner corner_res;

	PixelType l_x_ud_uv, l_y_ud_uv;
	l_x_ud_uv = (corner_.x - A_cam(0, 2)) / A_cam(0, 0);
	l_y_ud_uv = (corner_.y - A_cam(1, 2)) / A_cam(0, 0);
	Eigen::Vector3d pos(l_x_ud_uv, l_y_ud_uv, 1);
	auto p_ud = rect_camera.inverse() * pos;
	Corner pixel_ud = Corner(p_ud(0) / p_ud(2), p_ud(1) / p_ud(2));

	double r_xy = (pixel_ud.x * pixel_ud.x + pixel_ud.y * pixel_ud.y);
	auto k1 = camera_distortion(0);
	auto k2 = camera_distortion(1);
	auto p1 = camera_distortion(2);
	auto p2 = camera_distortion(3);
	auto x_d = pixel_ud.x * (1 + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p1)*pixel_ud.x * pixel_ud.y + (p2) * (r_xy + (2) * pixel_ud.x * pixel_ud.x);
	auto y_d = pixel_ud.y * ((1) + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p2)*pixel_ud.x * pixel_ud.y + (p1) * (r_xy + (2) * pixel_ud.y * pixel_ud.y);

	corner_res.x = camera_intrinsic(0, 0) * x_d + camera_intrinsic(0, 2);
	corner_res.y = camera_intrinsic(1, 1) * y_d + camera_intrinsic(1, 2);

	return corner_res;
}

struct ReprojectionErrorNonePlane
{
	ReprojectionErrorNonePlane(const Eigen::Matrix3d& R,
		const Eigen::Vector3d& t,
		const Eigen::Matrix3d& A_cam,
		const Corner& m,
		const Eigen::Vector3d& M)
		: R(R)
		, t(t)
		, A_cam(A_cam)
		, m(m)
		, M(M)
	{
	}

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const
	{
		T R_relative[9];
		T t_relative[3];

		Pose2RT(pose, R_relative, t_relative);

		T R_now[9];
		T t_now[3];
		T RR[9];
		for (int i = 0; i < 9; ++i)
			RR[i] = T(R(i / 3, i % 3));
		T tt[3];
		for (int i = 0; i < 3; ++i)
			tt[i] = T(t(i));
		MatMulMat(RR, R_relative, R_now);
		VecAddVec(tt, t_relative, t_now);

		T A[9];
		for (int i = 0; i < 9; ++i)
			A[i] = T(A_cam(i / 3, i % 3));

		T AR[9];
		MatMulMat(A, R_now, AR);

		T MM[3];
		MM[0] = T(M.x());
		MM[1] = T(M.y());
		MM[2] = T(M.z());

		T ARM[3];
		MatMulVec(AR, MM, ARM);
		T AT[3];
		MatMulVec(A, t_now, AT);
		T MMM[3];
		for (int i = 0; i < 3; ++i)
			MMM[i] = T(ARM[i]) + T(AT[i]);

		residuals[0] = T(m.x) - MMM[0] / MMM[2];
		residuals[1] = T(m.y) - MMM[1] / MMM[2];

		return true;
	}

	const Eigen::Matrix3d R;
	const Eigen::Vector3d t;
	const Eigen::Matrix3d A_cam;
	const Corner m;
	const Eigen::Vector3d M;
};


void optInTwoImagesNonePlane(const Corners& left_corners_2D, const std::vector<Eigen::Vector3d>& left_points_3D, const Corners& right_corners_2D,
	const std::vector<Eigen::Vector3d>& right_points_3D, const std::vector<Eigen::Matrix3d>& R_inits, const std::vector<Eigen::Vector3d>& t_inits, const Eigen::Matrix3d& A_cam,
	const double& t_cam, Eigen::Matrix3d& R_res, Eigen::Vector3d& t_res, double& cost)
{
	auto rotRPY = [](const double phi, const double theta, const double psi) {
		Eigen::Matrix3d rot;
		rot << cos(phi) * cos(theta), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), sin(phi)* sin(psi) + cos(phi) * sin(theta) * cos(psi),
			sin(phi)* cos(theta), cos(phi)* cos(psi) + sin(phi) * sin(theta) * sin(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi),
			-sin(theta), cos(theta)* sin(psi), cos(theta)* cos(psi);

		return rot;
	};

	double test_rpy[7][3] = { {0,0,0}, {1.5, 0, 0}, {-1.5, 0, 0}, {0, 1.5, 0},
		{0, -1.5, 0}, {0, 0, 1.5}, {0, 0, -1.5} };
	cost = 1e10;
	for (size_t ti = 0; ti < t_inits.size(); ti++)
	{
		Eigen::Matrix3d R_init = R_inits.at(ti);
		R_init = Eigen::Matrix3d::Identity();
		Eigen::Vector3d t_init = t_inits.at(ti);
		for (size_t ri = 0; ri < 7; ri++)
		{
			double pose[6] = { 0.0, 0, 0, 0, 0, 0 };
			Eigen::Matrix3d R_test = R_init * rotRPY(test_rpy[ri][0], test_rpy[ri][1], test_rpy[ri][2]);

			ceres::Problem problem;
			auto problem_count = 0;
			for (int i = 0; i < left_corners_2D.size(); ++i)
			{
				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<ReprojectionErrorNonePlane, 2, 6>(
						new ReprojectionErrorNonePlane(R_test, t_init, A_cam, left_corners_2D.at(i), left_points_3D.at(i))),
					nullptr,
					pose);
				++problem_count;
			}
			for (int i = 0; i < right_corners_2D.size(); ++i)
			{
				Eigen::Vector3d bias(-t_cam, 0, 0);
				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<ReprojectionErrorNonePlane, 2, 6>(
						new ReprojectionErrorNonePlane(R_test, t_init + bias, A_cam, right_corners_2D.at(i), right_points_3D.at(i))),
					nullptr,
					pose);
				++problem_count;
			}

			ceres::Solver::Options options;
			options.trust_region_strategy_type = ceres::DOGLEG;// LEVENBERG_MARQUARDT;//
			options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = false;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			//std::cout << summary.BriefReport() << "\n";

			double cost_x = sqrt(/*2.0 * */summary.final_cost / problem_count);
			if (cost_x < cost)
			{
				cost = cost_x;
				R_res = R_test * rotRPY(pose[0], pose[1], pose[2]);
				t_res = t_init + Eigen::Vector3d(pose[3], pose[4], pose[5]);
			}
			//if (cost < 2.0)
			//{
			//	break;
			//}
		}
	}
}

void findSquares_adapt(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares)
{
	squares.clear();

	cv::Mat image_binary;
	cv::adaptiveThreshold(image, image_binary, 255,
		cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 21, 7);//21, 7);//

	detectRectangles(image_binary, squares);
}

int perimeter(std::vector<cv::Point>& a)
{
	int sum = 0;
	for (unsigned int i = 0; i < a.size(); i++)
	{
		int i2 = (i + 1) % a.size();
		sum += sqrt((a[i].x - a[i2].x) * (a[i].x - a[i2].x) + (a[i].y - a[i2].y) * (a[i].y - a[i2].y));
	}
	return sum;
}

void drawContour(cv::Mat& in, std::vector<cv::Point>& contour, cv::Scalar color)
{
	for (unsigned int i = 0; i < contour.size(); i++)
	{
		cv::rectangle(in, contour[i], contour[i], color);
	}
}

void detectRectangles(const cv::Mat& thresImg, std::vector<std::vector<cv::Point>>& OutMarkerContours)
{
	//calcualte the min_max contour sizes
	int minSize = 40;// 2e-2 * std::max(thresImg.cols, thresImg.rows) * 4;
	int maxSize = 100000;// 0.25 * std::max(thresImg.cols, thresImg.rows) * 4;
	std::vector<std::vector<cv::Point> > contours2;
	std::vector<cv::Vec4i> hierarchy2;
	cv::Mat thres2;

	thresImg.copyTo(thres2);
	cv::findContours(thres2, contours2, hierarchy2, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
	std::vector<cv::Point> approxCurve;
	///for each contour, analyze if it is a paralelepiped likely to be the marker

	std::vector<std::vector<cv::Point>> MarkerCanditates;
	std::vector<int> candidate_idxs;

	for (unsigned int i = 0; i < contours2.size(); i++)
	{
		//check it is a possible element by first checking is has enough points
		if (minSize < contours2[i].size() && contours2[i].size() < maxSize)
		{
			//approximate to a poligon
			cv::approxPolyDP(contours2[i], approxCurve, double(contours2[i].size()) * 0.05, true);

			//check that the poligon has 4 points
			if (approxCurve.size() == 4)
			{
				//and is convex
				if (cv::isContourConvex(cv::Mat(approxCurve)))
				{
					// ensure that the distace between consecutive points is large enough
					float minDist = 1e10;
					for (int j = 0; j < 4; j++)
					{
						float d = std::sqrt((float)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) * (approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
							(approxCurve[j].y - approxCurve[(j + 1) % 4].y) * (approxCurve[j].y - approxCurve[(j + 1) % 4].y));
						if (d < minDist) minDist = d;
					}
					//check that distance is not very small
					if (minDist > 20)
					{
						//add the points
						candidate_idxs.push_back(i);
						std::vector<cv::Point> marker_candidate;
						for (int j = 0; j < 4; j++)
						{
							marker_candidate.push_back(approxCurve[j]);
						}
						if (100 > (std::abs(marker_candidate[0].x - marker_candidate[2].x) +
							std::abs(marker_candidate[0].y - marker_candidate[2].y)))
							continue;
						MarkerCanditates.push_back(marker_candidate);
					}
				}
			}
		}
	}

		///sort the points in anti-clockwise order
	std::valarray<bool> swapped(false, MarkerCanditates.size());//used later
	for (unsigned int i = 0; i < MarkerCanditates.size(); i++)
	{

		//trace a line between the first and second point.
		//if the thrid point is at the right side, then the points are anti-clockwise
		double dx1 = MarkerCanditates[i][1].x - MarkerCanditates[i][0].x;
		double dy1 = MarkerCanditates[i][1].y - MarkerCanditates[i][0].y;
		double dx2 = MarkerCanditates[i][2].x - MarkerCanditates[i][0].x;
		double dy2 = MarkerCanditates[i][2].y - MarkerCanditates[i][0].y;
		double o = (dx1 * dy2) - (dy1 * dx2);

		if (o < 0.0)		 //if the third point is in the left side, then sort in anti-clockwise order
		{
			swap(MarkerCanditates[i][1], MarkerCanditates[i][3]);
			swapped[i] = true;
		}
	}

	/// remove these elements whise corners are too close to each other
	//first detect candidates

	std::vector<std::pair<int, int>  > TooNearCandidates;
	for (unsigned int i = 0; i < MarkerCanditates.size(); i++)
	{
		// 	cout<<"Marker i="<<i<<MarkerCanditates[i]<<endl;
		//calculate the average distance of each corner to the nearest corner of the other marker candidate
		for (unsigned int j = i + 1; j < MarkerCanditates.size(); j++)
		{
			float dist = 0;
			for (int c = 0; c < 4; c++)
				dist += sqrt((MarkerCanditates[i][c].x - MarkerCanditates[j][c].x) *
					(MarkerCanditates[i][c].x - MarkerCanditates[j][c].x) +
					(MarkerCanditates[i][c].y - MarkerCanditates[j][c].y) *
					(MarkerCanditates[i][c].y - MarkerCanditates[j][c].y));
			dist /= 4;
			//if distance is too small
			if (dist < 100)
			{
				TooNearCandidates.push_back(std::pair<int, int>(i, j));
			}
		}
	}

	//mark for removal the element of  the pair with smaller perimeter
	std::valarray<bool> toRemove(false, MarkerCanditates.size());
	for (unsigned int i = 0; i < TooNearCandidates.size(); i++)
	{
		if (perimeter(MarkerCanditates[TooNearCandidates[i].first]) >
			perimeter(MarkerCanditates[TooNearCandidates[i].second]))
			toRemove[TooNearCandidates[i].second] = true;
		else toRemove[TooNearCandidates[i].first] = true;
	}

	//remove the invalid ones
//     removeElements ( MarkerCanditates,toRemove );
	//finally, assign to the remaining candidates the contour
	OutMarkerContours.reserve(MarkerCanditates.size());
	for (size_t i = 0; i < MarkerCanditates.size(); i++) {
		if (!toRemove[i]) {
			OutMarkerContours.push_back(MarkerCanditates.at(i));//.push_back(contours2[candidate_idxs.at(i)]);        
		}
	}
}

bool find8Points(const cv::Mat& image, std::vector<std::vector<cv::Point2f>>& result,
	std::vector<std::vector<cv::Point>>& resRect) {
	// pR: points radius
	// parameters:
	std::vector<std::vector<cv::Point>> contoursRect;
	findSquares_adapt(image, contoursRect);
	bool circle_res = findCircles_comb(contoursRect, image, result, resRect);
	return circle_res;
}

bool findCircles_comb(const std::vector < std::vector<cv::Point>>& contoursRect, const cv::Mat& image,
	std::vector<std::vector<cv::Point2f>>& result, std::vector<std::vector<cv::Point>>& resRect)
{
	for (const auto& cR : contoursRect)
	{
		std::vector<cv::Point2f> temp_result;
		if (findCircles_new_sub(cR, image, temp_result))
		{
			result.emplace_back(temp_result);
			resRect.emplace_back(cR);
		}
		else
		{
			std::vector<cv::Point2f> temp_result2;
			if (findCircles_sub(cR, image, temp_result2))
			{
				result.emplace_back(temp_result2);
				resRect.emplace_back(cR);
			}
		}
	}

	if (result.size() == 0) {
		return false;
	}

	return true;
}

bool distinguish8PointsV2(const std::vector<cv::Point2f>& pointsIn,
	const std::vector<cv::Point> cnr_rect, PatternContainer& pointsOut, int& Pattern_ID)
{
	if (pointsIn.size() != 8) {
		return false;
	}
	//寻找边界四个点
	std::vector<std::pair<float, int>> min_dis;
	min_dis.resize(4);
	cv::Point2f p_tem;
	float ti, tj;
	auto dis_res = [](cv::Point2f p1, int i, cv::Point2f p2, cv::Point2f p3, int j) {
		float ti = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
		float tj = sqrt((p3.x - p2.x) * (p3.x - p2.x) + (p3.y - p2.y) * (p3.y - p2.y));
		std::pair<float, int> res;
		if (ti > tj)
		{
			res.first = tj;
			res.second = j;
		}
		else
		{
			res.first = ti;
			res.second = i;
		}
		return res;
	};
	//for (size_t i = 0;i< pointsIn.size();i++)
	{
		for (size_t j = 1; j < pointsIn.size(); j++)
		{
			min_dis[0] = dis_res(pointsIn[min_dis[0].second], min_dis[0].second, cnr_rect[0], pointsIn[j], j);
			min_dis[1] = dis_res(pointsIn[min_dis[1].second], min_dis[1].second, cnr_rect[1], pointsIn[j], j);
			min_dis[2] = dis_res(pointsIn[min_dis[2].second], min_dis[2].second, cnr_rect[2], pointsIn[j], j);
			min_dis[3] = dis_res(pointsIn[min_dis[3].second], min_dis[3].second, cnr_rect[3], pointsIn[j], j);
		}
	}

	//分离边角4点和剩余4点
	std::vector<cv::Point2f> corner_4, surplus_4;
	corner_4.emplace_back(pointsIn[min_dis[0].second]);
	corner_4.emplace_back(pointsIn[min_dis[1].second]);
	corner_4.emplace_back(pointsIn[min_dis[2].second]);
	corner_4.emplace_back(pointsIn[min_dis[3].second]);

	for (size_t i = 0; i < pointsIn.size(); i++)
	{
		if (min_dis[0].second == i || min_dis[1].second == i ||
			min_dis[2].second == i || min_dis[3].second == i)
		{
		}
		else
		{
			surplus_4.emplace_back(pointsIn[i]);
		}
	}

	//寻找确定坐标系的第5个点——原点
	cv::Point2f P5;

	std::pair<float, int> pair1, pair2, cter1;
	cter1.first = 1000.f;
	for (size_t i = 0; i < surplus_4.size(); i++)
	{
		pair1 = dis_res((corner_4[0] + corner_4[1]) / 2, 0, surplus_4[i], (corner_4[1] + corner_4[2]) / 2, 1);
		pair2 = dis_res((corner_4[2] + corner_4[3]) / 2, 2, surplus_4[i], (corner_4[3] + corner_4[0]) / 2, 3);
		if (pair1.first > pair2.first)
			pair1 = pair2;
		if (cter1.first > pair1.first)
		{
			cter1.first = pair1.first;
			cter1.second = i;
		}
		if (cter1.first < 2.f)
			break;
	}
	P5 = surplus_4[cter1.second];

	//确定边角四点顺序
	cv::Point2f P1, P2, P3, P4;
	Eigen::Vector4i P4_order;
	switch (pair1.second)
	{
	case 0:
	{
		P4_order << 0, 3, 2, 1;
		break;
	}
	case 1:
	{
		P4_order << 1, 0, 3, 2;
		break;
	}
	case 2:
	{
		P4_order << 2, 1, 0, 3;
		break;
	}
	case 3:
	{
		P4_order << 3, 2, 1, 0;
		break;
	}
	}
	P1 = pointsIn[min_dis[P4_order[0]].second];
	P2 = pointsIn[min_dis[P4_order[1]].second];
	P3 = pointsIn[min_dis[P4_order[2]].second];
	P4 = pointsIn[min_dis[P4_order[3]].second];

	//识别剩余的3个点,
	std::vector<cv::Point2f> surplus_3;
	for (size_t i = 0; i < surplus_4.size(); i++)
		if (i == cter1.second)
		{
		}
		else
		{
			surplus_3.emplace_back(surplus_4[i]);
		}
	// 剩余的3个点可能出现的位置有9个，一共有20个判别点，其中内部有12个，分别为
	std::vector<cv::Point2f> cter_for_3;
	p_tem = -1.75f / 5.5f * (P5 - P4) + 9.f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = 1.75f / 5.5f * (P5 - P4) + 9.f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = -3.5f / 5.5f * (P5 - P4) + 7.5f / 11.f * (P3 - P4) + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = 7.5f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = 3.5f / 5.5f * (P5 - P4) + 7.5f / 11.f * (P2 - P1) + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = -1.75f / 5.5f * (P5 - P4) + 6.f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = 1.75f / 5.5f * (P5 - P4) + 6.f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = -3.5f / 5.5f * (P5 - P4) + 4.5f / 11.f * (P3 - P4) + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = 4.5f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = 3.5f / 5.5f * (P5 - P4) + 4.5f / 11.f * (P2 - P1) + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = -1.75f / 5.5f * (P5 - P4) + 3.f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);
	p_tem = 1.75f / 5.5f * (P5 - P4) + 3.f / 11.f * (P2 + P3 - P1 - P4) / 2.f + P5;
	cter_for_3.emplace_back(p_tem);

	// 在20个判据中，其中外部有8个，分别为边角4点以及相邻边角点的中心点
	std::vector<int> ID;
	// for Position 

	for (size_t i = 0; i < surplus_3.size(); i++)
	{
		cv::Point2f P = surplus_3[i];
		if (IsPointInRect(P, cter_for_3[2], cter_for_3[0], P3))
		{
			ID.emplace_back(1);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[0], cter_for_3[3], cter_for_3[1], (P3 + P2) / 2.f))
		{
			ID.emplace_back(2);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[1], cter_for_3[4], P2))
		{
			ID.emplace_back(3);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[7], cter_for_3[5], cter_for_3[2], (P3 + P4) / 2.f))
		{
			ID.emplace_back(4);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[5], cter_for_3[8], cter_for_3[6], cter_for_3[3]))
		{
			ID.emplace_back(5);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[4], cter_for_3[6], cter_for_3[9], (P1 + P2) / 2.f))
		{
			ID.emplace_back(6);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[10], cter_for_3[7], P4))
		{
			ID.emplace_back(7);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[11], cter_for_3[8], cter_for_3[10], P5))
		{
			ID.emplace_back(8);
			continue;
		}
		if (IsPointInRect(P, cter_for_3[9], cter_for_3[11], P1))
		{
			ID.emplace_back(9);
		}
	}
	if (ID.size() == 3)
	{
	}
	else
	{
		return false;
	}

	// 提取编号
	cv::Point2f P6, P7, P8;

	if (ID[0] > ID[1] && ID[1] > ID[2])
	{
		Pattern_ID = ID[2] * 100 + ID[1] * 10 + ID[0];
		P6 = surplus_3[2]; P7 = surplus_3[1]; P8 = surplus_3[0];
	}
	else if (ID[0] > ID[2] && ID[2] > ID[1])
	{
		Pattern_ID = ID[1] * 100 + ID[2] * 10 + ID[0];
		P6 = surplus_3[1]; P7 = surplus_3[2]; P8 = surplus_3[0];
	}
	else if (ID[1] > ID[2] && ID[2] > ID[0])
	{
		Pattern_ID = ID[0] * 100 + ID[2] * 10 + ID[1];
		P6 = surplus_3[0]; P7 = surplus_3[2]; P8 = surplus_3[1];
	}
	else if (ID[1] > ID[0] && ID[0] > ID[2])
	{
		Pattern_ID = ID[2] * 100 + ID[0] * 10 + ID[1];
		P6 = surplus_3[2]; P7 = surplus_3[0]; P8 = surplus_3[1];
	}
	else if (ID[2] > ID[0] && ID[0] > ID[1])
	{
		Pattern_ID = ID[1] * 100 + ID[0] * 10 + ID[2];
		P6 = surplus_3[1]; P7 = surplus_3[0]; P8 = surplus_3[2];
	}
	else if (ID[2] > ID[1] && ID[1] > ID[0])
	{
		Pattern_ID = ID[0] * 100 + ID[1] * 10 + ID[2];
		P6 = surplus_3[0]; P7 = surplus_3[1]; P8 = surplus_3[2];
	}
	pointsOut.p1 = P1;
	pointsOut.p2 = P2;
	pointsOut.p3 = P3;
	pointsOut.p4 = P4;
	pointsOut.p5 = P5;
	pointsOut.p6 = P6;
	pointsOut.p7 = P7;
	pointsOut.p8 = P8;

	return true;

}

bool IsPointInRect(cv::Point2f P, cv::Point2f A, cv::Point2f B, cv::Point2f C, cv::Point2f D)
{
	int a, b, c, d;
	a = (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x);
	b = (C.x - B.x) * (P.y - B.y) - (C.y - B.y) * (P.x - B.x);
	c = (A.x - C.x) * (P.y - C.y) - (A.y - C.y) * (P.x - C.x);
	if (abs(D.x) + abs(D.y) < 1.f)
		d = c;
	else
	{
		c = (D.x - C.x) * (P.y - C.y) - (D.y - C.y) * (P.x - C.x);
		d = (A.x - D.x) * (P.y - D.y) - (A.y - D.y) * (P.x - D.x);
	}
	if ((a >= 0 && b >= 0 && c >= 0 && d >= 0) || (a <= 0 && b <= 0 && c <= 0 && d <= 0))
	{
		return true;
	}
	return false;
}

void identifyMarkerIdV2(const std::vector<std::vector<cv::Point2f>>& points8,
	const std::vector<std::vector<cv::Point>> cnr_rect, std::vector<int>& markerId_temp,
	std::vector<std::vector<cv::Point2f>>& markerPoints_temp,
	const std::vector<std::vector<cv::Point2f>>& registrated_pcs)
{
	for (size_t i = 0; i < points8.size(); i++) {
		PatternContainer temp_leftPC;
		int temp_id;
		if (distinguish8PointsV2(points8[i], cnr_rect[i], temp_leftPC, temp_id))
		{
			markerId_temp.emplace_back(temp_id);

			std::vector<cv::Point2f> lPts;
			lPts.emplace_back(temp_leftPC.p1);
			lPts.emplace_back(temp_leftPC.p2);
			lPts.emplace_back(temp_leftPC.p3);
			lPts.emplace_back(temp_leftPC.p4);
			lPts.emplace_back(temp_leftPC.p5);
			lPts.emplace_back(temp_leftPC.p6);
			lPts.emplace_back(temp_leftPC.p7);
			lPts.emplace_back(temp_leftPC.p8);
			markerPoints_temp.emplace_back(lPts);
		}
	}
}

void markerIdentify(const cv::Mat& leftSrc, const cv::Mat& rightSrc,
	const cv::Rect& gb_rect_l, const cv::Rect& gb_rect_r, const bool& is_global,
	const std::vector<std::vector<cv::Point2f>>& registrated_pcs,
	std::vector<int>& leftMarkerId, std::vector<int>& rightMarkerId,
	std::vector<std::vector<cv::Point2f>>& leftMarkerPoints,
	std::vector<std::vector<cv::Point2f>>& rightMarkerPoints) {

	/* find 8 points.*/
	std::vector<std::vector<cv::Point2f>> leftPoints, rightPoints;
	std::vector<float> leftPR, rightPR;
	cv::Mat leftCopy, rightCopy;
	cv::cvtColor(leftSrc(gb_rect_l), leftCopy, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(leftCopy, leftCopy, cv::Size(3, 3), 1);
	cv::cvtColor(rightSrc(gb_rect_r), rightCopy, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(rightCopy, rightCopy, cv::Size(5, 5), 3);
	std::vector<std::vector<cv::Point>> cnr_rect_l, cnr_rect_r;

	//printf("#############1.1\n");
	find8Points(leftCopy, leftPoints, cnr_rect_l);
	//printf("#############1.2\n");
	find8Points(rightCopy, rightPoints, cnr_rect_r);
	//printf("#############1.3\n");

	for (size_t i = 0; i < cnr_rect_l.size(); i++)
	{
		for (size_t j = 0; j < leftPoints.at(i).size(); j++)
		{
			leftPoints.at(i).at(j).x = leftPoints.at(i).at(j).x + gb_rect_l.x;
			leftPoints.at(i).at(j).y = leftPoints.at(i).at(j).y + gb_rect_l.y;
		}
		for (size_t j = 0; j < cnr_rect_l.at(i).size(); j++)
		{
			cnr_rect_l.at(i).at(j).x = cnr_rect_l.at(i).at(j).x + gb_rect_l.x;
			cnr_rect_l.at(i).at(j).y = cnr_rect_l.at(i).at(j).y + gb_rect_l.y;
		}
	}
	for (size_t i = 0; i < cnr_rect_r.size(); i++)
	{
		for (size_t j = 0; j < rightPoints.at(i).size(); j++)
		{
			rightPoints.at(i).at(j).x = rightPoints.at(i).at(j).x + gb_rect_r.x;
			rightPoints.at(i).at(j).y = rightPoints.at(i).at(j).y + gb_rect_r.y;
		}
		for (size_t j = 0; j < cnr_rect_r.at(i).size(); j++)
		{
			cnr_rect_r.at(i).at(j).x = cnr_rect_r.at(i).at(j).x + gb_rect_r.x;
			cnr_rect_r.at(i).at(j).y = cnr_rect_r.at(i).at(j).y + gb_rect_r.y;
		}
	}

	//epm::MarkerLocations mk_locs;
	std::vector<int> leftMarkerId_wlf, rightMarkerId_wlf;
	std::vector<std::vector<cv::Point2f>> leftMarkerPoints_wlf, rightMarkerPoints_wlf;

	identifyMarkerIdV2(leftPoints, cnr_rect_l, leftMarkerId_wlf, leftMarkerPoints_wlf, registrated_pcs);
	identifyMarkerIdV2(rightPoints, cnr_rect_r, rightMarkerId_wlf, rightMarkerPoints_wlf, registrated_pcs);

	if (leftMarkerId_wlf.size() > 0)
	{
		for (size_t i = 0; i < leftMarkerId_wlf.size(); i++)
		{
			bool is_repeat = false;
			for (size_t j = 0; j < leftMarkerId.size(); j++)
			{
				if (leftMarkerId.at(j) == leftMarkerId_wlf.at(i))
				{
					is_repeat = true;
				}
			}
			if (!is_repeat)
			{
				leftMarkerId.push_back(leftMarkerId_wlf.at(i));
				leftMarkerPoints.push_back(leftMarkerPoints_wlf.at(i));
			}
		}
	}
	if (rightMarkerId_wlf.size() > 0)
	{
		for (size_t i = 0; i < rightMarkerId_wlf.size(); i++)
		{
			bool is_repeat = false;
			for (size_t j = 0; j < rightMarkerId.size(); j++)
			{
				if (rightMarkerId.at(j) == rightMarkerId_wlf.at(i))
				{
					is_repeat = true;
				}
			}
			if (!is_repeat)
			{
				rightMarkerId.push_back(rightMarkerId_wlf.at(i));
				rightMarkerPoints.push_back(rightMarkerPoints_wlf.at(i));
			}
		}
	}
	for (int i = leftMarkerId.size() - 1; i > -1; i--)
	{
		std::vector<int>::iterator it;
		it = std::find(rightMarkerId.begin(), rightMarkerId.end(), leftMarkerId[i]);
		if (it == rightMarkerId.end())
		{
			leftMarkerId.erase(leftMarkerId.begin() + i);
			leftMarkerPoints.erase(leftMarkerPoints.begin() + i);
		}
	}
	for (int i = rightMarkerId.size() - 1; i > -1; i--)
	{
		std::vector<int>::iterator it;
		it = std::find(leftMarkerId.begin(), leftMarkerId.end(), rightMarkerId[i]);
		if (it == leftMarkerId.end())
		{
			rightMarkerId.erase(rightMarkerId.begin() + i);
			rightMarkerPoints.erase(rightMarkerPoints.begin() + i);
		}
	}
	return;
}


