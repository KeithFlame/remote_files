#include "marker_identifier_new.h"
#include "eight_point_marker_locator.h"
#include <valarray>  
#include <time.h>

static std::vector<gripperRois> left_gp_3rois(3);
static std::vector<gripperRois> right_gp_3rois(3);

static epm::EightPointMarkerLocator locator_left;
static epm::EightPointMarkerLocator locator_right;
const float SCALE_RATE = 1.06;
const int MAX_SCALE_T = 10;

static void refineCircleCenters(const cv::Mat gray_img, const cv::Rect& gb_rect, std::vector<std::vector<cv::Point2f>>& markerPoints)
{
	if (markerPoints.size() > 0)
	{
		for (size_t i = 0; i < markerPoints.size(); i++)
		{
			int half_len = (int)(std::min(cv::norm(markerPoints.at(i).at(1) - markerPoints.at(i).at(0)),
				cv::norm(markerPoints.at(i).at(2) - markerPoints.at(i).at(0))) * 2.4 / 7.68);
			if (half_len > 0)
			{
				for (size_t j = 0; j < markerPoints.at(i).size(); j++)
				{
					int temp_minx = std::max((int)(markerPoints.at(i).at(j).x - gb_rect.x - half_len), 0);
					int temp_miny = std::max((int)(markerPoints.at(i).at(j).y - gb_rect.y - half_len), 0);
					int temp_maxx = std::min((int)(markerPoints.at(i).at(j).x - gb_rect.x + half_len), gray_img.cols - 1);
					int temp_maxy = std::min((int)(markerPoints.at(i).at(j).y - gb_rect.y + half_len), gray_img.rows - 1);
					cv::Mat img_roi = gray_img(cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx,
						temp_maxy - temp_miny)).clone();

					cv::threshold(img_roi, img_roi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
					//float thresh2 = cv::mean(img_roi)[0];
					//img_roi = img_roi > 0.8 * thresh2;

					float x_sum = 0.0, y_sum = 0.0, count_ = 0.0;
					for (size_t row_ = 0;  row_ < img_roi.rows;  row_++)
					{
						for (size_t col_ = 0; col_ < img_roi.cols; col_++)
						{
							float pixel_val = (float)img_roi.ptr<uchar>(row_)[col_];
							if (pixel_val > 0.5)
							{
								x_sum += ((float)col_);// *pixel_val;
								y_sum += ((float)row_);// *pixel_val;
								count_ += 1;// pixel_val;
							}
						}
					}
					if (count_ > 1e-6)
					{
						markerPoints.at(i).at(j).x = x_sum / count_ + temp_minx + gb_rect.x;
						markerPoints.at(i).at(j).y = y_sum / count_ + temp_miny + gb_rect.y;
					}
				}
			}
		}
	}
}

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
	cv::Mat img_roi = image(cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx,
		temp_maxy - temp_miny));// .clone();

	//cv::Mat imageSrc;
	//cv::cvtColor(img_roi, imageSrc, cv::COLOR_GRAY2BGR);

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
	//cv::waitKey(0);

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
	//cv::imshow("imageSrc", imageSrc);
	//cv::waitKey(0);

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
	//cv::imshow("imageSrc1", imageSrc1);
	//cv::waitKey(0);

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

//


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

int getId(const PatternContainer& sorted_pcs, const std::vector<std::vector<cv::Point2f>>& registrated_pcs) 
{
	int id = 0;
	if ((sorted_pcs.p1.x < 0 || sorted_pcs.p2.x < 0 || sorted_pcs.p3.x < 0 || sorted_pcs.p4.x < 0 ||
		sorted_pcs.p5.x < 0 || sorted_pcs.p6.x < 0 || sorted_pcs.p7.x < 0 || sorted_pcs.p8.x < 0) ||
		registrated_pcs.size() < 6) {
		//std::cout << "PatternContainer is not initialized..." << std::endl;
		return -1;
	}

	cv::Mat xy = (cv::Mat_<float>(2, 2) <<
		(sorted_pcs.p2.x - sorted_pcs.p1.x), (sorted_pcs.p3.x - sorted_pcs.p1.x),
		(sorted_pcs.p2.y - sorted_pcs.p1.y), (sorted_pcs.p3.y - sorted_pcs.p1.y));  // [p2-p1 p4-p1] * [a;b] = pi - p1; xy * [a;b] = c.
	cv::Mat xy_inv = xy.inv();

	// 6
	cv::Mat c = (cv::Mat_<float>(2, 1) <<
		(sorted_pcs.p6.x - sorted_pcs.p1.x),
		(sorted_pcs.p6.y - sorted_pcs.p1.y));
	cv::Mat ab = xy_inv * c;
	cv::Point2f pos6(ab.at<float>(0, 0), ab.at<float>(1, 0));

	// 7
	c = (cv::Mat_<float>(2, 1) <<
		(sorted_pcs.p7.x - sorted_pcs.p1.x),
		(sorted_pcs.p7.y - sorted_pcs.p1.y));
	ab = xy_inv * c;
	cv::Point2f pos7(ab.at<float>(0, 0), ab.at<float>(1, 0));

	// 8
	c = (cv::Mat_<float>(2, 1) <<
		(sorted_pcs.p8.x - sorted_pcs.p1.x),
		(sorted_pcs.p8.y - sorted_pcs.p1.y));
	ab = xy_inv * c;
	cv::Point2f pos8(ab.at<float>(0, 0), ab.at<float>(1, 0));

	float thresh = 0.6;
	float temp;

	// 72
	temp = norm(pos6 - registrated_pcs.at(3).at(0)) +
		norm(pos7 - registrated_pcs.at(3).at(1)) +
		norm(pos8 - registrated_pcs.at(3).at(2));
	if (temp < thresh) {
		thresh = temp;
		id = 72;
	}
	// 73
	temp = norm(pos6 - registrated_pcs.at(4).at(0)) +
		norm(pos7 - registrated_pcs.at(4).at(1)) +
		norm(pos8 - registrated_pcs.at(4).at(2));
	if (temp < thresh) {
		thresh = temp;
		id = 73;
	}
	// 74
	temp = norm(pos6 - registrated_pcs.at(5).at(0)) +
		norm(pos7 - registrated_pcs.at(5).at(1)) +
		norm(pos8 - registrated_pcs.at(5).at(2));
	if (temp < thresh) {
		thresh = temp;
		id = 74;
	}

	return id;
}

void identifyMarkerId(const std::vector<std::vector<cv::Point2f>>& points8, 
	std::vector<int>& markerId_temp, std::vector<std::vector<cv::Point2f>>& markerPoints_temp,
	const std::vector<std::vector<cv::Point2f>>& registrated_pcs)
{
	for (size_t i = 0; i < points8.size(); i++) {
		PatternContainer temp_leftPC;
		distinguish8Points(points8[i], temp_leftPC);
		int temp_id = getId(temp_leftPC, registrated_pcs);

#ifdef DEBUG_OUTPUT 
		if (temp_id == -1) {
			printf("Left marker: %d; didn't distinguish 8 points.\n", i);
		}
		else if (temp_id == 0) {
			printf("Left marker: %d; didn't match id.\n", i);
		}
		else 
#endif // DEBUG_OUTPUT		
		{
			markerId_temp.emplace_back(temp_id);

			std::vector<cv::Point2f> lPts;
			lPts.emplace_back(temp_leftPC.p1);
			lPts.emplace_back(temp_leftPC.p2);
			lPts.emplace_back(temp_leftPC.p3);
			lPts.emplace_back(temp_leftPC.p5);
			lPts.emplace_back(temp_leftPC.p4);
			lPts.emplace_back(temp_leftPC.p6);
			lPts.emplace_back(temp_leftPC.p7);
			lPts.emplace_back(temp_leftPC.p8);
			markerPoints_temp.emplace_back(lPts);
		}
	}
}

void findSquares(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares)
{
	auto angle = [](const cv::Point& pt1, const cv::Point& pt2, const cv::Point& pt0)
	{
		double dx1 = static_cast<double>(pt1.x) - pt0.x;
		double dy1 = static_cast<double>(pt1.y) - pt0.y;
		double dx2 = static_cast<double>(pt2.x) - pt0.x;
		double dy2 = static_cast<double>(pt2.y) - pt0.y;
		return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
	};


	//int thresh = 100;
	// int N = 1;  // 5;  // try 5 different thresholds.
	squares.clear();

	cv::Mat dst, gray_one, gray;
	gray_one = cv::Mat(image.size(), CV_8U);
	dst = image.clone();

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	// search the rectangle in each channel.
	for (int c = 0; c < image.channels(); c++)
	{
		int ch[] = { c, 0 };

		// convert to one channel image
		cv::mixChannels(&dst, 1, &gray_one, 1, ch, 1);

		// method 1
		// sobel edge detection
		cv::medianBlur(gray_one, gray_one, 5);
		cv::Mat img_x, img_y, img_sobel;
		cv::Sobel(gray_one, img_x, CV_16S, 1, 0); // be careful to use CV_16S here, maintaining useful information.
		cv::Sobel(gray_one, img_y, CV_16S, 0, 1);
		cv::convertScaleAbs(img_x, img_x); // absolute value.
		cv::convertScaleAbs(img_y, img_y);
		cv::addWeighted(img_x, 0.5, img_y, 0.5, 0.0, img_sobel);
		gray = img_sobel > 15;//10, 15, 20
		//
		//cv::Canny(gray, gray, 5, 100, 5);

		// findContours(gray, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		cv::findContours(gray, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

		std::vector<cv::Point> approx;

		// verify contours
		for (size_t i = 0; i < contours.size(); i++)
		{
			// poly approximation
			cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.05, true);

			// calculate contour area size, and then get 4 points
			if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 5000 && fabs(cv::contourArea(cv::Mat(approx))) < 150000 && cv::isContourConvex(cv::Mat(approx)))
			{
				double maxCosine = 0;

				for (int j = 2; j < 5; j++)
				{
					// find the max angle
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}

				if (maxCosine < 0.7)
				{
					// unique test.
					bool unique = true;
					for (size_t j = 0; j < squares.size(); j++) {
						std::vector<cv::Point> approx_j = squares[j];
						if ((fabs(cv::contourArea(cv::Mat(approx))) - fabs(cv::contourArea(cv::Mat(approx_j)))) < fabs(cv::contourArea(cv::Mat(approx))) * 0.001 &&
							(pow((approx[0].x + approx[1].x + approx[2].x + approx[3].x) - (approx_j[0].x + approx_j[1].x + approx_j[2].x + approx_j[3].x), 2) +
								pow((approx[0].y + approx[1].y + approx[2].y + approx[3].y) - (approx_j[0].y + approx_j[1].y + approx_j[2].y + approx_j[3].y), 2)) < fabs(cv::contourArea(cv::Mat(approx))) * 0.01)
							unique = false;
						break;
					}
					if (unique == 1) {
						squares.push_back(approx);
					}
				}
			}
		}
	}

	// show result
	//cv::Mat out;
	//cv::cvtColor(image, out, cv::COLOR_GRAY2BGR);
	//for (size_t i = 0; i < squares.size(); i++)
	//{
	//	const cv::Point* p = &squares[i][0];

	//	int n = (int)squares[i].size();
	//	if (p->x > 3 && p->y > 3)
	//	{
	//		cv::polylines(out, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
	//	}
	//}
	//cv::imwrite("result.jpg", out);
}

void findSquares_adapt(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares)
{
	squares.clear();

	cv::Mat image_binary;
	cv::adaptiveThreshold(image, image_binary, 255,
		cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 15, 5);//21, 7);//

	//cv::Mat imageBinary2;
	//cv::erode(imageBinary, imageBinary2, cv::Mat());
	//imageBinary2.copyTo(imageBinary); //vs thres=thres2;

	//cv::namedWindow("binary", cv::WINDOW_NORMAL);
	//cv::imshow("binary", image_binary);
	//cv::waitKey(0);

	detectRectangles(image_binary, squares);
	//std::cout << "outCountors.size(): " << squares.size() << std::endl;

	// draw result
	//cv::Mat image_src;
	//cv::cvtColor(image, image_src, cv::COLOR_GRAY2BGR);
	//for (int i = 0; i < squares.size(); i++)
	//{
	//	for (int j = 0; j < squares.at(i).size(); j++)
	//	{
	//		cv::line(image_src, squares.at(i).at(j), squares.at(i).at((j + 1) % 4),
	//			cv::Scalar(0, 255, 255), 2);

	//	}
	//}
	//cv::namedWindow("imageSrc", cv::WINDOW_NORMAL);
	//cv::imshow("imageSrc", image_src);
	//cv::waitKey(0);
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
	int maxSize = 10000;// 0.25 * std::max(thresImg.cols, thresImg.rows) * 4;
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
		//cv::Mat input;
		//cv::cvtColor(thresImg, input, cv::COLOR_GRAY2BGR);
		//drawContour(input, contours2[i], cv::Scalar(255, 0, 225));
		//cv::namedWindow("input");
		//cv::imshow("input", input);
		//cv::waitKey(10);
		//check it is a possible element by first checking is has enough points
		if (minSize < contours2[i].size() && contours2[i].size() < maxSize)
		{
			//approximate to a poligon
			cv::approxPolyDP(contours2[i], approxCurve, double(contours2[i].size()) * 0.05, true);
			// 				drawApproxCurve(copy,approxCurve,Scalar(0,0,255));
			//check that the poligon has 4 points
			if (approxCurve.size() == 4)
			{
				//and is convex
				if (cv::isContourConvex(cv::Mat(approxCurve)))
				{
					// 					      drawApproxCurve(input,approxCurve,Scalar(255,0,255));
					// ensure that the distace between consecutive points is large enough
					float minDist = 1e10;
					for (int j = 0; j < 4; j++)
					{
						float d = std::sqrt((float)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) * (approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
							(approxCurve[j].y - approxCurve[(j + 1) % 4].y) * (approxCurve[j].y - approxCurve[(j + 1) % 4].y));
						// 		norm(Mat(approxCurve[i]),Mat(approxCurve[(i+1)%4]));
						if (d < minDist) minDist = d;
					}
					//check that distance is not very small
					if (minDist > 20)
					{
						//add the points
						// 	      cout<<"ADDED"<<endl;

						candidate_idxs.push_back(i);
						std::vector<cv::Point> marker_candidate;
						for (int j = 0; j < 4; j++)
						{
							marker_candidate.push_back(approxCurve[j]);
						}
						MarkerCanditates.push_back(marker_candidate);
					}
				}
			}
		}
	}

	// 		 		  namedWindow("input");
	//  		imshow("input",input);
	//  						waitKey(0);
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
			if (dist < 20)
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
	//return false;

#ifdef DEBUG_OUTPUT 
	clock_t start, end;
	start = std::clock();
#endif // DEBUG_OUTPUT
	findSquares_adapt(image, contoursRect);
#ifdef DEBUG_OUTPUT
	end = clock();
	printf("findSquares_adapt's time: %.3f s.\n", double(end - start) / CLOCKS_PER_SEC);

	start = std::clock();
#endif // DEBUG_OUTPUT
	//printf("#####find8Point #1\n");
	bool circle_res = findCircles_comb(contoursRect, image, result, resRect);

#ifdef DEBUG_OUTPUT 
	end = clock();
	printf("findCircle's time: %.3f s.\n", double(end - start) / CLOCKS_PER_SEC);
#endif // DEBUG_OUTPUT
	return circle_res;
}

bool findCircles_new(const std::vector < std::vector<cv::Point>>& contoursRect, const cv::Mat& image, 
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
	}	
	if (result.size() == 0) {
		return false;
	}

	return true;
}

bool findCircles(const std::vector < std::vector<cv::Point>>& contoursRect, const cv::Mat& image, 
	std::vector<std::vector<cv::Point2f>>& result, std::vector<std::vector<cv::Point>>& resRect)
{
	for (const auto& cR : contoursRect) 
	{
		std::vector<cv::Point2f> temp_result;
		if (findCircles_sub(cR, image, temp_result))
		{
			result.emplace_back(temp_result);
			resRect.emplace_back(cR);
		}		
	}

	if (result.size() == 0) {
		return false;
	}

	return true;
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


bool distinguish8Points(const std::vector<cv::Point2f>& pointsIn, PatternContainer& pointsOut) {
	//PatternContainer pointsOut;

	if (pointsIn.size() != 8) {
		//std::cout << "distinguish8Points: input size wrong!" << std::endl;
		return false;
	}

	// find X coordinate.
	// bool findX = 0;
	for (const auto& a : pointsIn) {
		for (const auto& b : pointsIn) {
			if (b == a) {
				continue;
			}

			// criteria1: the remaining points are to the left of the {Vec_ab}
			// criteria2: theta_a < 100 deg.
			bool cri2 = 1;
			float theta_ac = 0;
			// float theta_c = 4;
			for (const auto& c : pointsIn) {
				if (c == a || c == b) {
					continue;
				}
				if (((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) >= 0) {  // on the right side
					cri2 = 0;
					break;
				}

				float temp_ac;
				temp_ac = acos(((b.x - a.x) * (c.x - a.x) + (b.y - a.y) * (c.y - a.y)) /
					sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)) /
					sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)));
				if (temp_ac > theta_ac) {  // max a
					theta_ac = temp_ac;
				}
			}
			if (!cri2 || theta_ac > 135.0 * CV_PI / 180.0) {
				continue;
			}

			// find b(2) and c(3)
			float dis_ab = 1000, dis_ac = 1000;
			cv::Point2f p2, p3;
			//int cri_x_num = 0;
			for (const auto& c : pointsIn) {
				if (c == a) {
					continue;
				}
				float temp_cos = ((b.x - a.x) * (c.x - a.x) + (b.y - a.y) * (c.y - a.y)) /
					sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)) /
					sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y));
				float temp_a = acos(temp_cos > 1 ? 1 : temp_cos);
				if (fabs(temp_a) < (5.0 / 180.0 * CV_PI)) {  // b
					//cri_x_num++;
					float temp_dis = sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y));
					if (temp_dis < dis_ab) {
						dis_ab = temp_dis;
						p2 = c;
					}
				}
				else if (fabs(theta_ac - temp_a) < (5.0 / 180.0 * CV_PI)) {  // c
					float temp_dis = sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y));
					if (temp_dis < dis_ac) {
						dis_ac = temp_dis;
						p3 = c;
					}
				}
			}
			float pR = (dis_ac+dis_ab) * 0.1;
			/*
			if (cri_x_num != 2) {
				continue;
			}*/


			// criteria 3: d(4) and e(5) is at the position.
			cv::Point2f p4, p5;
			bool find_d = false, find_e = false;
			for (const auto& c : pointsIn) {
				if (c == a || norm(c - p2) < pR || norm(c - p3) < pR) {
					continue;
				}
				cv::Point2f d = p2 + p3 - a;
				cv::Point2f e = 2.4 * p2 + 2.4 * p3 - 3.8 * a;
				float temp_dis_d = sqrt((d.x - c.x) * (d.x - c.x) + (d.y - c.y) * (d.y - c.y));
				float temp_dis_e = sqrt((e.x - c.x) * (e.x - c.x) + (e.y - c.y) * (e.y - c.y));
				if (temp_dis_d < pR * 1.3) {
					p4 = c;
					find_d = true;
					continue;
				}
				else if (temp_dis_e < pR * 2) {
					p5 = c;
					find_e = true;
					continue;
				}
			}
			if (find_d == false || find_e == false) {
				continue;
			}

			// all criterias are met
			// findX = 1;
			pointsOut.p1 = a;
			pointsOut.p2 = p2;
			pointsOut.p3 = p3;
			pointsOut.p4 = p4;
			pointsOut.p5 = p5;
			std::vector<cv::Point2f> points2;
			for (const auto& c : pointsIn) {
				if (c == a || norm(c - p2) < pR || norm(c - p3) < pR ||
					norm(c - p4) < pR || norm(c - p5) < pR) {
					continue;
				}
				points2.emplace_back(c);  // p6, p7 and p8 are emplaced back.
			}
			float cos_temp = ((p2.x - a.x) * (points2[0].x - a.x) + (p2.y - a.y) * (points2[0].y - a.y)) /
				sqrt((p2.x - a.x) * (p2.x - a.x) + (p2.y - a.y) * (p2.y - a.y)) /
				sqrt((points2[0].x - a.x) * (points2[0].x - a.x) + (points2[0].y - a.y) * (points2[0].y - a.y));
			float temp0 = acos(cos_temp < 1 ? cos_temp : 1);
			cos_temp = ((p2.x - a.x) * (points2[1].x - a.x) + (p2.y - a.y) * (points2[1].y - a.y)) /
				sqrt((p2.x - a.x) * (p2.x - a.x) + (p2.y - a.y) * (p2.y - a.y)) /
				sqrt((points2[1].x - a.x) * (points2[1].x - a.x) + (points2[1].y - a.y) * (points2[1].y - a.y));
			float temp1 = acos(cos_temp < 1 ? cos_temp : 1);
			cos_temp = ((p2.x - a.x) * (points2[2].x - a.x) + (p2.y - a.y) * (points2[2].y - a.y)) /
				sqrt((p2.x - a.x) * (p2.x - a.x) + (p2.y - a.y) * (p2.y - a.y)) /
				sqrt((points2[2].x - a.x) * (points2[2].x - a.x) + (points2[2].y - a.y) * (points2[2].y - a.y));
			float temp2 = acos(cos_temp);
			if (temp0 < temp1 && temp0 < temp2) {
				pointsOut.p8 = points2[0];
				if (temp1 < temp2) {
					pointsOut.p7 = points2[1];
					pointsOut.p6 = points2[2];
				}
				else {
					pointsOut.p7 = points2[2];
					pointsOut.p6 = points2[1];
				}
			}
			else if (temp1 < temp0 && temp1 < temp2) {
				pointsOut.p8 = points2[1];
				if (temp0 < temp2) {
					pointsOut.p7 = points2[0];
					pointsOut.p6 = points2[2];
				}
				else {
					pointsOut.p7 = points2[2];
					pointsOut.p6 = points2[0];
				}
			}
			else {
				pointsOut.p8 = points2[2];
				if (temp0 < temp1) {
					pointsOut.p7 = points2[0];
					pointsOut.p6 = points2[1];
				}
				else {
					pointsOut.p7 = points2[1];
					pointsOut.p6 = points2[0];
				}
			}
			return true;

		}
	}

	// didn't find X coordinate
	//std::cout << "distinguish8Points: can't find X coordinate..." << std::endl;
	return false;
}


std::vector<int> crossCheck(std::vector<PatternContainer>& leftPoints, std::vector<PatternContainer>& rightPoints,
	const std::vector<std::vector<cv::Point2f>>& registrated_pcs)
{
	// right pattern id order that matches left pattern id
	std::vector<int> rightMatchLeft(leftPoints.size(), -1);
	for (size_t i = 0; i < leftPoints.size(); i++) {
		int temp_leftid = getId(leftPoints[i], registrated_pcs);
		if (temp_leftid < 1) {
			continue;
		}
		for (size_t j = 0; j < rightPoints.size(); j++) {
			int temp_rightid = getId(rightPoints[j], registrated_pcs);
			if (temp_rightid == temp_leftid) {
				rightMatchLeft[i] = j;
			}
		}
	}

	return rightMatchLeft;
}

std::vector<cv::Point3f> uv2xyz(const std::vector<cv::Point2f>& lPts, const std::vector<cv::Point2f>& rPts,
	const cv::Mat& cameraMatrix, double t) {

	std::vector<cv::Point3f> pts3D;

	const cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
		1., 0., 0., 0.,
		0., 1., 0., 0.,
		0., 0., 1., 0.);
	const cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
		1., 0., 0., -t,
		0., 1., 0., 0.,
		0., 0., 1., 0.);

	// undistortion
	std::vector<cv::Point2f> lPts_ud, rPts_ud;
	lPts_ud = lPts;
	rPts_ud = rPts;

	// projection matrix: from world (left camera) coordinate, to image coordinate.
	cv::Mat proMl(3, 4, CV_32F), proMr(3, 4, CV_32F);
	proMl = cameraMatrix * T1;
	proMr = cameraMatrix * T2;
	//proMl = T1;
	//proMr = T2;
	cv::Mat pts4D(4, 3, CV_64F);
	cv::Mat camlpnts(1, 3, CV_64FC2);
	cv::Mat camrpnts(1, 3, CV_64FC2);
	camlpnts.at<cv::Vec2d>(0, 0) = cv::Vec2d(lPts_ud[0].x, lPts_ud[0].y);
	camlpnts.at<cv::Vec2d>(0, 1) = cv::Vec2d(lPts_ud[1].x, lPts_ud[1].y);
	camlpnts.at<cv::Vec2d>(0, 2) = cv::Vec2d(lPts_ud[2].x, lPts_ud[2].y);
	camrpnts.at<cv::Vec2d>(0, 0) = cv::Vec2d(rPts_ud[0].x, rPts_ud[0].y);
	camrpnts.at<cv::Vec2d>(0, 1) = cv::Vec2d(rPts_ud[1].x, rPts_ud[1].y);
	camrpnts.at<cv::Vec2d>(0, 2) = cv::Vec2d(rPts_ud[2].x, rPts_ud[2].y);
	// calculate 3D position.
	cv::triangulatePoints(proMl, proMr, camlpnts, camrpnts, pts4D);

	//pts4D.at<cv::Vec4d>(0, 1)[0];
	for (int i = 0; i < 3; i++)
	{
		cv::Point3f pointTmp(pts4D.at<double>(0, i) / pts4D.at<double>(3, i),
			pts4D.at<double>(1, i) / pts4D.at<double>(3, i), pts4D.at<double>(2, i) / pts4D.at<double>(3, i));
		pts3D.push_back(pointTmp);
	}

	return pts3D;
}


cv::Mat Tinit(const cv::Point3f& pts0, const cv::Point3f& pts1, const cv::Point3f& pts2) {
	// calculate Tcam_marker.
	cv::Point3f xCoor = (pts1 - pts0) / norm(pts1 - pts0);
	cv::Point3f yCoor = (pts2 - pts0) / norm(pts2 - pts0);
	cv::Point3f zCoor = xCoor.cross(yCoor);
	zCoor /= norm(zCoor);
	yCoor = zCoor.cross(xCoor);
	yCoor /= norm(yCoor);
	cv::Mat temp_Tcam_marker = (cv::Mat_<float>(4, 4) <<
		xCoor.x, yCoor.x, zCoor.x, pts0.x,
		xCoor.y, yCoor.y, zCoor.y, pts0.y,
		xCoor.z, yCoor.z, zCoor.z, pts0.z,
		0.0, 0.0, 0.0, 1.0);
	return temp_Tcam_marker;
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

	std::vector<std::vector<cv::Point2f>> leftPoints_roi, rightPoints_roi;
	std::vector<std::vector<cv::Point>> cnr_rect_l_roi, cnr_rect_r_roi;
	std::vector<std::vector<cv::Point>> cnts_l_temp, cnts_r_temp;
	for (size_t i = 0; i < 3; i++)
	{
		if (left_gp_3rois.at(i).try_ > 0)
		{
			std::vector<cv::Point> cnt_temp;
			for (size_t j = 0; j < 4; j++)
			{
				int xxt = std::min(std::max((int)left_gp_3rois.at(i).verts[j][0] - gb_rect_l.x,
					0), leftCopy.cols - 1);
				int yyt = std::min(std::max((int)left_gp_3rois.at(i).verts[j][1] - gb_rect_l.y,
					0), leftCopy.rows - 1);
				cnt_temp.push_back(cv::Point(xxt,yyt));
			}
			cnts_l_temp.push_back(cnt_temp);
		}
		if (right_gp_3rois.at(i).try_ > 0)
		{
			std::vector<cv::Point> cnt_temp;
			for (size_t j = 0; j < 4; j++)
			{
				int xxt = std::min(std::max((int)right_gp_3rois.at(i).verts[j][0] - gb_rect_r.x,
					0), rightCopy.cols - 1);
				int yyt = std::min(std::max((int)right_gp_3rois.at(i).verts[j][1] - gb_rect_r.y,
					0), rightCopy.rows - 1);
				cnt_temp.push_back(cv::Point(xxt, yyt));
			}
			cnts_r_temp.push_back(cnt_temp);
		}
	}
	if (findCircles_comb(cnts_l_temp, leftCopy, leftPoints_roi, cnr_rect_l_roi))
	{
		leftPoints.insert(leftPoints.end(), leftPoints_roi.begin(), leftPoints_roi.end());
		cnr_rect_l.insert(cnr_rect_l.end(), cnr_rect_l_roi.begin(), cnr_rect_l_roi.end());
	}
	if (findCircles_comb(cnts_r_temp, rightCopy, rightPoints_roi, cnr_rect_r_roi))
	{
		rightPoints.insert(rightPoints.end(), rightPoints_roi.begin(), rightPoints_roi.end());
		cnr_rect_r.insert(cnr_rect_r.end(), cnr_rect_r_roi.begin(), cnr_rect_r_roi.end());
	}

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

	//printf("Left found %zu candidates; right found %zu candidates.\n", leftPoints.size(), rightPoints.size());
	//printf("#############2\n");

	/* distinguish 8 points.*/
	std::vector<int> leftMarkerId_temp, rightMarkerId_temp;
	std::vector<std::vector<cv::Point2f>> leftMarkerPoints_temp, rightMarkerPoints_temp;
	identifyMarkerId(leftPoints, leftMarkerId_temp, leftMarkerPoints_temp, registrated_pcs);
	identifyMarkerId(rightPoints, rightMarkerId_temp, rightMarkerPoints_temp, registrated_pcs);

	bool is_gripper_detected = false;
	//left
	for (size_t i = 0; i < 3; i++)
	{
		left_gp_3rois.at(i).try_--;
		if (left_gp_3rois.at(i).try_ < 0)
		{
			left_gp_3rois.at(i).try_ = 0;
		}
	}
	for (size_t i = 0; i < leftMarkerId_temp.size(); i++)
	{
		bool is_repeat = false;
		for (size_t j = 0; j < leftMarkerId.size(); j++)
		{
			if (leftMarkerId.at(j) == leftMarkerId_temp.at(i))
			{
				is_repeat = true;
			}
		}
		if (!is_repeat)
		{
			leftMarkerId.push_back(leftMarkerId_temp.at(i));
			leftMarkerPoints.push_back(leftMarkerPoints_temp.at(i));
			if (leftMarkerId_temp.at(i) > 71 && leftMarkerId_temp.at(i) < 75)
			{
				is_gripper_detected = true;
				for (size_t k = 0; k < 4; k++)
				{
					left_gp_3rois.at(leftMarkerId_temp.at(i) - 72).verts[k][0] = cnr_rect_l.at(i).at(k).x;
					left_gp_3rois.at(leftMarkerId_temp.at(i) - 72).verts[k][1] = cnr_rect_l.at(i).at(k).y;
					left_gp_3rois.at(leftMarkerId_temp.at(i) - 72).try_ = MAX_SCALE_T;
				}
			}
		}
	}
	for (size_t i = 0; i < 3; i++)
	{
		if (left_gp_3rois.at(i).try_ < MAX_SCALE_T && left_gp_3rois.at(i).try_ > 0)
		{
			float x_avg = 0.25 * (left_gp_3rois.at(i).verts[0][0] + left_gp_3rois.at(i).verts[1][0] + 
				left_gp_3rois.at(i).verts[2][0] + left_gp_3rois.at(i).verts[3][0]);
			float y_avg = 0.25 * (left_gp_3rois.at(i).verts[0][1] + left_gp_3rois.at(i).verts[1][1] +
				left_gp_3rois.at(i).verts[2][1] + left_gp_3rois.at(i).verts[3][1]);
			for (size_t j = 0; j < 4; j++)
			{
				left_gp_3rois.at(i).verts[j][0] = left_gp_3rois.at(i).verts[j][0] + SCALE_RATE *
					(left_gp_3rois.at(i).verts[j][0] - x_avg);
				left_gp_3rois.at(i).verts[j][1] = left_gp_3rois.at(i).verts[j][1] + SCALE_RATE *
					(left_gp_3rois.at(i).verts[j][1] - y_avg);
			}
		}
	}

	//right
	for (size_t i = 0; i < 3; i++)
	{
		right_gp_3rois.at(i).try_--;
		if (right_gp_3rois.at(i).try_ < 0)
		{
			right_gp_3rois.at(i).try_ = 0;
		}
	}
	for (size_t i = 0; i < rightMarkerId_temp.size(); i++)
	{
		bool is_repeat = false;
		for (size_t j = 0; j < rightMarkerId.size(); j++)
		{
			if (rightMarkerId.at(j) == rightMarkerId_temp.at(i))
			{
				is_repeat = true;
			}
		}
		if (!is_repeat)
		{
			rightMarkerId.push_back(rightMarkerId_temp.at(i));
			rightMarkerPoints.push_back(rightMarkerPoints_temp.at(i));
			if (rightMarkerId_temp.at(i) > 71 && rightMarkerId_temp.at(i) < 75)
			{
				is_gripper_detected = is_gripper_detected && true;
				for (size_t k = 0; k < 4; k++)
				{
					right_gp_3rois.at(rightMarkerId_temp.at(i) - 72).verts[k][0] = cnr_rect_r.at(i).at(k).x;
					right_gp_3rois.at(rightMarkerId_temp.at(i) - 72).verts[k][1] = cnr_rect_r.at(i).at(k).y;
					right_gp_3rois.at(rightMarkerId_temp.at(i) - 72).try_ = MAX_SCALE_T;
				}
			}
		}
	}
	for (size_t i = 0; i < 3; i++)
	{
		if (right_gp_3rois.at(i).try_ < MAX_SCALE_T && right_gp_3rois.at(i).try_ > 0)
		{
			float x_avg = 0.25 * (right_gp_3rois.at(i).verts[0][0] + right_gp_3rois.at(i).verts[1][0] +
				right_gp_3rois.at(i).verts[2][0] + right_gp_3rois.at(i).verts[3][0]);
			float y_avg = 0.25 * (right_gp_3rois.at(i).verts[0][1] + right_gp_3rois.at(i).verts[1][1] +
				right_gp_3rois.at(i).verts[2][1] + right_gp_3rois.at(i).verts[3][1]);
			for (size_t j = 0; j < 4; j++)
			{
				right_gp_3rois.at(i).verts[j][0] = right_gp_3rois.at(i).verts[j][0] + SCALE_RATE *
					(right_gp_3rois.at(i).verts[j][0] - x_avg);
				right_gp_3rois.at(i).verts[j][1] = right_gp_3rois.at(i).verts[j][1] + SCALE_RATE *
					(right_gp_3rois.at(i).verts[j][1] - y_avg);
			}
		}
	}

	// use wlf
	if (is_global || !is_gripper_detected)
	{
		epm::MarkerLocations mk_locs;
		std::vector<int> leftMarkerId_wlf, rightMarkerId_wlf;
		std::vector<std::vector<cv::Point2f>> leftMarkerPoints_wlf, rightMarkerPoints_wlf;

		mk_locs = locator_left.locateMarkers(leftCopy, gb_rect_l, is_global);
		identifyMarkerId(mk_locs, leftMarkerId_wlf, leftMarkerPoints_wlf, registrated_pcs);
		mk_locs = locator_right.locateMarkers(rightCopy, gb_rect_r, is_global);
		identifyMarkerId(mk_locs, rightMarkerId_wlf, rightMarkerPoints_wlf, registrated_pcs);

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
	}

	//select the biggest marker
	//int leftMarkerId_gripper_final;
	//std::vector<cv::Point2f> leftMarkerPoints_gripper_final;
	//std::vector<int> leftMarkerId_tempxx;
	//std::vector<std::vector<cv::Point2f>> leftMarkerPoints_tempxx;
	//float max_distance = 0;
	//for (size_t i = 0; i < leftMarkerId.size(); i++)
	//{
	//	if (leftMarkerId.at(i) > 71 && leftMarkerId.at(i) < 75)
	//	{
	//		float distance_mk = cv::norm(leftMarkerPoints.at(i).at(1) - leftMarkerPoints.at(i).at(0)) +
	//			cv::norm(leftMarkerPoints.at(i).at(2) - leftMarkerPoints.at(i).at(0));
	//		if (distance_mk > max_distance)
	//		{
	//			max_distance = distance_mk;
	//			leftMarkerId_gripper_final = leftMarkerId.at(i);
	//			leftMarkerPoints_gripper_final = leftMarkerPoints.at(i);
	//		}
	//	}
	//	else
	//	{
	//		leftMarkerId_tempxx.push_back(leftMarkerId.at(i));
	//		leftMarkerPoints_tempxx.push_back(leftMarkerPoints.at(i));
	//	}
	//}
	//if (max_distance > 1e-6)
	//{
	//	leftMarkerId_tempxx.push_back(leftMarkerId_gripper_final);
	//	leftMarkerPoints_tempxx.push_back(leftMarkerPoints_gripper_final);
	//}
	//leftMarkerId = leftMarkerId_tempxx;
	//leftMarkerPoints = leftMarkerPoints_tempxx;
	
	for (size_t i = 0; i < 4; i++)
	{
		refineCircleCenters(leftCopy, gb_rect_l, leftMarkerPoints);
		refineCircleCenters(rightCopy, gb_rect_r, rightMarkerPoints);
	}

#ifdef DEBUG_OUTPUT
	printf("Left found %d markers; right found %d markers.\n", leftMarkerId.size(), rightMarkerId.size());
#endif // DEBUG_OUTPUT
	return;
}


