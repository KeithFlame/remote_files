#include "detector.h"
#include "../alg/math_helper.h"
#include "../alg/timer.h"
#include <Eigen/Dense>
#include <array>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

#ifdef USE_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#endif

int init_width0 = 8;
const int FIRST_POINT_MAX_TEST_TIMES = 250;
const PixelType SEARCH_CHANGED_ANGLE_THRED = PI / 12;
const PixelType SWM_SEARCH_SIDE_ANGLE_THRED = PI / 8;
const PixelType DWM_SEARCH_SIDE_ANGLE_THRED = PI / 8;
const int ANGLE_BIN_NUM = 28;
const int MAX_UNFOUND_COUNT = 10;
const PixelType SIDE_LENGTH_RATE = 1.0 / 2.0; // default: 2.0 / 3.0

Detector::Detector(const cv::Size size, const MarkerType marker_type)
	: SIZE(size)
	, marker_type(marker_type)
	, SIGMA(2)
	, HALF_PATCH_SIZE(2)
	, PATCH_X(calcPatchX())
	, WIDTH_MIN(8)
	, CORR_THRESHOLD(0.4f)
	, MIN_NUM_FIRST_EXIT(5)
	, MIN_NUM_ONE_START(3)
	, MIN_NUM_EXIT(4)
	, NMS_THRESHOLD_SIGMA_2(0.08f)
	, TRY_FIRST_WIDTH_MAX(150)
	, EXPAND_RATE(1.1)
	, MAX_TEMPLATE_WIDTH(60)
	, ROI_WIDTH_RATE(6)
	, rect{ cv::Range(0, size.width), cv::Range(0, size.height) }
{
#ifdef USE_CUDA
	initCuda(size);
#endif
	init_width0 = WIDTH_MIN;
	unfound_count = 100;
	first_ang1 = 0;
	first_ang2 = 0;
	try_first_width = TRY_FIRST_WIDTH_MAX;
}

CornersSorted Detector::process(const cv::Mat image)
{
	auto image_roi = image(rect.range_y, rect.range_x).clone();

	auto [corners, is_vaild] = detectCornersRectRoi(image_roi);

	/*转换为绝对角点坐标*/
	for (auto& corner : corners)
		corner.point += Corner(rect.range_x.start, rect.range_y.start);

	if (is_vaild)
	{
		unfound_count = 0;
		PixelType width_sum = 0;
		Corner point_sum(0, 0);
		for (const auto& corner : corners)
		{
			point_sum += corner.point;
			width_sum += corner.width;
		}
		auto px_avg = point_sum.x / corners.size();
		auto py_avg = point_sum.y / corners.size();
		auto width_avg = width_sum / corners.size();
		try_first_width = 2.0 * width_avg;

		/*下一帧裁剪*/
		rect.range_x = cv::Range(
			std::max(int(px_avg - width_avg * ROI_WIDTH_RATE - WIDTH_MIN), 0),
			std::min(int(px_avg + width_avg * ROI_WIDTH_RATE + WIDTH_MIN), image.cols));
		rect.range_y = cv::Range(
			std::max(int(py_avg - width_avg * ROI_WIDTH_RATE - WIDTH_MIN), 0),
			std::min(int(py_avg + width_avg * ROI_WIDTH_RATE + WIDTH_MIN), image.rows));

		init_width0 = (width_avg * 0.6) > WIDTH_MIN ? (width_avg * 0.6) : WIDTH_MIN;
		first_ang1 = corners.at(0).angle1;
		first_ang2 = corners.at(0).angle2;
	}
	else
	{
		unfound_count++;
		int x_center = (rect.range_x.start + rect.range_x.end) / 2;
		int y_center = (rect.range_y.start + rect.range_y.end) / 2;
		int x_range = EXPAND_RATE * (rect.range_x.end - rect.range_x.start);
		int y_range = EXPAND_RATE * (rect.range_y.end - rect.range_y.start);

		rect.range_x = cv::Range(std::max(int(x_center - x_range / 2), 0), std::min(int(x_center + x_range / 2), image.cols));
		rect.range_y = cv::Range(std::max(int(y_center - y_range / 2), 0), std::min(int(y_center + y_range / 2), image.rows));

		//init_width0 = WIDTH_MIN;
		if (unfound_count >= MAX_UNFOUND_COUNT)
		{
			unfound_count = 100;
			first_ang1 = 0;
			first_ang2 = 0;
		}
		try_first_width = EXPAND_RATE * try_first_width;
		if (try_first_width > TRY_FIRST_WIDTH_MAX)
		{
			try_first_width = TRY_FIRST_WIDTH_MAX;
		}
	}

	CornersSorted res, res_inter;
	CornersSorted res_x, res_y;
	if (is_vaild)
	{
		for (auto&& p : corners)
		{
			CornerSorted res_temp(0, 0);

			/*match with matlab starting from (1, 1)*/
			res_temp.point = p.point + Corner(1, 1);
			res_temp.sepeNum = p.sepeNum;
			//res_temp.firstIdx = p.firstIdx;
			res_x.emplace_back(res_temp); //p.point
			res_y.emplace_back(res_temp); //p.point
		}
		//res.assign(res_x.begin(), res_x.end());

		//sort
		std::sort(res_x.begin(), res_x.end(),
			[](const auto& lhs, const auto& rhs) { return lhs.point.x < rhs.point.x; });

		std::sort(res_y.begin(), res_y.end(),
			[](const auto& lhs, const auto& rhs) { return lhs.point.y < rhs.point.y; });
		if (res_x.at(res_x.size() - 1).point.x - res_x.at(0).point.x <
			res_y.at(res_y.size() - 1).point.y - res_y.at(0).point.y)
		{
			res_inter.assign(res_y.begin(), res_y.end());
		}
		else
		{
			res_inter.assign(res_x.begin(), res_x.end());
		}

		// first or end corner
		float width_s1 = sqrtf(powf(res_inter.at(1).point.x - res_inter.at(0).point.x, 2.0) +
			powf(res_inter.at(1).point.y - res_inter.at(0).point.y, 2.0));
		float width_s2 = sqrtf(powf(res_inter.at(2).point.x - res_inter.at(1).point.x, 2.0) +
			powf(res_inter.at(2).point.y - res_inter.at(1).point.y, 2.0));
		int start_skip = 0;
		if (width_s1 > width_s2)
		{
			start_skip = 1;
		}

		int cr_num_all = res_inter.size();
		float width_e1 = sqrtf(powf(res_inter.at(cr_num_all - 1).point.x - res_inter.at(cr_num_all - 2).point.x, 2.0) +
			powf(res_inter.at(cr_num_all - 1).point.y - res_inter.at(cr_num_all - 2).point.y, 2.0));
		float width_e2 = sqrtf(powf(res_inter.at(cr_num_all - 2).point.x - res_inter.at(cr_num_all - 3).point.x, 2.0) +
			powf(res_inter.at(cr_num_all - 2).point.y - res_inter.at(cr_num_all - 3).point.y, 2.0));
		int end_skip = 0;
		if (width_e1 > width_e2)
		{
			end_skip = 1;
		}
		for (int i = start_skip; i < cr_num_all - end_skip; i++)
		{
			res.push_back(res_inter.at(i));
		}

		std::vector<float> width_arr;
		for (int i = 0; i < res.size() - 1; i++)
		{
			float width_tmp = sqrtf(powf(res.at(i + 1).point.x - res.at(i).point.x, 2.0) +
				powf(res.at(i + 1).point.y - res.at(i).point.y, 2.0));
			width_arr.push_back(width_tmp);
		}
		std::sort(width_arr.begin(), width_arr.end(),
			[](const auto& lhs, const auto& rhs) { return lhs > rhs; });
		auto mid_width = width_arr.at(width_arr.size() / 2 - 1);
		for (int i = 0; i < width_arr.size(); i++)
		{
			if (width_arr.at(i) < 1.5 * mid_width)
			{
				res.at(i).sepeNum = 0;
			}
			else if (width_arr.at(i) < 2.5 * mid_width)
			{
				res.at(i).sepeNum = 1;
			}
			else if (width_arr.at(i) < 3.5 * mid_width)
			{
				res.at(i).sepeNum = 2;
			}
			else
			{
				return CornersSorted();
			}
		}
		res.at(res.size() - 1).sepeNum = 0;
	}

	return res;
}

std::tuple<CornersTemplate, bool> Detector::detectCornersRectRoi(const cv::Mat& image)
{
	//auto t0 = tic();
	gray_image = convertToGray(image);
	//std::cout << toc(t0, "t0:") << std::endl;

#ifdef USE_CUDA
	std::tie(I_angle, I_weight, cmax_sigma_2) = secondDerivCornerMetricCuda();
#else
	std::tie(I_angle, I_weight, cmax_sigma_2) = secondDerivCornerMetricRoi(gray_image);
#endif

	corner_candidates = nonMaximumSuppression(cmax_sigma_2, WIDTH_MIN / 2, WIDTH_MIN / 2, NMS_THRESHOLD_SIGMA_2);

	//Maximas corners_sigma_2;
	//std::tie(corners_sigma_2, I_angle, I_weight, cmax_sigma_2) = parallelGetCandidates(gray_image);
	std::sort(corner_candidates.begin(), corner_candidates.end(),
		[](const auto& lhs, const auto& rhs) { return lhs.val > rhs.val; });

	auto [marker_corners, is_vaild] = detectCornersOnMarker();

	if (is_vaild)
	{
		/*bug!!!*/
		if (marker_corners.size() <= MIN_NUM_EXIT)
		{
			return { CornersTemplate(), false };
		}

		return { marker_corners, true };
	}
	else
	{
		return { CornersTemplate(), false };
	}

	return { CornersTemplate(), false };
}

std::tuple<CornersTemplate, bool> Detector::detectCornersOnMarker()
{
	CornersTemplate corners_temp;
	CornersTemplate corners_selected_2_temp;

	int dir_first = 0, dir_second = 0;
	CornerTemplate end_point_first(Corner(0, 0), 0), end_point_second(Corner(0, 0), 0);


	//cv::Mat img_color;
	//cv::cvtColor(gray_image*255, img_color, cv::COLOR_GRAY2BGR);
	//for (const auto& p : corner_candidates)
	//{
	//	cv::circle(img_color, p.corner, 4, cv::Scalar(255, 0, 0), -1);
	//}
	//cv::imwrite("test_.jpg", img_color);

	//for (const auto& p : corners)
	for (int travel_i = 0; travel_i < corner_candidates.size(); travel_i++)
	{
		if (travel_i > FIRST_POINT_MAX_TEST_TIMES)
		{
			break;
		}

		auto p = corner_candidates.at(travel_i);

		auto [corner_first, corner_second, dir] = findFirstSecondCorners(p.corner);
		if (dir != 0)
		{
			CornersTemplate corners_selected_1;

			corners_selected_1.push_back(corner_first);
			corners_selected_1.push_back(corner_second);

			std::array<std::pair<int, CornerTemplate>, 2> comps = {
				std::make_pair(dir, corner_second),
				std::make_pair(-dir, corner_first) };

			int pos_detected_count = 2;
			int neg_detected_count = 0;
			bool search_first_end = false;
			for (auto& comp : comps)
			{
				while (true)
				{
					auto corner_next = predictNextCorner(comp.second, comp.second.width, comp.first);
					if (corner_next.corr < CORR_THRESHOLD)
					{
						if (!search_first_end)
						{
							search_first_end = true;
							dir_first = comp.first;
							end_point_first = comp.second;
						}
						else
						{
							dir_second = comp.first;
							end_point_second = comp.second;
						}
						break;
					}
					else
					{
						if (!search_first_end)
						{
							pos_detected_count++;
						}
						else
						{
							neg_detected_count++;
						}

						comp.second = corner_next;
						corners_selected_1.push_back(corner_next);
					}

				}
			}

			int count_s1 = corners_selected_1.size();
			if (count_s1 > MIN_NUM_FIRST_EXIT)
			{
				CornersTemplate corners_res;

				for (int i = 0; i < neg_detected_count; i++)
				{
					corners_res.push_back(corners_selected_1.at(count_s1 - 1 - i));
				}
				for (int ii = 0; ii < pos_detected_count; ii++)
				{
					corners_res.push_back(corners_selected_1.at(ii));
				}
				
				return { corners_selected_1, true };
			}
		}
	}

	return { CornersTemplate(), false };
}

std::tuple<cv::Mat, cv::Mat, cv::Mat> Detector::secondDerivCornerMetricRoi(const cv::Mat& img_gray)
{
	cv::Mat gaussian_image;// = gray_image;
	cv::GaussianBlur(img_gray, gaussian_image, cv::Size(7 * SIGMA + 1, 7 * SIGMA + 1), SIGMA);

	cv::Mat dx = (cv::Mat_<PixelType>(1, 3) << -1, 0, 1);
	//cv::Mat dx = (cv::Mat_<PixelType>(3, 3) << -1.0 / 3.0, 0, 1.0 / 3.0, -1.0 / 3.0, 0, 1.0 / 3.0, -1.0 / 3.0, 0, 1.0 / 3.0);
	cv::Mat dy;
	cv::transpose(dx, dy);

	// first derivative
	auto Ix = conv2(gaussian_image, dx, "same");
	auto Iy = conv2(gaussian_image, dy, "same");
	auto I_45 = Ix * cos(PI / 4) + Iy * sin(PI / 4);
	auto I_n45 = Ix * cos(-PI / 4) + Iy * sin(-PI / 4);

	// second derivative
	auto Ixy = conv2(Ix, dy, "same");
	auto I_45_x = conv2(I_45, dx, "same");
	auto I_45_y = conv2(I_45, dy, "same");
	auto I_45_45 = I_45_x * cos(-PI / 4) + I_45_y * sin(-PI / 4);

	auto cxy_sigma_2 = static_cast<cv::Mat>(pow(SIGMA, 2) * cv::abs(Ixy) - 1.5 * SIGMA * (cv::abs(I_45) + cv::abs(I_n45)));
	auto c45_sigma_2 = static_cast<cv::Mat>(pow(SIGMA, 2) * cv::abs(I_45_45) - 1.5 * SIGMA * (cv::abs(Ix) + cv::abs(Iy)));
	auto cmax_sigma_2 = static_cast<cv::Mat>(cv::max(cxy_sigma_2, c45_sigma_2));
	cv::Mat zeros_mat = cv::Mat::zeros(cmax_sigma_2.size(), MatType);
	cmax_sigma_2 = cv::max(cmax_sigma_2, zeros_mat);

	cv::Mat I_angle, I_weight;
	cv::phase(Ix, Iy, I_angle);
	cv::magnitude(Ix, Iy, I_weight);

	return { I_angle, I_weight, cmax_sigma_2 };
}


std::tuple<Maximas, cv::Mat, cv::Mat, cv::Mat> Detector::parallelGetCandidates(const cv::Mat& img, const int step_h, const int step_w)
{
	const auto img_height = img.rows;
	const auto img_width = img.cols;

	const int height_num = std::max((int)(cvCeil(img_height / ((PixelType)step_h))), 1);
	const int width_num = std::max((int)(cvCeil(img_width / ((PixelType)step_w))), 1);

	const PixelType step_h_used = img_height / ((PixelType)height_num);
	const PixelType step_w_used = img_width / ((PixelType)width_num);
	const int overlap = WIDTH_MIN;// (7 * SIGMA + 1) + 1;

	std::vector<cv::Mat> img_blocks;// (height_num * width_num);
	std::vector<DetectRectangle> roi_arr;
	for (int h = 0; h < height_num; h++)
	{
		for (int w = 0; w < width_num; w++)
		{
			DetectRectangle rect_;
			rect_.range_x.start = std::max((int)(step_w_used * w - overlap), 0);
			rect_.range_x.end = std::min((int)(step_w_used * (w + 1) + overlap), img_width);
			rect_.range_y.start = std::max((int)(step_h_used * h - overlap), 0);
			rect_.range_y.end = std::min((int)(step_h_used * (h + 1) + overlap), img_height);
			cv::Mat img_x = img(rect_.range_y, rect_.range_x).clone();

			img_blocks.push_back(img_x);
			roi_arr.push_back(rect_);
		}
	}

	Maximas res;
	cv::Mat cmax_sigma_2 = cv::Mat::zeros(img.size(), MatType);
	cv::Mat I_angle = cv::Mat::zeros(img.size(), MatType);
	cv::Mat I_weight = cv::Mat::zeros(img.size(), MatType);
	int block_num = img_blocks.size();
	std::vector<Maximas> maximas_arr(block_num);

	parallel_for_(cv::Range(0, block_num), [&](const cv::Range& range) {

		for (int i = range.start; i < range.end; i++) {
			auto [I_angle_, I_weight_, cmax_] = secondDerivCornerMetricRoi(img_blocks.at(i));
			maximas_arr.at(i) = nonMaximumSuppression(cmax_, WIDTH_MIN / 2, WIDTH_MIN / 2, NMS_THRESHOLD_SIGMA_2);

			DetectRectangle cmax_rect, img_rect;
			if (roi_arr.at(i).range_x.start < 1)
			{
				cmax_rect.range_x.start = 0;
				img_rect.range_x.start = 0;
			}
			else
			{
				cmax_rect.range_x.start = std::min(overlap, cmax_.cols);
				img_rect.range_x.start = std::min(roi_arr.at(i).range_x.start + overlap, img_width);
			}
			if (roi_arr.at(i).range_x.end > img_width - 1)
			{
				cmax_rect.range_x.end = cmax_.cols;
				img_rect.range_x.end = img_width;
			}
			else
			{
				cmax_rect.range_x.end = std::max(cmax_.cols - overlap, 0);
				img_rect.range_x.end = std::max(roi_arr.at(i).range_x.end - overlap, 0);
			}

			if (roi_arr.at(i).range_y.start < 1)
			{
				cmax_rect.range_y.start = 0;
				img_rect.range_y.start = 0;
			}
			else
			{
				cmax_rect.range_y.start = std::min(overlap, cmax_.rows);
				img_rect.range_y.start = std::min(roi_arr.at(i).range_y.start + overlap, img_height);
			}
			if (roi_arr.at(i).range_y.end > img_height - 1)
			{
				cmax_rect.range_y.end = cmax_.rows;
				img_rect.range_y.end = img_height;
			}
			else
			{
				cmax_rect.range_y.end = std::max(cmax_.rows - overlap, 0);
				img_rect.range_y.end = std::max(roi_arr.at(i).range_y.end - overlap, 0);
			}

			cmax_(cmax_rect.range_y, cmax_rect.range_x).copyTo(cmax_sigma_2(img_rect.range_y, img_rect.range_x));
			I_angle_(cmax_rect.range_y, cmax_rect.range_x).copyTo(I_angle(img_rect.range_y, img_rect.range_x));
			I_weight_(cmax_rect.range_y, cmax_rect.range_x).copyTo(I_weight(img_rect.range_y, img_rect.range_x));
		}
		});

	Maximas overlap_res;
	for (int i = 0; i < block_num; i++)
	{
		if (maximas_arr.at(i).size() > 0)
		{
			auto start_x = roi_arr.at(i).range_x.start;
			auto end_x = roi_arr.at(i).range_x.end;
			auto start_y = roi_arr.at(i).range_y.start;
			auto end_y = roi_arr.at(i).range_y.end;

			for (int j = 0; j < maximas_arr.at(i).size(); j++)
			{
				Maxima maxima_tmp = maximas_arr.at(i).at(j);


				maxima_tmp.corner.x += start_x;
				maxima_tmp.corner.y += start_y;

				bool is_overlap = false;
				if ((maxima_tmp.corner.x < start_x + 2 * overlap || maxima_tmp.corner.x > end_x - 2 * overlap) ||
					(maxima_tmp.corner.y < start_y + 2 * overlap || maxima_tmp.corner.y > end_y - 2 * overlap))
				{
					if (overlap_res.size() > 0)
					{
						for (int k = 0; k < overlap_res.size(); k++)
						{
							if (sqrtf(pow(maxima_tmp.corner.x - overlap_res.at(k).corner.x, 2) +
								pow(maxima_tmp.corner.y - overlap_res.at(k).corner.y, 2)) < 1)
							{
								is_overlap = true;
							}
						}
					}

					if (!is_overlap)
					{
						overlap_res.push_back(maxima_tmp);
					}
				}

				if (!is_overlap)
				{
					res.push_back(maxima_tmp);
				}
			}
		}
	}

	return { res, I_angle, I_weight, cmax_sigma_2 };
}

bool Detector::isMarkerCorners(CornerTemplate& checked_corner)
{
	Eigen::Vector2f vect1, vect2, vect_search;
	vect1 << cosf(checked_corner.angle1), sinf(checked_corner.angle1);
	vect2 << cosf(checked_corner.angle2), sinf(checked_corner.angle2);
	vect_search << cosf(checked_corner.search_angle), sinf(checked_corner.search_angle);

	auto angle_s_1 = acosf(fabs(vect_search.dot(vect1)));
	auto angle_s_2 = acosf(fabs(vect_search.dot(vect2)));

	bool res = false;
	if (marker_type > SWM)
	{
		if (fabs(angle_s_1) > DWM_SEARCH_SIDE_ANGLE_THRED && fabs(angle_s_2) > DWM_SEARCH_SIDE_ANGLE_THRED)
		{
			res = true;
		}
	}
	else
	{
		if (fabs(angle_s_1) < SWM_SEARCH_SIDE_ANGLE_THRED || fabs(angle_s_2) < SWM_SEARCH_SIDE_ANGLE_THRED)
		{
			res = true;
		}
	}

	return res;
}


cv::Mat Detector::convertToGray(const cv::Mat& image)
{
	cv::Mat gray;


	/*G 与 B 通道的数据互换*/
	//cv::Mat color_exchanged = image.clone();
	//std::vector<cv::Mat> chs;
	//cv::split(color_exchanged, chs);
	//std::swap(chs[0], chs[1]);
	//cv::merge(chs, color_exchanged);
	//cv::cvtColor(color_exchanged, gray, cv::COLOR_BGR2GRAY);

	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	gray.convertTo(gray, MatType);
	gray = gray / 255;

	return gray;
}

Maximas Detector::nonMaximumSuppression(const cv::Mat& img, int n, int margin, PixelType tau)
{
	auto width = img.cols;
	auto height = img.rows;

	Maximas maxima;
	for (int i = n + margin; i < width - n - margin; i += n + 1)
	{
		for (int j = n + margin; j < height - n - margin; j += n + 1)
		{
			auto max_i = i;
			auto max_j = j;
			auto max_val = img.ptr<PixelType>(j)[i];

			for (int i2 = i; i2 <= i + n; ++i2)
			{
				for (int j2 = j; j2 <= j + n; ++j2)
				{
					auto curr_val = img.ptr<PixelType>(j2)[i2];
					if (curr_val > max_val)
					{
						max_i = i2;
						max_j = j2;
						max_val = curr_val;
					}
				}
			}

			if (max_val < tau)
				continue;

			bool failed = false;
			for (int i2 = max_i - n;
				i2 <= std::min(max_i + n, width - margin - 1);
				i2++)
			{
				for (int j2 = max_j - n;
					j2 <= std::min(max_j + n, height - margin - 1);
					j2++)
				{
					if (img.ptr<PixelType>(j2)[i2] > max_val &&
						(i2 < i || i2 > i + n || j2 < j || j2 > j + n))
					{
						failed = true;
						break;
					}
				}
				if (failed)
					break;
			}

			if (failed)
				continue;

			maxima.emplace_back(max_i, max_j, max_val);
		}
	}

	return maxima;
}

Eigen::MatrixXf Detector::calcPatchX()
{
	std::vector<int> vec;
	for (int i = -HALF_PATCH_SIZE; i <= HALF_PATCH_SIZE; ++i)
		vec.push_back(i);

	auto size = 2 * HALF_PATCH_SIZE + 1;
	Eigen::MatrixXf XX = Eigen::MatrixXf(size * size, 6);
	for (int i = 0; i < size * size; ++i)
	{
		auto x = vec.at(i / size);
		auto y = vec.at(i % size);
		XX(i, 0) = x * x;
		XX(i, 1) = y * y;
		XX(i, 2) = x;
		XX(i, 3) = y;
		XX(i, 4) = x * y;
		XX(i, 5) = 1;
	}

	return (XX.transpose() * XX).inverse() * XX.transpose();
}

std::tuple<CornerTemplate, CornerTemplate, int> Detector::findFirstSecondCorners(const Corner& point)
{
	CornerTemplate corner_first(subPixelLocation(point), init_width0);

	PixelType angle1 = first_ang1, angle2 = first_ang2;
	std::tie(angle1, angle2) = findEdgeAngles(corner_first.point, corner_first.width);
	if (fabs(angle1) < 1e-7 && fabs(angle2) < 1e-7)
		return { corner_first, corner_first, 0 };
	
	//std::cout << "angle2_refined - angle1_refined: " << (angle2 - angle1) * (180 / PI) << std::endl;

	int width_temp1 = std::min((int)(init_width0 * SIDE_LENGTH_RATE), (int)(MAX_TEMPLATE_WIDTH));
	auto corr = calcBolicCorrelation(corner_first.point, width_temp1, angle1, -(angle2 - PI / 2));
	//auto [corr, theta_x1, phi_x1] = calcBolicCorrelationRefineAngle(corner_first.point, width_temp1, angle1, -(angle2 - PI / 2));
	//angle1 = theta_x1; angle2 = PI / 2 - phi_x1;

	if (corr <= CORR_THRESHOLD)
		return { corner_first, corner_first, 0 };
	corner_first.corr = corr;

	// optimize width and angle
	const auto DOUBLE_WIDTH_MIN = 2 * init_width0;
	const auto CORR_THRESHOLD_EXT = 0.6f;

	int width_temp2 = std::min((int)(DOUBLE_WIDTH_MIN * SIDE_LENGTH_RATE), (int)(MAX_TEMPLATE_WIDTH));
	auto corr_x = calcBolicCorrelation(corner_first.point, width_temp2,	angle1, -(angle2 - PI / 2));
	//auto [corr_x, theta_x1_x, phi_x1_x] = calcBolicCorrelationRefineAngle(corner_first.point, width_temp2, angle1, -(angle2 - PI / 2));
	//angle1 = theta_x1_x; angle2 = PI / 2 - phi_x1_x;

	PixelType init_width = init_width0;
	if (corr_x > CORR_THRESHOLD)
	{
		init_width = width_temp2;
		corner_first.width = width_temp2;
		//std::tie (angle1, angle2) = findEdgeAngles(corner_first.point, corner_first.width, first_Ix, first_Iy);
	}
	corner_first.angle1 = angle1;
	corner_first.angle2 = angle2;

	/*set search direction, may diagonal direction*/
	PixelType search_angle1 = 0.0, search_angle2 = 0.0;
	if (marker_type > SWM)
	{
		search_angle1 = 0.5 * (corner_first.angle1 + corner_first.angle2);
		search_angle2 = search_angle1 + PI / 2.0;
		if (search_angle2 > PI)
		{
			search_angle2 = search_angle2 - PI;
		}
		//search_angle2 = search_angle2 > PI ? (search_angle2 - PI) : search_angle2;
	}
	else
	{
		search_angle1 = corner_first.angle1;
		search_angle2 = corner_first.angle2;
	}

	auto corner_second = corner_first;
	/* first--angle, second--direction */
	std::array<std::pair<PixelType, int>, 4> comps = {
		std::make_pair(search_angle1, -1),
		std::make_pair(search_angle1, 1),
		std::make_pair(search_angle2, -1),
		std::make_pair(search_angle2, 1) };

	for (const auto& comp : comps)
	{
		corner_first.width = init_width;
		corner_first.search_angle = comp.first;
		int dir = comp.second;

		while (corner_first.width < try_first_width/*TRY_FIRST_WIDTH_MAX*/)
		{
			auto next_corners = findNextCorners(corner_first, corner_first.width, dir);
			
			for (int i = 0; i < next_corners.size(); i++)
			{
				auto next_corner = next_corners.at(i);

				auto width_temp = cv::norm(next_corner - corner_first.point);
				if (width_temp <= WIDTH_MIN)
				{
					continue;
				}

				auto [angle1_next, angle2_next] = findEdgeAngles(next_corner, width_temp);

				auto width_test0 = std::max(decltype(width_temp)(WIDTH_MIN), decltype(width_temp)(width_temp * SIDE_LENGTH_RATE));
				int width_temp3 = std::min((int)width_test0, (int)(MAX_TEMPLATE_WIDTH));

				auto corr_test_next = calcBolicCorrelation(next_corner, width_temp3, angle1_next, -(angle2_next - PI / 2));
				//auto [corr_test_next, theta_x1_test, phi_x1_test] = calcBolicCorrelationRefineAngle(next_corner, 
				//	width_temp3, angle1_next, -(angle2_next - PI / 2));
				//angle1_next = theta_x1_test;
				//angle2_next = PI / 2 - phi_x1_test;

				if (corr_test_next < CORR_THRESHOLD)
				{
					continue;
				}

				corner_second.point = subPixelLocation(next_corner);

				/*update search_angle*/
				auto search_angle_temp = atan2f((corner_second.point.y - corner_first.point.y),
					(corner_second.point.x - corner_first.point.x));
				if (search_angle_temp < 0)
				{
					search_angle_temp = search_angle_temp + PI;
					dir = -1;
				}
				else
				{
					dir = 1;
				}

				corner_first.search_angle = search_angle_temp;
				corner_second.search_angle = search_angle_temp;
				corner_second.angle1 = angle1_next;
				corner_second.angle2 = angle2_next;

				if (isMarkerCorners(corner_second))
				{
					corner_first.width = width_temp;
					corner_second.width = width_temp;
					corner_second.corr = corr_test_next;

					return { corner_first, corner_second, dir };
				}

			}

			corner_first.width *= 2;
			continue;
			
		}
	}

	return { corner_first, corner_second, 0 };
}

Corner Detector::subPixelLocation(const Corner& point)
{
	if (point.x < HALF_PATCH_SIZE ||
		point.y < HALF_PATCH_SIZE ||
		point.x > cmax_sigma_2.cols - HALF_PATCH_SIZE - 1 ||
		point.y > cmax_sigma_2.rows - HALF_PATCH_SIZE - 1)
	{
		return Corner(point.x, point.y);
	}

	auto width = cmax_sigma_2.cols, height = cmax_sigma_2.rows;
	auto patch = cmax_sigma_2(
		cv::Range(std::max((int)(point.y - HALF_PATCH_SIZE), 0), std::min((int)(point.y + HALF_PATCH_SIZE + 1), height)),
		cv::Range(std::max((int)(point.x - HALF_PATCH_SIZE), 0), std::min((int)(point.x + HALF_PATCH_SIZE + 1), width)));
	if (patch.cols < 2 * HALF_PATCH_SIZE + 1 || patch.rows < 2 * HALF_PATCH_SIZE + 1)
	{
		return Corner(point.x, point.y);
	}

	Eigen::MatrixXf e_patch;
	cv::cv2eigen(patch, e_patch);
	Eigen::Map<Eigen::RowVectorXf> v_patch(e_patch.data(), e_patch.size());
	auto beta = PATCH_X * v_patch.transpose();
	auto A = beta(0), B = beta(1), C = beta(2), D = beta(3), E = beta(4);
	auto delta = 4 * A * B - E * E;
	if (abs(delta) < 1e-7)
		return Corner(point.x, point.y);

	auto x = -(2 * B * C - D * E) / delta;
	auto y = -(2 * A * D - C * E) / delta;
	if (abs(x) > HALF_PATCH_SIZE || abs(y) > HALF_PATCH_SIZE)
		return Corner(point.x, point.y);

	return Corner(point.x + x, point.y + y);
}

std::tuple<PixelType, PixelType> Detector::findEdgeAngles(const Corner& point, const PixelType corner_width)
{
	auto r = WIDTH_MIN;
	if (1.0 * corner_width / 3.0 > r)
	{
		r = 1.0 * corner_width / 3.0;
	}
	auto width = I_angle.cols, height = I_angle.rows;

	int cu = round(point.x), cv = round(point.y);
	auto v_range = cv::Range(std::max(cv - r, 0), std::min(cv + r + 1, height));
	auto u_range = cv::Range(std::max(cu - r, 0), std::min(cu + r + 1, width));

	return edgeOrientation(I_angle(v_range, u_range), I_weight(v_range, u_range));
}


std::tuple<PixelType, PixelType> Detector::edgeOrientation(const cv::Mat& img_angle, const cv::Mat& img_weight)
{
	const auto BIN_NUM = ANGLE_BIN_NUM;
	using Histogram = std::array<PixelType, BIN_NUM>;
	Histogram angle_hist = {};

	/* pair: first--index, second--hist_smoothed(index) */
	using Mode = std::vector<std::pair<int, PixelType>>;

	for (int u = 0; u < img_angle.cols; ++u)
	{
		for (int v = 0; v < img_angle.rows; ++v)
		{
			auto val = [](PixelType angle) {
				angle += PI / 2;
				while (angle > PI)
					angle -= PI;
				return angle;
			}(img_angle.ptr<PixelType>(v)[u]);

			auto bin = std::max(
				std::min(
					static_cast<int>(floor(val / PI * BIN_NUM)),
					BIN_NUM - 1),
				0);

			angle_hist.at(bin) += img_weight.ptr<PixelType>(v)[u];
		}
	}

	auto findModesMeanShift = [&angle_hist, &BIN_NUM](int sigma) {
		Histogram hist_smoothed = {};
		Mode modes;

		for (int i = 0; i < BIN_NUM; ++i)
		{
			for (int j = -2 * sigma; j <= 2 * sigma; ++j)
			{
				auto id = (i + j + BIN_NUM) % BIN_NUM;
				hist_smoothed.at(i) += angle_hist.at(id) * normpdf(j, 0, sigma);
			}
		}

		auto is_all_zeros = [&hist_smoothed]() {
			for (const auto& hist : hist_smoothed)
				if (abs(hist - hist_smoothed.front()) >= 1e-5)
					return false;

			return true;
		};
		if (is_all_zeros())
			return modes;

		for (int i = 0; i < BIN_NUM; ++i)
		{
			auto j = i;
			while (true)
			{
				auto h0 = hist_smoothed.at(j);
				auto j1 = (j + 1 + BIN_NUM) % BIN_NUM;
				auto j2 = (j - 1 + BIN_NUM) % BIN_NUM;
				auto h1 = hist_smoothed.at(j1);
				auto h2 = hist_smoothed.at(j2);

				if (h1 >= h0 && h1 >= h2)
					j = j1;
				else if (h2 > h0 && h2 > h1)
					j = j2;
				else
					break;
			}

			auto contains = [&modes](int j) {
				for (const auto& e : modes)
					if (e.first == j)
						return true;

				return false;
			};
			if (modes.empty() || !contains(j))
			{
				modes.emplace_back(std::make_pair(j, hist_smoothed.at(j)));
			}
		}

		std::sort(modes.begin(), modes.end(),
			[](const auto& lhs, const auto& rhs) { return lhs.second > rhs.second; });

		return modes;
	};
	auto modes = findModesMeanShift(1);

	if (modes.size() <= 1)
		return { 0, 0 };

	PixelType angle1 = modes.at(0).first * PI / BIN_NUM;
	PixelType angle2 = modes.at(1).first * PI / BIN_NUM;
	if (angle1 > angle2)
		std::swap(angle1, angle2);

	auto delta_angle = angle2 - angle1;// std::min(angle2 - angle1, angle2 - angle1 + PI);
	if (delta_angle <= 0.5f)
		return { 0, 0 };

	return { angle1, angle2 };
}

std::tuple<PixelType, PixelType, PixelType> Detector::calcBolicCorrelationRefineAngle(const Corner& point, int theta_width, 
	PixelType theta, PixelType phi, const Eigen::Vector2f search_vect)
{
	PixelType refine_bias_angle = 1.0 * (PI / ANGLE_BIN_NUM);
	std::vector<PixelType> theta_bias_arr, phi_bias_arr;
	theta_bias_arr.push_back(0);	theta_bias_arr.push_back(-refine_bias_angle);	theta_bias_arr.push_back(refine_bias_angle);
	theta_bias_arr.push_back(-2 * refine_bias_angle);	theta_bias_arr.push_back(2 * refine_bias_angle);
	theta_bias_arr.push_back(-3 * refine_bias_angle);	theta_bias_arr.push_back(3 * refine_bias_angle);
	theta_bias_arr.push_back(-4 * refine_bias_angle);	theta_bias_arr.push_back(4 * refine_bias_angle);
	phi_bias_arr.push_back(0);	phi_bias_arr.push_back(-refine_bias_angle);	phi_bias_arr.push_back(refine_bias_angle);
	phi_bias_arr.push_back(-2 * refine_bias_angle);	phi_bias_arr.push_back(2 * refine_bias_angle);
	phi_bias_arr.push_back(-3 * refine_bias_angle);	phi_bias_arr.push_back(3 * refine_bias_angle);
	phi_bias_arr.push_back(-4 * refine_bias_angle);	phi_bias_arr.push_back(4 * refine_bias_angle);

	PixelType corr_max = -100, corr_res = -100;
	PixelType theta_res = theta, phi_res = phi;

	if (unfound_count < MAX_UNFOUND_COUNT)
	{
		for (int theta_i = 0; theta_i < 5 + unfound_count * 2; theta_i++)
		{
			if (theta_i < theta_bias_arr.size())
			{
				auto theta_test = theta + theta_bias_arr.at(theta_i);
				auto phi_test = phi - theta_bias_arr.at(theta_i);
				auto corr_temp = calcBolicCorrelation(point, theta_width, theta_test, phi_test, search_vect);

				if (fabs(corr_temp) > corr_max)
				{
					theta_res = theta_test;
					phi_res = phi_test;
					corr_max = fabs(corr_temp);
					corr_res = corr_temp;

					if (corr_max > 0.85)
					{
						break;
					}
				}
			}

			if (corr_max > 0.85)
			{
				break;
			}
		}
	}
	else
	{
		for (int theta_i = 0; theta_i < theta_bias_arr.size(); theta_i++)
		{
			auto theta_test = theta + theta_bias_arr.at(theta_i);
			for (int phi_i = 0; phi_i < phi_bias_arr.size(); phi_i++)
			{
				auto phi_test = phi + phi_bias_arr.at(phi_i);
				auto corr_temp = calcBolicCorrelation(point, theta_width, theta_test, phi_test, search_vect);

				if (fabs(corr_temp) > corr_max)
				{
					theta_res = theta_test;
					phi_res = phi_test;
					corr_max = fabs(corr_temp);
					corr_res = corr_temp;

					if (corr_max > 0.85)
					{
						break;
					}
				}
			}

			if (corr_max > 0.85)
			{
				break;
			}
		}
	}


	return { corr_res, theta_res, phi_res };

}

PixelType Detector::calcBolicCorrelation(const Corner& point, int side_width, PixelType theta, PixelType phi, const Eigen::Vector2f search_vect)
{
	auto fun_hyperbolic_tangent_scaled =
		[&theta, &phi](PixelType dx, PixelType dy, PixelType alpha = 1, PixelType beta = 1) {
		auto fun_hyperbolic_tangent = [&]() {
			auto u = -dx * sin(theta) + dy * cos(theta);
			auto v = dx * cos(phi) - dy * sin(phi);
			return tanh(alpha * u) * tanh(beta * v);
		};
		return (fun_hyperbolic_tangent() + 1) / 2; // convert to range(0, 1)
	};

	double raw_sum = 0, bolic_sum = 0;
	auto count = 0;

	auto side_angle1 = theta;
	auto side_angle2 = PI / 2.0 - phi;
	Eigen::Vector2f side1(cosf(side_angle1), sinf(side_angle1));
	Eigen::Vector2f side2(cosf(side_angle2), sinf(side_angle2));
	PixelType side_mutual_angle = acosf(fabs(side1.dot(side2)));
	for (int x = -side_width; x <= side_width; ++x)
	{
		for (int y = -side_width; y <= side_width; ++y)
		{
			if (fabs(y) + fabs(x) < side_width)
			{
				PixelType delta_x = x * cosf(side_angle1) + y * cosf(side_angle2);
				PixelType delta_y = x * sinf(side_angle1) + y * sinf(side_angle2);
				int input_x = round(point.x + delta_x);
				int input_y = round(point.y + delta_y);

				if (input_x > 0 && input_x < gray_image.cols && input_y > 0 && input_y < gray_image.rows)
				{
					raw_sum += gray_image.ptr<PixelType>(input_y)[input_x];
					//raw_sum += imgAt(gray_image, input_x, input_y);
					auto htm_model = fun_hyperbolic_tangent_scaled(delta_x, delta_y);

					Eigen::Vector2f temp_vect(delta_x, delta_y);
					if (marker_type > SWM && temp_vect.norm() > 0.1)
					{
						temp_vect = temp_vect / temp_vect.norm();
						PixelType search_side1_angle = acosf(fabs(side1.dot(temp_vect)));
						PixelType search_side2_angle = acosf(fabs(side2.dot(temp_vect)));

						if (search_side1_angle > side_mutual_angle || search_side2_angle > side_mutual_angle)
						{
							if (marker_type == DWM_A)
							{
								if (htm_model > 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}
							else
							{
								if (htm_model < 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}
						}
						else
						{
							if (marker_type == DWM_A)
							{
								if (htm_model < 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}
							else
							{
								if (htm_model > 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}

						}
					}

					bolic_sum += htm_model;
					++count;
				}
			}
		}
	}
	auto raw_avg = raw_sum / count;
	auto bolic_avg = bolic_sum / count;

	double cov = 0, var_bolic = 0, var_raw = 0;
	for (int x = -side_width; x <= side_width; ++x)
	{
		for (int y = -side_width; y <= side_width; ++y)
		{
			if (fabs(y) + fabs(x) < side_width)
			{
				PixelType delta_x = x * cosf(side_angle1) + y * cosf(side_angle2);
				PixelType delta_y = x * sinf(side_angle1) + y * sinf(side_angle2);
				int input_x = round(point.x + delta_x), input_y = round(point.y + delta_y);

				if (input_x > 0 && input_x < gray_image.cols && input_y > 0 && input_y < gray_image.rows)
				{
					auto diff_raw = gray_image.ptr<PixelType>(input_y)[input_x] - raw_avg;
					// auto diff_raw = imgAt(gray_image, input_x, input_y) - raw_avg;
					auto htm_model = fun_hyperbolic_tangent_scaled(delta_x, delta_y);

					Eigen::Vector2f temp_vect(delta_x, delta_y);
					if (marker_type > SWM && temp_vect.norm() > 0.1)
					{
						temp_vect = temp_vect / temp_vect.norm();
						PixelType search_side1_angle = acosf(fabs(side1.dot(temp_vect)));
						PixelType search_side2_angle = acosf(fabs(side2.dot(temp_vect)));

						if (search_side1_angle > side_mutual_angle || search_side2_angle > side_mutual_angle)
						{
							if (marker_type == DWM_A)
							{
								if (htm_model > 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}
							else
							{
								if (htm_model < 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}
						}
						else
						{
							if (marker_type == DWM_A)
							{
								if (htm_model < 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}
							else
							{
								if (htm_model > 0.5)
								{
									htm_model = 1.0 - htm_model;
								}
							}

						}
					}
					auto diff_bolic = htm_model - bolic_avg;

					cov += diff_raw * diff_bolic;
					var_raw += pow(diff_raw, 2);
					var_bolic += pow(diff_bolic, 2);
				}
			}
		}
	}

	if (var_raw > 0 && var_bolic > 0)
	{
		if (marker_type > SWM)
		{
			return /*abs*/(cov / (sqrt(var_raw) * sqrt(var_bolic)));
		}
		else
		{
			return abs(cov / (sqrt(var_raw) * sqrt(var_bolic)));
		}
	}
	return 0;
}

Corners Detector::findNextCorners(const CornerTemplate& current, const PixelType search_length, int dir)
{
	auto width = cmax_sigma_2.cols, height = cmax_sigma_2.rows;
	int predict_x = (int)(current.point.x + dir * search_length * cos(current.search_angle));
	int predict_y = (int)(current.point.y + dir * search_length * sin(current.search_angle));
	Corner c(predict_x, predict_y);
	auto side = std::max(search_length / 3.0, WIDTH_MIN / 3.0);

	//image_roi = image(rect.range_y, rect.range_x).clone();
	DetectRectangle sub_roi_rect;
	sub_roi_rect.range_x = cv::Range(std::max((int)(c.x - 6 * side), 0), std::min((int)(c.x + 6 * side), gray_image.cols));
	sub_roi_rect.range_y = cv::Range(std::max((int)(c.y - 6 * side), 0), std::min((int)(c.y + 6 * side), gray_image.rows));
	cv::Mat sub_img_roi = gray_image(sub_roi_rect.range_y, sub_roi_rect.range_x).clone();
#ifdef USE_CUDA
	std::tie(I_angle, I_weight, cmax_sigma_2) = secondDerivCornerMetricCuda();
#else
	auto [sub_I_angle, sub_I_weight, sub_cmax_sigma_2] = secondDerivCornerMetricRoi(sub_img_roi);
#endif

	auto sub_corner_candidates = nonMaximumSuppression(sub_cmax_sigma_2, WIDTH_MIN / 2, WIDTH_MIN / 2, NMS_THRESHOLD_SIGMA_2);

	//Maximas corners_sigma_2;
	//std::tie(corners_sigma_2, I_angle, I_weight, cmax_sigma_2) = parallelGetCandidates(gray_image);
	std::sort(sub_corner_candidates.begin(), sub_corner_candidates.end(),
		[](const auto& lhs, const auto& rhs) { return lhs.val > rhs.val; });

	Corners res;
	for (int i = 0; i < sub_corner_candidates.size(); i++)
	{
		auto p = Corner(sub_corner_candidates.at(i).corner.x, sub_corner_candidates.at(i).corner.y)
			+ Corner(sub_roi_rect.range_x.start, sub_roi_rect.range_y.start);
		auto dx = p.x - c.x;
		auto dy = p.y - c.y;
		if (fabs(dx) < side && fabs(dy) < side)
		{
			if (sqrtf(powf(p.x - c.x, 2.0) + powf(p.y - c.y, 2.0)) < side)
			{
				res.push_back(p);
			}
		}
	}
	return res;
}

CornerTemplate Detector::predictNextCorner(const CornerTemplate& current, const PixelType search_length, int dir)
{
	auto next_corners = findNextCorners(current, search_length, dir);

	for (int i = 0; i < next_corners.size(); i++)
	{
		auto next_corner = next_corners.at(i);

		CornerTemplate corner_next(subPixelLocation(next_corner), WIDTH_MIN);
		auto [angle1, angle2] = findEdgeAngles(corner_next.point, current.width);

		auto width_test0 = std::max(decltype(current.width)(WIDTH_MIN / 2.0), 
			decltype(current.width)(current.width * SIDE_LENGTH_RATE));
		int width_temp3 = std::min((int)width_test0, (int)(MAX_TEMPLATE_WIDTH));
		auto corr_next = calcBolicCorrelation(corner_next.point, width_temp3, angle1, -(angle2 - PI / 2));
		//auto [corr_next, theta_x1_next, phi_x1_next] = calcBolicCorrelationRefineAngle(corner_next.point, width_temp3, 
		//	angle1, -(angle2 - PI / 2));
		//angle1 = theta_x1_next;
		//angle2 = PI / 2 - phi_x1_next;

		if (corr_next > CORR_THRESHOLD)
		{
			PixelType width_temp = cv::norm(corner_next.point - current.point);

			/*update search_angle*/
			auto search_angle_temp = atan2f((corner_next.point.y - current.point.y), (corner_next.point.x - current.point.x));
			if (fabs(search_angle_temp + PI - current.search_angle) < fabs(search_angle_temp - current.search_angle))
			{
				search_angle_temp = search_angle_temp + PI;
			}
			else if (fabs(search_angle_temp - PI - current.search_angle) < fabs(search_angle_temp - current.search_angle))
			{
				search_angle_temp = search_angle_temp - PI;
			}

			corner_next.search_angle = search_angle_temp;
			corner_next.angle1 = angle1;
			corner_next.angle2 = angle2;
			corner_next.width = width_temp;

			if (isMarkerCorners(corner_next) && fabs(corner_next.search_angle - current.search_angle) 
				< SEARCH_CHANGED_ANGLE_THRED)
			{
				corner_next.corr = corr_next;

				return corner_next;
			}
		}
	}
	
	return CornerTemplate(Corner(0, 0), WIDTH_MIN);
}

/////////////////////////////////////CUDA/////////////////////////////////////
#ifdef USE_CUDA
void Detector::initCuda(const cv::Size& size)
{
	/* cuda first initialization */
	auto A = cv::Mat::ones(cv::Size(3, 3), MatType);
	auto B = cv::Mat::ones(cv::Size(2, 2), MatType);
	cv::cuda::GpuMat gA(A), gR;
	cv::cuda::add(gA, gA, gR);

	cv::Mat F = cudaFilter(A, B);

	/* filter initialization */
	cv::Mat dx = (cv::Mat_<PixelType>(1, 3) << -1, 0, 1);
	cv::Mat dy;
	cv::transpose(dx, dy);

	auto filter = [](const cv::Mat& kernel, cv::Ptr<cv::cuda::Filter>& f) {
		cv::Mat flip_kernel;
		cv::flip(kernel, flip_kernel, -1);
		cv::Point anchor(flip_kernel.cols - flip_kernel.cols / 2 - 1, flip_kernel.rows - flip_kernel.rows / 2 - 1);

		f = cv::cuda::createLinearFilter(MatType, MatType, flip_kernel, anchor, cv::BORDER_CONSTANT);
	};
	filter(dx, filter_dx);
	filter(dy, filter_dy);
	filter_G = cv::cuda::createGaussianFilter(MatType, MatType, cv::Size(7 * SIGMA + 1, 7 * SIGMA + 1), SIGMA);

	/* mat initialization */
	cv::Mat ones_mat = cv::Mat::ones(size, MatType);
	cv::Mat zeros_mat = cv::Mat::zeros(size, MatType);
	g_ones.upload(ones_mat);
	g_zeros.upload(zeros_mat);
}

std::tuple<cv::Mat, cv::Mat, cv::Mat> Detector::secondDerivCornerMetricCuda()
{
	auto ttt1 = tic();
	cv::cuda::GpuMat g_gray_image(gray_image), g_gaussian_image;
	filter_G->apply(g_gray_image, g_gaussian_image);
	toc(ttt1, "ttt1");

	// first derivative
	auto ttt2 = tic();
	cv::cuda::GpuMat g_Ix, g_Iy, g_I_45, g_I_n45;
	filter_dx->apply(g_gaussian_image, g_Ix);
	filter_dy->apply(g_gaussian_image, g_Iy);
	cv::cuda::GpuMat temp1, temp2;
	cv::cuda::multiply(g_Ix, g_ones, temp1, cos(PI / 4));
	cv::cuda::multiply(g_Iy, g_ones, temp2, sin(PI / 4));
	cv::cuda::add(temp1, temp2, g_I_45);
	cv::cuda::subtract(temp1, temp2, g_I_n45);
	toc(ttt2, "ttt2");

	// second derivative
	auto ttt3 = tic();
	cv::cuda::GpuMat g_Ixy, g_I_45_x, g_I_45_y, g_I_45_45;
	filter_dy->apply(g_Ix, g_Ixy);
	filter_dx->apply(g_I_45, g_I_45_x);
	filter_dy->apply(g_I_45, g_I_45_y);
	cv::cuda::multiply(g_I_45_x, g_ones, temp1, cos(-PI / 4));
	cv::cuda::multiply(g_I_45_y, g_ones, temp2, sin(-PI / 4));
	cv::cuda::add(temp1, temp2, g_I_45_45);
	toc(ttt3, "ttt3");

	// cmax_sigma_2
	auto ttt4 = tic();
	auto sigma_2 = pow(SIGMA, 2), sigma_n15 = -1.5 * SIGMA;
	cv::cuda::GpuMat g_cxy, g_c45, g_cmax_sigma_2;
	cv::cuda::abs(g_I_45, temp1);
	cv::cuda::abs(g_I_n45, temp2);
	cv::cuda::add(temp1, temp2, temp1);
	cv::cuda::multiply(temp1, g_ones, temp1, sigma_n15);
	cv::cuda::abs(g_Ixy, temp2);
	cv::cuda::scaleAdd(temp2, sigma_2, temp1, g_cxy);

	cv::cuda::abs(g_Ix, temp1);
	cv::cuda::abs(g_Iy, temp2);
	cv::cuda::add(temp1, temp2, temp1);
	cv::cuda::multiply(temp1, g_ones, temp1, sigma_n15);
	cv::cuda::abs(g_I_45_45, temp2);
	cv::cuda::scaleAdd(temp2, sigma_2, temp1, g_c45);

	cv::cuda::max(g_cxy, g_c45, g_cmax_sigma_2);
	cv::cuda::max(g_cmax_sigma_2, g_zeros, g_cmax_sigma_2);

	cv::cuda::GpuMat g_I_angle, g_I_weight;
	cv::cuda::phase(g_Ix, g_Iy, g_I_angle);
	cv::cuda::magnitude(g_Ix, g_Iy, g_I_weight);
	toc(ttt4, "ttt4");

	// download
	auto ttt5 = tic();
	cv::Mat I_angle, I_weight, cmax_sigma_2;
	g_I_angle.download(I_angle);
	g_I_weight.download(I_weight);
	g_cmax_sigma_2.download(cmax_sigma_2);
	toc(ttt5, "ttt5");

	return { I_angle, I_weight, cmax_sigma_2 };
}
#endif
/////////////////////////////////////CUDA/////////////////////////////////////