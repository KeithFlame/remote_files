#pragma once
#include "measurement_system_params.h"

class ImageDetection
{
public:
	ImageDetection() {};
	~ImageDetection() {};
	bool markerIdentify(const cv::Mat& src, const cv::Rect& gb_rect,
		std::vector<PATTERN_CONTAINER>& marker_decal);
private:
	bool find8Points(const cv::Mat& image, std::vector<std::vector<cv::Point2f>>& result,
		std::vector<std::vector<cv::Point>>& contoursRect);
	void identifyMarkerId(const std::vector<std::vector<cv::Point2f>>& points8, const std::vector<std::vector<cv::Point>> cnr_rect,
		std::vector<PATTERN_CONTAINER>& markerPoints_temp);

	// find8Points函数用得到
	void findSquares_adapt(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares);
	bool findCircles_comb(const std::vector < std::vector<cv::Point>>& contoursRect, const cv::Mat& image,
		std::vector<std::vector<cv::Point2f>>& result, std::vector<std::vector<cv::Point>>& resRect);

	// findSquares_adapt函数用得到
	void detectRectangles(const cv::Mat& thresImg, std::vector<std::vector<cv::Point>>& OutMarkerContours);
	int perimeter(std::vector<cv::Point>& a);

	// findCircles_comb函数用得到
	bool findCircles_new_sub(const std::vector<cv::Point>& cR, const cv::Mat& image,
		std::vector<cv::Point2f>& temp_result);
	bool findCircles_sub(const std::vector<cv::Point>& cR, const cv::Mat& image,
		std::vector<cv::Point2f>& temp_result);
	float minDistancePointRect(const cv::Point2f& px, const std::vector<cv::Point>& rect);

	// identifyMarkerIdV2函数用得到
	bool distinguish8Points(const std::vector<cv::Point2f>& pointsIn, const std::vector<cv::Point> cnr_rect,
		PATTERN_CONTAINER& pointsOut);

	// distinguish8Points函数用得到
	bool IsPointInRect(cv::Point2f P, cv::Point2f A, cv::Point2f B, cv::Point2f C, cv::Point2f D = cv::Point2f(0.f, 0.f));
	void sortBubble(std::vector<PATTERN_CONTAINER>& marker_detected);

};

