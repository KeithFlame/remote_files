#ifndef EIGHT_POINT_MARKER_LOCATOR_H_LF
#define EIGHT_POINT_MARKER_LOCATOR_H_LF
#include <vector>
#include <opencv2/opencv.hpp>
#include "eight_point_marker.h"

#define DEBUG_MARKER_LOCATOR 0
#define ENABLE_SHOW_RESULTS 1

namespace epm {

#if DEBUG_MARKER_LOCATOR
    inline void show(std::string win_name, const cv::Mat& mat, float scale = 0.3)
    {
        int w = mat.cols;
        int h = mat.rows;
        cv::Mat tmp;
        cv::resize(mat, tmp, cv::Size((int)w * scale, (int)h * scale));
        cv::imshow(win_name, tmp);
    }

#define MTRACE(fmt, ...) \
    printf("[%s][%s][%d] " fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#endif


    /**
     * An array of Eight-Point-Marker
     */
    using MarkerLocations = std::vector<std::vector<cv::Point2f>>;


    /**
     * @brief The EightPointMarkerLocator class
     */
    class EightPointMarkerLocator
    {
    public:
        EightPointMarkerLocator();
        ~EightPointMarkerLocator();

        MarkerLocations locateMarkers(const cv::Mat& image, const cv::Rect& rect, bool is_first);


#if ENABLE_SHOW_RESULTS
        void showResults(int wait_time = 0);
#endif

    private:
        cv::Mat findMarkers(const cv::Mat& image, uint8_t thresh, const cv::Rect& rect_);
        cv::Mat trackMarkers(cv::Mat& image, const cv::Rect& rect_);

        std::vector<EightPointMarker> _markers;

        uint16_t _width;
        uint16_t _height;

#if ENABLE_SHOW_RESULTS
        cv::Mat _image;
        cv::Rect _rect;
#endif
    };

} // namespace::epm
#endif // EIGHT_POINT_MARKER_LOCATOR_H_LF
