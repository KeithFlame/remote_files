#include "eight_point_marker_locator.h"
#include "eight_point_marker_util.h"
#include <cstdint>
#include <algorithm>

namespace epm {

    constexpr uint8_t  MARKER_BG = 25;
    constexpr uint8_t  MARKER_BG2 = 35;
    constexpr uint8_t  MARKER_BG3 = 45;
    constexpr uint8_t  MARKER_BG4 = 55;
    constexpr uint16_t MARKER_AREA_MIN = 36 * 36;
    constexpr uint16_t MARKER_PT_AREA_MIN = 20;
    constexpr uint16_t MARKER_PT_AREA_MAX = 3000;

    bool isMarker(cv::Mat& bw, const cv::Mat& gray,
        MarkerPointLocations& pt_locations, MarkerPointAreas& pt_areas,
        const EightPointMarker* prev_marker);


    EightPointMarkerLocator::EightPointMarkerLocator()
        : _width(1920)
        , _height(1080)
    {
        _markers.clear();
    }


    EightPointMarkerLocator::~EightPointMarkerLocator()
    {
    }


    MarkerLocations EightPointMarkerLocator::locateMarkers(const cv::Mat& image, 
        const cv::Rect& rect, bool is_first)
    {
        if (image.empty()) return MarkerLocations();

        _width = image.cols;
        _height = image.rows;
        //printf("_width: %d, _height:%d\n", _width, _height);
#if ENABLE_SHOW_RESULTS
        _image = image;
        _rect = rect;
#endif
        cv::Mat gray = image.clone();
        //cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        //cv::GaussianBlur(gray, gray, cv::Size(5, 5), 3);

        if (_markers.size() > 0) {
#if DEBUG_MARKER_LOCATOR
            MTRACE("Track markers first\n");
#endif
                for (auto& marker : _markers) {
                    MarkerPointLocations pts = marker.pts();
                    for (auto& pt : pts) {
                        pt = pt - cv::Point2f(rect.x, rect.y);
                    }
                    marker.update(pts);
                }
   
            gray = trackMarkers(gray, rect);

            //if (_markers.size() > 0)
            //{
            //    MarkerLocations marker_locations;
            //    for (auto& marker : _markers) {
            //        marker_locations.push_back(marker.pts());
            //    }
            //    return marker_locations;
            //}
        }
        if (is_first || _markers.size() == 0) {
#if DEBUG_MARKER_LOCATOR
            MTRACE("Find markers iter1\n");
#endif
            gray = findMarkers(gray, MARKER_BG, rect);
#if DEBUG_MARKER_LOCATOR
            MTRACE("Find markers iter2\n");
#endif
            gray = findMarkers(gray, MARKER_BG2, rect);
            #if DEBUG_MARKER_LOCATOR
                    MTRACE("Find markers iter3\n");
            #endif
                    gray = findMarkers(gray, MARKER_BG3, rect);
            #if DEBUG_MARKER_LOCATOR
                    MTRACE("Find markers iter4\n");
            #endif
                    findMarkers(gray, MARKER_BG4, rect);

        }

        MarkerLocations marker_locations;
        for (auto& marker : _markers) {
            marker_locations.push_back(marker.pts());
        }
        return marker_locations;
    }


#if ENABLE_SHOW_RESULTS
    void EightPointMarkerLocator::showResults(int wait_time)
    {
        for (size_t i = 0; i < _markers.size(); i++) {
            auto& marker = _markers[i];
            printf("\t#%ld marker: size=%ld\n", i, marker.pts().size());
            cv::Scalar color = marker.color();
            for (size_t j = 0; j < marker.pts().size(); j++) {
                cv::Point2f pt = marker.pts()[j] - cv::Point2f(_rect.x, _rect.y);
                cv::circle(_image, pt, 3, color, 3);
                //printf("\t\t Pt%ld:[%f,%f]\n", j, pt.x, pt.y);
            }
        }
        cv::imshow("Results", _image);
        cv::waitKey(wait_time);
    }
#endif


    cv::Mat EightPointMarkerLocator::findMarkers(const cv::Mat& gray, uint8_t thresh, const cv::Rect& rect_)
    {
        cv::Mat BW = gray < thresh;
#if DEBUG_MARKER_LOCATOR
        show("findMarkers:BW", BW);
#endif

        cv::Mat labels, stats, centroids;
        int num = cv::connectedComponentsWithStats(BW, labels, stats, centroids);
        for (uint16_t i = 1; i < num; i++) {
            int area = stats.at<int>(i, 4);
            if (area < MARKER_AREA_MIN) {
                continue;
            }
            cv::Mat bw = (labels == i);

            MarkerPointLocations pt_locations;
            MarkerPointAreas pt_areas;
            if (isMarker(bw, gray, pt_locations, pt_areas, nullptr)) {
                EightPointMarker marker(pt_locations, pt_areas);
                // Refine the marker
                cv::Rect rect = marker.rect(_width, _height);
                cv::Mat roi_gray = gray(rect);
                cv::Mat roi_bw;
                float thresh1 = cv::threshold(roi_gray, roi_bw, 0, 255,
                    cv::THRESH_OTSU | cv::THRESH_BINARY_INV);

                float thresh2 = cv::mean(roi_gray)[0];
                float refined_thresh = MIN(thresh1, 0.8 * thresh2);
                roi_bw = roi_gray < refined_thresh;

#if DEBUG_MARKER_LOCATOR
                MTRACE("Find a valid marker: pt.size:%ld, area.size:%ld\n",
                    pt_locations.size(), pt_areas.size());
                printf("\tthresh: %f, mean_thresh:%f\n", thresh1, 0.8 * thresh2);
                show("findMarker:roi_gray", roi_gray);
                show("findMarker:isMarker", bw);
                show("findMarker:isMarker_refine", roi_bw); cv::waitKey(0);
#endif
                isMarker(roi_bw, roi_gray, pt_locations, pt_areas, nullptr);
                if (pt_locations.size() == 8) {
#if DEBUG_MARKER_LOCATOR
                    MTRACE("Refine the marker done: pt.size:%ld, area.size:%ld\n",
                        pt_locations.size(), pt_areas.size());
#endif
                    for (auto& pt : pt_locations) {
                        pt = cv::Point2f(pt.x + rect.x + rect_.x, pt.y + rect.y + rect_.y);
                    }
                    marker.update(pt_locations, pt_areas);
                    _markers.push_back(marker);

                    cv::Mat white = cv::Mat(rect.height, rect.width, gray.type(), cv::Scalar(255));
                    white.copyTo(gray(rect));

                    //printf("\tfind done\n");
                }
            }
        }
        return gray;
    }


    cv::Mat EightPointMarkerLocator::trackMarkers(cv::Mat& gray, const cv::Rect& rect_)
    {
        size_t i = 0;
        while (true) {
            if (i >= _markers.size()) break;

            auto& marker = _markers[i];
            cv::Rect rect = marker.rect(_width, _height);
            if (rect.x < 0 || rect.y < 0 || (rect.width < 0 || rect.width > rect_.width) 
                || (rect.height < 0 || rect.height > rect_.height))
            {
                _markers = epm::rmElement(_markers, i);
                continue;
            }

            cv::Mat roi_gray;
            try
            {
                roi_gray = gray(rect);
            }
            catch (const std::exception&)
            {
                printf("xiang wang.\n");
            }

            //cv::Mat roi_gray = gray(rect);
            cv::Mat roi_bw;
            float thresh1 = cv::threshold(roi_gray, roi_bw, 0, 255,
                cv::THRESH_OTSU | cv::THRESH_BINARY_INV);
            float thresh2 = cv::mean(roi_gray)[0];
            float refined_thresh = MIN(thresh1, 0.8 * thresh2);
            roi_bw = roi_gray < refined_thresh;

#if DEBUG_MARKER_LOCATOR
            show("trackMarker:roi_bw", roi_bw);
            printf("\t Track thresh: %f, mean_thresh:%f\n", thresh1, 0.8 * thresh2);
#endif
            MarkerPointLocations pt_locations;
            MarkerPointAreas pt_areas;
            if (isMarker(roi_bw, roi_gray, pt_locations, pt_areas, &marker)) {
                for (auto& pt : pt_locations) {
                    pt = cv::Point2f(pt.x + rect.x + rect_.x, pt.y + rect.y + rect_.y);
                }
                marker.update(pt_locations, pt_areas);

                i++;
                cv::Mat white = cv::Mat(rect.height, rect.width, gray.type(), cv::Scalar(255));
                white.copyTo(gray(rect));
#if DEBUG_MARKER_LOCATOR
                MTRACE("Track marker successfully: pt.size:%ld, area.size:%ld\n",
                    pt_locations.size(), pt_areas.size());
                show("trackMarker:isMarker", roi_bw);
                show("trackMarker:", gray); cv::waitKey(0);
#endif
                //printf("\ttrack done\n");
            }
            else {
                _markers = epm::rmElement(_markers, i);
            }
        }
        return gray;
    }


    /*****************************************************************************/
    std::vector<int> filterFlags(std::vector<int>& flags, float mean_area, int count)
    {
        for (uint16_t i = 0; i < flags.size(); i++) {
            if (!flags[i]) continue;
#if DEBUG_CODES
            printf("area: %d\t", flags[i]);
#endif
            flags[i] = abs(flags[i] - mean_area) + 1;
#if DEBUG_CODES
            printf("delta area: %d\n", flags[i]);
#endif
        }

        for (uint16_t i = 0; i < count - 8; i++) {
            flags[findMaxValueId(flags)] = 0;
        }

        return flags;
    }


    std::vector<int> filterIndices(std::vector<int>& indicies, const cv::Mat& stats)
    {
        size_t i = 0;
        while (i < indicies.size()) {
            int id = indicies[i];
            int w = stats.at<int>(id, 2);
            int h = stats.at<int>(id, 3);
            float ratio = 1.0 * MIN(w, h) / MAX(w, h);
            if (ratio < 0.35) {
                indicies = rmElement(indicies, i);
            }
            else {
                i++;
            }
        }
        return indicies;
    }


    bool filterIndices(std::vector<int>& indices, const cv::Mat& stats,
        const cv::Mat& centroids, const cv::Mat& gray)
    {
        auto meanValue = [&gray](int x1, int x2, int y) -> float {
            float sum = 0;
            int count = 0;
            for (int x = x1; x <= x2; x++) {
                sum += gray.at<uchar>(y, x);
                count++;
            }
            return sum / count;
        };

        std::vector<float> P_is_circle(indices.size());
        for (size_t i = 0; i < indices.size(); i++) {
            int id = indices[i];
            int xc = centroids.at<double>(id, 0);
            int yc = centroids.at<double>(id, 1);
            float center = gray.at<uchar>(yc, xc);

            int x = stats.at<int>(id, 0);
            int y = stats.at<int>(id, 1);
            int w = stats.at<int>(id, 2);
            int h = stats.at<int>(id, 3);
            int d = h * 0.08;
            int y1 = MAX(y - d, 0);
            int y2 = MIN(y + h + d, gray.rows - 1);
            float surround = 0.5 * meanValue(x, x + w-1, y1) +
                0.5 * meanValue(x, x + w-1, y2);
            P_is_circle[i] = center / surround;
#if DEBUG_MARKER_LOCATOR
            printf("\t#%ld Contrast: %f\n", i, P_is_circle[i]);
#endif
        }

        size_t i = 0;
        while (i < indices.size()) {
            float p = P_is_circle[i];
            if (p < 1.3) {
                P_is_circle = rmElement(P_is_circle, i);
                indices = rmElement(indices, i);
            }
            else {
                i++;
            }
        }
#if DEBUG_MARKER_LOCATOR
        printf("\tAfter filtering by contrast: %ld\n", indices.size());
#endif
        if (indices.size() < 8) return false;

        while (indices.size() > 8) {
            size_t id = findMinValueId(P_is_circle);
            P_is_circle = rmElement(P_is_circle, id);
            indices = rmElement(indices, id);
        }
        return true;
    }


    bool isCentroidValid(std::vector<int>& indices, const cv::Mat& centroids)
    {
        float min_xc = centroids.at<double>(indices[0], 0);
        float max_xc = min_xc;
        float min_yc = centroids.at<double>(indices[0], 1);
        float max_yc = min_yc;
        for (size_t i = 1; i < indices.size(); i++) {
            int id = indices[i];
            float xc = centroids.at<double>(id, 0);
            float yc = centroids.at<double>(id, 1);
            if (xc > max_xc) { max_xc = xc; }
            else if (xc < min_xc) { min_xc = xc; }

            if (yc > max_yc) { max_yc = yc; }
            else if (yc < min_yc) { min_yc = yc; }
        }
        float max_range = MAX(max_xc - min_xc, max_yc - min_yc);
        float min_range = MIN(max_xc - min_xc, max_yc - min_yc);
        if (min_range / max_range < 0.3 || max_range > 360) {
            return false;
        }
        return true;
    }


    bool isAreaValid(std::vector<int>& indices, const cv::Mat& stats)
    {
        int area_min = stats.at<int>(indices[0], 4);
        int area_max = area_min;
        for (size_t i = 1; i < indices.size(); i++) {
            int area = stats.at<int>(indices[i], 4);
            if (area > area_max) {
                area_max = area;
            }
            else if (area < area_min) {
                area_min = area;
            }
        }
        if (area_min - area_max > 400) {
            return false;
        }
        return true;
    }


    bool isMarker(cv::Mat& bw, const cv::Mat& gray,
        MarkerPointLocations& pt_locations, MarkerPointAreas& pt_areas,
        const EightPointMarker* prev_marker)
    {
        pt_locations.clear();
        pt_locations.reserve(8);
        pt_areas.clear();
        pt_areas.reserve(8);

        uint16_t pt_area_min = MARKER_PT_AREA_MIN;
        uint16_t pt_area_max = MARKER_PT_AREA_MAX;
        if (prev_marker) {
            pt_area_min = prev_marker->ptAreaMin();
            pt_area_max = prev_marker->ptAreaMax();
        }

        bw = ~bw;
#if DEBUG_MARKER_LOCATOR
        printf("isMarker: a new roi_bw\n");
        show("isMarker::~roi_bw", bw); cv::waitKey(0);
#endif

        cv::Mat labels, stats, centroids;
        int num = cv::connectedComponentsWithStats(bw, labels, stats, centroids);
        std::vector<int> indices;
        for (uint16_t i = 1; i < num; i++) {
            int area = stats.at<int>(i, 4);
#if DEBUG_MARKER_LOCATOR
            if (prev_marker) printf("\tarea: %d\n", area);
#endif
            if (area > pt_area_min && area < pt_area_max) {
                indices.push_back(i);
#if DEBUG_MARKER_LOCATOR
                printf("\t\tarea: %d\n", area);
                show("isMarker:current_bw", labels == i);
                cv::waitKey(0);
#endif
            }
        }
#if DEBUG_MARKER_LOCATOR
        printf("\tPossible circle: %ld\n", indices.size());
#endif
        if (indices.size() < 8) {
            return false;
        }

        // Filter by local shape
        indices = filterIndices(indices, stats);
#if DEBUG_MARKER_LOCATOR
        printf("\tAfter filtering by shape: %ld\n", indices.size());
#endif
        if (indices.size() < 8) return false;
        // Filter by local contrast
        if (!filterIndices(indices, stats, centroids, gray))
            return false;
#if DEBUG_MARKER_LOCATOR
        printf("\tPass contrast check: pt.size:%ld\n", indices.size());
#endif

        // Filter the final centroid
        if (!isCentroidValid(indices, centroids))
            return false;

        // Filter the final area
        if (!isAreaValid(indices, stats))
            return false;


        // Get the centroid and area
        int xc = 0, yc = 0;
        for (size_t i = 0; i < indices.size(); i++) {
            int id = indices[i];
            xc = centroids.at<double>(id, 0);
            yc = centroids.at<double>(id, 1);
            pt_locations.push_back(cv::Point2f(xc, yc));

            int area = stats.at<int>(id, 4);
            pt_areas.push_back(area);
        }
        return true;
    }

} // namespace::epm
