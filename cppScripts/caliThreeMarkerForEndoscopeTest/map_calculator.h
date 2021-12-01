#ifndef MAP_CALCULATOR_H_
#define MAP_CALCULATOR_H_
#include <opencv2/opencv.hpp>
//#include "camera_parameters.h"
#include <fstream>

/** @brief Create the map tor rectify the image
  This class is designed as a virtual class
*/
class MapCalculator
{
public:
    MapCalculator(std::string parameterPath, int i = 0, uint16_t image_width = 2592, uint16_t image_height = 1944);

    ~MapCalculator() {}

    /** @brief Update the map for rectification.
    */
    void updateMap(int disparity = 0);

    const cv::Mat& getCPUMapx() const { return cpu_mapx; }
    const cv::Mat& getCPUMapy() const { return cpu_mapy; }
    void getCameraParameters(std::string parameterPath = NULL, int i = 0);
    cv::Mat rectify(cv::Mat mat);

protected:
    /** @brief Initialize all the Mat
    */
    void initMat();

    /** @brief Calculate the image map.
      the map include map(x&y) in both 2D and 3D, however 3D map is same as 2D actually.
    */
    void calcImageMap(cv::Mat& mapx, cv::Mat& mapy);

protected:
    cv::Mat					double_mapx;	//!< the map in x dimension
    cv::Mat					double_mapy;	//!< the map in y dimension

    uint16_t    image_width;
    uint16_t    image_height;

    cv::Mat		cpu_mapx;			//!< the roi region in 'm_double_mapx'
    cv::Mat		cpu_mapy;			//!< the roi region in 'm_double_mapy'


    cv::Mat         A;                      //!< intrinsic parameters 3x3
    cv::Mat         D;                      //!< distortion parameters 1x5
    cv::Mat         R;                      //!< rectification parameters 3x3
    cv::Rect        ROI;                    //!< region of interest [x,y,width,height]
    cv::Mat         Anew;                   //!< new intrinsic parameters 3x3
    std::vector<std::string> split(const std::string& s, const std::string& seperator);
};

#endif // MAP_CALCULATOR_H_
