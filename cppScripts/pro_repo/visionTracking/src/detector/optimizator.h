#pragma once
#include "../def/type_def.h"
#include "ippe.h"

struct PatternResult
{
    PatternResult()
        : pc_idx(0)
    {
        corners_ = Corners();
    }

    int pc_idx;
    Corners corners_;
};

struct OneImageReturnParas
{
    Corners cr_2d;
    std::vector<Eigen::Vector3d> p_3d;
    Eigen::Matrix3d rot;
    Eigen::Vector3d trans;
    int axial_idx;

    PatternResult detected_pc;
};

class Optimizator
{
public:
    Optimizator(const double trigger_dis = 35, const double thred0 = 2, const PixelType dis_second_line = 3.0, 
        const PixelType axial_side_length = 2.0,const PixelType marker_radius = 4.138, const int nums = 24, const int sepe_num = 4 );

    /* 3d center, cost */
    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, PatternResult, PatternResult, Corners, Corners>
    process(const CornersSorted left_in, const CornersSorted right_in, const cv::Mat left_img, const cv::Mat right_img);


    Corners getReprojectImagePoint(const std::vector<Eigen::Vector3d>& points3d, const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans) const;
    Corners disRectifyCorners(const Corners& corners2d, bool is_left) const;
    Corner rectifyOneCorner(const Corner& cs, const Eigen::Matrix3d& rect_rot, const Eigen::Matrix3d& cam_intrins, Eigen::Vector4d& cam_distor) const;


private:
    Corner rectifySinglePixel(const Corner pixel, const Eigen::Vector4d distortion_coeff) const;

    void adjustOptiThreshold(const CornersSorted& left, const CornersSorted& right);
    std::tuple<CornersSorted, CornersSorted>
        flipCornersWhenNecessary(const CornersSorted& left, const CornersSorted& right);

    std::tuple<CornersSorted, CornersSorted>
        rectifyCorners(const CornersSorted& left, const CornersSorted& right) const;

    //Corner rectifyOneCorner(const Corner& cs, const Eigen::Matrix3d& rect_rot, const Eigen::Matrix3d& cam_intrins, Eigen::Vector4d& cam_distor) const;

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, Corners, std::vector<Eigen::Vector3d>>
        getInitPoseInSingleImage_IPPE(const CornersSorted& corners_sorted) const;

    std::tuple <Corners, std::vector<Eigen::Vector3d>> getRelative2Dto3DCorrespondings(const CornersSorted& corners_sorted) const;

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d>
        calcAnalyticPosebyIPPE(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D) const;

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
        optimizeInitMarkerPoseInSingleImage(const std::vector<Eigen::Matrix3d>& R_init, const std::vector<Eigen::Vector3d>& t_init,
            const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D) const;

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
        optInSingleImage(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D, 
            const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double pose[6]) const;

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
        optInSingleImageNonePlane(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D,
            const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double pose[6]) const;

    /* R, t, cost */
    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
        optInTwoImagesNonePlane(const Corners& left_corners_2D, const std::vector<Eigen::Vector3d>& left_points_3D, const Corners& right_corners_2D, 
            const std::vector<Eigen::Vector3d>& right_points_3D, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double pose[6]) const;

    std::tuple<Corner, Eigen::Vector3d, int, int, Corners> predictAndRefineAxialCorner(const CornersSorted& corners_sorted, const cv::Mat& img,
        const Eigen::Matrix3d& rot_init, const Eigen::Vector3d& trans_init, bool left_flag) const;

    std::vector<PatternCorners> generatePatternCorresGroup(const CornersSorted& corners_sorted, const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans) const;

    Corner disRectifyOneCorner(const Corner& corner_, bool is_left) const;

    std::vector<PatternCorners> disRectifyAndSortPatterns(const std::vector<PatternCorners>& pattern_groups, const cv::Mat& img, bool left_flag) const;

    std::vector<PatternCorners> getROIAndCalcMaxCL(const cv::Mat& img, const std::vector<PatternCorners>& pattern_groups) const;

    std::tuple<double, Corner> getMaxCLValueAndLoc(const cv::Mat& img, const int checked_width) const;

    cv::Mat secondDerivCornerMetricRoi(const cv::Mat& img_roi) const;

    Corner subPixelLocationbyCV(const Corner& point, const cv::Mat& gray_img) const;
    Corner subPixelLocation(const Corner& point, const cv::Mat& cmax) const;

    Eigen::MatrixXf calcPatchX() const;

    std::tuple<int, int, double> calcCorrWithSixPatterns(const PatternCorners& pc) const;

    OneImageReturnParas processOneImage(const CornersSorted& crs_in, const cv::Mat& img_in, bool left_flag) const;

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> optiMarkerPosebyAll(const OneImageReturnParas& left_init, const OneImageReturnParas& right_init) const;

    std::tuple<Eigen::Matrix3d, std::vector<Eigen::Vector3d>> rectifyRotAndPointsbyAxialSign(const Eigen::Matrix3d& rot, const std::vector<Eigen::Vector3d>& p_3d, int bias_idx) const;

    double calcReprojectErrorbyRT(const Eigen::Matrix3d R, const Eigen::Vector3d t, const Corners corners_, const std::vector<Eigen::Vector3d>& points_) const;
private:
    double OPTI_COST_THRESHOLD;
    double OPTI_COST_THRESHOLD0;
    double trigger_distance;

    PixelType r; // radius
    int circle_num;// 36;
    int SEPE_PATTERN_NUM;

    PixelType L; // arch
    PixelType dis_axial_corner;
    PixelType axial_rect_side_length;
    const Eigen::MatrixXf PATCH_X;

    CornersSorted left_detected_crs;
    CornersSorted right_detected_crs;

    std::vector<Eigen::Vector3d> coordinate_points;
};