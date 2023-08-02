#include "optimizator.h"
#include "../config/config.h"
#include "../alg/rotation.h"
#include <Eigen/Dense>
#include <algorithm>
#include <ceres/ceres.h>
//#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "PNPSolver.h"
//#include "lm_opencv.h"
#include "../alg/math_helper.h"
#include <future>

//using namespace cv;
//using namespace std;
 
//const auto r = 9.5 / 2; // radius
//const int circle_num = 28;// 36;
//const int SEPE_FIRST_NUM = 7;
//
//const auto L = 2 * PI * r / circle_num; // arch

const int MIN_OPTI_CORNRE_NUM = 4;
const auto MAX_COST_EXIT = 0.5;

const double MIN_PATTERN_WIDTH = 8.0;
const int HALF_PATCH_SIZE = 2;
const double MIN_EXIT_CORR_THRED = 0.90;
const int JUDGE_USE_THRED = -10;

Optimizator::Optimizator(const double trigger_dis, const double thred0, const PixelType dis_second_line, const PixelType axial_side_length, 
    const PixelType marker_radius, const int nums, const int sepe_num)
    :r(marker_radius),
    circle_num(nums),
    SEPE_PATTERN_NUM(sepe_num),
    trigger_distance(trigger_dis),
    OPTI_COST_THRESHOLD0(thred0),
    OPTI_COST_THRESHOLD(thred0),
    dis_axial_corner(dis_second_line),
    axial_rect_side_length(1 * axial_side_length),
    PATCH_X(calcPatchX())
{
    L = 2 * PI * r / circle_num;

    coordinate_points.push_back(Eigen::Vector3d(0, 0, 0));
    coordinate_points.push_back(Eigen::Vector3d(1.8 * r, 0, 0));
    coordinate_points.push_back(Eigen::Vector3d(0, 1.8 * r, 0));
    coordinate_points.push_back(Eigen::Vector3d(0, 0, 1.8 * r));
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, PatternResult, PatternResult, Corners, Corners> Optimizator::process(const CornersSorted left_in, const CornersSorted right_in,
    const cv::Mat left_img, const cv::Mat right_img)
{
    /*1. Optimize pose from lens to marker in left and/or right image, separately. 
      2. With the optimized pose by left and/or right image, 
         predict and refine the location of axial graphic pattern that indicates the absolute axial sign. 
      3. If left and right corners are both detected, align the corners between left and right image by the absolute axial sign.
      4. Using the aligned corners and the corner belonging to the sole axial graphic pattern, re-optimize the pose.
      5. Output the optimized pose and the location of the detected axial graphic pattern.
    */
    if (left_img.empty() || right_img.empty() || (left_in.empty() && right_in.empty()))
    {
        return { Eigen::Matrix3d(), Eigen::Vector3d(), 200, PatternResult(), PatternResult(), Corners(), Corners() };
    }

    //rectify corners' location using calibrated camera params
    auto [left, right] = rectifyCorners(left_in, right_in);

    /*adjust the opti-threshold*/
    adjustOptiThreshold(left, right);

    OneImageReturnParas left_init_res, right_init_res;
    left_init_res.axial_idx = JUDGE_USE_THRED;
    right_init_res.axial_idx = JUDGE_USE_THRED;

    if (left.size() >= MIN_OPTI_CORNRE_NUM && right.size() >= MIN_OPTI_CORNRE_NUM)
    {
        tie(left_detected_crs, right_detected_crs) = flipCornersWhenNecessary(left, right);

        /*left: optimize and find absolute axial sign*/
#ifdef USE_MULTI_THREAD
        auto fut_left = std::async(std::launch::async, [&]() { return processOneImage(left_detected_crs, left_img, true); });
        auto fut_right = std::async(std::launch::async, [&]() { return processOneImage(right_detected_crs, right_img, false); });
        left_init_res = fut_left.get();
        right_init_res = fut_right.get();
#else
        left_init_res = processOneImage(left, left_img, true);
        right_init_res = processOneImage(right, right_img, false);
#endif
    }
    else if(left.size() >= MIN_OPTI_CORNRE_NUM)
    {
        left_init_res = processOneImage(left, left_img, true);
    }
    else if (right.size() >= MIN_OPTI_CORNRE_NUM)
    {
        right_init_res = processOneImage(right, right_img, false);
    }
    else
    {
        return { Eigen::Matrix3d(), Eigen::Vector3d(), 200, PatternResult(), PatternResult(), Corners(), Corners() };
    }


    //return { Eigen::Matrix3d(), Eigen::Vector3d(), 200, PatternResult(), PatternResult(), Corners(), Corners() };
    auto [rot_res, trans_res, err_res] = optiMarkerPosebyAll(left_init_res, right_init_res);

    //coordinate system
    Corners left_coordinate_crs, right_coordinate_crs;
    if (err_res < 100)
    {
        Corners left_crs_rectified = getReprojectImagePoint(coordinate_points, rot_res, trans_res);
        left_coordinate_crs = disRectifyCorners(left_crs_rectified, true);

        Eigen::Vector3d trans_res_right = trans_res - Eigen::Vector3d(b_dis, 0, 0);
        Corners right_crs_rectified = getReprojectImagePoint(coordinate_points, rot_res, trans_res_right);
        right_coordinate_crs = disRectifyCorners(right_crs_rectified, false);

        return { rot_res, trans_res, err_res, left_init_res.detected_pc, right_init_res.detected_pc, left_coordinate_crs, right_coordinate_crs };
    }

    return { Eigen::Matrix3d(), Eigen::Vector3d(), 200, PatternResult(), PatternResult(), Corners(), Corners() };
    
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, Corners, std::vector<Eigen::Vector3d>> Optimizator::getInitPoseInSingleImage_IPPE(const CornersSorted& corners_sorted) const
{
    /*calc 3D point in model plane*/
    auto [corr_2DCorners, corr_3DPoints] = getRelative2Dto3DCorrespondings(corners_sorted);

    /*IPPE*/
    auto [rotm_1, trans_1, rotm_2, trans_2 ] = calcAnalyticPosebyIPPE(corr_2DCorners, corr_3DPoints);

    /*LM Optimization*/
    std::vector<Eigen::Matrix3d> rot_init;
    std::vector<Eigen::Vector3d> trans_init;
    rot_init.push_back(rotm_1);
    trans_init.push_back(trans_1);
    rot_init.push_back(rotm_2);
    trans_init.push_back(trans_2);

    //std::cout << "reprojerr: " << calcReprojectErrorbyRT(rotm_1, trans_1, corr_2DCorners, corr_3DPoints) << std::endl;
    auto [res_rot, res_trans, res_cost] = optimizeInitMarkerPoseInSingleImage(rot_init, trans_init, corr_2DCorners, corr_3DPoints);
    
    return { res_rot, res_trans, res_cost, corr_2DCorners, corr_3DPoints };
}

std::tuple <Corners, std::vector<Eigen::Vector3d>> Optimizator::getRelative2Dto3DCorrespondings(const CornersSorted& corners_sorted) const
{
    Corners corners_2D;
    std::vector<Eigen::Vector3d> points_3D;

    int sepe_sum = 0;
    for (int i = 0; i < corners_sorted.size(); i++)
    {
        corners_2D.push_back(corners_sorted.at(i).point);

        double angle = (sepe_sum + i) * L / r;
        points_3D.push_back(Eigen::Vector3d(r * cos(angle), r * sin(angle), 0));
        sepe_sum += corners_sorted.at(i).sepeNum;
    }

    return { corners_2D, points_3D };
    //return std::tuple <Corners, std::vector<Eigen::Vector3d>>();
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d>
Optimizator::calcAnalyticPosebyIPPE(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D) const
{
    std::vector<cv::Point3f> object3DPoints;//存储四个点的世界坐标
    std::vector<cv::Point2f> image2DPoints;//存储四个点的图像坐标
    cv::Mat camera_matrix;//内参数矩阵
    cv::Mat distortion_coefficients;//畸变系数
    camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0] = A_cam(0, 0);
    camera_matrix.ptr<double>(0)[2] = A_cam(0, 2);
    camera_matrix.ptr<double>(1)[1] = A_cam(1, 1);
    camera_matrix.ptr<double>(1)[2] = A_cam(1, 2);
    camera_matrix.ptr<double>(2)[2] = 1.0f;
    distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

    //设置特征点的世界坐标,图像坐标
    for (int i = 0; i < points_3D.size(); i++)
    {
        //三维坐标的单位是毫米
        object3DPoints.push_back(cv::Point3f(points_3D.at(i)(0), points_3D.at(i)(1), 0));
        image2DPoints.push_back(corners_2D.at(i));
    }

    IPPE::PoseSolver planePoseSolver;
    cv::Mat rvec1, tvec1; //first pose
    cv::Mat rvec2, tvec2; //second pose
    float err1, err2; //RMS reprojection errors of first and second poses

    planePoseSolver.solveGeneric(object3DPoints, image2DPoints, camera_matrix, distortion_coefficients, rvec1, tvec1, err1, rvec2, tvec2, err2);

    /*******************提取旋转矩阵*********************/
    Eigen::Matrix3d matrixTemp1;
    Eigen::Vector3d vectTemp1;
    cv::Mat RoteM1, TransM1;
    double rm1[9];
    RoteM1 = cv::Mat(3, 3, CV_64FC1, rm1);
    Rodrigues(rvec1, RoteM1);
    TransM1 = tvec1;
    cv2eigen(RoteM1, matrixTemp1);
    cv2eigen(TransM1, vectTemp1);

    //another
    Eigen::Matrix3d matrixTemp2;
    Eigen::Vector3d vectTemp2;
    cv::Mat RoteM2, TransM2;
    double rm2[9];
    RoteM2 = cv::Mat(3, 3, CV_64FC1, rm2);
    Rodrigues(rvec2, RoteM2);
    TransM2 = tvec2;
    cv2eigen(RoteM2, matrixTemp2);
    cv2eigen(TransM2, vectTemp2);

    //calc right reprojErr

    //float err_r = 0;

    //cv::InputArray object3DPoints_rx = (cv::InputArray)object3DPoints;
    //size_t n = object3DPoints_rx.rows() * object3DPoints_rx.cols();
    //float dx, dy;
    //if (n > 2)
    //{
    //    cv::Mat tvec1_r = tvec1; //first pose
    //    cv::Mat projectedPoints;
    //    cv::InputArray image2DPoints_x = (cv::InputArray)image2DPoints;
    //    cv::Mat image2DPoints_rx = image2DPoints_x.getMat();
    //    cv::projectPoints(object3DPoints_rx, rvec1, /*(cv::InputArray)*/tvec1_r, camera_matrix, distortion_coefficients, projectedPoints);

    //    for (size_t i = 0; i < n; i++) {
    //        if (projectedPoints.depth() == CV_32FC1) {
    //            dx = projectedPoints.at<Vec2f>(i)[0] - image2DPoints_rx.at<Vec2f>(i)[0];
    //            dy = projectedPoints.at<Vec2f>(i)[1] - image2DPoints_rx.at<Vec2f>(i)[1];
    //        }
    //        else {
    //            dx = projectedPoints.at<Vec2d>(i)[0] - image2DPoints_rx.at<Vec2d>(i)[0];
    //            dy = projectedPoints.at<Vec2d>(i)[1] - image2DPoints_rx.at<Vec2d>(i)[1];
    //        }

    //        err_r += dx * dx + dy * dy;

    //    }
    //    err_r = sqrt(err_r / (2.0f * n));
    //}
    //else
    //{
    //    err_r = 0;
    //}

    //std::cout << "err1: " << err1 << "\terr2: " << err2 << "\terr_re: " << err_r << std::endl;

    return { matrixTemp1, vectTemp1, matrixTemp2, vectTemp2 };
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> Optimizator::optimizeInitMarkerPoseInSingleImage(const std::vector<Eigen::Matrix3d>& R_init, 
    const std::vector<Eigen::Vector3d>& t_init, const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D) const
{
    Eigen::Vector3d point_mid = points_3D.at(points_3D.size() / 2);
    point_mid(2) = 0;
    point_mid = point_mid / point_mid.norm();

    auto costMin = 400.0;
    Eigen::Matrix3d R_test;
    Eigen::Vector3d t_test;
    Eigen::Matrix3d R_candidates;
    Eigen::Vector3d t_candidates;
    for (int i = 0; i < R_init.size(); ++i)
    {
        R_test = R_init.at(i);
        t_test = t_init.at(i);
        if (t_test.z() < 0)
        {
            t_test = -t_test;
        }

        Eigen::Vector3d point_cam_0 = R_test * point_mid;
        if (point_cam_0.z() >= 0)
        {
            Eigen::Matrix3d rotZ;
            rotZ << cos(PI), -sin(PI), 0,
                sin(PI), cos(PI), 0,
                0, 0, 1;

            R_test = R_test * rotZ;
        }

        // init relative pose
        double init_pose[6] = { 0, 0, 0, 0, 0, 0 };
        auto [R_relative, t_relative, cost] = optInSingleImage(corners_2D, points_3D, R_test, t_test, init_pose);

        R_test = R_test * R_relative;
        t_test = t_test + t_relative;

        // judge
        Eigen::Vector3d point_cam = R_test * point_mid;
        if (point_cam.z() < 0 && t_test.z() > 0)
        {
            if (cost < MAX_COST_EXIT)
            {
                return { R_test, t_test, cost };
            }
            else
            {
                if (cost < costMin)
                {
                    costMin = cost;
                    R_candidates = R_test;
                    t_candidates = t_test;
                }
            }
        }
    }

    if (costMin < 400.0)
    {
        return { R_candidates, t_candidates, costMin };
    }
    else
    {
        return { Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), costMin };
    }
    //return std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>();
}

struct ReprojectionError
{
    ReprojectionError(const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t,
        const Corner& m,
        const Eigen::Vector3d& M)
        : R(R)
        , t(t)
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

        T E[9];
        [&R_now, &t_now](T E[9]) {
            for (int i = 0; i < 9; ++i)
                E[i] = R_now[i];

            E[2] = t_now[0];
            E[5] = t_now[1];
            E[8] = t_now[2];
        }(E);

        T H[9];
        T A[9];
        for (int i = 0; i < 9; ++i)
            A[i] = T(A_cam(i / 3, i % 3));
        MatMulMat(A, E, H);

        T MM[3];
        MM[0] = T(M.x());
        MM[1] = T(M.y());
        MM[2] = T(1/*M.z()*/);
        T MMM[3];
        MatMulVec(H, MM, MMM);

        residuals[0] = T(m.x) - MMM[0] / MMM[2];
        residuals[1] = T(m.y) - MMM[1] / MMM[2];

        /* constrain of outside */
        T POINT_CAM[3];
        T POINT_LOCAL[3];
        POINT_LOCAL[0] = T(M.x());
        POINT_LOCAL[1] = T(M.y());
        POINT_LOCAL[2] = T(0);
        MatMulVec(R_now, POINT_LOCAL, POINT_CAM);
        //residuals[2] = T(0.0);
        if (POINT_CAM[2] >= T(0.0))
        {
            residuals[2] = T(10.0) + T(10.0) * POINT_CAM[2];
        }
        else
        {
            residuals[2] = T(0.0);
        }

        return true;
    }

    const Eigen::Matrix3d R;
    const Eigen::Vector3d t;
    const Corner m;
    const Eigen::Vector3d M;
};

struct ReprojectionErrorNonePlane
{
    ReprojectionErrorNonePlane(const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t,
        const Corner& m,
        const Eigen::Vector3d& M)
        : R(R)
        , t(t)
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


        //residuals[0] = T(m.x) - MMM[0] / MMM[2];
        //residuals[1] = T(m.y) - MMM[1] / MMM[2];

        ///* constrain of outside */
        //T POINT_CAM[3];
        //MatMulVec(R_now, MM, POINT_CAM);
        ////residuals[2] = T(0.0);
        //if (POINT_CAM[2] >= T(0.0))
        //{
        //    residuals[2] = T(10.0) + T(10.0) * POINT_CAM[2];
        //}
        //else
        //{
        //    residuals[2] = T(0.0);
        //}

        return true;
    }

    const Eigen::Matrix3d R;
    const Eigen::Vector3d t;
    const Corner m;
    const Eigen::Vector3d M;
};

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> Optimizator::optInSingleImage(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D,
    const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double pose[6]) const
{
    ceres::Problem problem;
    auto problem_count = 0;
    for (int i = 0; i < corners_2D.size() ; ++i)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ReprojectionError, 3, 6>(
                new ReprojectionError(R, t, corners_2D.at(i), points_3D.at(i))),
            nullptr,
            pose);
        ++problem_count;
    }

    ///* x, y in [-400 mm, 400 mm] */
    //problem.SetParameterLowerBound(pose, 3, -400 - t(0));
    //problem.SetParameterUpperBound(pose, 3, 400 - t(0));
    //problem.SetParameterLowerBound(pose, 4, -400 - t(1));
    //problem.SetParameterUpperBound(pose, 4, 400 - t(1));
    ///* z_depth in [10 mm, 400 mm] */
    //problem.SetParameterLowerBound(pose, 5, 10 - t(2));
    //problem.SetParameterUpperBound(pose, 5, 400 - t(2));

    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG;//LEVENBERG_MARQUARDT;//
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    //options.num_threads = 8;
    //options.function_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.BriefReport() << std::endl;

    auto rotRPY = [](const double phi, const double theta, const double psi) {
        Eigen::Matrix3d rot;
        rot << cos(phi) * cos(theta), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), sin(phi)* sin(psi) + cos(phi) * sin(theta) * cos(psi),
            sin(phi)* cos(theta), cos(phi)* cos(psi) + sin(phi) * sin(theta) * sin(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi),
            -sin(theta), cos(theta)* sin(psi), cos(theta)* cos(psi);

        return rot;
    };

    /*the returned cost larger than the actual average cost because sqrt((x1*x1 + x2*x2) / 2) >= (abs(x1) + abs(x2)) / 2 */
    return { rotRPY(pose[0], pose[1], pose[2]), Eigen::Vector3d(pose[3], pose[4], pose[5]), sqrtf(/*2.0 * */summary.final_cost / problem_count) };
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> Optimizator::optInSingleImageNonePlane(const Corners& corners_2D, 
    const std::vector<Eigen::Vector3d>& points_3D, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double pose[6]) const
{
    ceres::Problem problem;
    auto problem_count = 0;
    for (int i = 0; i < points_3D.size(); ++i)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ReprojectionErrorNonePlane, 2, 6>(
                new ReprojectionErrorNonePlane(R, t, corners_2D.at(i), points_3D.at(i))),
            nullptr,
            pose);
        ++problem_count;
    }

    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG;//LEVENBERG_MARQUARDT;//
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    auto rotRPY = [](const double phi, const double theta, const double psi) {
        Eigen::Matrix3d rot;
        rot << cos(phi) * cos(theta), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), sin(phi)* sin(psi) + cos(phi) * sin(theta) * cos(psi),
            sin(phi)* cos(theta), cos(phi)* cos(psi) + sin(phi) * sin(theta) * sin(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi),
            -sin(theta), cos(theta)* sin(psi), cos(theta)* cos(psi);

        return rot;
    };

    /*the returned cost larger than the actual average cost because sqrt((x1*x1 + x2*x2) / 2) >= (abs(x1) + abs(x2)) / 2 */
    return { rotRPY(pose[0], pose[1], pose[2]), Eigen::Vector3d(pose[3], pose[4], pose[5]), sqrt(/*2.0 * */summary.final_cost / problem_count) };
}

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

Corner Optimizator::rectifySinglePixel(const Corner pixel, const Eigen::Vector4d distortion_coeff) const
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

void Optimizator::adjustOptiThreshold(const CornersSorted& left, const CornersSorted& right)
{
    if (left.size() >= MIN_OPTI_CORNRE_NUM)
    {
        auto dis_temp = sqrt((left.at(0).point.x - left.at(1).point.x) * (left.at(0).point.x - left.at(1).point.x)
            + (left.at(0).point.y - left.at(1).point.y) * (left.at(0).point.y - left.at(1).point.y));
        OPTI_COST_THRESHOLD = OPTI_COST_THRESHOLD0 * dis_temp / trigger_distance;
    }
    else if (right.size() >= MIN_OPTI_CORNRE_NUM)
    {
        auto dis_temp = sqrt((right.at(0).point.x - right.at(1).point.x) * (right.at(0).point.x - right.at(1).point.x)
            + (right.at(0).point.y - right.at(1).point.y) * (right.at(0).point.y - right.at(1).point.y));
        OPTI_COST_THRESHOLD = OPTI_COST_THRESHOLD0 * dis_temp / trigger_distance;
    }
    else
    {
        OPTI_COST_THRESHOLD = OPTI_COST_THRESHOLD0;
    }
    if (OPTI_COST_THRESHOLD < OPTI_COST_THRESHOLD0 / 2)
    {
        OPTI_COST_THRESHOLD = OPTI_COST_THRESHOLD0 / 2;
    }
}

std::tuple<CornersSorted, CornersSorted> Optimizator::flipCornersWhenNecessary(const CornersSorted& left, const CornersSorted& right)
{
    CornersSorted leftSorted = left;
    CornersSorted rightSorted = right;
    /* flip when the sorting directions in two images are different */
    if ((leftSorted.at(0).point.x - leftSorted.at(leftSorted.size() - 1).point.x) * (rightSorted.at(0).point.x - rightSorted.at(rightSorted.size() - 1).point.x)
        + (leftSorted.at(0).point.y - leftSorted.at(leftSorted.size() - 1).point.y) * (rightSorted.at(0).point.y - rightSorted.at(rightSorted.size() - 1).point.y) < 0)
    {
        reverse(rightSorted.begin(), rightSorted.end());
        for (int i = 0; i < rightSorted.size(); i++)
        {
            if (rightSorted.at(i).sepeNum > 0)
            {
                if (i > 0)
                {
                    rightSorted.at(i - 1).sepeNum = rightSorted.at(i).sepeNum;
                    rightSorted.at(i).sepeNum = 0;
                }
                break;
            }
        }
    }

    return { leftSorted, rightSorted };
}

Corner Optimizator::rectifyOneCorner(const Corner& cs, 
    const Eigen::Matrix3d& rect_rot, const Eigen::Matrix3d& cam_intrins, Eigen::Vector4d& cam_distor) const
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

std::tuple<CornersSorted, CornersSorted> Optimizator::rectifyCorners(const CornersSorted& left, const CornersSorted& right) const
{
    CornersSorted corners_left, corners_right;

    /*left*/
    if (left.size() > MIN_OPTI_CORNRE_NUM)
    {
        for (const auto& cs : left)
        {
            CornerSorted temp_corner = cs;
            temp_corner.point = rectifyOneCorner(cs.point, rect_left_camera, left_camera_intrinsic, left_camera_distortion);

            corners_left.push_back(temp_corner);
        }
    }
    

    /*right*/
    if (right.size() > MIN_OPTI_CORNRE_NUM) 
    {
        for (const auto& cs : right)
        {
            CornerSorted temp_corner = cs;
            temp_corner.point = rectifyOneCorner(cs.point, rect_right_camera, right_camera_intrinsic, right_camera_distortion);

            corners_right.push_back(temp_corner);
        }
    }
    

    return { corners_left, corners_right };
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
Optimizator::optInTwoImagesNonePlane(const Corners& left_corners_2D, const std::vector<Eigen::Vector3d>& left_points_3D, const Corners& right_corners_2D,
    const std::vector<Eigen::Vector3d>& right_points_3D, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double pose[6]) const
{
    ceres::Problem problem;
    auto problem_count = 0;
    for (int i = 0; i < left_corners_2D.size(); ++i)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ReprojectionErrorNonePlane, 2, 6>(
                new ReprojectionErrorNonePlane(R, t, left_corners_2D.at(i), left_points_3D.at(i))),
            nullptr,
            pose);
        ++problem_count;
    }
    for (int i = 0; i < right_corners_2D.size(); ++i)
    {
        Eigen::Vector3d bias(-b_dis, 0, 0);
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ReprojectionErrorNonePlane, 2, 6>(
                new ReprojectionErrorNonePlane(R, t + bias, right_corners_2D.at(i), right_points_3D.at(i))),
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

    auto rotRPY = [](const double phi, const double theta, const double psi) {
        Eigen::Matrix3d rot;
        rot << cos(phi) * cos(theta), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), sin(phi)* sin(psi) + cos(phi) * sin(theta) * cos(psi),
            sin(phi)* cos(theta), cos(phi)* cos(psi) + sin(phi) * sin(theta) * sin(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi),
            -sin(theta), cos(theta)* sin(psi), cos(theta)* cos(psi);

        return rot;
    };

    /*the returned cost larger than the actual average cost because sqrt((x1*x1 + x2*x2) / 2) >= (abs(x1) + abs(x2)) / 2 */
    return { rotRPY(pose[0], pose[1], pose[2]), Eigen::Vector3d(pose[3], pose[4], pose[5]), sqrt(/*2.0 * */summary.final_cost / problem_count) };
}

std::tuple<Corner, Eigen::Vector3d, int, int, Corners> Optimizator::predictAndRefineAxialCorner(const CornersSorted& corners_sorted, 
    const cv::Mat& img, const Eigen::Matrix3d& rot_init, const Eigen::Vector3d& trans_init, bool left_flag) const
{
    Corner best_axial_corner(-1, -1);
    Eigen::Vector3d best_axial_point = Eigen::Vector3d();

    auto pattern_groups = generatePatternCorresGroup(corners_sorted, rot_init, trans_init);
    auto pattern_groups_sorted = disRectifyAndSortPatterns(pattern_groups, img, left_flag);

    double max_corr = 0.650;
    int best_axial_idx = -1;
    int best_pattern_idx = -1;
    Corners best_pc_corners;

    for (int i = 0; i < pattern_groups_sorted.size(); i++) 
    {

        if (pattern_groups_sorted.at(i).max_cl_val > -100)
        {
            auto [axial_idx_temp, pattern_idx_temp, corr_temp] = calcCorrWithSixPatterns(pattern_groups_sorted.at(i));

            if (corr_temp > max_corr)
            {
                max_corr = corr_temp;
                best_axial_idx = axial_idx_temp;
                best_pattern_idx = pattern_idx_temp;
                best_axial_corner = pattern_groups_sorted.at(i).center_;
                best_axial_point = pattern_groups_sorted.at(i).points_.at(0);

                //best_pc_corners.clear();
                best_pc_corners = pattern_groups_sorted.at(i).corners_;
            }

            if (max_corr > MIN_EXIT_CORR_THRED)
            {

                //std::cout << "max_corr: " << max_corr << "\ti: " << i << "\tsize: " << pattern_groups_sorted.size() << std::endl;
                break;
            }
        }
    }

    /*rectify center corner*/
    if (max_corr > 0.6500001)
    {
        if (left_flag)
        {
            best_axial_corner = rectifyOneCorner(best_axial_corner, rect_left_camera, left_camera_intrinsic, left_camera_distortion);
        }
        else
        {
            best_axial_corner = rectifyOneCorner(best_axial_corner, rect_right_camera, right_camera_intrinsic, right_camera_distortion);
        }
    }

    return { best_axial_corner, best_axial_point, best_axial_idx, best_pattern_idx, best_pc_corners };
}

std::vector<PatternCorners> Optimizator::generatePatternCorresGroup(const CornersSorted& corners_sorted, const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans) const
{
    auto calc_pattern_corr_by_one_point = [&](const double angle_x) {
        PatternCorners pattern_pos, pattern_neg;
        Eigen::Vector3d point3d_temp_pos_c(r * cos(angle_x), r * sin(angle_x), dis_axial_corner);
        pattern_pos.points_.push_back(point3d_temp_pos_c);

        Eigen::Vector3d point3d_temp_neg_c(r * cos(angle_x), r * sin(angle_x), -dis_axial_corner);
        pattern_neg.points_.push_back(point3d_temp_neg_c);
        
        double d_ang_unit = axial_rect_side_length / r;
        double pos_dz_arr[8] = { 0.5 * axial_rect_side_length, 0.5 * axial_rect_side_length, 0.5 * axial_rect_side_length, 0.0 * axial_rect_side_length,
                            -0.5 * axial_rect_side_length, -0.5 * axial_rect_side_length, -0.5 * axial_rect_side_length, 0.0 * axial_rect_side_length };
        double pos_dang_arr[8] = { 0.5 * d_ang_unit, 0.0 * d_ang_unit, -0.5 * d_ang_unit, -0.5 * d_ang_unit,
                                 -0.5 * d_ang_unit, 0.0 * d_ang_unit, 0.5 * d_ang_unit, 0.5 * d_ang_unit };

        double neg_dz_arr[8] = { -0.5 * axial_rect_side_length, -0.5 * axial_rect_side_length, -0.5 * axial_rect_side_length, 0.0 * axial_rect_side_length,
                            0.5 * axial_rect_side_length, 0.5 * axial_rect_side_length, 0.5 * axial_rect_side_length, 0.0 * axial_rect_side_length };
        double neg_dang_arr[8] = { -0.5 * d_ang_unit, 0.0 * d_ang_unit, 0.5 * d_ang_unit, 0.5 * d_ang_unit,
                                 0.5 * d_ang_unit, 0.0 * d_ang_unit, -0.5 * d_ang_unit, -0.5 * d_ang_unit };
        for (int i = 0; i < 8; i++)
        {
            Eigen::Vector3d point3d_temp_pos(r * cos(angle_x + pos_dang_arr[i]), r * sin(angle_x + pos_dang_arr[i]), dis_axial_corner + pos_dz_arr[i]);
            pattern_pos.points_.push_back(point3d_temp_pos);

            Eigen::Vector3d point3d_temp_neg(r * cos(angle_x + neg_dang_arr[i]), r * sin(angle_x + neg_dang_arr[i]), -dis_axial_corner + neg_dz_arr[i]);
            pattern_neg.points_.push_back(point3d_temp_neg);
        }

        pattern_pos.corners_ = getReprojectImagePoint(pattern_pos.points_, rot, trans);
        pattern_neg.corners_ = getReprojectImagePoint(pattern_neg.points_, rot, trans);

        return std::tuple{ pattern_pos, pattern_neg };
    };

    std::vector<PatternCorners> corres_group;

    std::vector<Eigen::Vector3d> center_points_3d;
    Corners center_corner_2d;
    int sepe_sum = 0;
    for (int i = 0; i < corners_sorted.size(); i++)
    {
        double angle = (sepe_sum + i) * L / r;
        auto [temp_pattern_pos, temp_pattern_neg] = calc_pattern_corr_by_one_point(angle);
        temp_pattern_pos.axial_idx = sepe_sum + i;
        temp_pattern_neg.axial_idx = sepe_sum + i;

        corres_group.push_back(temp_pattern_pos);
        corres_group.push_back(temp_pattern_neg);
        
        if (corners_sorted.at(i).sepeNum > 0)
        {
            for (int j = 0; j < corners_sorted.at(i).sepeNum; j++)
            {
                sepe_sum += 1;
                double angle_s = (sepe_sum + i) * L / r;
                auto [temp_pattern_pos_s, temp_pattern_neg_s] = calc_pattern_corr_by_one_point(angle_s);
                temp_pattern_pos_s.axial_idx = sepe_sum + i;
                temp_pattern_neg_s.axial_idx = sepe_sum + i;

                corres_group.push_back(temp_pattern_pos_s);
                corres_group.push_back(temp_pattern_neg_s);
            }
        }
    }

    return corres_group;
}

Corners Optimizator::getReprojectImagePoint(const std::vector<Eigen::Vector3d>& points3d, const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans) const
{
    cv::Mat camera_matrix;//内参数矩阵
    cv::Mat distortion_coefficients;//畸变系数
    camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0] = A_cam(0, 0);
    camera_matrix.ptr<double>(0)[2] = A_cam(0, 2);
    camera_matrix.ptr<double>(1)[1] = A_cam(1, 1);
    camera_matrix.ptr<double>(1)[2] = A_cam(1, 2);
    camera_matrix.ptr<double>(2)[2] = 1.0f;
    distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

    std::vector<cv::Point3f> object3DPoints;
    for (int i = 0; i < points3d.size(); i++)
    {
        object3DPoints.push_back(cv::Point3f(points3d.at(i)(0), points3d.at(i)(1), points3d.at(i)(2)));
    }

    cv::Mat projectedPoints;
    cv::Mat RoteM, TransM;
    cv::Mat rvec, tvec;
    double rm[9];
    RoteM = cv::Mat(3, 3, CV_64FC1, rm);
    eigen2cv(rot, RoteM);
    Rodrigues(RoteM, rvec);
    //rvec = rot2vec_sub(RoteM);
    eigen2cv(trans, TransM);
    tvec = TransM;

    cv::InputArray object3DPoints_x = (cv::InputArray)object3DPoints;
    cv::projectPoints(object3DPoints, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);

    Corners corners_res = Corners();
    size_t n = object3DPoints_x.rows() * object3DPoints_x.cols();
    for (size_t i = 0; i < n; i++)
    {
        corners_res.push_back(Corner(projectedPoints.at<cv::Vec2f>(i)[0], projectedPoints.at<cv::Vec2f>(i)[1]));
    }

    return corners_res;
}

Corners Optimizator::disRectifyCorners(const Corners& corners2d, bool is_left) const
{
    Corners corners_res;

    for (const auto& cs : corners2d)
    {
        Corner temp_corner = disRectifyOneCorner(cs, is_left);

        corners_res.push_back(temp_corner);
    }

    return corners_res;
}

Corner Optimizator::disRectifyOneCorner(const Corner& corner_, bool is_left) const
{
    Corner corner_res;

    if (is_left)
    {
        PixelType l_x_ud_uv, l_y_ud_uv;
        l_x_ud_uv = (corner_.x - A_cam(0, 2)) / A_cam(0, 0);
        l_y_ud_uv = (corner_.y - A_cam(1, 2)) / A_cam(0, 0);
        Eigen::Vector3d pos(l_x_ud_uv, l_y_ud_uv, 1);
        auto p_ud = rect_left_camera.inverse() * pos;
        Corner pixel_ud = Corner(p_ud(0) / p_ud(2), p_ud(1) / p_ud(2));

        double r_xy = (pixel_ud.x * pixel_ud.x + pixel_ud.y * pixel_ud.y);
        auto k1 = left_camera_distortion(0);
        auto k2 = left_camera_distortion(1);
        auto p1 = left_camera_distortion(2);
        auto p2 = left_camera_distortion(3);
        auto x_d = pixel_ud.x * (1 + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p1)*pixel_ud.x * pixel_ud.y + (p2) * (r_xy + (2) * pixel_ud.x * pixel_ud.x);
        auto y_d = pixel_ud.y * ((1) + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p2)*pixel_ud.x * pixel_ud.y + (p1) * (r_xy + (2) * pixel_ud.y * pixel_ud.y);

        corner_res.x = left_camera_intrinsic(0, 0) * x_d + left_camera_intrinsic(0, 2);
        corner_res.y = left_camera_intrinsic(1, 1) * y_d + left_camera_intrinsic(1, 2);
    }
    else
    {
        PixelType l_x_ud_uv, l_y_ud_uv;
        l_x_ud_uv = (corner_.x - A_cam(0, 2)) / A_cam(0, 0);
        l_y_ud_uv = (corner_.y - A_cam(1, 2)) / A_cam(0, 0);
        Eigen::Vector3d pos(l_x_ud_uv, l_y_ud_uv, 1);
        auto p_ud = rect_right_camera.inverse() * pos;
        Corner pixel_ud = Corner(p_ud(0) / p_ud(2), p_ud(1) / p_ud(2));

        double r_xy = (pixel_ud.x * pixel_ud.x + pixel_ud.y * pixel_ud.y);
        auto k1 = right_camera_distortion(0);
        auto k2 = right_camera_distortion(1);
        auto p1 = right_camera_distortion(2);
        auto p2 = right_camera_distortion(3);
        auto x_d = pixel_ud.x * (1 + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p1)*pixel_ud.x * pixel_ud.y + (p2) * (r_xy + (2) * pixel_ud.x * pixel_ud.x);
        auto y_d = pixel_ud.y * ((1) + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p2)*pixel_ud.x * pixel_ud.y + (p1) * (r_xy + (2) * pixel_ud.y * pixel_ud.y);
        
        corner_res.x = right_camera_intrinsic(0, 0) * x_d + right_camera_intrinsic(0, 2);
        corner_res.y = right_camera_intrinsic(1, 1) * y_d + right_camera_intrinsic(1, 2);
    }

    return corner_res;
}

std::vector<PatternCorners> Optimizator::disRectifyAndSortPatterns(const std::vector<PatternCorners>& pattern_groups, const cv::Mat& img, bool left_flag) const
{
    auto comp_cl = [](const PatternCorners& pc1, const PatternCorners& pc2) { return pc1.max_cl_val > pc2.max_cl_val; };

    std::vector<PatternCorners> pattern_groups_res = pattern_groups;
    /*for each group*/
    for (int i = 0; i < pattern_groups.size(); i++)
    {
        pattern_groups_res.at(i).corners_ = disRectifyCorners(pattern_groups.at(i).corners_, left_flag);
    }
    pattern_groups_res = getROIAndCalcMaxCL(img, pattern_groups_res);

    //sort 
    //std::sort(pattern_groups_res.begin(), pattern_groups_res.end(), comp_cl);

    return pattern_groups_res;
}

std::vector<PatternCorners> Optimizator::getROIAndCalcMaxCL(const cv::Mat& img, const std::vector<PatternCorners>& pattern_groups) const
{
    auto cornersWidth = [](Corner corner1, Corner corner2) {
        return sqrtf(powf(corner1.x - corner2.x, 2.0) + powf(corner1.y - corner2.y, 2.0));
    };

    std::vector<PatternCorners> pattern_groups_res = pattern_groups;

    int img_width = img.cols;
    int img_height = img.rows;

    for (int i = 0; i < pattern_groups.size(); i++)
    {
        double dis_13 = cornersWidth(pattern_groups.at(i).corners_.at(1), pattern_groups.at(i).corners_.at(3));
        double dis_35 = cornersWidth(pattern_groups.at(i).corners_.at(3), pattern_groups.at(i).corners_.at(5));
        double dis_57 = cornersWidth(pattern_groups.at(i).corners_.at(5), pattern_groups.at(i).corners_.at(7));
        double dis_71 = cornersWidth(pattern_groups.at(i).corners_.at(7), pattern_groups.at(i).corners_.at(1));
        double roi_half_width = 1.5 * (0.5 * 0.25 * (dis_13 + dis_35 + dis_57 + dis_71));
        double min_dis = std::min(std::min(dis_13, dis_35), std::min(dis_57, dis_71));
        
        int x_start = (int)(pattern_groups.at(i).corners_.at(0).x - roi_half_width);
        int x_end = (int)(pattern_groups.at(i).corners_.at(0).x + roi_half_width + 1);
        int y_start = (int)(pattern_groups.at(i).corners_.at(0).y - roi_half_width);
        int y_end = (int)(pattern_groups.at(i).corners_.at(0).y + roi_half_width + 1);

        if ( (x_start > 0 && x_start < img_width) && (x_end > 0 && x_end < img_width) && 
            (y_start > 0 && y_start < img_height) && (y_end > 0 && y_end < img_height) &&
            (min_dis > MIN_PATTERN_WIDTH) )
        {
            cv::Mat roi_rgb = img(cv::Range(y_start, y_end), cv::Range(x_start, x_end)).clone();
            cv::Mat roi_gray;
            cv::cvtColor(roi_rgb, roi_gray, cv::COLOR_BGR2GRAY);
            roi_gray.convertTo(roi_gray, CV_32FC1);
            roi_gray = roi_gray / 255.0;
            roi_gray.copyTo(pattern_groups_res.at(i).pattern_roi);

            auto [max_cl, max_cl_loc_bias] = getMaxCLValueAndLoc(pattern_groups_res.at(i).pattern_roi, ((int)(min_dis/4)));
            pattern_groups_res.at(i).max_cl_val = max_cl;
            pattern_groups_res.at(i).center_.x = pattern_groups_res.at(i).corners_.at(0).x;// +max_cl_loc_bias.x;
            pattern_groups_res.at(i).center_.y = pattern_groups_res.at(i).corners_.at(0).y;// +max_cl_loc_bias.y;
        }
        else
        {
            pattern_groups_res.at(i).max_cl_val = -100;
        }
    }

    return pattern_groups_res;
}

std::tuple<double, Corner> Optimizator::getMaxCLValueAndLoc(const cv::Mat& img, const int checked_width) const
{
    auto cmax_sigma_2 = secondDerivCornerMetricRoi(img);

    int x_start = (int)((cmax_sigma_2.rows - 1 - checked_width) / 2.0);
    int x_end = (int)((cmax_sigma_2.rows - 1 + checked_width) / 2.0) + 1;
    int y_start = (int)((cmax_sigma_2.cols - 1 - checked_width) / 2.0);
    int y_end = (int)((cmax_sigma_2.cols - 1 + checked_width) / 2.0) + 1;
    //cv::Range range_x = cv::Range(std::max(x_start, 0), std::min(x_end, cmax_sigma_2.cols));
    //cv::Range range_y = cv::Range(std::max(y_start, 0), std::min(y_end, cmax_sigma_2.rows));

    cv::Mat cmax_sigma_2_sub = cmax_sigma_2(cv::Range(x_start, x_end), cv::Range(y_start, y_end));
    
    cv::Point max_pos;
    double maxVal = 0;
    cv::minMaxLoc(cmax_sigma_2_sub, nullptr, &maxVal, nullptr, &max_pos);
    auto corner_loc = subPixelLocation(Corner(max_pos.x, max_pos.y), cmax_sigma_2_sub);

    Corner loc_bias = Corner(corner_loc.x - (cmax_sigma_2_sub.cols - 1) / 2.0, corner_loc.y - (cmax_sigma_2_sub.rows - 1) / 2.0);

    return {maxVal, loc_bias};
}

cv::Mat Optimizator::secondDerivCornerMetricRoi(const cv::Mat& img_roi) const
{
    cv::Mat gaussian_image;// = gray_image;
    int SIGMA = 2;
    cv::GaussianBlur(img_roi, gaussian_image, cv::Size(7 * SIGMA + 1, 7 * SIGMA + 1), SIGMA);

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

    return cmax_sigma_2;
}


Corner Optimizator::subPixelLocationbyCV(const Corner& point, const cv::Mat& gray_img) const
{
    if (gray_img.cols < 5*HALF_PATCH_SIZE || gray_img.rows < 5 * HALF_PATCH_SIZE)
    {
        return Corner(point.x, point.y);
    }

    Corners points; points.push_back(point);
    cv::Size subPixWinSize(HALF_PATCH_SIZE, HALF_PATCH_SIZE);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.1);

    cv::cornerSubPix(gray_img, points, subPixWinSize, cv::Size(-1, -1), termcrit);

    if (points.size() > 0)
    {
        return points.at(0);
    }
    return point;
}

Corner Optimizator::subPixelLocation(const Corner& point, const cv::Mat& cmax) const
{
    if (point.x < HALF_PATCH_SIZE ||
        point.y < HALF_PATCH_SIZE ||
        point.x > cmax.cols - HALF_PATCH_SIZE - 1 ||
        point.y > cmax.rows - HALF_PATCH_SIZE - 1)
    {
        return Corner(point.x, point.y);
    }

    auto width = cmax.cols, height = cmax.rows;
    auto patch = cmax(
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
    if (fabs(delta) < 1e-7)
        return Corner(point.x, point.y);

    auto x = -(2 * B * C - D * E) / delta;
    auto y = -(2 * A * D - C * E) / delta;
    if (fabs(x) > HALF_PATCH_SIZE || fabs(y) > HALF_PATCH_SIZE)
        return Corner(point.x, point.y);

    return Corner(point.x + x, point.y + y);
}

Eigen::MatrixXf Optimizator::calcPatchX() const
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

std::tuple<int, int, double> Optimizator::calcCorrWithSixPatterns(const PatternCorners& pc) const
{
    auto cornersWidth = [](Corner corner1, Corner corner2) {
        return sqrtf(powf(corner1.x - corner2.x, 2.0) + powf(corner1.y - corner2.y, 2.0));
    };
    auto in_quad = [](Corner cr_x, Corner cr1, Corner cr2, Corner cr3, Corner cr4) {
        bool isIn = true;

        Eigen::Vector3d vec1(cr1.x - cr_x.x, cr1.y - cr_x.y, 0);
        Eigen::Vector3d vec2(cr2.x - cr_x.x, cr2.y - cr_x.y, 0);
        Eigen::Vector3d vec3(cr3.x - cr_x.x, cr3.y - cr_x.y, 0);
        Eigen::Vector3d vec4(cr4.x - cr_x.x, cr4.y - cr_x.y, 0);

        Eigen::Vector3d prod12 = vec1.cross(vec2);
        Eigen::Vector3d prod23 = vec2.cross(vec3);
        Eigen::Vector3d prod34 = vec3.cross(vec4);
        Eigen::Vector3d prod41 = vec4.cross(vec1);

        isIn = isIn && (prod12.dot(prod23) > 0);
        isIn = isIn && (prod23.dot(prod34) > 0);
        isIn = isIn && (prod34.dot(prod41) > 0);

        return isIn;
    };

    struct PatternComb
    {
        int idx;
        double corr;
    };

    int axial_idx_res = -1, pattern_idx_res = -1;
    double corr_res = -1;
    std::vector<PatternComb> sixPcCombs;

    /*preparation*/
    auto gray_img = pc.pattern_roi;
    int img_width = gray_img.cols, img_height = gray_img.rows;
    Corner img_center((img_width - 1) / 2.0, (img_height - 1) / 2.0);
    Corners side_crs;
    for (int i = 0; i < pc.corners_.size() - 1; i++)
    {
        Corner cr_temp;
        cr_temp.x = pc.corners_.at(i + 1).x - pc.corners_.at(0).x + img_center.x;
        cr_temp.y = pc.corners_.at(i + 1).y - pc.corners_.at(0).y + img_center.y;

        side_crs.push_back(cr_temp);
    }

    /*set differ area according to the image's width*/
    Corners use_pc_outer_crs;
    double dis_13 = cornersWidth(side_crs.at(0), side_crs.at(2));
    double dis_35 = cornersWidth(side_crs.at(2), side_crs.at(4));
    double dis_57 = cornersWidth(side_crs.at(4), side_crs.at(6));
    double dis_71 = cornersWidth(side_crs.at(6), side_crs.at(0));
    double pc_width_avg = 0.25 * (dis_13 + dis_35 + dis_57 + dis_71);

    if (pc_width_avg < 40)
    {
        //all
        use_pc_outer_crs.push_back(side_crs.at(0));
        use_pc_outer_crs.push_back(side_crs.at(2));
        use_pc_outer_crs.push_back(side_crs.at(4));
        use_pc_outer_crs.push_back(side_crs.at(6));
    }
    else
    {
        //half
        use_pc_outer_crs.push_back(side_crs.at(1));
        use_pc_outer_crs.push_back(side_crs.at(4));
        use_pc_outer_crs.push_back(side_crs.at(5));
        use_pc_outer_crs.push_back(side_crs.at(6));
    }
    //std::cout << "img_width: " << img_width << std::endl;

    /*calc six corrs together*/
    double raw_sum = 0, pc_A_sum = 0, pc_B_sum = 0, pc_C_sum = 0, pc_D_sum = 0, pc_E_sum = 0, pc_F_sum = 0;
    int count = 0;
    for (int x = 0; x < img_width; x++)
    {
        for (int y = 0; y < img_height; y++)
        {
            Corner cr_temp(x, y);
            if (in_quad(cr_temp, use_pc_outer_crs.at(0), use_pc_outer_crs.at(1), use_pc_outer_crs.at(2), use_pc_outer_crs.at(3)))
            {
                raw_sum += gray_img.ptr<PixelType>(y)[x];

                //A, F
                bool temp_1_1 = (in_quad(cr_temp, img_center, side_crs.at(2), side_crs.at(3), side_crs.at(4)));
                bool temp_1_2 = (in_quad(cr_temp, img_center, side_crs.at(6), side_crs.at(7), side_crs.at(0)));
                if (temp_1_1 || temp_1_2)
                {
                    pc_A_sum += 1.0;
                    pc_F_sum += 0.0;
                }
                else
                {
                    pc_A_sum += 0.0;
                    pc_F_sum += 1.0;
                }

                //B, E
                if (in_quad(cr_temp, img_center, side_crs.at(1), side_crs.at(2), side_crs.at(3)) ||
                    in_quad(cr_temp, img_center, side_crs.at(5), side_crs.at(6), side_crs.at(7)))
                {
                    pc_B_sum += 0.0;
                    pc_E_sum += 1.0;
                }
                else
                {
                    pc_B_sum += 1.0;
                    pc_E_sum += 0.0;
                }

                //C
                Corner cr_e_C1(0.5 * (side_crs.at(1).x + side_crs.at(2).x), 0.5 * (side_crs.at(1).y + side_crs.at(2).y));
                Corner cr_e_C2(0.5 * (side_crs.at(4).x + side_crs.at(5).x), 0.5 * (side_crs.at(4).y + side_crs.at(5).y));
                Corner cr_e_C3(0.5 * (side_crs.at(5).x + side_crs.at(6).x), 0.5 * (side_crs.at(5).y + side_crs.at(6).y));
                Corner cr_e_C4(0.5 * (side_crs.at(0).x + side_crs.at(1).x), 0.5 * (side_crs.at(0).y + side_crs.at(1).y));
                if (in_quad(cr_temp, img_center, cr_e_C1, side_crs.at(3), cr_e_C2) ||
                    in_quad(cr_temp, img_center, cr_e_C3, side_crs.at(7), cr_e_C4))
                {
                    pc_C_sum += 0.0;
                }
                else
                {
                    pc_C_sum += 1.0;
                }

                //D
                Corner cr_e_D1(0.5 * (side_crs.at(7).x + side_crs.at(0).x), 0.5 * (side_crs.at(7).y + side_crs.at(0).y));
                Corner cr_e_D2(0.5 * (side_crs.at(2).x + side_crs.at(3).x), 0.5 * (side_crs.at(2).y + side_crs.at(3).y));
                Corner cr_e_D3(0.5 * (side_crs.at(3).x + side_crs.at(4).x), 0.5 * (side_crs.at(3).y + side_crs.at(4).y));
                Corner cr_e_D4(0.5 * (side_crs.at(6).x + side_crs.at(7).x), 0.5 * (side_crs.at(6).y + side_crs.at(7).y));
                if (in_quad(cr_temp, img_center, cr_e_D1, side_crs.at(1), cr_e_D2) ||
                    in_quad(cr_temp, img_center, cr_e_D3, side_crs.at(5), cr_e_D4))
                {
                    pc_D_sum += 0.0;
                }
                else
                {
                    pc_D_sum += 1.0;
                }

                count++;
            }
        }
    }
    auto raw_avg = raw_sum / count;
    auto pc_A_avg = pc_A_sum / count, pc_B_avg = pc_B_sum / count, pc_C_avg = pc_C_sum / count, 
        pc_D_avg = pc_D_sum / count, pc_E_avg = pc_E_sum / count, pc_F_avg = pc_F_sum / count;
    
    double raw_var = 0, pc_A_var = 0, pc_B_var = 0, pc_C_var = 0, pc_D_var = 0, pc_E_var = 0, pc_F_var = 0;
    double pc_A_cov = 0, pc_B_cov = 0, pc_C_cov = 0, pc_D_cov = 0, pc_E_cov = 0, pc_F_cov = 0;
    for (int x = 0; x < img_width; x++)
    {
        for (int y = 0; y < img_height; y++)
        {
            Corner cr_temp(x, y);
            if (in_quad(cr_temp, side_crs.at(0), side_crs.at(2), side_crs.at(4), side_crs.at(6)))
            {
                auto diff_raw = gray_img.ptr<PixelType>(y)[x] - raw_avg;
                raw_var += powf(diff_raw, 2.0);

                //A, F
                if (in_quad(cr_temp, img_center, side_crs.at(2), side_crs.at(3), side_crs.at(4)) ||
                    in_quad(cr_temp, img_center, side_crs.at(6), side_crs.at(7), side_crs.at(0)))
                {
                    auto pc_A_diff = 1.0 - pc_A_avg;
                    auto pc_F_diff = 0.0 - pc_F_avg;
                    pc_A_var += powf(pc_A_diff, 2.0);
                    pc_F_var += powf(pc_F_diff, 2.0);
                    pc_A_cov += diff_raw * pc_A_diff;
                    pc_F_cov += diff_raw * pc_F_diff;

                }
                else
                {
                    auto pc_A_diff = 0.0 - pc_A_avg;
                    auto pc_F_diff = 1.0 - pc_F_avg;
                    pc_A_var += powf(pc_A_diff, 2.0);
                    pc_F_var += powf(pc_F_diff, 2.0);
                    pc_A_cov += diff_raw * pc_A_diff;
                    pc_F_cov += diff_raw * pc_F_diff;
                }

                //B, E
                if (in_quad(cr_temp, img_center, side_crs.at(1), side_crs.at(2), side_crs.at(3)) ||
                    in_quad(cr_temp, img_center, side_crs.at(5), side_crs.at(6), side_crs.at(7)))
                {
                    auto pc_B_diff = 0.0 - pc_B_avg;
                    auto pc_E_diff = 1.0 - pc_E_avg;
                    pc_B_var += powf(pc_B_diff, 2.0);
                    pc_E_var += powf(pc_E_diff, 2.0);
                    pc_B_cov += diff_raw * pc_B_diff;
                    pc_E_cov += diff_raw * pc_E_diff;
                }
                else
                {
                    auto pc_B_diff = 1.0 - pc_B_avg;
                    auto pc_E_diff = 0.0 - pc_E_avg;
                    pc_B_var += powf(pc_B_diff, 2.0);
                    pc_E_var += powf(pc_E_diff, 2.0);
                    pc_B_cov += diff_raw * pc_B_diff;
                    pc_E_cov += diff_raw * pc_E_diff;
                }

                //C
                Corner cr_e_C1(0.5 * (side_crs.at(1).x + side_crs.at(2).x), 0.5 * (side_crs.at(1).y + side_crs.at(2).y));
                Corner cr_e_C2(0.5 * (side_crs.at(4).x + side_crs.at(5).x), 0.5 * (side_crs.at(4).y + side_crs.at(5).y));
                Corner cr_e_C3(0.5 * (side_crs.at(5).x + side_crs.at(6).x), 0.5 * (side_crs.at(5).y + side_crs.at(6).y));
                Corner cr_e_C4(0.5 * (side_crs.at(0).x + side_crs.at(1).x), 0.5 * (side_crs.at(0).y + side_crs.at(1).y));
                if (in_quad(cr_temp, img_center, cr_e_C1, side_crs.at(3), cr_e_C2) ||
                    in_quad(cr_temp, img_center, cr_e_C3, side_crs.at(7), cr_e_C4))
                {
                    auto pc_C_diff = 0.0 - pc_C_avg;
                    pc_C_var += powf(pc_C_diff, 2.0);
                    pc_C_cov += diff_raw * pc_C_diff;
                }
                else
                {
                    auto pc_C_diff = 1.0 - pc_C_avg;
                    pc_C_var += powf(pc_C_diff, 2.0);
                    pc_C_cov += diff_raw * pc_C_diff;
                }

                //D
                Corner cr_e_D1(0.5 * (side_crs.at(7).x + side_crs.at(0).x), 0.5 * (side_crs.at(7).y + side_crs.at(0).y));
                Corner cr_e_D2(0.5 * (side_crs.at(2).x + side_crs.at(3).x), 0.5 * (side_crs.at(2).y + side_crs.at(3).y));
                Corner cr_e_D3(0.5 * (side_crs.at(3).x + side_crs.at(4).x), 0.5 * (side_crs.at(3).y + side_crs.at(4).y));
                Corner cr_e_D4(0.5 * (side_crs.at(6).x + side_crs.at(7).x), 0.5 * (side_crs.at(6).y + side_crs.at(7).y));
                if (in_quad(cr_temp, img_center, cr_e_D1, side_crs.at(1), cr_e_D2) ||
                    in_quad(cr_temp, img_center, cr_e_D3, side_crs.at(5), cr_e_D4))
                {
                    auto pc_D_diff = 0.0 - pc_D_avg;
                    pc_D_var += powf(pc_D_diff, 2.0);
                    pc_D_cov += diff_raw * pc_D_diff;
                }
                else
                {
                    auto pc_D_diff = 1.0 - pc_D_avg;
                    pc_D_var += powf(pc_D_diff, 2.0);
                    pc_D_cov += diff_raw * pc_D_diff;
                }
            }
        }
    }

    auto raw_var_sqrt = sqrtf(raw_var);
    PatternComb pc_A_comb, pc_B_comb, pc_C_comb, pc_D_comb, pc_E_comb, pc_F_comb;
    pc_A_comb.idx = 1; pc_A_comb.corr = pc_A_cov / (raw_var_sqrt * sqrtf(pc_A_var));
    pc_B_comb.idx = 2; pc_B_comb.corr = pc_B_cov / (raw_var_sqrt * sqrtf(pc_B_var));
    pc_C_comb.idx = 3; pc_C_comb.corr = pc_C_cov / (raw_var_sqrt * sqrtf(pc_C_var));
    pc_D_comb.idx = 4; pc_D_comb.corr = pc_D_cov / (raw_var_sqrt * sqrtf(pc_D_var));
    pc_E_comb.idx = 5; pc_E_comb.corr = pc_E_cov / (raw_var_sqrt * sqrtf(pc_E_var));
    pc_F_comb.idx = 6; pc_F_comb.corr = pc_F_cov / (raw_var_sqrt * sqrtf(pc_F_var));
    sixPcCombs.push_back(pc_A_comb);
    sixPcCombs.push_back(pc_B_comb);
    sixPcCombs.push_back(pc_C_comb);
    sixPcCombs.push_back(pc_D_comb);
    sixPcCombs.push_back(pc_E_comb);
    sixPcCombs.push_back(pc_F_comb);
    
    auto comp_cl = [](const PatternComb& pc1, const PatternComb& pc2) { return pc1.corr > pc2.corr; };
    std::sort(sixPcCombs.begin(), sixPcCombs.end(), comp_cl);

    axial_idx_res = pc.axial_idx;
    pattern_idx_res = sixPcCombs.at(0).idx;
    corr_res = sixPcCombs.at(0).corr;

    return { axial_idx_res, pattern_idx_res, corr_res };
}

OneImageReturnParas Optimizator::processOneImage(const CornersSorted& crs_in, const cv::Mat& img_in, bool left_flag) const
{
    OneImageReturnParas res;
    res.axial_idx = JUDGE_USE_THRED;

    double init_reproj_err = 100;
    tie(res.rot, res.trans, init_reproj_err, res.cr_2d, res.p_3d) = getInitPoseInSingleImage_IPPE(crs_in);
    if (init_reproj_err < OPTI_COST_THRESHOLD)
    {
        //std::cout << "here cost:" << init_reproj_err << std::endl;
        //std::cout << "rot: " << res.rot << std::endl;
        //std::cout << "trans: " << res.trans << std::endl;

        //predict and refine axial-corner location
        auto [axial_corner_2D, axial_point_3D, axial_idx, pc_idx, pc_corners] =
            predictAndRefineAxialCorner(crs_in, img_in, res.rot, res.trans, left_flag);

        res.cr_2d.push_back(axial_corner_2D);
        res.p_3d.push_back(axial_point_3D);
        if (axial_corner_2D.x > 0 && axial_corner_2D.y > 0)
        {
            //std::cout << "here cost:" << init_reproj_err << std::endl;

            res.axial_idx = axial_idx;
            res.detected_pc.pc_idx = pc_idx;
            res.detected_pc.corners_ = pc_corners;
            //disRectifyOneCorner
            res.detected_pc.corners_.at(0) = disRectifyOneCorner(axial_corner_2D, left_flag);
        }
    }

    return res;// OneImageReturnParas();
}

//
std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> Optimizator::optiMarkerPosebyAll(const OneImageReturnParas& left_init, 
    const OneImageReturnParas& right_init) const
{
    const auto left_cr_2d = left_init.cr_2d;
    auto left_p_3d = left_init.p_3d;
    auto left_rot = left_init.rot;
    const auto left_trans = left_init.trans;
    const auto left_axial_idx = left_init.axial_idx;
    const auto left_pc_idx = left_init.detected_pc.pc_idx;

    const auto right_cr_2d = right_init.cr_2d;
    auto right_p_3d = right_init.p_3d;
    auto right_rot = right_init.rot;
    const auto right_trans = right_init.trans;
    const auto right_axial_idx = right_init.axial_idx;
    const auto right_pc_idx = right_init.detected_pc.pc_idx;

    /*the result is based on the first pattern*/
    if (left_axial_idx > JUDGE_USE_THRED && right_axial_idx > JUDGE_USE_THRED)
    {
        int pos_or_neg = (left_p_3d.at(left_p_3d.size() - 1).z() > 0) ? 1 : -1;
        int left_bias_idx = - pos_or_neg * SEPE_PATTERN_NUM * (left_pc_idx - 1) + left_axial_idx;
        auto [left_rot_rectified, left_points_rectified] = rectifyRotAndPointsbyAxialSign(left_rot, left_p_3d, left_bias_idx);
        if (pos_or_neg < 0)
        {
            Eigen::Matrix3d rotX;
            rotX << 1, 0, 0,
                0, cos(PI), -sin(PI),
                0, sin(PI), cos(PI);

            left_rot_rectified = left_rot_rectified * rotX;

            for (int ix = 0; ix < left_points_rectified.size(); ix++)
            {
                left_points_rectified.at(ix).y() = -left_points_rectified.at(ix).y();
                left_points_rectified.at(ix).z() = -left_points_rectified.at(ix).z();
            }
        }
        
        pos_or_neg = (right_p_3d.at(right_p_3d.size() - 1).z() > 0) ? 1 : -1;
        int right_bias_idx = -pos_or_neg * SEPE_PATTERN_NUM * (right_pc_idx - 1) + right_axial_idx;
        auto [right_rot_rectified, right_points_rectified] = rectifyRotAndPointsbyAxialSign(right_rot, right_p_3d, right_bias_idx);
        if (pos_or_neg < 0)
        {
            Eigen::Matrix3d rotX;
            rotX << 1, 0, 0,
                0, cos(PI), -sin(PI),
                0, sin(PI), cos(PI);

            right_rot_rectified = right_rot_rectified * rotX;

            for (int ix = 0; ix < right_points_rectified.size(); ix++)
            {
                right_points_rectified.at(ix).y() = -right_points_rectified.at(ix).y();
                right_points_rectified.at(ix).z() = -right_points_rectified.at(ix).z();
            }
        }


        /*calc avg rot and trans*/
        Eigen::Vector3d right_trans_bias(b_dis, 0, 0);
        Eigen::Vector3d trans_avg = (left_trans + right_trans + right_trans_bias) / 2.0;
        Eigen::Matrix3d dR = right_rot_rectified * left_rot_rectified.transpose();
        Eigen::AngleAxisd rot_vect;
        rot_vect.fromRotationMatrix(dR);
        Eigen::AngleAxisd half_rot_vect = rot_vect;
        half_rot_vect.angle() = rot_vect.angle() / 2.0;
        Eigen::Matrix3d rot_avg = (half_rot_vect.toRotationMatrix()) * left_rot_rectified;

        // init relative pose
        double init_pose[6] = { 0.0, 0, 0, 0, 0, 0 };
        auto [R_relative, t_relative, cost] = optInTwoImagesNonePlane(left_cr_2d, left_points_rectified, 
            right_cr_2d, right_points_rectified, rot_avg, trans_avg, init_pose);
        Eigen::Matrix3d R_res = rot_avg * R_relative;
        Eigen::Vector3d t_res = trans_avg + t_relative;

        //if (pos_or_neg < 0)
        //{
        //    Eigen::Matrix3d rotX;
        //    rotX << 1, 0, 0,
        //        0, cos(PI), -sin(PI),
        //        0, sin(PI), cos(PI);

        //    R_res = R_res * rotX;
        //}

        return { R_res, t_res, cost };

    }
    //else if (left_axial_idx > JUDGE_USE_THRED)
    //{
    //    int pos_or_neg = (left_p_3d.at(left_p_3d.size() - 1).z() > 0) ? 1 : -1;
    //    int left_bias_idx = -pos_or_neg * SEPE_PATTERN_NUM * (left_pc_idx - 1) + left_axial_idx;
    //    auto [left_rot_rectified, left_points_rectified] = rectifyRotAndPointsbyAxialSign(left_rot, left_p_3d, left_bias_idx);

    //    // init relative pose
    //    double init_pose[6] = { 0.0, 0, 0, 0, 0, 0 };
    //    auto [R_relative, t_relative, cost] = optInSingleImageNonePlane(left_cr_2d, left_points_rectified, left_rot_rectified, left_trans, init_pose);
    //    Eigen::Matrix3d R_res = left_rot_rectified * R_relative;
    //    Eigen::Vector3d t_res = left_trans + t_relative;

    //    if (pos_or_neg < 0)
    //    {
    //        Eigen::Matrix3d rotX;
    //        rotX << 1, 0, 0,
    //            0, cos(PI), -sin(PI),
    //            0, sin(PI), cos(PI);

    //        R_res = R_res * rotX;
    //    }

    //    return { R_res, t_res, cost };
    //}
    //else if (right_axial_idx > JUDGE_USE_THRED)
    //{
    //    int pos_or_neg = (right_p_3d.at(right_p_3d.size() - 1).z() > 0) ? 1 : -1;
    //    int right_bias_idx = -pos_or_neg * SEPE_PATTERN_NUM * (right_pc_idx - 1) + right_axial_idx;
    //    auto [right_rot_rectified, right_points_rectified] = rectifyRotAndPointsbyAxialSign(right_rot, right_p_3d, right_bias_idx);

    //    // init relative pose
    //    double init_pose[6] = { 0, 0, 0, 0, 0, 0 };
    //    auto [R_relative, t_relative, cost] = optInSingleImageNonePlane(right_cr_2d, right_points_rectified, right_rot_rectified, right_trans, init_pose);
    //    
    //    Eigen::Vector3d right_trans_bias(b_dis, 0, 0);
    //    Eigen::Matrix3d R_res = right_rot_rectified * R_relative;
    //    Eigen::Vector3d t_res = right_trans + t_relative + right_trans_bias;

    //    if (pos_or_neg < 0)
    //    {
    //        Eigen::Matrix3d rotX;
    //        rotX << 1, 0, 0,
    //            0, cos(PI), -sin(PI),
    //            0, sin(PI), cos(PI);

    //        R_res = R_res * rotX;
    //    }
    //    return { R_res, t_res, cost };
    //}
    else
    {
        return { Eigen::Matrix3d(), Eigen::Vector3d(), 200 };
    }

    return { Eigen::Matrix3d(), Eigen::Vector3d(), 200 };
}

std::tuple<Eigen::Matrix3d, std::vector<Eigen::Vector3d>> Optimizator::rectifyRotAndPointsbyAxialSign(const Eigen::Matrix3d& rot, const std::vector<Eigen::Vector3d>& p_3d, int bias_idx) const
{
    Eigen::Matrix3d rot_rectified;
    std::vector<Eigen::Vector3d> p_rectified;
    double bias_angle = bias_idx * L / r;

    Eigen::Matrix3d rotZ;
    rotZ << cos(bias_angle), -sin(bias_angle), 0,
        sin(bias_angle), cos(bias_angle), 0,
        0, 0, 1;

    rot_rectified = rot * rotZ;
    for (int i = 0; i < p_3d.size(); i++)
    {
        Eigen::Vector3d p_temp;
        p_temp = rotZ.transpose() * p_3d.at(i);

        p_rectified.push_back(p_temp);
    }


    return { rot_rectified, p_rectified };
}

double Optimizator::calcReprojectErrorbyRT(const Eigen::Matrix3d R, const Eigen::Vector3d t, const Corners corners_, const std::vector<Eigen::Vector3d>& points_) const
{
    Eigen::Matrix3d pose_extend, H_left;
    pose_extend.col(0) = R.col(0);
    pose_extend.col(1) = R.col(1);
    pose_extend.col(2) = t;
    H_left = A_cam * pose_extend;
    //cout << "homography after: " << H_left << endl;

    double err_sum = 0;
    int count = 0;
    for (int i = 0; i < corners_.size(); i++)
    {
        Eigen::Vector3d point_temp(points_.at(i).x(), points_.at(i).y(), 1);
        Eigen::Vector3d corner_m = H_left * point_temp;
        double ww = corner_m(2);
        if (fabs(ww) > DBL_EPSILON)
        {
            Eigen::Vector2d corner_err(0, 0);
            corner_err(0) = corners_.at(i).x - corner_m(0) / ww;
            corner_err(1) = corners_.at(i).y - corner_m(1) / ww;

            err_sum += powf(corner_err.norm(), 2);
            count++;
        }
        else
        {
            return 10000.0;
        }
    }

    if (count != 0)
    {
        return sqrtf(err_sum / (2*count));
    }
    else
    {
        return 10000.0;
    }
}

