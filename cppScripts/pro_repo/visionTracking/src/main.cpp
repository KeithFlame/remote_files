#include "alg/math_helper.h"
#include "alg/timer.h"
#include "config/config.h"
#include "detector/detector.h"
#include "detector/optimizator.h"
#include "detector/rectifier.h"
//#include "detector/lm_opencv.h"
#include "def/timer.h"
#include "def/triple_buffer.h"
#include "def/ringbuffer.h"
#include <vector>
#include <thread>
#include <mutex>

#include <future>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Windows.h>
#include <fstream>

#include "sr_common.h"
#include "sr_mbx.h"
#include "globaldefine.h"
#include <stddef.h>
#include <stdio.h>
#include <direct.h>
//#define CALIBRATE_Z_AXIS 

using namespace sr;
CSrMbx clientImage;
CSrMbx clientMotionPara;
CSrMbx severVisionCommand;
#define SHOW_RESULT_IMAGE
//#define WRITE_VIDEO
//#define OPTIMIZATION_TEST
#define OPEN_MULTI_THREAD_CODE
constexpr auto MAX_RUNTIME = 100;
constexpr auto SHOW_TIME_COUNT = 200;

#define USING_MULTI_MARKERS
struct   TimerAvrg {
    std::vector<double> times;
    size_t curr = 0, n;
    std::chrono::high_resolution_clock::time_point begin, end;
    TimerAvrg(int _n = 30) { n = _n; times.reserve(n); }
    inline void start() { begin = std::chrono::high_resolution_clock::now(); }
    inline void stop() {
        end = std::chrono::high_resolution_clock::now();
        double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
        if (times.size() < n) times.push_back(duration); else { times[curr] = duration; curr++; if (curr >= times.size()) curr = 0; }
    }
    double getAvrg() {
        double sum = 0; for (auto t : times) sum += t;
        return sum / double(times.size());
    }
};
TimerAvrg Fps;

#ifdef OPEN_MULTI_THREAD_CODE

typedef struct ImageCombo
{
    cv::Mat left;
    cv::Mat right;

    int tag;
}ImageCombo;

typedef struct ImageCornersCombo
{
    cv::Mat img_left;
    cv::Mat img_right;

    std::vector<CornersSorted> cnr_left;
    std::vector<CornersSorted> cnr_right;

    int tag;
}ImageCornersCombo;

typedef struct ImageExposureTime
{
    cv::Mat img;

    float time;
}ImageExposureTime;

std::tuple<cv::Mat, cv::Mat, cv::Mat> showBothResult(const cv::String& window_name, const ImageCornersCombo& cb_cnr_img, int good_percentage,
    std::vector<PatternResult> pc_left, std::vector<PatternResult> pc_right,
    std::vector<Corners> coordi_crs_left, std::vector<Corners> coordi_crs_right,
    std::vector<Eigen::Vector3d> t_res, std::vector<Eigen::Matrix3d> R_res, std::vector<double> cost_res_arr)
{
    //static bool writed_flag = false;
    static std::string str_last[3];

    /*resize image*/
    double resize_scale = 1.0;
    double text_scale = 1.3;
    int point_radius = 4;
    int line_width = 8;

    cv::Mat img_left_resized, img_right_resized;
    cv::Mat img_left_return = cb_cnr_img.img_left;
    cv::Mat img_right_return = cb_cnr_img.img_right;

    const int COLOR_NUM = 5;
    cv::Scalar axial_color_arr[6] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 165, 255), cv::Scalar(0, 255, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 139) };
    cv::Scalar color_arr[COLOR_NUM] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(192, 112, 0), cv::Scalar(0, 255, 255) };
    
    int axis_corner_left[5] = { 0 }, axis_corner_right[5] = { 0 };
    if (cb_cnr_img.cnr_left.size() > 0 && cb_cnr_img.cnr_left.size() < COLOR_NUM)
    {
        //int marker_i = 0;
        for (int marker_i = 0; marker_i < cb_cnr_img.cnr_left.size(); marker_i++)
        {
            /*left*/
            for (const auto& sc : cb_cnr_img.cnr_left.at(marker_i))
            {
                cv::circle(img_left_return, Corner(sc.point.x, sc.point.y), point_radius, color_arr[marker_i], -1);
            }
            if (marker_i < pc_left.size())
            {
                if (pc_left.at(marker_i).corners_.size() > 2)
                {
                    axis_corner_left[marker_i] = 1;
                    cv::circle(img_left_return, Corner(pc_left.at(marker_i).corners_.at(0).x, 
                        pc_left.at(marker_i).corners_.at(0).y), point_radius, 
                        axial_color_arr[pc_left.at(marker_i).pc_idx - 1], -1);
  
                }
            }

            /*right*/
            if (marker_i < cb_cnr_img.cnr_right.size())
            {
                for (const auto& sc : cb_cnr_img.cnr_right.at(marker_i))
                {
                    cv::circle(img_right_return, Corner(sc.point.x, sc.point.y), 
                        point_radius, color_arr[marker_i], -1);
                }
            }
            if (marker_i < pc_right.size())
            {
                if (pc_right.at(marker_i).corners_.size() > 2)
                {
                    axis_corner_right[marker_i] = 1;
                    cv::circle(img_right_return, Corner(pc_right.at(marker_i).corners_.at(0).x, 
                        pc_right.at(marker_i).corners_.at(0).y), point_radius, 
                        axial_color_arr[pc_right.at(marker_i).pc_idx - 1], -1);
                }
            }
        }

        //draw coordinate system
        for (int marker_i = 0; marker_i < coordi_crs_left.size(); marker_i++)
        {
            if (coordi_crs_left.at(marker_i).size() == 4)
            {
                if ((coordi_crs_left.at(marker_i).at(0).x > 0 && coordi_crs_left.at(marker_i).at(0).x < img_left_return.cols - 1) &&
                    (coordi_crs_left.at(marker_i).at(0).y > 0 && coordi_crs_left.at(marker_i).at(0).y < img_left_return.rows - 1) &&
                    (coordi_crs_left.at(marker_i).at(1).x > 0 && coordi_crs_left.at(marker_i).at(1).x < img_left_return.cols - 1) &&
                    (coordi_crs_left.at(marker_i).at(1).y > 0 && coordi_crs_left.at(marker_i).at(1).y < img_left_return.rows - 1) &&
                    (coordi_crs_left.at(marker_i).at(2).x > 0 && coordi_crs_left.at(marker_i).at(2).x < img_left_return.cols - 1) &&
                    (coordi_crs_left.at(marker_i).at(2).y > 0 && coordi_crs_left.at(marker_i).at(2).y < img_left_return.rows - 1) &&
                    (coordi_crs_left.at(marker_i).at(3).x > 0 && coordi_crs_left.at(marker_i).at(3).x < img_left_return.cols - 1) &&
                    (coordi_crs_left.at(marker_i).at(3).y > 0 && coordi_crs_left.at(marker_i).at(3).y < img_left_return.rows - 1))
                {
                    cv::line(img_left_return, Corner(coordi_crs_left.at(marker_i).at(0).x, coordi_crs_left.at(marker_i).at(0).y), Corner(coordi_crs_left.at(marker_i).at(1).x, coordi_crs_left.at(marker_i).at(1).y),
                        cv::Scalar(0, 0, 255), line_width);
                    cv::line(img_left_return, Corner(coordi_crs_left.at(marker_i).at(0).x, coordi_crs_left.at(marker_i).at(0).y), Corner(coordi_crs_left.at(marker_i).at(2).x, coordi_crs_left.at(marker_i).at(2).y),
                        cv::Scalar(0, 255, 0), line_width);
                    cv::line(img_left_return, Corner(coordi_crs_left.at(marker_i).at(0).x, coordi_crs_left.at(marker_i).at(0).y), Corner(coordi_crs_left.at(marker_i).at(3).x, coordi_crs_left.at(marker_i).at(3).y),
                        cv::Scalar(255, 0, 0), line_width);
                }

                if ((coordi_crs_right.at(marker_i).at(0).x > 0 && coordi_crs_right.at(marker_i).at(0).x < img_right_return.cols - 1) &&
                    (coordi_crs_right.at(marker_i).at(0).y > 0 && coordi_crs_right.at(marker_i).at(0).y < img_right_return.rows - 1) &&
                    (coordi_crs_right.at(marker_i).at(1).x > 0 && coordi_crs_right.at(marker_i).at(1).x < img_right_return.cols - 1) &&
                    (coordi_crs_right.at(marker_i).at(1).y > 0 && coordi_crs_right.at(marker_i).at(1).y < img_right_return.rows - 1) &&
                    (coordi_crs_right.at(marker_i).at(2).x > 0 && coordi_crs_right.at(marker_i).at(2).x < img_right_return.cols - 1) &&
                    (coordi_crs_right.at(marker_i).at(2).y > 0 && coordi_crs_right.at(marker_i).at(2).y < img_right_return.rows - 1) &&
                    (coordi_crs_right.at(marker_i).at(3).x > 0 && coordi_crs_right.at(marker_i).at(3).x < img_right_return.cols - 1) &&
                    (coordi_crs_right.at(marker_i).at(3).y > 0 && coordi_crs_right.at(marker_i).at(3).y < img_right_return.rows - 1))
                {
                    cv::line(img_right_return, Corner(coordi_crs_right.at(marker_i).at(0).x, coordi_crs_right.at(marker_i).at(0).y), Corner(coordi_crs_right.at(marker_i).at(1).x, coordi_crs_right.at(marker_i).at(1).y),
                        cv::Scalar(0, 0, 255), line_width);
                    cv::line(img_right_return, Corner(coordi_crs_right.at(marker_i).at(0).x, coordi_crs_right.at(marker_i).at(0).y), Corner(coordi_crs_right.at(marker_i).at(2).x, coordi_crs_right.at(marker_i).at(2).y),
                        cv::Scalar(0, 255, 0), line_width);
                    cv::line(img_right_return, Corner(coordi_crs_right.at(marker_i).at(0).x, coordi_crs_right.at(marker_i).at(0).y), Corner(coordi_crs_right.at(marker_i).at(3).x, coordi_crs_right.at(marker_i).at(3).y),
                        cv::Scalar(255, 0, 0), line_width);
                }
            }
        }
    }

    cv::resize(img_left_return, img_left_resized, cv::Size(), resize_scale, resize_scale);
    cv::resize(img_right_return, img_right_resized, cv::Size(), resize_scale, resize_scale);
    /*joint image*/
    cv::Mat joint_image;
    //cv::Mat cropped_left = img_left_resized(cv::Range(350, img_left_resized.rows-50), cv::Range(0, img_left_resized.cols-1000)).clone();
    //cv::Mat cropped_right = img_right_resized(cv::Range(350, img_right_resized.rows-50), cv::Range(0, img_right_resized.cols-1000)).clone();
    //hconcat(cropped_left, cropped_right, joint_image);
    hconcat(img_left_resized, img_right_resized, joint_image); 

    //char wm_name[3] = { 'A', 'B', 'C' };
    std::string wm_name[3] = { "SWM", "DWM_B", "DWM_A" };
    //int i = 0;
    cv::Scalar joint_color_arr[3] = { cv::Scalar(64, 187, 208), cv::Scalar(105, 208, 64), cv::Scalar(208, 187, 64) };
    
    for (int i = 0; i < cost_res_arr.size(); i++)
    {
        if (i < 3)
        {
            char* chCode;
            std::string str2;
            chCode = new(std::nothrow)char[200];
            if (cost_res_arr.at(i) < 100)
            {
                Eigen::Quaterniond R_quat(R_res.at(i));
                std::sprintf(chCode, "%s: left %d, right %d, position [%.1lf,%.1lf,%.1lf], orientation [%.2lf,%.2lf,%.2lf,%.2lf]",
                    wm_name[i], cb_cnr_img.cnr_left.at(i).size() + axis_corner_left[i], cb_cnr_img.cnr_right.at(i).size() + axis_corner_right[i], t_res.at(i)(0), t_res.at(i)(1), t_res.at(i)(2), R_quat.w(), R_quat.x(), R_quat.y(), R_quat.z());

                str2 = std::string(chCode);
                str_last[i] = str2;
            }
            else
            {
                str2 = str_last[i];
            }
            delete[]chCode;

            cv::putText(img_left_resized, str2, cv::Point_(10, 50 + 50 * i), cv::FONT_HERSHEY_COMPLEX, text_scale, joint_color_arr[i], 2);
            cv::putText(joint_image, str2, cv::Point_(10, 50 + 50 * i), cv::FONT_HERSHEY_COMPLEX, text_scale, joint_color_arr[i], 2);

        }

    }

    //cv::imshow(" as", joint_image);
    
    //std::cout << "-->clientImage.send" << std::endl;
    //std::cout << joint_image.rows << "    " << joint_image.cols <<joint_image.channels()<< std::endl;
    DWORD t_start, t_end;

    //bool rc = clientImage.send((S8*)img_left_resized.data, 2592 * 1944 * 3 * sizeof(unsigned char));
    //std::cout <<"send: "<< rc << std::endl;
    //joint_image.release();

    return { img_left_resized, img_right_resized, joint_image };
}

struct ThreadShowImage
{
    ThreadShowImage() : tag_last(-1) {};

    void thread_showImg()
    {
        //Optimizator opti_fun_arr[3] = { Optimizator(25, 2.0, 3.0, 2.3, 4.6, 24, 4), Optimizator(25, 2.0, 3.0, 2.3, 4.6, 24, 4), Optimizator(25, 2.0, 3.0, 2.3, 4.6, 24, 4)};
        Optimizator opti_fun_arr[3] = { Optimizator(25, 4.0, 2.747, 2.394, 4.546, 24, 4)/*Optimizator(25, 2.0, 2.4, 2.1, 4.1, 24, 4)*/, Optimizator(25, 2.0, 2.4, 2.1, 4.1, 24, 4), Optimizator(25, 2.0, 2.5, 2.1, 4.1, 24, 4) };
        //Optimizator optimizator1(25, 2.5, 4.75, 20, 5);
        //Optimizator optimizator2(25, 2.5);

        bool write_video_flag = false;
        cv::VideoWriter writerRes("VideoRes_20201115_1.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60.0, cv::Size(2592, 972));
        //cv::namedWindow("left + right", cv::WINDOW_NORMAL);
        int count = 0, good_count = 0;

        /*record res*/
        std::ofstream record_pose_data;
        record_pose_data.open("pose_data.raw");
        int record_count = 1;

        auto show_avg_time = 0.0;

        PixelType dis_1_2 = 0.0, dis_1_3 = 0.0;

        //cv::namedWindow("left + right", 1);
        while (true)
        {
            auto start_timepoint = mtimer::getDurationSinceEpoch();

            ImageCornersCombo cb_cnrImg;
            cb_cnrImg = cnrImg_buf.readLast();
           // cv::Mat imge_raw = cnrImg_buf.readLast().img_left.clone();
            if ( (!cb_cnrImg.img_left.empty() && !cb_cnrImg.img_right.empty()) && cb_cnrImg.tag != tag_last)
            {
                auto start_ = tic();

                tag_last = cb_cnrImg.tag;
                count++;
                if (count > 1000000)
                {
                    count = 0;
                }

                /*calc pose*/
                std::vector<Eigen::Matrix3d> R_res_arr;
                std::vector<Eigen::Vector3d> t_res_arr;
                std::vector<double> cost_res_arr;
                std::vector<PatternResult> left_pc_arr;
                std::vector<PatternResult> right_pc_arr;
                std::vector<Corners> left_coordi_crs_arr;
                std::vector<Corners> right_coordi_crs_arr;

                for (int i = 0; i < cb_cnrImg.cnr_left.size(); i++)
                {
                    if ((!cb_cnrImg.cnr_left.at(i).empty() || !cb_cnrImg.cnr_right.at(i).empty()) && i < 3)
                    {
                        auto[R_res, t_res, cost_res, left_pc_res, right_pc_res, left_coordi_crs, right_coordi_crs] = opti_fun_arr[i].process(cb_cnrImg.cnr_left.at(i),
                            cb_cnrImg.cnr_right.at(i), cb_cnrImg.img_left, cb_cnrImg.img_right);

                        //std::cout << "cost: " << cost_res << std::endl;

                        R_res_arr.push_back(R_res);
                        t_res_arr.push_back(t_res);
                        cost_res_arr.push_back(cost_res);
                        left_pc_arr.push_back(left_pc_res);
                        right_pc_arr.push_back(right_pc_res);
                        left_coordi_crs_arr.push_back(left_coordi_crs);
                        right_coordi_crs_arr.push_back(right_coordi_crs);
                    }
                    else
                    {
                        R_res_arr.push_back(Eigen::Matrix3d::Identity());
                        t_res_arr.push_back(Eigen::Vector3d::Zero());
                        cost_res_arr.push_back(0.1);
                        left_pc_arr.push_back(PatternResult());
                        right_pc_arr.push_back(PatternResult());
                        left_coordi_crs_arr.push_back(Corners());
                        right_coordi_crs_arr.push_back(Corners());
                    }
                }
                
                int good_percentage = 0;
                if (count > 0)
                {
                    good_percentage = (int)(100 * good_count / (double)count);
                }
                else
                {
                    good_count = 0;
                    count = 0;
                }

                /*show result*/
                cv::Mat img_left_res, img_right_res, joint_res; 
                //std::cout << "-->Debug layer 4" << std::endl;
                std::tie(img_left_res, img_right_res, joint_res) = showBothResult("left + right", cb_cnrImg, good_percentage, left_pc_arr,
                    right_pc_arr, left_coordi_crs_arr, right_coordi_crs_arr, t_res_arr, R_res_arr, cost_res_arr);
                
                //std::cout << "-->Debug layer 4" << std::endl;
                if (cost_res_arr.size() > 0 )
                {
                    //std::cout << "-->Debug layer 4.1" << std::endl;
                    if (cost_res_arr.at(0) < 100)
                    {
                        //std::cout << "-->Debug layer 4.2" << std::endl;
                        good_count++;

                        if (write_video_flag)
                        {
                            //std::cout << "-->Debug layer 4.3" << std::endl;
                            writerRes.write(joint_res);
                        }
                    }
                }
                //std::cout << "-->Debug layer 5" << std::endl;
                Eigen::Quaterniond R_quat(R_res_arr.at(0));
                float pose[8];
                //std::cout << "-->Debug layer 6" << std::endl;
                pose[0] = t_res_arr.at(0)(0);
                pose[1] = t_res_arr.at(0)(1);
                pose[2] = t_res_arr.at(0)(2);
                //std::cout << "-->Debug layer 7" << std::endl;
                pose[3] = R_quat.w();
                pose[4] = R_quat.x();
                pose[5] = R_quat.y();
                //std::cout << "-->Debug layer 8" << std::endl;
                pose[6] = R_quat.z();
                pose[7] = cost_res_arr.at(0);

                //std::cout << "-->clientMotionPara.send" << std::endl;
                clientMotionPara.send((S8*)&pose, 8 * sizeof(float));

                auto key = cv::waitKey(1);
                if (key == 'q' || key == 'Q')
                {
                    exit(0);
                    break;
                }
                else if (key == 'r' || key == 'R')
                {
                    if (cost_res_arr.at(0) < 100)
                    {
                        

                        record_pose_data << t_res_arr.at(0)(0) << "\t" << t_res_arr.at(0)(1) << "\t" << t_res_arr.at(0)(2) << "\t" << R_quat.w()
                            << "\t" << R_quat.x() << "\t" << R_quat.y() << "\t" << R_quat.z() << "\t" << cost_res_arr.at(0) << std::endl;

                        std::cout << record_count << std::endl;
                        record_count++;
                    }
                }
                else if (key == 'v' || key == 'V')
                {
                    write_video_flag = !write_video_flag;
                    if (write_video_flag)
                    {
                        std::cout << "Start save video!!!!!!!!!!!!!!!!!!" << std::endl;
                    }
                    else
                    {
                        std::cout << "Stop save video!!!!!!!!!!!!!!!!!!" << std::endl;
                    }
                }
                else if (key == 'w' || key == 'W')
                {
                    for (int i = 0; i < cost_res_arr.size(); i++)
                    {
                        if (cost_res_arr.at(i) < 100)
                        {
                            std::cout << "marker #" << i + 1 << ": \n" << t_res_arr.at(i) << std::endl
                                << R_res_arr.at(i) << std::endl << cost_res_arr.at(i) << std::endl;

                            std::cout << "left_corners: " << std::endl;
                            for (int ixx = 0; ixx < cb_cnrImg.cnr_left.at(i).size(); ixx++)
                            {
                                std::cout << cb_cnrImg.cnr_left.at(i).at(ixx).point << std::endl;
                            }
                            std::cout << "right_corners: " << std::endl;
                            for (int ixx = 0; ixx < cb_cnrImg.cnr_right.at(i).size(); ixx++)
                            {
                                std::cout << cb_cnrImg.cnr_right.at(i).at(ixx).point << std::endl;
                            }
                        }
                    }
                    cv::imwrite("left_1.jpg", img_left_res);
                    cv::imwrite("right_1.jpg", img_right_res);

                    char joint_res_name[50] = { 0 };
                    sprintf(joint_res_name, "joint_res_%d.jpg", count);
                    cv::imwrite(joint_res_name, joint_res);
                    std::cout << "writed" << std::endl;
                }
                else if (key == 'p' || key == 'P')
                {
                    //std::cout << "total: " << count << ",  showImg average time: " << show_avg_time / count
                    //    << ", good percentage: " << (double)good_count / count << std::endl;
                    for (int i = 0; i < cost_res_arr.size(); i++)
                    {
                        if (cost_res_arr.at(i) < 100)
                        {
                            std::cout << "marker #" << i + 1 << ": \n" << R_res_arr.at(i) << std::endl;
                        }
                    }
                }
                else
                {
                    ;
                }

                show_avg_time += toc(start_, "start_");

                //std::cout << "count in show: " << count << endl;

                if (count % SHOW_TIME_COUNT == 0 && count != 0)
                {
                    std::cout << "total: " << count << ",  showImg average time: " << show_avg_time / SHOW_TIME_COUNT
                        <<", good percentage: " << (double)good_count / count << std::endl;
                    show_avg_time = 0;
                }
            }

            auto ms = mtimer::getDurationSince(start_timepoint);
            if (ms < MAX_RUNTIME)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RUNTIME - (int)ms));
            }
        }
    }

    void setImage(ImageCornersCombo& cnrImg)
    {
        cnrImg_buf.update(cnrImg);
    }

    int tag_last;
    TripleBuffer<ImageCornersCombo> cnrImg_buf;
};


struct ThreadDetection
{
    ThreadDetection() : tag_last(-1) {
        //t_cPose = std::thread(&ThreadCalcPose::thread_calcPose, &thread_cPose);
        //t_cPose.detach();
        ////Sleep(1000);

        t_shw = std::thread(&ThreadShowImage::thread_showImg, &thread_shw);
        t_shw.detach();

        //Sleep(1000);
    };

    void thread_detection()
    {
        cv::Size size(2592, 1944);
        auto detector_left1 = std::make_unique<Detector>(size, SWM);
        auto detector_right1 = std::make_unique<Detector>(size, SWM);
        
#ifdef USING_MULTI_MARKERS
        auto detector_left2 = std::make_unique<Detector>(size, DWM_A);
        auto detector_right2 = std::make_unique<Detector>(size, DWM_A);
        auto detector_left3 = std::make_unique<Detector>(size, DWM_B);
        auto detector_right3 = std::make_unique<Detector>(size, DWM_B);
#endif // USING_MULTI_MARKERS
        int count = 1;
        auto detect_avg_time = 0.0;

        bool is_detect=true;

        int isOpen=0;
        U32 len = severVisionCommand.recv((S8*)&isOpen, sizeof(int), 2);
        std::cout <<"-->isOpen: "<< isOpen<<"len: "<<len << std::endl;
        if (isOpen == 1)
        {
            is_detect = true;
        }
        else
        {
            is_detect = false;
        }
        //std::cout << "-->Debug layer 0" << std::endl;
        while (true)
        {
            U32 len = severVisionCommand.recv((S8*)&isOpen, sizeof(int), 2);
            
            if (isOpen == 1)
            {
                is_detect = true;
                std::cout << "-->isOpen: " << isOpen << "    len: " << len << std::endl;
            }
            else
            {
                is_detect = false;
            }
            auto start_timepoint = mtimer::getDurationSinceEpoch();


            ImageCombo cb1 = image.readLast();
#ifdef USING_MULTI_MARKERS
            ImageCombo cb2 = image.readLast();
            ImageCombo cb3 = image.readLast();
#endif // USING_MULTI_MARKERS

            if ((!cb1.left.empty() && !cb1.right.empty()) && cb1.tag != tag_last)
            {
                auto start_ = tic();

                tag_last = cb1.tag;
                count++;
                if (count > 1000000)
                {
                    count = 1;
                }

                ImageCornersCombo cb_cnrImg;
                cb1.left.copyTo(cb_cnrImg.img_left);
                cb1.right.copyTo(cb_cnrImg.img_right);

                //std::cout << "-->Debug layer 2" << std::endl;
                if (is_detect)
                {

#ifdef USE_MULTI_THREAD
                {
                    auto fut_left1 = std::async(std::launch::async, [&]() { return detector_left1->process(cb1.left); });
                    auto fut_right1 = std::async(std::launch::async, [&]() { return detector_right1->process(cb1.right); });
                    auto corners_left_local1 = fut_left1.get();
                    auto corners_right_local1 = fut_right1.get();

                    cb_cnrImg.cnr_left.push_back(corners_left_local1);
                    cb_cnrImg.cnr_right.push_back(corners_right_local1);
                }
                
#ifdef USING_MULTI_MARKERS
                auto fut_left2 = std::async(std::launch::async, [&]() { return detector_left2->process(cb2.left); });
                auto fut_right2 = std::async(std::launch::async, [&]() { return detector_right2->process(cb2.right); });
                auto fut_left3 = std::async(std::launch::async, [&]() { return detector_left3->process(cb3.left); });
                auto fut_right3 = std::async(std::launch::async, [&]() { return detector_right3->process(cb3.right); });

                auto corners_left_local2 = fut_left2.get();
                auto corners_right_local2 = fut_right2.get();
                auto corners_left_local3 = fut_left3.get();
                auto corners_right_local3 = fut_right3.get();
#endif // USING_MULTI_MARKERS

#else
                auto corners_left_local = detector_left->process(left_image_local);
                auto corners_right_local = detector_right->process(right_image_local);
#endif // USE_MULTI_THREAD


#ifdef USING_MULTI_MARKERS
                cb_cnrImg.cnr_left.push_back(corners_left_local2);
                cb_cnrImg.cnr_right.push_back(corners_right_local2);
                cb_cnrImg.cnr_left.push_back(corners_left_local3);
                cb_cnrImg.cnr_right.push_back(corners_right_local3);
#endif // USING_MULTI_MARKERS

                }
                else
                {
                    cb_cnrImg.cnr_left.push_back(CornersSorted());
                    cb_cnrImg.cnr_right.push_back(CornersSorted());
                }
                //std::cout << "-->Debug layer 3" << std::endl;
                
                cb_cnrImg.tag = count;
                thread_shw.setImage(cb_cnrImg);

                detect_avg_time += toc(start_, "start_");

                //std::cout << "count in detection: " << count << endl;
                if (count % SHOW_TIME_COUNT == 0 && count != 0)
                {
                    std::cout << "total: " << count << ",  detection average time: " << detect_avg_time / SHOW_TIME_COUNT << std::endl;
                    detect_avg_time = 0;
                }

            }
            
            auto ms = mtimer::getDurationSince(start_timepoint);
            if (ms < MAX_RUNTIME)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RUNTIME - (int)ms));
            }
        }

    }

    void setImage(ImageCombo& img)
    {
        image.update(img);
    }

    int tag_last;
    TripleBuffer<ImageCombo> image;

    //std::thread t;

    //ThreadCalcPose thread_cPose;
    //std::thread t_cPose;
    ThreadShowImage thread_shw;
    std::thread t_shw;
};

struct ThreadRead 
{
    ThreadRead() {
        t = std::thread(&ThreadDetection::thread_detection, &thread_detect);
        t.detach();
        Sleep(5000);
    }
    void thread_read()
    {
        cv::Size size(2592, 1944);
        cv::VideoCapture capture_left, capture_right;
        //capture_left.open("../../video/VideoLeft_20210611_1.avi");// //VideoLeft_20200801_2.avi");//VideoLeft_20200705_4
        //capture_right.open("../../video/VideoRight_20210611_1.avi");// //VideoRight_20200801_2.avi");//VideoRight_20200705_4
        std::cout << "cap1 loading ..." << std::endl;
        capture_left.open(1/*,  cv::CAP_DSHOW*/);
        std::cout << "cap1 loaded,cap2 loading ..." << capture_left .isOpened()<< std::endl;
        capture_right.open(0/*, cv::CAP_DSHOW*/);
        std::cout << "cap2 loaded." << capture_right .isOpened()<< std::endl;

        capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
        capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
        capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
        capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);
        //capture_left.set(cv::CAP_PROP_SHARPNESS, 3.0);
        //capture_left.set(cv::CAP_PROP_EXPOSURE, -6);
        //capture_left.set(cv::CAP_PROP_FPS, 15);

        std::cout << "over21" << std::endl;
        capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
        capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
        capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
        capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);
        //capture_right.set(cv::CAP_PROP_SHARPNESS, 3.0);
        //capture_right.set(cv::CAP_PROP_EXPOSURE, -6);
        //capture_right.set(cv::CAP_PROP_FPS, 15);

        std::cout << "over22" << std::endl;
        int count = 1;
        auto read_avg_time = 0.0;
        while (true)
        {
            auto start_timepoint = mtimer::getDurationSinceEpoch();
            auto start_ = tic();
            std::cout << "over23" << std::endl;
            ImageCombo frame_read;
#ifdef USE_MULTI_THREAD
            //capture_left >> frame_read.left;
            //capture_right >> frame_read.right;
            std::cout << "over24" << std::endl;
            //cv::waitKey(10);
            //imshow("", frame_read.right);
            //std::cout << "over" << std::endl;
            auto fut_read_left = std::async(std::launch::async, [&]() { return capture_left.read(frame_read.left); });
            auto fut_read_right = std::async(std::launch::async, [&]() { return capture_right.read(frame_read.right); });
            auto left_readed = fut_read_left.get();
            auto right_readed = fut_read_right.get();

            //auto fut_read_left = std::async(std::launch::async, [&]() { return 1; });
            //auto fut_read_right = std::async(std::launch::async, [&]() { return 1; });
            //auto left_readed = fut_read_left.get();
            //auto right_readed = fut_read_right.get();
#else
            auto left_readed = capture_left.read(frame_left);
            auto right_readed = capture_right.read(frame_right);
#endif // USE_MULTI_THREAD

            std::cout << "over25" << std::endl;
            if (!left_readed || !right_readed)
            //if(!capture_left.read(frame_read.left) || !capture_right.read(frame_read.right))
            {
                std::cout << "????" << std::endl;
                continue;
            }
            else
            {
                std::cout << "over26" << std::endl;
                count++;
                if (count > 1000000)
                {
                    count = 1;
                }

                frame_read.tag = count;
                thread_detect.setImage(frame_read);
                
                //break;
                //std::cout << "readed count: " << count << std::endl;
            }
            read_avg_time += toc(start_, "start_");

            if (count % SHOW_TIME_COUNT == 0 && count != 0)
            {
                std::cout << "total: " << count << ",  read average time: " << read_avg_time / SHOW_TIME_COUNT << std::endl;
                read_avg_time = 0.0;
            }

            auto ms = mtimer::getDurationSince(start_timepoint);
            if (ms < MAX_RUNTIME)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RUNTIME - (int)ms));
            }

        }

        printf("enter any key to exit program.\n");

        char ssst = getchar();
        exit(0);
    }

    //ThreadRectify thread_rect;
    
    ThreadDetection thread_detect;
    std::thread t;
};

void test_multi_thread()
{
    ThreadRead read_;
    
    read_.thread_read();

    //thread t1(thread_read);
    //thread t2(thread_rectify);
    //thread t3(thread_detection);
    //thread t4(thread_calcPose);
    //thread t5(thread_showImg);    

    //t1.join();
    //t2.join();
    //t3.join();
    //t4.join();
    //t5.join();
}

#endif // OPEN_MULTI_THREAD_CODE
float dis_second_line1 = 4.58; // 
float axial_side_length = 3.96;
float marker_radius = 7.6012;

cv::Mat test_hdr(ImageExposureTime cap1, ImageExposureTime cap2)
{
    //! [Load images and exposure times]
    std::vector<cv::Mat> images;
    std::vector<float> times;

    images.push_back(cap1.img);
    images.push_back(cap2.img);
    times.push_back(cap1.time);
    times.push_back(cap2.time);

    //! [Estimate camera response]
    //cv::Mat response;
    //cv::Ptr<cv::CalibrateDebevec> calibrate = cv::createCalibrateDebevec();
    //calibrate->process(images, response, times);
    //! [Estimate camera response]

    //! [Make HDR image]
    //cv::Mat hdr;
    //cv::Ptr<cv::MergeDebevec> merge_debevec = cv::createMergeDebevec();
    //merge_debevec->process(images, hdr, times, response);
    ////! [Make HDR image]

    ////! [Tonemap HDR image]
    //cv::Mat ldr;
    //cv::Ptr<cv::Tonemap> tonemap = cv::createTonemap(2.2f);
    //tonemap->process(hdr, ldr);
    ////! [Tonemap HDR image]

    //! [Perform exposure fusion]
    cv::Mat fusion;
    cv::Ptr<cv::MergeMertens> merge_mertens = cv::createMergeMertens();
    merge_mertens->process(images, fusion);
    //! [Perform exposure fusion]

    //! [Write results]
    //fusion = fusion * 255;
    //cv::imwrite("test_fusioghvn.png", fusion);
    //cv::imwrite("test_ldr.png", ldr * 255);
    //cv::imwrite("test_hdr.hdr", hdr);
    //! [Write results]

    return fusion;
}


//������
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

double square_size(std::vector<cv::Point> square)
{
    if (square.size() != 4)
        return 0;

    double width = abs((square[0].x + square[1].x + square[2].x + square[3].x) / 4-square[0].x)*2;
    double height = abs((square[0].y + square[1].y + square[2].y + square[3].y) / 4 - square[0].y)*2;
    return width * height;

}

//��һ�������Ǵ����ԭʼͼ�񣬵ڶ��������ͼ��
void findSquares(const cv::Mat& image, cv::Mat& out, std::vector<std::vector<cv::Point>>& squares, int min_area=10000, int max_area=100000)
{
    int thresh = 50, N = 5;
    squares.clear();

    cv::Mat src, dst, gray_one, gray;

    src = image.clone();
    out = image.clone();
    gray_one = cv::Mat(src.size(), CV_8U);
    //�˲���ǿ��Ե���
    cv::medianBlur(src, dst, 9);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    //��ͼ���ÿ����ɫͨ���в��Ҿ���
    for (int c = 0; c < image.channels(); c++)
    {
        int ch[] = { c, 0 };

        //ͨ������
        cv::mixChannels(&dst, 1, &gray_one, 1, ch, 1);

        // ���Լ�����ֵ
        for (int l = 0; l < N; l++)
        {
            // ��canny()��ȡ��Ե
            if (l == 0)
            {
                //����Ե
                Canny(gray_one, gray, 5, thresh, 5);
                //��Û
                dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));
            }
            else
            {
                gray = gray_one >= (l + 1) * 255 / N;
            }

            // ��������
            cv::findContours(gray, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> approx;

            // ������ҵ�������
            for (size_t i = 0; i < contours.size(); i++)
            {
                //ʹ��ͼ����������ж�������
                approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

                //������������󣬵õ�����4������
                if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) < max_area && fabs(cv::contourArea(cv::Mat(approx))) > min_area && cv::isContourConvex(cv::Mat(approx)))
                {
                    //std::cout << cv::contourArea(cv::Mat(approx)) << std::endl;
                    //std::cout << approx << std::endl;
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++)
                    {
                        // ��������Ե֮��Ƕȵ��������
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if (maxCosine < 0.3)
                    {
                        // 2 < width/height < 3
                        if ((fabs((approx[0].x + approx[1].x + approx[2].x + approx[3].x) / 4 - approx[0].x) >
                            fabs((approx[0].y + approx[1].y + approx[2].y + approx[3].y) / 4 - approx[0].y) * 2
                            && fabs((approx[0].x + approx[1].x + approx[2].x + approx[3].x) / 4 - approx[0].x) <
                            fabs((approx[0].y + approx[1].y + approx[2].y + approx[3].y) / 4 - approx[0].y) * 3)
                            || (fabs((approx[0].y + approx[1].y + approx[2].y + approx[3].y) / 4 - approx[0].y) >
                                fabs((approx[0].x + approx[1].x + approx[2].x + approx[3].x) / 4 - approx[0].x) * 2
                                && fabs((approx[0].y + approx[1].y + approx[2].y + approx[3].y) / 4 - approx[0].y) <
                                fabs((approx[0].x + approx[1].x + approx[2].x + approx[3].x) / 4 - approx[0].x) * 3))
                        {
                            if (square_size(approx) < 45000 && square_size(approx) > 20000)
                                squares.emplace_back(approx);

                            break;
                        }
                        
                    }
                }
            }
        }
    }


    for (size_t i = 0; i < squares.size(); i++)
    {
        const cv::Point* p = &squares[i][0];

        int n = (int)squares[i].size();
        if (p->x > 3 && p->y > 3)
        {
            std::cout << "square size["<<i<<"] :" << square_size(squares[i]) << std::endl;
            polylines(out, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }
    }
    //std::cout << "squares:" << '\n'  << std::endl;
}

//order: downleft,downright,upleft,upright,
std::vector<int> distinguish_camera()
{
    cv::Size size(2592, 1944);
    cv::VideoCapture capture1, capture2, capture3, capture4;

    std::cout << "cap1 loading ..." << std::endl;
    capture1.open(0);
    std::cout << "cap1 loaded, cap2 loading ..." << capture1.isOpened() << std::endl;
    capture2.open(1);
    std::cout << "cap2 loaded, cap3 loading ..." << capture2.isOpened() << std::endl;
    capture3.open(2);
    std::cout << "cap3 loaded, cap4 loading ..." << capture3.isOpened() << std::endl;
    capture4.open(3);
    std::cout << "cap4 loaded ..." << capture4.isOpened() << std::endl;

    capture1.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture1.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture1.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture1.set(cv::CAP_PROP_BUFFERSIZE, 3);

    capture2.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture2.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture2.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture2.set(cv::CAP_PROP_BUFFERSIZE, 3);
    
    capture3.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture3.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture3.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture3.set(cv::CAP_PROP_BUFFERSIZE, 3);

    capture4.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture4.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture4.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture4.set(cv::CAP_PROP_BUFFERSIZE, 3);

    std::vector<int> camera_order; //order: downleft,downright,upleft,upright,
    std::vector<std::vector<cv::Point>> squares1, squares2, squares3, squares4, square;
    while (1)
    {

        cv::Mat frame1, frame2, frame3, frame4;
        cv::Mat out1, out2, out3, out4;
        capture1.read(frame1);
        capture2.read(frame2);
        capture3.read(frame3);
        capture4.read(frame4);


        
        if (squares1.empty())
        {
            findSquares(frame1, out1, squares1);
            cv::imwrite("./conf/vision/cap1.jpg", out1);
            std::cout << "findSquares ...1" << std::endl;
        }
            
        
        if (squares2.empty())
        {
            findSquares(frame2, out2, squares2);
            cv::imwrite("./conf/vision/cap2.jpg", out2);
            std::cout << "findSquares ...2" << std::endl;
        }
            
        
        if (squares3.empty())
        {
            findSquares(frame3, out3, squares3);
            std::cout << "findSquares ...3" << std::endl;
            cv::imwrite("./conf/vision/cap3.jpg", out3);
        }
            
        
        if (squares4.empty())
        {
            findSquares(frame4, out4, squares4);
            std::cout << "findSquares ...4" << std::endl;
            cv::imwrite("./conf/vision/cap4.jpg", out4);
        }
            
        
        camera_order.clear();
        if (squares1.size() == 0 || squares2.size() == 0 || squares3.size() == 0 || squares4.size() == 0)
            continue;
                               
        square.emplace_back(squares1[0]);
        square.emplace_back(squares2[0]);
        square.emplace_back(squares3[0]);
        square.emplace_back(squares4[0]);

        //std::cout << square[0] << std::endl;
        //std::cout << square[1] << std::endl;
        //std::cout << square[2] << std::endl;
        //std::cout << square[3] << std::endl;

        // if width<height, emplace forward; else emplace back.
        if (fabs((square[0][0].x + square[0][1].x + square[0][2].x + square[0][3].x) / 4 - square[0][0].x) <
            fabs((square[0][0].y + square[0][1].y + square[0][2].y + square[0][3].y) / 4 - square[0][0].y))
            camera_order.insert(camera_order.begin(), 0);
        else
            camera_order.emplace_back(0);
        if (fabs((square[1][0].x + square[1][1].x + square[1][2].x + square[1][3].x) / 4 - square[1][0].x) <
            fabs((square[1][0].y + square[1][1].y + square[1][2].y + square[1][3].y) / 4 - square[1][0].y))
            camera_order.insert(camera_order.begin(), 1);
        else
            camera_order.emplace_back(1);
        if (fabs((square[2][0].x + square[2][1].x + square[2][2].x + square[2][3].x) / 4 - square[2][0].x) <
            fabs((square[2][0].y + square[2][1].y + square[2][2].y + square[2][3].y) / 4 - square[2][0].y))
            camera_order.insert(camera_order.begin(), 2);
        else
            camera_order.emplace_back(2);
        if (fabs((square[3][0].x + square[3][1].x + square[3][2].x + square[3][3].x) / 4 - square[3][0].x) <
            fabs((square[3][0].y + square[3][1].y + square[3][2].y + square[3][3].y) / 4 - square[3][0].y))
            camera_order.insert(camera_order.begin(), 3);
        else
            camera_order.emplace_back(3);

        // down, distinguish y
        if ((square[camera_order[0]][0].x + square[camera_order[0]][1].x + square[camera_order[0]][2].x + square[camera_order[0]][3].x) <
            (square[camera_order[1]][0].x + square[camera_order[1]][1].x + square[camera_order[1]][2].x + square[camera_order[1]][3].x))
            std::swap(camera_order[0], camera_order[1]);
        // up, distinguish x
        if ((square[camera_order[2]][0].x + square[camera_order[2]][1].x + square[camera_order[2]][2].x + square[camera_order[2]][3].x) <
            (square[camera_order[3]][0].x + square[camera_order[3]][1].x + square[camera_order[3]][2].x + square[camera_order[3]][3].x))
            std::swap(camera_order[2], camera_order[3]);
        

        break;
    }
    capture1.release();
    capture2.release();
    capture3.release();
    capture4.release();
    return camera_order;
}

void test_single_thread()
{
    cv::Size size(2592, 1944);
    cv::VideoCapture capture_left, capture_right;
    //capture_left.open("../../video/VideoLeft_20210611_1.avi");// //VideoLeft_20200801_2.avi");//VideoLeft_20200705_4
    //capture_right.open("../../video/VideoRight_20210611_1.avi");// //VideoRight_20200801_2.avi");//VideoRight_20200705_4
    std::cout << "cap1 loading ..." << std::endl;
    capture_left.open(1);
    std::cout << "cap1 loaded,cap2 loading ..." << capture_left.isOpened() << std::endl;
    capture_right.open(0);
    std::cout << "cap2 loaded." << capture_right.isOpened() << std::endl;

    capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);
    //capture_left.set(cv::CAP_PROP_SHARPNESS, 3.0);
    //capture_left.set(cv::CAP_PROP_EXPOSURE, -6);
    capture_left.set(cv::CAP_PROP_FPS, 15);

    std::cout << "over21" << std::endl;
    capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);
    //capture_right.set(cv::CAP_PROP_SHARPNESS, 3.0);
    //capture_right.set(cv::CAP_PROP_EXPOSURE, -6);
    capture_right.set(cv::CAP_PROP_FPS, 15);

    //detector
    auto detector_left1 = std::make_unique<Detector>(size, SWM);
    auto detector_right1 = std::make_unique<Detector>(size, SWM);

#ifdef USING_MULTI_MARKERS
    auto detector_left2 = std::make_unique<Detector>(size, DWM_B);
    auto detector_right2 = std::make_unique<Detector>(size, DWM_B);

#endif // USING_MULTI_MARKERS

    //float dis_second_line1 = 4.58; // 
    //float axial_side_length = 3.96;
    //float marker_radius = 7.6012;

#ifdef CALIBRATE_Z_AXIS
    float dis_second_line1 = 2.738; // 
    float axial_side_length = 2.386;
    float marker_radius = 4.531;
#else
    //float dis_second_line1 = 4.5407; // 
    //float axial_side_length = 3.9260;
    //float marker_radius = 7.5360;
#endif
    //optimizator
    Optimizator opti_fun[2] = { Optimizator(25, 4.0, dis_second_line1, axial_side_length, marker_radius, 24, 4)
        /*Optimizator(25, 2.0, 2.5, 2.1, 4.1, 24, 4)*/,
        Optimizator(25, 4.0,  dis_second_line1, axial_side_length, marker_radius, 24, 4) };

    //std::cout << "over22" << std::endl;
    int count = 1;
    auto read_avg_time = 0.0;
    bool is_detect = false;
    int isOpen = 0;

    std::ofstream record_pose_data;
    record_pose_data.open("pose_data.raw");
    int record_count = 1;
    int write_count = 1;

    while (true)
    {
        //std::cout << "over23" << std::endl;
        ImageCombo frame_read;

        
        auto fut_read_left = std::async(std::launch::async, [&]() { return capture_left.read(frame_read.left); });
        auto fut_read_right = std::async(std::launch::async, [&]() { return capture_right.read(frame_read.right); });
        auto left_readed = fut_read_left.get();
        auto right_readed = fut_read_right.get();



        if (! left_readed || !right_readed)
        {
            std::cout << "No image." << std::endl;
            break;
        }

        
        U32 len = severVisionCommand.recv((S8*)&isOpen, sizeof(int), 2);
        std::cout << "-->isOpen: " << isOpen << "  len: " << len << std::endl;
        if (isOpen == 1)
        {
            is_detect = true;
        }
        else
        {
            is_detect = false;
            //detector_left1->clearHistoryDatas(size);
            //detector_right1->clearHistoryDatas(size);
#ifdef USING_MULTI_MARKERS
            //detector_left2->clearHistoryDatas(size);
            //detector_right2->clearHistoryDatas(size);
#endif // USING_MULTI_MARKERS

        }

        ImageCornersCombo cb_cnrImg;
        frame_read.left.copyTo(cb_cnrImg.img_left);
        frame_read.right.copyTo(cb_cnrImg.img_right);

        std::cout << "over1" << std::endl;
        //detect marker
        if (is_detect)
        {
            
            //std::cout<<"go" <<std::endl;
            auto fut_left1 = std::async(std::launch::async, [&]() { return detector_left1->process(frame_read.left); });
            auto fut_right1 = std::async(std::launch::async, [&]() { return detector_right1->process(frame_read.right); });
            auto corners_left_local1 = fut_left1.get();
            auto corners_right_local1 = fut_right1.get();
            //std::cout << "isDetect: "<<is_detect << std::endl;
            cb_cnrImg.cnr_left.push_back(corners_left_local1);
            cb_cnrImg.cnr_right.push_back(corners_right_local1);

#ifdef USING_MULTI_MARKERS
            auto fut_left2 = std::async(std::launch::async, [&]() { return detector_left2->process(frame_read.left); });
            auto fut_right2 = std::async(std::launch::async, [&]() { return detector_right2->process(frame_read.right); });
            auto corners_left_local2 = fut_left2.get();
            auto corners_right_local2 = fut_right2.get();
            //std::cout << "isDetect: " << is_detect << std::endl;
            cb_cnrImg.cnr_left.push_back(corners_left_local2);
            cb_cnrImg.cnr_right.push_back(corners_right_local2);
#endif // USING_MULTI_MARKERS


        }
        else
        {
            cb_cnrImg.cnr_left.push_back(CornersSorted());
            cb_cnrImg.cnr_right.push_back(CornersSorted());
#ifdef USING_MULTI_MARKERS
            cb_cnrImg.cnr_left.push_back(CornersSorted());
            cb_cnrImg.cnr_right.push_back(CornersSorted());
#endif
        }
        std::cout << "over223" << std::endl;

        std::vector<Eigen::Matrix3d> R_res_arr;
        std::vector<Eigen::Vector3d> t_res_arr;
        std::vector<double> cost_res_arr;
        std::vector<PatternResult> left_pc_arr;
        std::vector<PatternResult> right_pc_arr;
        std::vector<Corners> left_coordi_crs_arr;
        std::vector<Corners> right_coordi_crs_arr;
        float poseTwo[16] = {0};
        for (int i = 0; i < cb_cnrImg.cnr_left.size(); i++)
        {
            if (!cb_cnrImg.cnr_left.at(i).empty() && !cb_cnrImg.cnr_right.at(i).empty())
            {

                auto [R_res, t_res, cost_res, left_pc_res, right_pc_res, left_coordi_crs, right_coordi_crs] = opti_fun[i].process(cb_cnrImg.cnr_left.at(i),
                    cb_cnrImg.cnr_right.at(i), cb_cnrImg.img_left, cb_cnrImg.img_right);


                //std::cout << "cost: " << cost_res << std::endl;

                R_res_arr.push_back(R_res);
                t_res_arr.push_back(t_res);
                cost_res_arr.push_back(cost_res);
                left_pc_arr.push_back(left_pc_res);
                right_pc_arr.push_back(right_pc_res);
                left_coordi_crs_arr.push_back(left_coordi_crs);
                right_coordi_crs_arr.push_back(right_coordi_crs);
                std::cout << "over213" << std::endl;
                Eigen::Quaterniond R_quat(R_res_arr.at(i));
                float pose[8];
                pose[0] = t_res_arr.at(i)(0);
                pose[1] = t_res_arr.at(i)(1);
                pose[2] = t_res_arr.at(i)(2);
                pose[3] = R_quat.w();
                pose[4] = R_quat.x();
                pose[5] = R_quat.y();
                pose[6] = R_quat.z();
                pose[7] = cost_res_arr.at(i);

                //std::cout << "-->clientMotionPara.send" << std::endl;
                std::cout << pose[0] << "  " << pose[1] << "  " << pose[2] << "  " << pose[3] << "  "
                    << pose[4] << "  " << pose[5] << "  " << pose[6] << "  " << pose[7] << std::endl;

                for (int iw = 0; iw < sizeof(pose) / sizeof(float); iw++)
                    poseTwo[i * 8 + iw] = pose[iw];
            }
            else
            {
                R_res_arr.push_back(Eigen::Matrix3d::Identity());
                t_res_arr.push_back(Eigen::Vector3d::Zero());
                cost_res_arr.push_back(0.1);
                left_pc_arr.push_back(PatternResult());
                right_pc_arr.push_back(PatternResult());
                left_coordi_crs_arr.push_back(Corners());
                right_coordi_crs_arr.push_back(Corners());
                std::cout << "Nothing here !" << std::endl;
            }
        }
        if (is_detect)
        {
            clientMotionPara.send((S8*)&poseTwo, sizeof(poseTwo), 20);
            std::cout << poseTwo[0] << "  " << poseTwo[1] << "  " << poseTwo[2] << "  " << poseTwo[3] << "  "
                << poseTwo[4] << "  " << poseTwo[5] << "  " << poseTwo[6] << "  " << poseTwo[7]
                << poseTwo[8] << "  " << poseTwo[9] << "  " << poseTwo[10] << "  " << poseTwo[11] << "  "
                << poseTwo[12] << "  " << poseTwo[13] << "  " << poseTwo[14] << "  " << poseTwo[15] << std::endl;
        }

        /*show result*/
        cv::Mat img_left_res, img_right_res, joint_res;
        if (cost_res_arr.size() > 0)
        {
            std::tie(img_left_res, img_right_res, joint_res) = showBothResult("left + right", cb_cnrImg, 1, left_pc_arr,
                right_pc_arr, left_coordi_crs_arr, right_coordi_crs_arr, t_res_arr, R_res_arr, cost_res_arr);
            //std::cout << "over193" << std::endl;
            //std::cout << "size: [ " << img_left_res.size()<<" ]" << std::endl;
            bool rc = clientImage.send((S8*)img_left_res.data, 2592 * 1944 * 3  * sizeof(unsigned char),50);
            //bool rc = clientImage.send((S8*)img_right_res.data, 2592 * 1944 * 3 / 4 * sizeof(unsigned char), 50);

        }
        //std::cout << "over203" << std::endl;

        auto key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            break;
        }
        else if (key == 'w' || key == 'W')
        {
        }
        else if (key == 'r' || key == 'R'||is_detect)
        {
            is_detect = false;
            
            Eigen::Quaterniond R_quat(R_res_arr.at(0));
            record_pose_data << t_res_arr.at(0)(0) << "\t" << t_res_arr.at(0)(1) << "\t" << t_res_arr.at(0)(2) << "\t" << R_quat.w()
                << "\t" << R_quat.x() << "\t" << R_quat.y() << "\t" << R_quat.z() << "\t" << cost_res_arr.at(0) << std::endl;

            std::cout << record_count << std::endl;
            record_count++;
        }
        else
        {
            ;
        }
        //imshow("  ",joint_res);

    }
    record_pose_data.close();
}

void test_single_image()
{
    cv::Size size(2592, 1944);

    /*read image*/
    int img_id = 1;
    char img_name_left[1000], img_name_right[1000], joint_save_name[1000];
    std::ofstream record_pose_data;
    record_pose_data.open("../../video/systemTest/pose_data.raw");
    int write_count = 1;
    std::cout << "layer: 1" << std::endl;
    for (int img_id = 0; img_id < 100; img_id++)
    {
        sprintf(img_name_left, "../../video/systemTest/left_%d.jpg", img_id);
        sprintf(img_name_right, "../../video/systemTest/right_%d.jpg", img_id);
        //sprintf(joint_save_name, "../../video/systemTest/joint_%d.jpg", img_id);

        cv::Mat img_left = cv::imread(img_name_left);
        cv::Mat img_right = cv::imread(img_name_right);
        //std::cout << "layer: 2  "<< img_left.rows << std::endl;
        if (img_left.rows != 1944 || img_right.rows != 1944)
            break;
        /*detect corners*/
        auto detector_left1 = std::make_unique<Detector>(size, SWM);
        auto detector_right1 = std::make_unique<Detector>(size, SWM);

#ifdef USING_MULTI_MARKERS
        auto detector_left2 = std::make_unique<Detector>(size, DWM_B);
        auto detector_right2 = std::make_unique<Detector>(size, DWM_B);
        //auto detector_left3 = std::make_unique<Detector>(size, DWM_A);
        //auto detector_right3 = std::make_unique<Detector>(size, DWM_A);
#endif // USING_MULTI_MARKERS

#ifdef USE_MULTI_THREAD
        auto fut_left1 = std::async(std::launch::async, [&]() { return detector_left1->process(img_left); });
        auto fut_right1 = std::async(std::launch::async, [&]() { return detector_right1->process(img_right); });
        auto corners_left_local1 = fut_left1.get();
        auto corners_right_local1 = fut_right1.get();

        //CornersSorted corners_right_local1;

#ifdef USING_MULTI_MARKERS
        auto fut_left2 = std::async(std::launch::async, [&]() { return detector_left2->process(img_left); });
        auto fut_right2 = std::async(std::launch::async, [&]() { return detector_right2->process(img_right); });
        //auto fut_left3 = std::async(std::launch::async, [&]() { return detector_left3->process(img_left); });
        //auto fut_right3 = std::async(std::launch::async, [&]() { return detector_right3->process(img_right); });

        auto corners_left_local2 = fut_left2.get();
        auto corners_right_local2 = fut_right2.get();
        //auto corners_left_local3 = fut_left3.get();
        //auto corners_right_local3 = fut_right3.get();
#endif // USING_MULTI_MARKERS

#else
        auto corners_left_local = detector_left->process(left_image_local);
        auto corners_right_local = detector_right->process(right_image_local);
#endif // USE_MULTI_THREAD
        ImageCornersCombo cb_cnrImg;
        img_left.copyTo(cb_cnrImg.img_left);
        img_right.copyTo(cb_cnrImg.img_right);

        CornersSorted corner_zuobis = corners_left_local1;
        corners_left_local1.clear();
        for (int xxi = 1; xxi < 8; xxi++)
        {
            corners_left_local1.push_back(corner_zuobis.at(xxi));
        }

        cb_cnrImg.cnr_left.push_back(corners_left_local1);
        cb_cnrImg.cnr_right.push_back(corners_right_local1);

        std::cout << "\ncorners_left: " << corners_left_local1.size() << std::endl;
        for (int i_left = 0; i_left < corners_left_local1.size(); i_left++)
        {
            std::cout << corners_left_local1.at(i_left).point << std::endl;
        }

        std::cout << "\ncorners_right: " << corners_right_local1.size() << std::endl;
        for (int i_right = 0; i_right < corners_right_local1.size(); i_right++)
        {
            std::cout << corners_right_local1.at(i_right).point << std::endl;
        }

#ifdef USING_MULTI_MARKERS
        cb_cnrImg.cnr_left.push_back(corners_left_local2);
        cb_cnrImg.cnr_right.push_back(corners_right_local2);
        //cb_cnrImg.cnr_left.push_back(corners_left_local3);
        //cb_cnrImg.cnr_right.push_back(corners_right_local3);
#endif // USING_MULTI_MARKERS
        
        /*optimize position and orientation*/
        Optimizator opti_fun[2] = { Optimizator(25, 4.0, dis_second_line1, axial_side_length, marker_radius, 24, 4)
            /*Optimizator(25, 2.0, 2.5, 2.1, 4.1, 24, 4)*/,
            Optimizator(25, 4.0,  dis_second_line1, axial_side_length, marker_radius, 24, 4) };
        std::vector<Eigen::Matrix3d> R_res_arr;
        std::vector<Eigen::Vector3d> t_res_arr;
        std::vector<double> cost_res_arr;
        std::vector<PatternResult> left_pc_arr;
        std::vector<PatternResult> right_pc_arr;
        std::vector<Corners> left_coordi_crs_arr;
        std::vector<Corners> right_coordi_crs_arr;

        for (int i = 0; i < cb_cnrImg.cnr_left.size(); i++)
        {
            if ((!cb_cnrImg.cnr_left.at(i).empty() || !cb_cnrImg.cnr_right.at(i).empty()) && i < 2)
            {
                auto [R_res, t_res, cost_res, left_pc_res, right_pc_res, left_coordi_crs, right_coordi_crs] = opti_fun[i].process(cb_cnrImg.cnr_left.at(i),
                    cb_cnrImg.cnr_right.at(i), cb_cnrImg.img_left, cb_cnrImg.img_right);

                //std::cout << "cost: " << cost_res << std::endl;

                R_res_arr.push_back(R_res);
                t_res_arr.push_back(t_res);
                cost_res_arr.push_back(cost_res);
                left_pc_arr.push_back(left_pc_res);
                right_pc_arr.push_back(right_pc_res);
                left_coordi_crs_arr.push_back(left_coordi_crs);
                right_coordi_crs_arr.push_back(right_coordi_crs);
            }
            else
            {
                R_res_arr.push_back(Eigen::Matrix3d::Identity());
                t_res_arr.push_back(Eigen::Vector3d::Zero());
                cost_res_arr.push_back(0.1);
                left_pc_arr.push_back(PatternResult());
                right_pc_arr.push_back(PatternResult());
                left_coordi_crs_arr.push_back(Corners());
                right_coordi_crs_arr.push_back(Corners());
            }
        }

        /*show result*/
        cv::Mat img_left_res, img_right_res, joint_res;
        if (cost_res_arr.size() > 0)
        {
            std::tie(img_left_res, img_right_res, joint_res) = showBothResult("left + right", cb_cnrImg, 1, left_pc_arr,
                right_pc_arr, left_coordi_crs_arr, right_coordi_crs_arr, t_res_arr, R_res_arr, cost_res_arr);
        }
        
        cv::Mat show_res;
        float resize_scale = 0.25;
        cv::resize(joint_res, show_res, cv::Size(), resize_scale, resize_scale);
        imshow("res", show_res);
        auto key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            ;
        }
        else if (key == 'w' || key == 'W'||1)
        {
            Eigen::Vector3d pen_tip(0, 0, 0);
            auto len = 18.5;
            //for (int i = 0; i < cost_res_arr.size(); i++)
            //{
            //    if (cost_res_arr.at(i) < 100)
            //    {
            //        std::cout << "marker #" << i + 1 << ": \n" << t_res_arr.at(i) << std::endl;
            //        pen_tip = t_res_arr.at(0) + R_res_arr.at(0) * Eigen::Vector3d(0, 0, len);
            //    }
            //}
            Eigen::Quaterniond R_quat(R_res_arr.at(0));
            record_pose_data << t_res_arr.at(0)(0) << "\t" << t_res_arr.at(0)(1) << "\t" << t_res_arr.at(0)(2) << "\t" << R_quat.w()
                << "\t" << R_quat.x() << "\t" << R_quat.y() << "\t" << R_quat.z() << "\t" << cost_res_arr.at(0) << std::endl;

            Eigen::Quaterniond R_quat1(R_res_arr.at(1));
            record_pose_data << t_res_arr.at(1)(0) << "\t" << t_res_arr.at(1)(1) << "\t" << t_res_arr.at(1)(2) << "\t" << R_quat1.w()
                << "\t" << R_quat1.x() << "\t" << R_quat1.y() << "\t" << R_quat1.z() << "\t" << cost_res_arr.at(1) << std::endl;
            record_pose_data << std::endl << std::endl;

            
            //cv::imwrite("left_1.jpg", img_left_res/*imge_raw*//*img_left_res*//*cb_cnrImg.img_left*/);
            //cv::imwrite("right_1.jpg", img_right_res/*img_right_res*//*cb_cnrImg.img_right*/);
            //cv::imwrite(joint_save_name, joint_res/*img_right_res*//*cb_cnrImg.img_right*/);
            write_count++;
            //record_pose_data << pen_tip.x() << "\t" << pen_tip.y() << "\t" << pen_tip.z() << std::endl;
            std::cout << "writed" << std::endl;
        }
        else if (key == 27)
        {
            //record_pose_data.close();
            //break;
        }
        else
        {
            ;
        }
    }

    record_pose_data.close();
}


void capture_video()
{
    cv::Size size(2592, 1944);
    cv::VideoCapture capture_left, capture_right;
    //capture_left.open("../../video/VideoLeft_20210611_1.avi");// //VideoLeft_20200801_2.avi");//VideoLeft_20200705_4
    //capture_right.open("../../video/VideoRight_20210611_1.avi");// //VideoRight_20200801_2.avi");//VideoRight_20200705_4
    std::cout << "cap1 loading ..." << std::endl;
    capture_left.open(1);
    std::cout << "cap1 loaded,cap2 loading ..." << capture_left.isOpened() << std::endl;
    capture_right.open(0);
    std::cout << "cap2 loaded." << capture_right.isOpened() << std::endl;

    capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);
    //capture_left.set(cv::CAP_PROP_SHARPNESS, 3.0);
    //capture_left.set(cv::CAP_PROP_EXPOSURE, -6);
    capture_left.set(cv::CAP_PROP_FPS, 15);

    std::cout << "over21" << std::endl;
    capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);
    //capture_right.set(cv::CAP_PROP_SHARPNESS, 3.0);
    //capture_right.set(cv::CAP_PROP_EXPOSURE, -6);
    capture_right.set(cv::CAP_PROP_FPS, 15);

    if (!capture_left.isOpened() || !capture_right.isOpened())
    {
        printf("can not open ...\n");
        return;
    }

    cv::namedWindow("left", cv::WINDOW_NORMAL);
    cv::namedWindow("right", cv::WINDOW_NORMAL);
    Rectifier rectifier(size);

    cv::VideoWriter writer_left("VideoLeft_20210925_2.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15.0, size);
    cv::VideoWriter writer_right("VideoRight_20210925_2.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15.0, size);

    cv::Mat frame_left, frame_right;
    bool write_flag = false;
    int count = 0;
    float pics_interVal = 1.f;
    float current_time_interval=0.f;
    Fps.start();
    bool isGetFrame = false;
    while (true)
    {
        if (!capture_left.read(frame_left) || !capture_right.read(frame_right))
            break;

        //frame_left = rectifier.rectify(frame_left, Rectifier::LEFT);
        //frame_right = rectifier.rectify(frame_right, Rectifier::RIGHT);

        if (write_flag)
        {
            writer_left.write(frame_left);
            writer_right.write(frame_right);
        }

        imshow("left", frame_left);
        imshow("right", frame_right);

        Fps.stop();
        current_time_interval += (float)Fps.getAvrg();
        //std::cout << current_time_interval << std::endl;
        if (current_time_interval > 1.f)
        {
            //ע����м���ʵ���ֶ�����
            //isGetFrame = true;
            current_time_interval = 0.f;
            Fps.start();
        }


        auto key = cv::waitKey(1);

        if (key == 'q' || key == 'Q')
        {
            break;
        }
        else if (key == 'v' || key == 'V')
        {
            write_flag = !write_flag;
            if (write_flag)
            {
                std::cout << "Start Record Video.\n";
            }
            else
            {
                std::cout << "Stop Record Video.\n";
            }
        }
        else if (key == 'w' || key == 'W'|| isGetFrame)
        {
            char write_name_left[1000] = {0};
            sprintf(write_name_left, "../../video/systemTest/left_%d.jpg", count);
            cv::imwrite(write_name_left, frame_left);
            char write_name_right[1000] = { 0 };
            sprintf(write_name_right, "../../video/systemTest/right_%d.jpg", count);
            cv::imwrite(write_name_right, frame_right);
            isGetFrame = false;
            std::cout << "write: " << count << std::endl;

            count++;
        }

    }

    std::cout << "capture video done!!!" << std::endl;
}


void test_single_thread_local(std::vector<int> camera_order)
{
    cv::Size size(2592, 1944);
    cv::VideoCapture capture_left, capture_right;
    //capture_left.open("../../video/VideoLeft_20210807_2.avi");// //VideoLeft_20200801_2.avi");//VideoLeft_20200705_4
    //capture_right.open("../../video/VideoRight_20210807_2.avi");// //VideoRight_20200801_2.avi");//VideoRight_20200705_4
    std::cout << "cap1 loading ..." << std::endl;  //order: downleft,downright,upleft,upright,
    capture_left.open(camera_order.at(2));
    std::cout << "cap1 loaded,cap2 loading ..." << capture_left.isOpened() << std::endl;
    capture_right.open(camera_order.at(3));
    std::cout << "cap2 loaded." << capture_right.isOpened() << std::endl;

    capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);

    std::cout << "over21" << std::endl;
    capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);

    //optimizator    

    //optimizator
    Optimizator opti_fun[3] = { Optimizator(25, 4.0, dis_second_line1, axial_side_length, marker_radius, 24, 4)
        /*Optimizator(25, 2.0, 2.5, 2.1, 4.1, 24, 4)*/,
        Optimizator(25, 4.0,  dis_second_line1, axial_side_length, marker_radius, 24, 4),
    Optimizator(25, 4.0,  dis_second_line1, axial_side_length, marker_radius, 24, 4) };

    //detector
    auto detector_left1 = std::make_unique<Detector>(size, SWM);
    auto detector_right1 = std::make_unique<Detector>(size, SWM);
#ifdef USING_MULTI_MARKERS
    auto detector_left2 = std::make_unique<Detector>(size, DWM_B);
    auto detector_right2 = std::make_unique<Detector>(size, DWM_B);

#endif // USING_MULTI_MARKERS
    //std::cout << "over22" << std::endl;
    int count = 1;
    auto read_avg_time = 0.0;
    bool is_detect = true;
    int isOpen = 1;
    std::ofstream record_pose_data;
    record_pose_data.open("pose_data.raw");
    int record_count = 1;
    int write_count = 1;
    while (true)
    {
        ImageCombo frame_read;

        Fps.start();
        auto fut_read_left = std::async(std::launch::async, [&]() { return capture_left.read(frame_read.left); });
        auto fut_read_right = std::async(std::launch::async, [&]() { return capture_right.read(frame_read.right); });
        auto left_readed = fut_read_left.get();
        auto right_readed = fut_read_right.get();

        Fps.stop();
        // chekc the speed by calculating the mean speed of all iterations
        std::cout << "\rTime Image Read =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl;


        if (!left_readed || !right_readed)
        {
            std::cout << "No image." << std::endl;
            break;
        }


        // U32 len = severVisionCommand.recv((S8*)&isOpen, sizeof(int), 2);
        // std::cout << "-->isOpen: " << isOpen << "  len: " << len << std::endl;
        if (isOpen == 1)
        {
            is_detect = true;
        }
        else
        {
            is_detect = false;
            //detector_left1->clearHistoryDatas(size);
            //detector_right1->clearHistoryDatas(size);
#ifdef USING_MULTI_MARKERS
            //detector_left2->clearHistoryDatas(size);
            //detector_right2->clearHistoryDatas(size);
#endif // USING_MULTI_MARKERS
        }

        ImageCornersCombo cb_cnrImg;
        frame_read.left.copyTo(cb_cnrImg.img_left);
        frame_read.right.copyTo(cb_cnrImg.img_right);

        //std::cout << "over1" << std::endl;
        //detect marker
        if (is_detect)
        {
            

            std::cout << "1: " << std::endl;
            //Fps.start();
            auto fut_left1 = std::async(std::launch::async, [&]() { return detector_left1->process(frame_read.left); });
            auto fut_right1 = std::async(std::launch::async, [&]() { return detector_right1->process(frame_read.right); });
            auto corners_left_local1 = fut_left1.get();
            auto corners_right_local1 = fut_right1.get();
            //std::cout << "isDetect: " << is_detect << std::endl;
            cb_cnrImg.cnr_left.push_back(corners_left_local1);
            cb_cnrImg.cnr_right.push_back(corners_right_local1);
            std::cout << "2: " << std::endl;

#ifdef USING_MULTI_MARKERS
            auto fut_left2 = std::async(std::launch::async, [&]() { return detector_left2->process(frame_read.left); });
            auto fut_right2 = std::async(std::launch::async, [&]() { return detector_right2->process(frame_read.right); });
            auto corners_left_local2 = fut_left2.get();
            auto corners_right_local2 = fut_right2.get();
            //std::cout << "isDetect: " << is_detect << std::endl;
            cb_cnrImg.cnr_left.push_back(corners_left_local2);
            cb_cnrImg.cnr_right.push_back(corners_right_local2);
#endif // USING_MULTI_MARKERS


            //Fps.stop();
            //std::cout << "\rTime Corner Detection =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl;

            std::cout << "3: " << std::endl;
            // chekc the speed by calculating the mean speed of all iterations
            //std::cout << "\rTime Marker Detection =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl
            /*
            cv::Mat left_x = frame_read.left.clone();
            for (int i = 0; i < corners_left_local1.size(); i++)
            {
                std::cout << "Corner: " << Corner(corners_left_local1.at(i).point.x, corners_left_local1.at(i).point.y) << std::endl;
                cv::circle(left_x, Corner(corners_left_local1.at(i).point.x, corners_left_local1.at(i).point.y), 
                    6, cv::Scalar(0, 255, 0), -1);
            }
            std::cout << "3.1" << std::endl;
            for (int i = 0; i < corners_left_local2.size(); i++)
            {
                std::cout << "Corner: " << Corner(corners_left_local2.at(i).point.x, corners_left_local2.at(i).point.y) << std::endl;
                cv::circle(left_x, Corner(corners_left_local2.at(i).point.x, corners_left_local2.at(i).point.y),
                    6, cv::Scalar(0, 0, 255), -1);
            }
            std::cout << "3.2" << std::endl;
            cv::imshow("test_left", left_x);

            cv::Mat right_x = frame_read.right.clone();
            for (int i = 0; i < corners_right_local1.size(); i++)
            {
                std::cout << "Corner: " << Corner(corners_right_local1.at(i).point.x, corners_right_local1.at(i).point.y) << std::endl;
                cv::circle(right_x, Corner(corners_right_local1.at(i).point.x, corners_right_local1.at(i).point.y),
                    6, cv::Scalar(0, 255, 0), -1);
            }
            std::cout << "3.3" << std::endl;
            for (int i = 0; i < corners_right_local2.size(); i++)
            {
                std::cout << "Corner: " << Corner(corners_right_local2.at(i).point.x, corners_right_local2.at(i).point.y) << std::endl;
                cv::circle(right_x, Corner(corners_right_local2.at(i).point.x, corners_right_local2.at(i).point.y),
                    6, cv::Scalar(0, 0, 255), -1);
            }
            std::cout << "3.4" << std::endl;
            cv::imshow("test_right", right_x);
            std::cout << "4: " << std::endl;
            */
        }
        else
        {
            cb_cnrImg.cnr_left.push_back(CornersSorted());
            cb_cnrImg.cnr_right.push_back(CornersSorted());
#ifdef USING_MULTI_MARKERS
            cb_cnrImg.cnr_left.push_back(CornersSorted());
            cb_cnrImg.cnr_right.push_back(CornersSorted());
#endif
        }
        //std::cout << "over223" << std::endl;

        std::vector<Eigen::Matrix3d> R_res_arr;
        std::vector<Eigen::Vector3d> t_res_arr;
        std::vector<double> cost_res_arr;
        std::vector<PatternResult> left_pc_arr;
        std::vector<PatternResult> right_pc_arr;
        std::vector<Corners> left_coordi_crs_arr;
        std::vector<Corners> right_coordi_crs_arr;

//#ifdef USING_MULTI_MARKERS
        for (int i = 0; i < cb_cnrImg.cnr_left.size(); i++)
        {
            if (!cb_cnrImg.cnr_left.at(i).empty() && !cb_cnrImg.cnr_right.at(i).empty())
            {
                auto [R_res, t_res, cost_res, left_pc_res, right_pc_res, left_coordi_crs, right_coordi_crs] = opti_fun[i].process(cb_cnrImg.cnr_left.at(i),
                    cb_cnrImg.cnr_right.at(i), cb_cnrImg.img_left, cb_cnrImg.img_right);

                R_res_arr.push_back(R_res);
                t_res_arr.push_back(t_res);
                cost_res_arr.push_back(cost_res);
                left_pc_arr.push_back(left_pc_res);
                right_pc_arr.push_back(right_pc_res);
                left_coordi_crs_arr.push_back(left_coordi_crs);
                right_coordi_crs_arr.push_back(right_coordi_crs);
            }
            else
            {
                R_res_arr.push_back(Eigen::Matrix3d::Identity());
                t_res_arr.push_back(Eigen::Vector3d::Zero());
                cost_res_arr.push_back(0.1);
                left_pc_arr.push_back(PatternResult());
                right_pc_arr.push_back(PatternResult());
                left_coordi_crs_arr.push_back(Corners());
                right_coordi_crs_arr.push_back(Corners());
            }
        }
//#else
//        if (!cb_cnrImg.cnr_left.at(0).empty() || !cb_cnrImg.cnr_right.at(0).empty())
//        {
//            Fps.start();
//            auto [R_res, t_res, cost_res, left_pc_res, right_pc_res, left_coordi_crs, right_coordi_crs] = opti_fun.process(cb_cnrImg.cnr_left.at(0),
//                cb_cnrImg.cnr_right.at(0), cb_cnrImg.img_left, cb_cnrImg.img_right);
//
//            Fps.stop();
//            // chekc the speed by calculating the mean speed of all iterations
//            std::cout << "\rTime Pose Optimization =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl;
//
//
//            R_res_arr.push_back(R_res);
//            t_res_arr.push_back(t_res);
//            cost_res_arr.push_back(cost_res);
//            left_pc_arr.push_back(left_pc_res);
//            right_pc_arr.push_back(right_pc_res);
//            left_coordi_crs_arr.push_back(left_coordi_crs);
//            right_coordi_crs_arr.push_back(right_coordi_crs);
//
//        }
//        else
//        {
//            R_res_arr.push_back(Eigen::Matrix3d::Identity());
//            t_res_arr.push_back(Eigen::Vector3d::Zero());
//            cost_res_arr.push_back(200);
//            left_pc_arr.push_back(PatternResult());
//            right_pc_arr.push_back(PatternResult());
//            left_coordi_crs_arr.push_back(Corners());
//            right_coordi_crs_arr.push_back(Corners());
//            //std::cout << "Nothing here !" << std::endl;
//        }
//#endif


        /*show result*/
        cv::Mat img_left_res, img_right_res, joint_res;
        if (cost_res_arr.size() > 0)
        {
            std::tie(img_left_res, img_right_res, joint_res) = showBothResult("left + right", cb_cnrImg, 1, left_pc_arr,
                right_pc_arr, left_coordi_crs_arr, right_coordi_crs_arr, t_res_arr, R_res_arr, cost_res_arr);
        }
        //std::cout << "over203" << std::endl;

        double resize_scale = 0.3;
        cv::resize(joint_res, joint_res, cv::Size(), resize_scale, resize_scale);
        imshow("  ", joint_res);

        auto key = cv::waitKey(1);
        static int count = 0;
        std::cout << "count: " << count++ << std::endl;
        if (key == 'q' || key == 'Q')
        {
            break;
        }
        else if (key == 'w' || key == 'W')
        {
            Eigen::Vector3d pen_tip(0, 0, 0);
            auto len = 18.5;
            for (int i = 0; i < cost_res_arr.size(); i++)
            {
                if (cost_res_arr.at(i) < 100)
                {
                    std::cout << "marker #" << i + 1 << ": \n" << t_res_arr.at(i) << std::endl;
                    pen_tip = t_res_arr.at(0) + R_res_arr.at(0) * Eigen::Vector3d(0, 0, len);
                }
            }
            cv::imwrite("left_"+std::to_string(write_count)+".jpg", img_left_res/*imge_raw*//*img_left_res*//*cb_cnrImg.img_left*/);
            cv::imwrite("right_" + std::to_string(write_count) + ".jpg", img_right_res/*img_right_res*//*cb_cnrImg.img_right*/);
            cv::imwrite("joint_res_"+std::to_string(write_count)+".jpg", joint_res/*img_right_res*//*cb_cnrImg.img_right*/);
            write_count++;
            //record_pose_data << pen_tip.x() << "\t" << pen_tip.y() << "\t" << pen_tip.z() << std::endl;
            std::cout << "writed:\t"<< write_count << std::endl;
        }
        else if (key == 'r' || key == 'R')
        {

            Eigen::Quaterniond R_quat(R_res_arr.at(0));
            record_pose_data << t_res_arr.at(0)(0) << "\t" << t_res_arr.at(0)(1) << "\t" << t_res_arr.at(0)(2) << "\t" << R_quat.w()
                << "\t" << R_quat.x() << "\t" << R_quat.y() << "\t" << R_quat.z() << "\t" << cost_res_arr.at(0) << std::endl;

            Eigen::Quaterniond R_quat1(R_res_arr.at(1));
            record_pose_data << t_res_arr.at(1)(0) << "\t" << t_res_arr.at(1)(1) << "\t" << t_res_arr.at(1)(2) << "\t" << R_quat1.w()
                << "\t" << R_quat1.x() << "\t" << R_quat1.y() << "\t" << R_quat1.z() << "\t" << cost_res_arr.at(1) << std::endl;
            record_pose_data << std::endl << std::endl;
            std::cout << record_count << std::endl;
            record_count++;
        }
        else
        {
            ;
        }

    }
}

void test_single_thread_for_multicamera(std::vector<int> camera_order)
{
    cv::Size size(2592, 1944);
    cv::VideoCapture capture_left, capture_right;
    //capture_left.open("../../video/VideoLeft_20210611_1.avi");// //VideoLeft_20200801_2.avi");//VideoLeft_20200705_4
    //capture_right.open("../../video/VideoRight_20210611_1.avi");// //VideoRight_20200801_2.avi");//VideoRight_20200705_4
    std::cout << "cap_top_1 loading ..." << std::endl;
    capture_left.open(camera_order.at(2));
    std::cout << "cap_top_1 loaded,cap_top_2 loading ..." << capture_left.isOpened() << std::endl;
    capture_right.open(camera_order.at(3));
    std::cout << "cap_top_2 loaded." << capture_right.isOpened() << std::endl;
    capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);
    //capture_left.set(cv::CAP_PROP_SHARPNESS, 3.0);
    //capture_left.set(cv::CAP_PROP_EXPOSURE, -6);
    capture_left.set(cv::CAP_PROP_FPS, 15);

    std::cout << "over21" << std::endl;
    capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
    capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
    capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
    capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);
    //capture_right.set(cv::CAP_PROP_SHARPNESS, 3.0);
    //capture_right.set(cv::CAP_PROP_EXPOSURE, -6);
    capture_right.set(cv::CAP_PROP_FPS, 15);

    //detector
    auto detector_left1 = std::make_unique<Detector>(size, SWM);
    auto detector_right1 = std::make_unique<Detector>(size, SWM);

#ifdef USING_MULTI_MARKERS
    auto detector_left2 = std::make_unique<Detector>(size, DWM_B);
    auto detector_right2 = std::make_unique<Detector>(size, DWM_B);

#endif // USING_MULTI_MARKERS

    //float dis_second_line1 = 4.58; // 
    //float axial_side_length = 3.96;
    //float marker_radius = 7.6012;

#ifdef CALIBRATE_Z_AXIS
    float dis_second_line1 = 2.738; // 
    float axial_side_length = 2.386;
    float marker_radius = 4.531;
#else
    //float dis_second_line1 = 4.5407; // 
    //float axial_side_length = 3.9260;
    //float marker_radius = 7.5360;
#endif
    //optimizator
    Optimizator opti_fun[2] = { Optimizator(25, 4.0, dis_second_line1, axial_side_length, marker_radius, 24, 4)
        /*Optimizator(25, 2.0, 2.5, 2.1, 4.1, 24, 4)*/,
        Optimizator(25, 4.0,  dis_second_line1, axial_side_length, marker_radius, 24, 4) };

    //std::cout << "over22" << std::endl;
    int count = 1;
    auto read_avg_time = 0.0;
    bool is_detect = false;
    int isOpen = 0;

    std::ofstream record_pose_data;
    record_pose_data.open("./conf/vision/pose_data.raw");
    int record_count = 1;
    int write_count = 1;

    while (true)
    {
        //std::cout << "over23" << std::endl;
        U32 len = severVisionCommand.recv((S8*)&isOpen, sizeof(int), 2);
        std::cout << "-->isOpen: " << isOpen << "  len: " << len << std::endl;
        //Fps.start();
        if (isOpen == 4)
        {
            capture_left.release();
            capture_right.release();
            loadConfigV2(true);
            std::cout << "cap_top_1 loading ..." << std::endl;
            capture_left.open(camera_order.at(2));
            std::cout << "cap_top_1 loaded,cap_top_2 loading ..." << capture_left.isOpened() << std::endl;
            capture_right.open(camera_order.at(3));
            std::cout << "cap_top_2 loaded." << capture_right.isOpened() << std::endl;
            isOpen = 0;
            capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
            capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
            capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
            capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);
            capture_left.set(cv::CAP_PROP_FPS, 15);

            std::cout << "over21" << std::endl;
            capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
            capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
            capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
            capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);
            capture_right.set(cv::CAP_PROP_FPS, 15);
        }
        else if (isOpen == 3)
        {
            capture_left.release();
            capture_right.release();
            loadConfigV2(false);
            std::cout << "cap_limit_1 loading ..." << std::endl;
            capture_left.open(camera_order.at(0));
            std::cout << "cap_limit_1 loaded,cap_limit_2 loading ..." << capture_left.isOpened() << std::endl;
            capture_right.open(camera_order.at(1));
            std::cout << "cap_limit_2 loaded." << capture_right.isOpened() << std::endl;
            isOpen = 0;
            capture_left.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
            capture_left.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
            capture_left.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
            capture_left.set(cv::CAP_PROP_BUFFERSIZE, 3);
            capture_left.set(cv::CAP_PROP_FPS, 15);

            std::cout << "over021" << std::endl;
            capture_right.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
            capture_right.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
            capture_right.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
            capture_right.set(cv::CAP_PROP_BUFFERSIZE, 3);
            capture_right.set(cv::CAP_PROP_FPS, 15);
        }
        //Fps.stop();
        //std::cout << "\rTime Image Read =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl;
        ImageCombo frame_read;


        auto fut_read_left = std::async(std::launch::async, [&]() { return capture_left.read(frame_read.left); });
        auto fut_read_right = std::async(std::launch::async, [&]() { return capture_right.read(frame_read.right); });
        auto left_readed = fut_read_left.get();
        auto right_readed = fut_read_right.get();



        if (!left_readed || !right_readed)
        {
            std::cout << "No image." << std::endl;
            break;
        }


        if (isOpen == 1)
        {
            is_detect = true;
        }
        else
        {
            is_detect = false;
            //detector_left1->clearHistoryDatas(size);
            //detector_right1->clearHistoryDatas(size);
#ifdef USING_MULTI_MARKERS
            //detector_left2->clearHistoryDatas(size);
            //detector_right2->clearHistoryDatas(size);
#endif // USING_MULTI_MARKERS

        }

        ImageCornersCombo cb_cnrImg;
        frame_read.left.copyTo(cb_cnrImg.img_left);
        frame_read.right.copyTo(cb_cnrImg.img_right);

        std::cout << "over1" << std::endl;
        //detect marker
        if (is_detect)
        {

            //std::cout<<"go" <<std::endl;
            auto fut_left1 = std::async(std::launch::async, [&]() { return detector_left1->process(frame_read.left); });
            auto fut_right1 = std::async(std::launch::async, [&]() { return detector_right1->process(frame_read.right); });
            auto corners_left_local1 = fut_left1.get();
            auto corners_right_local1 = fut_right1.get();
            //std::cout << "isDetect: "<<is_detect << std::endl;
            cb_cnrImg.cnr_left.push_back(corners_left_local1);
            cb_cnrImg.cnr_right.push_back(corners_right_local1);

#ifdef USING_MULTI_MARKERS
            auto fut_left2 = std::async(std::launch::async, [&]() { return detector_left2->process(frame_read.left); });
            auto fut_right2 = std::async(std::launch::async, [&]() { return detector_right2->process(frame_read.right); });
            auto corners_left_local2 = fut_left2.get();
            auto corners_right_local2 = fut_right2.get();
            //std::cout << "isDetect: " << is_detect << std::endl;
            cb_cnrImg.cnr_left.push_back(corners_left_local2);
            cb_cnrImg.cnr_right.push_back(corners_right_local2);
#endif // USING_MULTI_MARKERS


        }
        else
        {
            cb_cnrImg.cnr_left.push_back(CornersSorted());
            cb_cnrImg.cnr_right.push_back(CornersSorted());
#ifdef USING_MULTI_MARKERS
            cb_cnrImg.cnr_left.push_back(CornersSorted());
            cb_cnrImg.cnr_right.push_back(CornersSorted());
#endif
        }
        std::cout << "over223" << std::endl;

        std::vector<Eigen::Matrix3d> R_res_arr;
        std::vector<Eigen::Vector3d> t_res_arr;
        std::vector<double> cost_res_arr;
        std::vector<PatternResult> left_pc_arr;
        std::vector<PatternResult> right_pc_arr;
        std::vector<Corners> left_coordi_crs_arr;
        std::vector<Corners> right_coordi_crs_arr;
        float poseTwo[16] = { 0 };
        for (int i = 0; i < cb_cnrImg.cnr_left.size(); i++)
        {
            if (!cb_cnrImg.cnr_left.at(i).empty() && !cb_cnrImg.cnr_right.at(i).empty())
            {

                auto [R_res, t_res, cost_res, left_pc_res, right_pc_res, left_coordi_crs, right_coordi_crs] = opti_fun[i].process(cb_cnrImg.cnr_left.at(i),
                    cb_cnrImg.cnr_right.at(i), cb_cnrImg.img_left, cb_cnrImg.img_right);


                //std::cout << "cost: " << cost_res << std::endl;

                R_res_arr.push_back(R_res);
                t_res_arr.push_back(t_res);
                cost_res_arr.push_back(cost_res);
                left_pc_arr.push_back(left_pc_res);
                right_pc_arr.push_back(right_pc_res);
                left_coordi_crs_arr.push_back(left_coordi_crs);
                right_coordi_crs_arr.push_back(right_coordi_crs);
                std::cout << "over213" << std::endl;
                Eigen::Quaterniond R_quat(R_res_arr.at(i));
                float pose[8];
                pose[0] = t_res_arr.at(i)(0);
                pose[1] = t_res_arr.at(i)(1);
                pose[2] = t_res_arr.at(i)(2);
                pose[3] = R_quat.w();
                pose[4] = R_quat.x();
                pose[5] = R_quat.y();
                pose[6] = R_quat.z();
                pose[7] = cost_res_arr.at(i);

                //std::cout << "-->clientMotionPara.send" << std::endl;
                std::cout << pose[0] << "  " << pose[1] << "  " << pose[2] << "  " << pose[3] << "  "
                    << pose[4] << "  " << pose[5] << "  " << pose[6] << "  " << pose[7] << std::endl;

                for (int iw = 0; iw < sizeof(pose) / sizeof(float); iw++)
                    poseTwo[i * 8 + iw] = pose[iw];
            }
            else
            {
                R_res_arr.push_back(Eigen::Matrix3d::Identity());
                t_res_arr.push_back(Eigen::Vector3d::Zero());
                cost_res_arr.push_back(0.1);
                left_pc_arr.push_back(PatternResult());
                right_pc_arr.push_back(PatternResult());
                left_coordi_crs_arr.push_back(Corners());
                right_coordi_crs_arr.push_back(Corners());
                std::cout << "Nothing here !" << std::endl;
            }
        }
        if (is_detect)
        {
            clientMotionPara.send((S8*)&poseTwo, sizeof(poseTwo), 20);
            std::cout << poseTwo[0] << "  " << poseTwo[1] << "  " << poseTwo[2] << "  " << poseTwo[3] << "  "
                << poseTwo[4] << "  " << poseTwo[5] << "  " << poseTwo[6] << "  " << poseTwo[7]
                << poseTwo[8] << "  " << poseTwo[9] << "  " << poseTwo[10] << "  " << poseTwo[11] << "  "
                << poseTwo[12] << "  " << poseTwo[13] << "  " << poseTwo[14] << "  " << poseTwo[15] << std::endl;
        }

        /*show result*/
        cv::Mat img_left_res, img_right_res, joint_res;
        if (cost_res_arr.size() > 0)
        {
            std::tie(img_left_res, img_right_res, joint_res) = showBothResult("left + right", cb_cnrImg, 1, left_pc_arr,
                right_pc_arr, left_coordi_crs_arr, right_coordi_crs_arr, t_res_arr, R_res_arr, cost_res_arr);
            //std::cout << "over193" << std::endl;
            //std::cout << "size: [ " << img_left_res.size()<<" ]" << std::endl;
            bool rc = clientImage.send((S8*)img_left_res.data, 2592 * 1944 * 3 * sizeof(unsigned char), 50);
            //bool rc = clientImage.send((S8*)img_right_res.data, 2592 * 1944 * 3 / 4 * sizeof(unsigned char), 50);

        }
        //std::cout << "over203" << std::endl;

        auto key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            break;
        }
        else if (key == 'w' || key == 'W')
        {
        }
        else if (key == 'r' || key == 'R' || is_detect)
        {
            is_detect = false;

            Eigen::Quaterniond R_quat(R_res_arr.at(0));
            record_pose_data << t_res_arr.at(0)(0) << "\t" << t_res_arr.at(0)(1) << "\t" << t_res_arr.at(0)(2) << "\t" << R_quat.w()
                << "\t" << R_quat.x() << "\t" << R_quat.y() << "\t" << R_quat.z() << "\t" << cost_res_arr.at(0) << std::endl;

            std::cout << record_count << std::endl;
            record_count++;
        }
        else
        {
            ;
        }
        //imshow("  ",joint_res);

    }
    record_pose_data.close();
}

int main()
{
#if 0

    int sizeTem = 1;
    if (SR_FALSE == severVisionCommand.create((S8*)"ToVisionCommand", 512, sizeTem * sizeof(int)))
    {
        printf("Service creation failed!\n");
        printf("Press any key to continue.\n");
        getchar();
        return 0;
    }
    else
    {
        printf("Server creation complete!\n");
    }

    int isOpen(0),before_begin_counts(0);
    U32 len = severVisionCommand.recv((S8*)&isOpen,  sizeof(int), 2);
    while (isOpen != 2)
    {
        printf("--> wait commanding %03d s...\n", before_begin_counts++);
        severVisionCommand.recv((S8*)&isOpen, sizeof(int), 2);
        cv::waitKey(1000);
    }
    printf("--> %d go on...\n", isOpen);
    if (SR_FALSE == clientImage.open((S8*)("ImageFrame")))
    {
        printf("Error connecting to server!\n");
        printf("Press any key to continue.\n");
        getchar();
        return 0;
    }
    printf("Connecting to the clientImage is complete.\n");

    if (SR_FALSE == clientMotionPara.open((S8*)("MotionPara")))
    {
        printf("Error connecting to server!\n");
        printf("Press any key to continue.\n");
        getchar();
        return 0;
    }
    printf("Connecting to the clientMotionPara is complete.\n");
    ////////////////
    //cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
    ////////////////
    std::vector<int> camera_order;
    camera_order = distinguish_camera();
    std::cout << camera_order[0] << camera_order[1] << camera_order[2] << camera_order[3] << std::endl;


    char* buf;
    if ((buf = _getcwd(NULL, 0)) == NULL)
    {
        perror("getcwd error");
    }
    else
    {
        std::cout << buf<<std::endl;
        delete buf;
    }


    if (!loadConfigV2(true))
        return -1;



    //capture_video();
    //test_single_image();
    //test_single_thread();
    test_single_thread_for_multicamera(camera_order);
    //test_multi_thread();
    //test_hdr();
    //show_shape_sensing_result();
#else

/*    std::vector<int> camera_order;
    camera_order = distinguish_camera();
    std::cout << camera_order[0] << camera_order[1] << camera_order[2] << camera_order[3] << std::endl;
   */ //if (!loadConfig())
        //return -1;
    std::vector<int> camera_order;
    camera_order = distinguish_camera();
    std::cout << camera_order[0] << camera_order[1] << camera_order[2] << camera_order[3] << std::endl;


    char* buf;
    if ((buf = _getcwd(NULL, 0)) == NULL)
    {
        perror("getcwd error");
}
    else
    {
        std::cout << buf << std::endl;
        delete buf;
    }

    loadConfigV2(true);
    //capture_video();
    test_single_thread_local(camera_order);
    //test_single_image();
#endif       
    return 0;

}





