#pragma once
#include <Eigen/Core>
#include <vector>

extern Eigen::Matrix3d left_camera_intrinsic;
extern Eigen::Vector4d left_camera_distortion;
extern Eigen::Matrix3d right_camera_intrinsic;
extern Eigen::Vector4d right_camera_distortion;

extern Eigen::Matrix3d rect_left_camera;
extern Eigen::Matrix3d rect_right_camera;
extern Eigen::Matrix3d A_cam;
extern double b_dis;

bool loadConfig();

Eigen::Matrix3d getMatrix3dParameter(std::string matName, bool isUp=1);

Eigen::Vector4d getVector4dParameter(std::string vecName, bool isUp=1);

std::string getParaPath();

bool loadConfigV2(bool normal=true);

std::vector<std::string> split(const std::string& s, const std::string& seperator);