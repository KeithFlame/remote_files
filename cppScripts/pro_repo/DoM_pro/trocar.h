#pragma once
#include "eigen_extensions.h"
struct Trocar {
	//Eigen::Matrix4d initial_pose;
	Eigen::Vector2d curved_path;
	Eigen::Vector2d feeding_phi;
};