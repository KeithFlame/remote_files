#pragma once
#include <string>
#include "eigen_extensions.h"

namespace auxiliary_func {
	int isExists(const std::string file_path);
	std::string removeAngleBrackets(const std::string& input);
	std::string fromVec2String(const Eigen::Vector7d& input);
	template <typename T>
	T deg2rad(T a) {
		T b = a * M_PI / 180;
		return b;
	}

	template <typename T>
	T rad2deg(T a) {
		T b = a / M_PI * 180;
		return b;
	}
}