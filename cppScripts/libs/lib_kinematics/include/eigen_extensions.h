#pragma once

#include <Eigen/Core>

/* Eigen Matrix6f/Vector6f definition */
namespace Eigen
{
	using Matrix6f = Eigen::Matrix<float, 6, 6, 0, 6, 6>;
	using Vector6f = Eigen::Matrix<float, 6, 1, 0, 6, 1>;
	using Vector6i = Eigen::Matrix<int, 6, 1, 0, 6, 1>;
	using Vector6d = Eigen::Matrix<double, 6, 1, 0, 6, 1>;
}



