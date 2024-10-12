#pragma once
#include <Eigen/Core>
#include <Eigen/SVD>
#include "eigen_extensions.h"
#include <Eigen/Geometry>

namespace Keith {
	constexpr double PI = 3.1416;
	constexpr double PI_2 = PI / 2;
	constexpr double PI_4 = PI / 4;
	template<typename T>
	T deg2rad(T deg)
	{
		return deg * PI / 180f;
	}

	template <typename T>
	T rad2deg(T rad)
	{
		return rad * 180f / PI;
	}

	Eigen::Matrix3d skew_matrix(Eigen::Vector3d vec);
	Eigen::Vector3d invSkewMatrix_keith(Eigen::Matrix3d mat1, Eigen::Matrix3d mat2);
	Eigen::MatrixXd pinv(const Eigen::MatrixXd inMatrix);
	Eigen::Matrix4d fromQuat2T(Eigen::Vector7d vec);


}
