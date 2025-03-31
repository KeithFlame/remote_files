#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "eigen_extensions.h"
#include <Eigen/Geometry>
#include <vector>

namespace Keith {
	const double PI = 3.14159265358979;
	const double PI_2 = PI / 2;
	const double PI_4 = PI / 4;
	template<typename T>
	T deg2rad(T deg)
	{
		return deg * PI / 180.f;
	}

	template <typename T>
	T rad2deg(T rad)
	{
		return rad * 180.f / PI;
	}

	Eigen::Matrix3d skew_matrix(Eigen::Vector3d vec);
	Eigen::Vector3d invSkewMatrix_keith(Eigen::Matrix3d mat1, Eigen::Matrix3d mat2);
	Eigen::MatrixXd pinv(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& inMatrix);
	Eigen::MatrixXd invScale(const Eigen::MatrixXd& inMatrix);
	Eigen::Matrix4d fromQuat2T(Eigen::Vector7d vec);
	Eigen::Matrix4d fromX2T(const Eigen::Vector6d& X);
	Eigen::Vector6d fromT2X(const Eigen::Matrix4d& X);
	Eigen::Vector6d calcDeviationFrom2T(const Eigen::Matrix4d& cur_T, const Eigen::Matrix4d& tar_T);
	Eigen::Matrix3d axang2rotm(Eigen::Vector3d vec);
	Eigen::Vector4d getQuat(std::vector<Eigen::Vector4d> quat, int quatSize);
	Eigen::Vector4d getPose(std::vector<Eigen::Vector4d> pose, int quatSize);
	Eigen::Vector6d Psi2Curvature_keith(const Eigen::Vector6d& psi, const Eigen::Vector3d& L12Zeta);
	Eigen::Vector6d Curvature2Psi_keith(const Eigen::Vector6d& u,
		const Eigen::Vector3d& L12Zeta, const double& l);
	Eigen::Vector6d Curvature2Actuation_keith(const Eigen::Vector6d& u, const Eigen::Matrix46d& Gc);
	Eigen::Vector6d Actuation2Curvature_keith(const Eigen::Vector6d& qa, const Eigen::Matrix46d& Gc);

	Eigen::Vector6d Actuation2Psi_keith(const Eigen::Vector6d& qa, const Eigen::Vector3d& L12Zeta, const Eigen::Matrix46d& Gc);
	Eigen::Vector6d Psi2Actuation_keith(const Eigen::Vector6d& psi, const Eigen::Vector3d& L12Zeta, const Eigen::Matrix46d& Gc);

	Eigen::Matrix4d getSegmentPose(const double L, double theta1, double delta1);
	Eigen::Matrix4d getForwardKinematics(const Eigen::Vector6d& psi, const Eigen::Vector4d& L1r2g, const double zeta);
}
