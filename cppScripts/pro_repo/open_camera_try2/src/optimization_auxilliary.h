#pragma once
#pragma once
#include <opencv2/opencv.hpp>
#include "math_extensions.h"
#include <Eigen/Core>

using Corner = cv::Point_<float>;
using PixelType = float;
const PixelType PI = CV_PI;
struct CostFunctorDistortion {
	CostFunctorDistortion(const PixelType& x_d, const PixelType& y_d,
		const double& k1, const double& k2, const double& p1, const double& p2)
		:x_d(x_d), y_d(y_d), k1(k1), k2(k2), p1(p1), p2(p2) {}

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

struct ReprojectionErrorNonePlane {
	ReprojectionErrorNonePlane(const Eigen::Matrix3d& R,
		const Eigen::Vector3d& t,
		const Eigen::Matrix3d& A_cam,
		const Corner& m,
		const Eigen::Vector3d& M)
		: R(R)
		, t(t)
		, A_cam(A_cam)
		, m(m)
		, M(M)
	{}

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {
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

		return true;
	}

	const Eigen::Matrix3d R;
	const Eigen::Vector3d t;
	const Eigen::Matrix3d A_cam;
	const Corner m;
	const Eigen::Vector3d M;
};