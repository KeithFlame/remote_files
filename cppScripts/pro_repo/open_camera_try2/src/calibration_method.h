#pragma once
#include <Eigen/Core>
#include "model_parameter.h"
#include <vector>


class CalibrationMethod {
public:
	CalibrationMethod();
	~CalibrationMethod();
	void setInitValue(const Eigen::VectorXd&);
	const Eigen::VectorXd getFinalValue();
private:
	Eigen::Vector3d JvLCouple(double theta, double delta, double L1, double L0, double zeta);
	Eigen::MatrixXd JvSegCouple(double theta, double delta, double L1, double L0, double zeta);
	Eigen::Matrix3d S(const Eigen::Vector3d& p);
	Eigen::Matrix3d Expm(const Eigen::Vector3d& u);
	Eigen::Vector3d calcSegP(double theta, double delta, double L);
	Eigen::Matrix3d calcSegR(double theta, double delta);
    Kmp forward(const Eigen::VectorXd& psi, const Eigen::VectorXd& seg, double zeta, double d, double ga);
    Eigen::MatrixXd calcJacobian(const Eigen::VectorXd& psi, const Eigen::VectorXd& seg, double zeta, double d, double ga);
	Eigen::VectorXd calcPsiFromQ(const Eigen::VectorXd& qa, const Eigen::VectorXd& psi_in, const Eigen::VectorXd& x);
	Eigen::VectorXd limitPsiNum(Eigen::VectorXd psi);
	Eigen::VectorXd calcQFrompsi(Eigen::VectorXd psi, Eigen::VectorXd x);
	Eigen::MatrixXd pinv(const Eigen::MatrixXd& mat);
	Eigen::Matrix3d EulerToRotMat(const Eigen::Vector3d& euler);
	std::vector<Eigen::Matrix4d> forwardCalib(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Psi_ref, const MP& x);



private:
	MP mp;
};

