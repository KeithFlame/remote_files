#pragma once
#include <Eigen/dense>
#include "two_segment_tool.h"
#include "space_variables.h"
#include "math_extensions.h"

class CoupledActuation
{
public:
	CoupledActuation();
	CoupledActuation(TwoSegmentTool);
	~CoupledActuation();
	const Eigen::Matrix46d getGc(const double);
	const Eigen::Vector6d Psi2Curvature_keith(const Eigen::Vector6d& psi, const Eigen::Vector3d& L12Zeta);
	const Eigen::Vector6d Curvature2Psi_keith(const Eigen::Vector6d& u,
		const Eigen::Vector3d& L12Zeta, const double& l);
	const Eigen::Vector6d Curvature2Actuation_keith(const Eigen::Vector6d& u, const Eigen::Matrix46d& Gc);
	const Eigen::Vector6d Actuation2Curvature_keith(const Eigen::Vector6d& qa, const Eigen::Matrix46d& Gc);

	const Eigen::Vector6d Actuation2Psi_keith(const Eigen::Vector6d& qa, const Eigen::Vector3d& L12Zeta, const Eigen::Matrix46d& Gc);
	const Eigen::Vector6d Psi2Actuation_keith(const Eigen::Vector6d& psi, const Eigen::Vector3d& L12Zeta, const Eigen::Matrix46d& Gc);

	const Eigen::Matrix4d getSegmentPose(const double L, double theta1, double delta1);
	const Eigen::Matrix4d getForwardKinematics(const Eigen::Vector6d& psi, const Eigen::Vector4d& L1r2g, const double zeta);
private:
	Eigen::Matrix46d Gc;
	TwoSegmentTool* tool = nullptr;
	double feeding_length;
	double l1;
	double ls;
};

