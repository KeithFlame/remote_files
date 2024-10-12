#include "coupled_actuation.h"

CoupledActuation::CoupledActuation()
{
	Gc = Eigen::MatrixXd::Identity(4, 6);
	feeding_length = -10.0;
	l1 = 0.0;
	ls = 0.0;
}

CoupledActuation::CoupledActuation(TwoSegmentTool tool)
{
	this->tool = &tool;
	Gc = Eigen::MatrixXd::Identity(4, 6);
	feeding_length = -10.0;
	l1 = 0.0;
	ls = 0.0;
}

CoupledActuation::~CoupledActuation()
{
	delete tool;
}

const Eigen::Matrix46d CoupledActuation::getGc(const double feeding_length)
{
	if (this->feeding_length == feeding_length) {
		return Gc;
	}
	else {
		if (tool->getL1() > feeding_length) {
			l1 = feeding_length;
			ls = 0.0;
		}
		else {
			l1 = tool->getL1();
			ls = feeding_length - l1;
		}
	}
	Eigen::Matrix46d dGamma = Eigen::Matrix46d::Identity();
	Eigen::Matrix4d Ell = Eigen::Matrix4d::Identity();
	Eigen::MatrixXd THETA(6, 4);
	Eigen::Matrix6d Ka = Eigen::Matrix6d::Identity();
	Eigen::Vector3d zeta12 = tool->getK();
	Eigen::Vector4d L1r2g = tool->getL1r2g();
	dGamma.topLeftCorner(4, 3) = (l1 + ls * zeta12(0)) * tool->Q1.transpose();
	dGamma.bottomRightCorner(2, 3) = L1r2g(2) * tool->Q2.transpose();

	Eigen::Vector2d bl= tool->getBackboneLength();
	Ell.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity() * (bl(0));
	Ell.bottomRightCorner(2, 2) = Eigen::Matrix2d::Identity() * (bl(1));

	THETA.topLeftCorner(3, 4) = -tool->Ke1(2, 2) * tool->Q1;
	THETA.topRightCorner(3, 2) = Eigen::Matrix32d::Zero();
	THETA.bottomLeftCorner(3, 2) = Eigen::Matrix32d::Zero();
	THETA.bottomRightCorner(3, 2) = -tool->Ke2(2, 2) * tool->Q2;

	Ka.topLeftCorner(3, 3) = 4 * tool->Kb1 + 16 * tool->Kb2 + zeta12(1) * tool->Kb1;
	Ka.topRightCorner(3, 3) = -16 * tool->Kb2 - zeta12(2) * tool->Kb1;
	Ka.bottomRightCorner(3, 3) = 16 * tool->Kb2 + zeta12(2) * tool->Kb1;

	Gc = dGamma - Ell * Keith::pinv(THETA) / 2 * Ka;
	return Gc;
}