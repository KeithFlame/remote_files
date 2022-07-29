#pragma once
#ifndef INSTRUMENT_H_
#define INSTRUMENT_H_
#include "configration_space.h"
#include "joint_space.h"
#include "tool_component.h"
#include "kinematics_base.h"
#include "kinematics_error.h"
#include <vector>

class Instrument {
public:
	Instrument() = default;
	Instrument(ToolArm);
	Instrument(Instrument&) = default;
	void forwardKinematics(ConfigSpace, Eigen::Matrix4f&, bool);
	void forwardKinematics(JointSpace, Eigen::Matrix4f&, bool);
	bool inverseKinematics(Eigen::Matrix4f,  ConfigSpace&, bool);
	bool inverseKinematics(Eigen::Matrix4f,  JointSpace&, bool);
	bool inverseKinematicsPro(Eigen::Matrix4f, ConfigSpace&, bool);
	bool inverseKinematicsPro(Eigen::Matrix4f, JointSpace&, bool);
	void fromPsi2Actuation(ConfigSpace, JointSpace&);
	void fromActuation2Psi(JointSpace, ConfigSpace&);

private:
	Eigen::Matrix4f calcSingleSegment(SingleSegment);
	void dealConfigurationSpaceValue(ConfigSpace, std::vector<SingleSegment>&, bool);
	void calcCoupledMatrix(ConfigSpace, Eigen::MatrixXf&);
	Eigen::MatrixXf calcPseudoInverse(const Eigen::MatrixXf);
	bool doInverseKinematics(Eigen::Matrix4f, ConfigSpace&, bool);
	float calcClampAngleFromActuation(float);
	float calcRotationAngleFromActuation(float);
	Eigen::MatrixXf calcJacobian(const ConfigSpace, bool);
	Eigen::Vector3f calcJvRL();
	Eigen::Vector3f calcJwRL();
	Eigen::Vector3f calcJvPhi();
	Eigen::Vector3f calcJwPhi();
	Eigen::MatrixXf calcJv1(const ConfigSpace);
	Eigen::MatrixXf calcJw1(const ConfigSpace);
	Eigen::MatrixXf calcJv2(const ConfigSpace);
	Eigen::MatrixXf calcJw2(const ConfigSpace);
	Eigen::Vector6f calcXdot(const Eigen::Matrix4f&, const Eigen::Matrix4f&, bool flag = false);
	Eigen::Vector6f calcPsidot(const Eigen::Vector6f&, const Eigen::MatrixXf&);
	void calcPsiFromPsidot(ConfigSpace&, const Eigen::Vector6f);

private:
	ToolArm tool;
	KinematicsParameter kp;
};
#endif // !INSTRUMENT_H_
