#pragma once
#ifndef KINEMATICS_ERROR_H_
#define KINEMATICS_ERROR_H_
#include <Eigen/Core>
#include "eigen_extensions.h"
#include "tool_component.h"



class KinematicsError
{
public:
	static bool isSmaller(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform, const PoseError& threshold);
	static bool isReached(const Eigen::Vector6i& lhs, const Eigen::Vector6i& rhs);

	static Eigen::Vector3f calcErrorPos(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform);
	static Eigen::Matrix3f calcErrorRot(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform);
	static float calcErrorDistance(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform);
	static float calcErrorAngle(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform);
	static Eigen::Vector3f calcErrorAxis(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform);

private:
	KinematicsError();
};

#endif // !KINEMATICS_ERROR_H_