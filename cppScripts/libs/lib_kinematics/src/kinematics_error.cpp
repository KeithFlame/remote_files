#include "kinematics_error.h"
#include <Eigen/Geometry>

namespace
{
	inline Eigen::AngleAxisf calcErrorAngleAxis(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform)
	{
		const Eigen::Matrix3f error_rot = KinematicsError::calcErrorRot(target_transform, current_transform);
		Eigen::AngleAxisf error_rot_angleaxis;
		error_rot_angleaxis.fromRotationMatrix(error_rot);

		return error_rot_angleaxis;
	}
}


bool KinematicsError::isSmaller(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform, const PoseError& threshold)
{
	const float error_pos = calcErrorDistance(target_transform, current_transform);
	const float error_angle = calcErrorAngle(target_transform, current_transform);

	return (error_pos < threshold.epsilon_position) && (abs(error_angle) < threshold.epsilon_direction);
	
}


Eigen::Vector3f KinematicsError::calcErrorPos(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform)
{
	const Eigen::Vector3f target_pos = target_transform.block<3, 1>(0, 3);
	const Eigen::Vector3f current_pos = current_transform.block<3, 1>(0, 3);

	return target_pos - current_pos;
}

Eigen::Matrix3f KinematicsError::calcErrorRot(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform)
{
	const Eigen::Matrix3f target_rot = target_transform.block<3, 3>(0, 0);
	const Eigen::Matrix3f current_rot = current_transform.block<3, 3>(0, 0);

	return target_rot * current_rot.transpose();
}

float KinematicsError::calcErrorDistance(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform)
{
	return calcErrorPos(target_transform, current_transform).norm();
}

float KinematicsError::calcErrorAngle(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform)
{
	return calcErrorAngleAxis(target_transform, current_transform).angle();
}

Eigen::Vector3f KinematicsError::calcErrorAxis(const Eigen::Matrix4f& target_transform, const Eigen::Matrix4f& current_transform)
{
	return calcErrorAngleAxis(target_transform, current_transform).axis();
}

bool KinematicsError::isReached(const Eigen::Vector6i& lhs, const Eigen::Vector6i& rhs)
{
	if (abs(lhs(0) - rhs(0)) > 30000)
		return false;

	for (int i = 1; i < 6; ++i)
		if (abs(lhs(i) - rhs(i)) > 8000)
			return false;

	return true;
}
