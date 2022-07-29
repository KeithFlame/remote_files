#ifndef KINEMATICS_BASE_H_
#define KINEMATICS_BASE_H_
#include "eigen_extensions.h"

struct SingleSegment
{
	float theta;
	float delta;
	float L;
	float L0;
};

struct KinematicsParameter
{
	// Coordinate '01' equals 's' in 类图参数说明.docx
	// pos
	Eigen::Vector3f P0b_01;
	Eigen::Vector3f P01_1b;
	Eigen::Vector3f P1b_1e;
	Eigen::Vector3f P1e_2b;
	Eigen::Vector3f P2b_2e;
	Eigen::Vector3f P2e_g;

	// rot
	Eigen::Matrix3f R0b_01;
	Eigen::Matrix3f R01_1b;
	Eigen::Matrix3f R1b_1e;
	Eigen::Matrix3f R1e_2b;
	Eigen::Matrix3f R2b_2e;
	Eigen::Matrix3f R2e_g;

	// get 
	Eigen::Vector3f P2b_O2e_Og() const
	{
		return R2b_2e * P2e_g;
	}

	Eigen::Vector3f P1b_O1e_Og() const
	{
		return R1b_1e * (P1e_2b + R1e_2b * (P2b_2e + P2b_O2e_Og()));
	}

	Eigen::Vector3f P0b_O1b_Og() const
	{
		return R0b_01 * R01_1b * (P1b_1e + P1b_O1e_Og());
	}

	Eigen::Matrix3f R0b_1b() const
	{
		return R0b_01 * R01_1b;
	}

	Eigen::Matrix3f R0b_2b() const
	{
		return R0b_1b() * R1b_1e * R1e_2b;
	}
};

#endif /* KINEMATICS_BASE_H_ */
