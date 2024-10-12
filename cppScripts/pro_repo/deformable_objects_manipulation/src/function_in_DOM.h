#pragma once
#include <Eigen/Core>
#include<Eigen/SVD>
#include "eigen_extensions.h"
#include "SurgicalContinuumManipulator.h"

namespace Keith {
	Eigen::Matrix3d skew_matrix(Eigen::Vector3d vec);
	Eigen::Vector3d invSkewMatrix_keith(Eigen::Matrix3d mat1, Eigen::Matrix3d mat2);
	Eigen::MatrixXd pinv(const Eigen::MatrixXd inMatrix);
	Eigen::Matrix4d fromQuat2T(Eigen::Vector7d vec);
	Eigen::Vector12d shootingStiffOpt_keith(Eigen::Vector34d& Guess_sf,
		Eigen::Vector12d qa, Eigen::Matrix4d Tc, SurgicalContinuumManipulator& MBP1,
		SurgicalContinuumManipulator& MBP2, Eigen::Vector2d err);
	Eigen::Vector12d shootingInvOpt_keith(Eigen::Vector24d& Guess_inv,
		Eigen::Matrix4d target, Eigen::Vector12d Ksi, SurgicalContinuumManipulator& MBP1,
		SurgicalContinuumManipulator& MBP2, Eigen::Vector2d err, float Energy);
	Eigen::Matrix4d shootingFkOpt_keith(Eigen::Vector20d& Guess_fk,
		Eigen::Vector12d qa,  SurgicalContinuumManipulator& MBP1,
		SurgicalContinuumManipulator& MBP2, Eigen::Vector2d err);
	void setQA(Eigen::Vector12d QA, Eigen::Vector12d& qa);
	void shootingStiffOpt_keith(Eigen::Vector34d& Guess_sf, Eigen::Vector12d qa,
		Eigen::Matrix4d Tc, SurgicalContinuumManipulator& MBP1, SurgicalContinuumManipulator& MBP2,
		Eigen::Vector12d& Ksi);
	void intCosserat(Eigen::Vector22d& y, Eigen::Matrix34d& v, Eigen::Index seg_index, 
		Eigen::Vector5d ksi, SurgicalContinuumManipulator& MBP);
}