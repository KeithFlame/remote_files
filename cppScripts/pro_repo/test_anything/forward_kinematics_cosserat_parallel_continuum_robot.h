#pragma once
#ifndef FORWARD_KINEMATICS_COSSERAT_PARALLEL_CONTINUUM_ROBOT_H_
#define FORWARD_KINEMATICS_COSSERAT_PARALLEL_CONTINUUM_ROBOT_H_
#include <Eigen/Core>
#include "two_segment_multi_backbone_continuum_robot.h"
class FKCoPCR {
public:
	FKCoPCR();
	~FKCoPCR() {};
	Eigen::VectorXd shootOptimization(Eigen::VectorXd qa, Eigen::Matrix4d T_base, Eigen::Matrix4d T_base2, 
		Eigen::VectorXd FMfm, TwoSegmentMultiBackboneContinuumRobot tcr1,
		TwoSegmentMultiBackboneContinuumRobot tcr2, Eigen::VectorXd ksi, Eigen::VectorXd& yend);
	Eigen::VectorXd shootMethod(Eigen::VectorXd guess, Eigen::VectorXd qa, Eigen::Matrix4d T_base, Eigen::Matrix4d T_base2, 
		Eigen::VectorXd FMfm,TwoSegmentMultiBackboneContinuumRobot tcr1, TwoSegmentMultiBackboneContinuumRobot tcr2,
		Eigen::VectorXd ksi, Eigen::VectorXd& yend);
	Eigen::VectorXd shootOneRobot(Eigen::VectorXd y, Eigen::VectorXd fm, Eigen::MatrixXd v, bool is_main,
		TwoSegmentMultiBackboneContinuumRobot tcr, Eigen::VectorXd ksi, Eigen::VectorXd& qaaa);
	Eigen::VectorXd integrateCosseratRod(Eigen::VectorXd y0, Eigen::MatrixXd v, int segIdx, TwoSegmentMultiBackboneContinuumRobot tcr, 
		Eigen::VectorXd fm, Eigen::VectorXd ksi, Eigen::MatrixXd& y, Eigen::MatrixXd& U);
	Eigen::VectorXd optimizeStiffness(Eigen::VectorXd pose, Eigen::VectorXd qa, Eigen::Matrix4d T_base, Eigen::Matrix4d T_base2,
		Eigen::VectorXd FMfm, TwoSegmentMultiBackboneContinuumRobot tcr1,
		TwoSegmentMultiBackboneContinuumRobot tcr2, Eigen::VectorXd ksi, Eigen::VectorXd& yend);
	Eigen::VectorXd optimizeActuation(Eigen::VectorXd pose, Eigen::VectorXd qa, Eigen::Matrix4d T_base, Eigen::Matrix4d T_base2,
		Eigen::VectorXd FMfm, TwoSegmentMultiBackboneContinuumRobot tcr1,
		TwoSegmentMultiBackboneContinuumRobot tcr2, Eigen::VectorXd ksi, Eigen::VectorXd& yend);
	void setGuess0(Eigen::VectorXd);
	Eigen::VectorXd getPosefromVec6(Eigen::VectorXd);
	Eigen::VectorXd getPoseDeviationfromVec6(Eigen::VectorXd, Eigen::VectorXd);

private:
	Eigen::VectorXd guess0;
};
#endif // !FORWARD_KINEMATICS_COSSERAT_PARALLEL_CONTINUUM_ROBOT_H_
