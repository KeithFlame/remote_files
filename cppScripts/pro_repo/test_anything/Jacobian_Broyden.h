#pragma once
#ifndef JACOBIAN_BROYDEN_H_
#define JACOBIAN_BROYDEN_H_
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


class Jacobian_Broyden
{
public:
	Jacobian_Broyden();
	Jacobian_Broyden(Eigen::VectorXf);
	void getInitPsi(Eigen::VectorXf psi);
	void getInitPose(Eigen::VectorXf T);
	Eigen::VectorXf calcNextPsi(Eigen::VectorXf psi, Eigen::VectorXf T);
	bool is_convergent();
	void getTarget(Eigen::VectorXf);
private:
	void calcJacobian_Broyden(Eigen::VectorXf psi, Eigen::VectorXf T);
	Eigen::VectorXf recombineT(Eigen::VectorXf T1, Eigen::VectorXf T2);
	void checkT(bool);
	void pinv(Eigen::MatrixXf& outMatrix, const Eigen::MatrixXf& inMatrix);
	void limitPsiOutput(Eigen::VectorXf& psi);
public:
	Eigen::VectorXf psi_cur;
	Eigen::VectorXf T_cur;
	Eigen::VectorXf T_velocity_limit;
	Eigen::VectorXf psi_velocity_limit;
	Eigen::MatrixXf JB;
	Eigen::MatrixXf JB_without_dir;
	Eigen::VectorXf Target;
	bool is_direction;
	float alpha_J;
	float step;
	float error;
	float error_tar;

};
#endif 