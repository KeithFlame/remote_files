#include "Jacobian_Broyden.h"
#include<Eigen/SVD>
#include <iostream>

#define JB_PI 3.141592654f

Jacobian_Broyden::Jacobian_Broyden() :
	alpha_J(0.6),
	step(0.01f),
	error_tar(0.5f),
	error(100.f),
	is_direction(true)
{
	psi_cur = Eigen::VectorXf::Zero(6, 1);
	T_cur = Eigen::VectorXf::Zero(7, 1);
	JB_without_dir = Eigen::MatrixXf::Identity(3, 6);
	JB = Eigen::MatrixXf::Identity(6, 6);
	T_velocity_limit = Eigen::VectorXf::Zero(6, 1);
	T_velocity_limit << 100.f, 100.f, 100.f, 90.f, 90.f, 90.f;
	psi_velocity_limit = Eigen::VectorXf::Zero(6, 1);
	psi_velocity_limit << 0.1f, 2.f, 0.1f, 0.1f, 0.1f, 0.1f;
	Target = Eigen::VectorXf::Zero(7, 1);
}

Jacobian_Broyden::Jacobian_Broyden(Eigen::VectorXf vec) :
	alpha_J(0.6),
	step(0.01f),
	error_tar(0.5f),
	error(100.f),
	is_direction(true)
{
	psi_cur = Eigen::VectorXf::Zero(6, 1);
	T_cur = Eigen::VectorXf::Zero(7, 1);
	JB_without_dir = Eigen::MatrixXf::Identity(3, 6);
	JB = Eigen::MatrixXf::Identity(6, 6);
	T_velocity_limit = Eigen::VectorXf::Zero(6, 1);
	T_velocity_limit << 100.f, 100.f, 100.f, 90.f, 90.f, 90.f;
	psi_velocity_limit = Eigen::VectorXf::Zero(6, 1);
	psi_velocity_limit << 0.1f, 2.f, 0.1f, 0.1f, 0.1f, 0.1f;
	Eigen::Vector3f ea(vec(3), vec(4), vec(5));
	float angle = ea.norm();
	Eigen::Vector3f axis = ea / angle;
	Eigen::AngleAxisf axang(angle / 180.f * JB_PI, axis);
	Eigen::Quaternionf quat(axang);
	Target = Eigen::VectorXf::Zero(7, 1);
	Target << vec.head(3), quat.w(), quat.x(), quat.y(), quat.z();
}

void Jacobian_Broyden::calcJacobian_Broyden(Eigen::VectorXf psi, Eigen::VectorXf T)
{
	Eigen::VectorXf dT = recombineT(T, T_cur);
	Eigen::VectorXf dPsi_v(6, 1);
	float r2d = 180.f / JB_PI;
	dPsi_v << r2d, 1.f, r2d, r2d, r2d, r2d;
	Eigen::VectorXf dPsi = (psi - psi_cur).array() * dPsi_v.array();
	if (is_direction)
		JB = JB + alpha_J * (dT - JB * dPsi) / (dPsi.transpose() * dPsi) * dPsi.transpose();
	else
		JB_without_dir = JB_without_dir + alpha_J * (dT - JB_without_dir * dPsi)
		/ (dPsi.transpose() * dPsi) * dPsi.transpose();

	T_cur = T;
	psi_cur = psi;
}

Eigen::VectorXf Jacobian_Broyden::recombineT(Eigen::VectorXf T1, Eigen::VectorXf T2)
{
	Eigen::Vector3f vec1 = T1.head(3);
	Eigen::Vector3f vec2 = T2.head(3);
	Eigen::Vector3f vec0 = vec1 - vec2;
	if (is_direction)
	{
		Eigen::Quaternionf quat1(T1(3), T1(4), T1(5), T1(6)), quat2(T2(3), T2(4), T2(5), T2(6));
		quat1.normalize();
		quat2.normalize();
		Eigen::AngleAxisf axang(quat2.toRotationMatrix().transpose() * quat1.toRotationMatrix());
		Eigen::VectorXf res(6,1);
		res << vec0, axang.axis()* axang.angle() * 180.f / JB_PI;
		return res;
	}
	else
	{
		return vec0;
	}

}

Eigen::VectorXf Jacobian_Broyden::calcNextPsi(Eigen::VectorXf psi, Eigen::VectorXf T)
{
	calcJacobian_Broyden(psi, T);
	Eigen::VectorXf dT(T_cur);
	Eigen::VectorXf psi_out;
	dT = recombineT(T, Target);
	float k = 1.f;
	//Eigen::VectorXf dTv = dT.head(3).array() / dT.head(3).norm() * T_velocity_limit.head(3).array();
	//Eigen::VectorXf dTw = dT.tail(3).array() / dT.tail(3).norm() * T_velocity_limit.tail(3).array();
	//dT << dTv, dTw;

	
	
	//std::cout << "dT_limit: " << dT.transpose() << std::endl;
	Eigen::MatrixXf J_66(Eigen::MatrixXf::Identity(6, 6));
	Eigen::MatrixXf J_63(Eigen::MatrixXf::Identity(6, 3));
	float cterion = dT.head(3).dot((Target - T).head(3))/(Target - T).head(3).norm() / dT.head(3).norm();
	if (cterion < -0.9f)
		k = 80.f;
	else if (cterion < -0.5f)
		k = 30.f;


	if (is_direction)
	{
		pinv(J_66, JB.transpose() * JB + 1e-7f * J_66);
		J_66 = Eigen::MatrixXf::Identity(6, 6) * J_66 * JB.transpose();
		psi_out = k * step * J_66 * dT * JB_PI / 180.f;
	}
	else
	{
		pinv(J_66, JB_without_dir.transpose() * JB_without_dir + 1e-7f * J_66);
		J_63 = Eigen::MatrixXf::Identity(6, 6) * J_66 * JB_without_dir.transpose();
		psi_out = k * step * J_63 * dT * JB_PI / 180.f;
		
	}
	psi_out(1) = psi_out(1) / JB_PI * 180.f;
	//std::cout << "psi_out: " << psi_out.transpose() << std::endl;

	limitPsiOutput(psi_out);
	//std::cout << "psi_out_limit: " << psi_out.transpose() << std::endl << std::endl;
	return psi - psi_out;
}

void Jacobian_Broyden::checkT(bool flag)
{
	is_direction = flag;
}

void Jacobian_Broyden::getInitPsi(Eigen::VectorXf psi)
{
	psi_cur = psi;
}

void Jacobian_Broyden::getInitPose(Eigen::VectorXf T)
{
	T_cur = T;
}

void Jacobian_Broyden::getTarget(Eigen::VectorXf T)
{
	Eigen::Vector3f ea(T(3), T(4), T(5));
	Eigen::AngleAxisf ra(Eigen::AngleAxisf(ea(2), Eigen::Vector3f::UnitX()));
	Eigen::AngleAxisf pa(Eigen::AngleAxisf(ea(1), Eigen::Vector3f::UnitY()));
	Eigen::AngleAxisf ya(Eigen::AngleAxisf(ea(0), Eigen::Vector3f::UnitZ()));
	Eigen::Quaternionf quat = ya * pa * ra;
	Target << T.head(3), quat.w(), quat.x(), quat.y(), quat.z();
}


bool Jacobian_Broyden::is_convergent()
{
	
	Eigen::Quaternionf qt(Target(3), Target(4), Target(5), Target(6)), qc(T_cur(3), T_cur(4), T_cur(5), T_cur(6));
	Eigen::AngleAxisf axang(qt.toRotationMatrix().transpose()*qc.toRotationMatrix());
	error = (Target - T_cur).head(3).norm() + axang.angle() * 180.f / JB_PI;
	//std::cout << "Target: " << Target.transpose() << std::endl;
	//std::cout << "T: " << T.transpose() << std::endl;
	//
	//std::cout << "psi_cur: " << psi_cur.transpose() << std::endl;
	//std::cout << "T_cur: " << T_cur.transpose() << std::endl;
	//std::cout << "angle: " << axang.angle() * 180.f / JB_PI << std::endl;
	//std::cout << "position: " << (Target - T).head(3).norm() << std::endl;
	//std::cout << "error: " << error << std::endl << std::endl << std::endl << std::endl;
	std::cout << "cur_pose_jb: " << T_cur.transpose() << std::endl;
	std::cout << "cur_psi_jb: " << psi_cur.transpose() << std::endl;
	std::cout << std::endl << "->error: " << error << std::endl << std::endl << std::endl << std::endl;
	if (error < error_tar)
		return true;
	else
		return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//伪逆矩阵(Moore-Penrose pseudoinverse)A定义：
//A+=VD+UT,其中，U，D和V是矩阵A奇异值分解后得到的矩阵。对角矩阵D的伪逆D+是非零元素取倒数之后再转置得到的。
//
void Jacobian_Broyden::pinv(Eigen::MatrixXf& outMatrix, const Eigen::MatrixXf& inMatrix)
{
	double pinvtoler = 1.e-6; // choose your tolerance wisely!
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(inMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXf singularValues_inv = svd.singularValues();
	Eigen::VectorXf sv = svd.singularValues();
	for (Eigen::Index i = 0; i < svd.cols(); ++i)
	{
		if (sv(i) > pinvtoler)
		{
			singularValues_inv(i) = 1.f / sv(i);
		}
		else
		{
			singularValues_inv(i) = 0;
		}
	}
	outMatrix = (svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose());
	//return outMatrix;
}

void Jacobian_Broyden::limitPsiOutput(Eigen::VectorXf& psi)
{
	for (size_t i = 0; i < psi.size(); i++)
	{
		if (psi(i) > psi_velocity_limit(i))
			psi(i) = psi_velocity_limit(i);
		else if (psi(i) < -psi_velocity_limit(i))
			psi(i) = -psi_velocity_limit(i);
	}
}
