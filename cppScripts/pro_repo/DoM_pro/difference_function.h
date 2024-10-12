#pragma once
#include <QObject>
#include <QThread>
#include "math_extensions.h"
#include "SurgicalContinuumManipulator.h"
#include <vector>

//#include <iostream>
//#include "trocar.h"

const int ARM_NUM = 3;

class DifferenceMethod : public QObject
{
	Q_OBJECT

public:
	DifferenceMethod(QObject* parent = nullptr);
	inline bool integrateHalfStiffnessSegment(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, double& energy, const Eigen::Vector4d& vec_v,
		const double& ksi, const int arm_num = 0) {
		if (manipulators[arm_num]->getLso() < 1e-2)
			return false;
		Eigen::Matrix3d Kb1 = manipulators[arm_num]->getKb1();
		Eigen::Matrix3d Ke1 = manipulators[arm_num]->getKe1();
		Eigen::Matrix3d Kb2 = manipulators[arm_num]->getKb2();
		Eigen::Matrix3d Ke2 = manipulators[arm_num]->getKe2();
		Eigen::Matrix34d Q = manipulators[arm_num]->getQ1();
		//double K1 = ksi;
		double zeta = manipulators[arm_num]->getZeta();
		Eigen::Vector3d r11 = manipulators[arm_num]->getR11();
		Eigen::Vector3d r12 = manipulators[arm_num]->getR12();
		Eigen::Vector3d r21 = manipulators[arm_num]->getR21();
		Eigen::Vector3d r22 = manipulators[arm_num]->getR22();

		Eigen::Matrix34d v = Eigen::Matrix34d::Zero();
		v.row(2) = vec_v.transpose();
		Eigen::Vector3d v1 = v.col(0);
		Eigen::Vector3d v2 = v.col(1);
		Eigen::Vector3d v3 = v.col(2);
		Eigen::Vector3d v4 = v.col(3);



		int N = std::ceil(manipulators[arm_num]->getLso() / manipulators[arm_num]->getDiscreteElement());
		double step = manipulators[arm_num]->getLso() * 1e-3 / (double)(N - 1);
		Eigen::Matrix3d Kb = (4 * Kb1 + 16 * Kb2 + manipulators[arm_num]->getK1() * Kb1) / zeta;
		Eigen::Matrix3d Kb_ = Kb.inverse();
		// 开始积分
		Eigen::Vector3d ip; Eigen::Vector3d ip_dot;
		Eigen::Matrix3d iR; Eigen::Matrix3d iR_dot;
		Eigen::Vector3d in; Eigen::Vector3d in_dot;
		Eigen::Vector3d im; Eigen::Vector3d im_dot;
		Eigen::Vector4d iq; Eigen::Vector4d iq_dot;
		Eigen::Vector3d iu;
		double theta(0.0), delta(0.0), ct(1.0), cd(1.0), st(0.0), sd(0.0);
		for (size_t i = 0; i < N - 1; i++) {
			ip = y.topRightCorner(3, 1);
			iR = y.topLeftCorner(3, 3);
			in = fm0.head(3);
			im = fm0.tail(3);
			iq = y.row(3).transpose();

			iu = Kb_ * iR.transpose() * im; iu(2) = 0.0;
			theta = step * iu.norm(); delta = -atan2(iu(1), iu(0)) + Keith::PI / 2.0;
			ct = cos(theta); st = sin(theta); cd = cos(delta); sd = sin(delta);
			if (abs(theta) < 1e-15) {
				ip_dot = iR * Eigen::Vector3d{ 0.0, 0.0, step };
				iR_dot = Eigen::Matrix3d::Identity();
			}
			else {
				ip_dot = iR * (step / theta * Eigen::Vector3d{ cd * (1.0 - ct), sd * (ct - 1), st });
				iR_dot << cd * cd * ct + sd * sd, -sd * cd * (ct - 1), cd* st,
					sd* cd* (1 - ct), cd* cd + ct * sd * sd, -sd * st,
					-cd * st, sd* st, ct;
			}
			ip = ip + ip_dot;
			iR = iR * iR_dot;
			in_dot = Eigen::Vector3d::Zero();
			in = in + in_dot;
			im_dot = -Keith::skew_matrix(ip_dot) * in - 2 * iR * (
				(Keith::skew_matrix(iu) * r11).cross(Ke1 * v1) + r11.cross(Keith::skew_matrix(iu) * Ke1 * v1)
				+ (Keith::skew_matrix(iu) * r12).cross(Ke1 * v2) + r12.cross(Keith::skew_matrix(iu) * Ke1 * v2)
				+ (Keith::skew_matrix(iu) * r21).cross(Ke2 * v3) + r21.cross(Keith::skew_matrix(iu) * Ke2 * v3)
				+ (Keith::skew_matrix(iu) * r22).cross(Ke2 * v4) + r22.cross(Keith::skew_matrix(iu) * Ke2 * v4)
				) * step;
			im = im + im_dot;
			iq_dot = Q.transpose() * iu * step;
			iq = iq + iq_dot;
			y.topLeftCorner(3, 3) = iR;
			y.topRightCorner(3, 1) = ip;
			y.row(3) = iq.transpose();
			fm0.head(3) = in;
			fm0.tail(3) = im;
			energy += 10 * iu.transpose().dot(/*Kb * */10 * iu) * step;
		}
		return true;
	}
	inline bool integrateContinuumSegment1(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, double& energy, const Eigen::Vector4d& vec_v,
		const double& ksi, const int arm_num = 0) {
		if (manipulators[arm_num]->getL1o() < 1e-2)
			return false;
		Eigen::Matrix3d Kb1 = manipulators[arm_num]->getKb1();
		Eigen::Matrix3d Ke1 = manipulators[arm_num]->getKe1();
		Eigen::Matrix3d Kb2 = manipulators[arm_num]->getKb2();
		Eigen::Matrix3d Ke2 = manipulators[arm_num]->getKe2();
		Eigen::Matrix34d Q = manipulators[arm_num]->getQ1();
		//double K1 = ksi;
		Eigen::Vector3d r11 = manipulators[arm_num]->getR11();
		Eigen::Vector3d r12 = manipulators[arm_num]->getR12();
		Eigen::Vector3d r21 = manipulators[arm_num]->getR21();
		Eigen::Vector3d r22 = manipulators[arm_num]->getR22();

		Eigen::Matrix34d v = Eigen::Matrix34d::Zero();
		v.row(2) = vec_v.transpose();
		Eigen::Vector3d v1 = v.col(0);
		Eigen::Vector3d v2 = v.col(1);
		Eigen::Vector3d v3 = v.col(2);
		Eigen::Vector3d v4 = v.col(3);



		int N = std::ceil(manipulators[arm_num]->getL1o() / manipulators[arm_num]->getDiscreteElement());
		double step = manipulators[arm_num]->getL1o() * 1e-3 / (double)(N - 1);
		Eigen::Matrix3d Kb = (4 * Kb1 + 16 * Kb2 + manipulators[arm_num]->getK1() * Kb1);
		Eigen::Matrix3d Kb_ = Kb.inverse();
		// 开始积分
		Eigen::Vector3d ip; Eigen::Vector3d ip_dot;
		Eigen::Matrix3d iR; Eigen::Matrix3d iR_dot;
		Eigen::Vector3d in; Eigen::Vector3d in_dot;
		Eigen::Vector3d im; Eigen::Vector3d im_dot;
		Eigen::Vector4d iq; Eigen::Vector4d iq_dot;
		Eigen::Vector3d iu;
		double theta(0.0), delta(0.0), ct(1.0), cd(1.0), st(0.0), sd(0.0);
		for (size_t i = 0; i < N; i++) {
			ip = y.topRightCorner(3, 1);
			iR = y.topLeftCorner(3, 3);
			in = fm0.head(3);
			im = fm0.tail(3);
			iq = y.row(3).transpose();
			//std::cout << ip.transpose() << std::endl;
			iu = Kb_ * iR.transpose() * im; iu(2) = 0.0;
			theta = step * iu.norm(); delta = -atan2(iu(1), iu(0)) + Keith::PI / 2.0;
			if (i == N - 1) {
				iu = Eigen::Vector3d{ 0.0, 0.0, 0.0 };
				step = manipulators[arm_num]->getLr() * 1e-3;
				theta = 0.0;
				delta = 0.0;
			}
			ct = cos(theta); st = sin(theta); cd = cos(delta); sd = sin(delta);

			if (abs(theta) < 1e-25) {
				ip_dot = iR * Eigen::Vector3d{ 0.0, 0.0, step };
				iR_dot = Eigen::Matrix3d::Identity();
			}
			else {
				ip_dot = iR * (step / theta * Eigen::Vector3d{ cd * (1.0 - ct), sd * (ct - 1), st });
				iR_dot << cd * cd * ct + sd * sd, -sd * cd * (ct - 1), cd* st,
					sd* cd* (1 - ct), cd* cd + ct * sd * sd, -sd * st,
					-cd * st, sd* st, ct;
			}
			ip = ip + ip_dot;
			iR = iR * iR_dot;
			in_dot = Eigen::Vector3d::Zero();
			in = in + in_dot;
			im_dot = -Keith::skew_matrix(ip_dot) * in - 2 * iR * (
				(Keith::skew_matrix(iu) * r11).cross(Ke1 * v1) + r11.cross(Keith::skew_matrix(iu) * Ke1 * v1)
				+ (Keith::skew_matrix(iu) * r12).cross(Ke1 * v2) + r12.cross(Keith::skew_matrix(iu) * Ke1 * v2)
				+ (Keith::skew_matrix(iu) * r21).cross(Ke2 * v3) + r21.cross(Keith::skew_matrix(iu) * Ke2 * v3)
				+ (Keith::skew_matrix(iu) * r22).cross(Ke2 * v4) + r22.cross(Keith::skew_matrix(iu) * Ke2 * v4)
				) * step;
			if (i == N - 1) {
				im_dot = Keith::skew_matrix(ip_dot) * in +
					2 * ((iR * r11).cross(iR * Ke1 * v1) + (iR * r12).cross(iR * Ke1 * v2));
			}
			im = im + im_dot;
			iq_dot = Q.transpose() * iu * step;
			iq = iq + iq_dot;
			y.topLeftCorner(3, 3) = iR;
			y.topRightCorner(3, 1) = ip;
			y.row(3) = iq.transpose();
			fm0.head(3) = in;
			fm0.tail(3) = im;
			energy += iu.transpose().dot(/*Kb * */iu) * step;
		}
		energy += 2 * (v1.transpose().dot(/*Ke1 * */v1) + v2.transpose().dot(/*Ke1 * */v2)) *
			(manipulators[arm_num]->getLstem() + manipulators[arm_num]->getL1()) * 1e-3;
		return true;
	}
	inline bool integrateContinuumSegment2(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, double& energy, const Eigen::Vector4d& vec_v,
		const int arm_num = 0) {
		Eigen::Matrix3d Kb1 = manipulators[arm_num]->getKb1();
		Eigen::Matrix3d Kb2 = manipulators[arm_num]->getKb2();
		Eigen::Matrix3d Ke2 = manipulators[arm_num]->getKe2();
		Eigen::Matrix32d Q = manipulators[arm_num]->getQ2();
		double K2 = manipulators[arm_num]->getK2();
		Eigen::Vector3d r21 = manipulators[arm_num]->getR21();
		Eigen::Vector3d r22 = manipulators[arm_num]->getR22();

		Eigen::Matrix32d v = Eigen::Matrix32d::Zero();
		v.row(2) = vec_v.tail(2).transpose();
		Eigen::Vector3d v1 = v.col(0);
		Eigen::Vector3d v2 = v.col(1);


		int N = std::ceil(manipulators[arm_num]->getL2() / manipulators[arm_num]->getDiscreteElement());
		double step = manipulators[arm_num]->getL2() * 1e-3 / (double)(N - 1);
		Eigen::Matrix3d Kb = (16 * Kb2 + K2 * Kb1);
		Eigen::Matrix3d Kb_ = Kb.inverse();
		// 开始积分
		Eigen::Vector3d ip; Eigen::Vector3d ip_dot;
		Eigen::Matrix3d iR; Eigen::Matrix3d iR_dot;
		Eigen::Vector3d in; Eigen::Vector3d in_dot;
		Eigen::Vector3d im; Eigen::Vector3d im_dot;
		Eigen::Vector2d iq; Eigen::Vector2d iq_dot;
		Eigen::Vector3d iu;
		Eigen::Vector3d iv;
		double theta(0.0), delta(0.0), ct(1.0), cd(1.0), st(0.0), sd(0.0);
		for (size_t i = 0; i < N; i++) {
			ip = y.topRightCorner(3, 1);
			iR = y.topLeftCorner(3, 3);
			in = fm0.head(3);
			im = fm0.tail(3);
			iq = y.bottomRightCorner(1, 2).transpose();

			iu = Kb_ * iR.transpose() * im; iu(2) = 0.0;
			theta = step * iu.norm(); delta = -atan2(iu(1), iu(0)) + Keith::PI / 2.0;
			if (i == N - 1) {
				iu = Eigen::Vector3d{ 0.0, 0.0, 0.0 };
				step = manipulators[arm_num]->getLg() * 1e-3;
				theta = 0.0;
				delta = 0.0;
			}
			ct = cos(theta); st = sin(theta); cd = cos(delta); sd = sin(delta);
			if (abs(theta) < 1e-15) {
				ip_dot = iR * Eigen::Vector3d{ 0.0, 0.0, step };
				iR_dot = Eigen::Matrix3d::Identity();
			}
			else {
				ip_dot = iR * (step / theta * Eigen::Vector3d{ cd * (1.0 - ct), sd * (ct - 1), st });
				iR_dot << cd * cd * ct + sd * sd, -sd * cd * (ct - 1), cd* st,
					sd* cd* (1 - ct), cd* cd + ct * sd * sd, -sd * st,
					-cd * st, sd* st, ct;
			}
			ip = ip + ip_dot;
			iR = iR * iR_dot;
			in_dot = Eigen::Vector3d::Zero();
			in = in + in_dot;
			im_dot = -Keith::skew_matrix(ip_dot) * in - 2 * iR * (
				(Keith::skew_matrix(iu) * r21).cross(Ke2 * v1) + r21.cross(Keith::skew_matrix(iu) * Ke2 * v1)
				+ (Keith::skew_matrix(iu) * r22).cross(Ke2 * v2) + r22.cross(Keith::skew_matrix(iu) * Ke2 * v2)
				) * step;
			if (i == N - 1) {
				im_dot = Keith::skew_matrix(ip_dot) * in +
					2 * ((iR * r21).cross(iR * Ke2 * v1) + (iR * r22).cross(iR * Ke2 * v2));
			}
			im = im + im_dot;
			iq_dot = Q.transpose() * iu * step;
			iq = iq + iq_dot;
			y.topLeftCorner(3, 3) = iR;
			y.topRightCorner(3, 1) = ip;
			y.bottomRightCorner(1, 2) = iq.transpose();
			fm0.head(3) = in;
			fm0.tail(3) = im;
			energy += iu.transpose().dot(/*Kb * */iu) * step;
		}
		energy += 2 * (v1.transpose().dot(/*Ke2 * */v1) + v2.transpose().dot(/*Ke2 * */v2)) *
			(manipulators[arm_num]->getLstem() + manipulators[arm_num]->getL1() +
				manipulators[arm_num]->getLr() + manipulators[arm_num]->getL2()) * 1e-3;
		return true;
	}
	inline bool integrateElasticRod(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, double& energy,
		const Eigen::Vector7d& ksi, const int arm_num = 0) {
		Eigen::DiagonalMatrix<double, Eigen::Dynamic> diag_Kb(ksi.segment(1, 3));
		//Eigen::DiagonalMatrix<double, Eigen::Dynamic> diag_Ke(ksi.tail(3));
		Eigen::Matrix3d Kb = 3 * diag_Kb * manipulators[arm_num]->getKb2() * 100 * 50;
		Eigen::Matrix3d Ke = manipulators[arm_num]->getKe2() * 100;
		//Ke(2, 2) = Ke(2, 2) * ksi(4);
		Eigen::Vector3d f0 = Eigen::Vector3d{ 0, 0,0 };//::Zero();// 
		//f0 = y.topLeftCorner(3, 3)*f0;

		int N = std::ceil(manipulators[arm_num]->getLdo() / manipulators[arm_num]->getDiscreteElement());
		double step = manipulators[arm_num]->getLdo() * 1e-3 / (double)(N - 1);
		Eigen::Matrix3d Kb_ = Kb.inverse();
		Eigen::Matrix3d Ke_ = Ke.inverse();
		// 开始积分
		Eigen::Vector3d ip; Eigen::Vector3d ip_dot;
		Eigen::Matrix3d iR; Eigen::Matrix3d iR_dot;
		Eigen::Vector3d in; Eigen::Vector3d in_dot;
		Eigen::Vector3d im; Eigen::Vector3d im_dot;
		Eigen::Vector3d iu; Eigen::Vector3d u0 = Eigen::Vector3d::Zero();
		Eigen::Vector3d iv; Eigen::Vector3d v0 = Eigen::Vector3d{ 0.0, 0.0, 1.0 };
		double theta(0.0), delta(0.0), phi(0.0), ct(1.0), cd(1.0), st(0.0), sd(0.0);
		Eigen::Matrix3d R_phi;
		Eigen::AngleAxisd rotation(phi, Eigen::Vector3d::UnitZ());
		for (size_t i = 0; i < N - 1; i++) {
			ip = y.topRightCorner(3, 1);
			iR = y.topLeftCorner(3, 3);
			in = fm0.head(3);
			im = fm0.tail(3);

			iu = Kb_ * iR.transpose() * im + u0;
			iv = Ke_ * iR.transpose() * in + v0;
			iv = iv * step;
			//iv[0] = 0.0; iv[1] = 0.0;
			theta = step * iu.head(2).norm(); delta = -atan2(iu(1), iu(0)) + Keith::PI / 2.0; phi = step * iu(2);
			rotation.angle() = phi;
			rotation.axis() = Eigen::Vector3d::UnitZ();
			R_phi = rotation.matrix();
			ct = cos(theta); st = sin(theta); cd = cos(delta); sd = sin(delta);
			if (abs(theta) < 1e-15) {
				ip_dot = iR * Eigen::Vector3d{ 0.0, 0.0, iv(2) };
				iR_dot = Eigen::Matrix3d::Identity() * R_phi;
			}
			else {

				iR_dot << cd * cd * ct + sd * sd, -sd * cd * (ct - 1), cd* st,
					sd* cd* (1 - ct), cd* cd + ct * sd * sd, -sd * st,
					-cd * st, sd* st, ct;
				iR_dot = iR_dot * R_phi;
				ip_dot = iR * (iv(2) / theta * Eigen::Vector3d{ cd * (1.0 - ct), sd * (ct - 1), st });
			}
			//iR_dot = iR*Keith::skew_matrix(iu);
			//ip_dot = iR * iv;
			in_dot = -f0 * step;
			im_dot = -Keith::skew_matrix(ip_dot) * in;
			iR = iR * iR_dot;
			ip = ip + ip_dot;
			in = in + in_dot;
			im = im + im_dot;
			y.topLeftCorner(3, 3) = iR;
			y.topRightCorner(3, 1) = ip;
			fm0.head(3) = in;
			fm0.tail(3) = im;
			energy += (iu.transpose().dot(iu) + iv.transpose().dot(iv)) * step;
		}
		return true;
	}
	bool shootOneSubchain(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, Eigen::Vector8d& v_phi_l, double& energy, const Eigen::Matrix3d& Rg_t,
		const Eigen::Vector13d& ksi, const int arm_num = 0);
	void shootPCKC(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, double& energy,
		std::vector<Eigen::Vector6d>& qa, Eigen::Matrix4d& target, const int arm_num = 2, const int mode = 0, const bool is_Jacobian = true);
	bool shootStiffnessOptimization(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
		Eigen::Matrix4d target);
	bool shootStiffnessOptimization2(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
		Eigen::Matrix4d target);
	bool shootInverseOptimization(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
		double& energy_rsd, Eigen::Matrix4d target = Eigen::Matrix4d::Identity());
	bool shootForwardOptimization(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
		double& energy, Eigen::Matrix4d& target);
	bool inverseKinematicsPlanB(std::vector<Eigen::Vector6d>& cur_qa, const Eigen::Matrix4d& cur_pose,
		const Eigen::Matrix4d& target, const int iter);
	bool getEnergyConstant(Eigen::Vector12d& dE, const std::vector<Eigen::Vector6d>& cur_qa);

	void setParams(const std::vector<Eigen::Vector12d>& paras);
	const std::vector<Eigen::Vector12d> getParams();

public:
	SurgicalContinuumManipulator* manipulators[ARM_NUM];
	//Trocar* tc[ARM_NUM];

private:
	std::vector<Eigen::Vector12d> paras;
	Eigen::MatrixXd Jaco_broy;

public slots:
	//void doWork()
	//{
	//    // 执行需要在单独线程中运行的工作
	//    // ...
	//}

};