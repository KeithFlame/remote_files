#pragma once
#include <QObject>
#include <QThread>
#include "math_extensions.h"
#include "SurgicalContinuumManipulator.h"
#include <vector>
#include "info_define.h"

class DifferenceMethod : public QObject
{
	Q_OBJECT

public:
	DifferenceMethod(QObject* parent = nullptr);
	inline bool integrateHalfStiffnessSegment(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, const Eigen::Vector4d& vec_v) {
		if (manipulators->getLso() < 1e-2)
			return false;
		Eigen::Matrix3d Kb1 = manipulators->getKb1();
		Eigen::Matrix3d Ke1 = manipulators->getKe1();
		Eigen::Matrix3d Kb2 = manipulators->getKb2();
		Eigen::Matrix3d Ke2 = manipulators->getKe2();
		Eigen::Matrix34d Q = manipulators->getQ1();
		//double K1 = ksi;
		double zeta = manipulators->getZeta();
		Eigen::Vector3d r11 = manipulators->getR11();
		Eigen::Vector3d r12 = manipulators->getR12();
		Eigen::Vector3d r21 = manipulators->getR21();
		Eigen::Vector3d r22 = manipulators->getR22();

		Eigen::Matrix34d v = Eigen::Matrix34d::Zero();
		v.row(2) = vec_v.transpose();
		Eigen::Vector3d v1 = v.col(0);
		Eigen::Vector3d v2 = v.col(1);
		Eigen::Vector3d v3 = v.col(2);
		Eigen::Vector3d v4 = v.col(3);

		int N = std::ceil(manipulators->getLso() / manipulators->getDiscreteElement());
		double step = manipulators->getLso() * 1e-3 / (double)(N - 1);
		Eigen::Matrix3d Kb = (4 * Kb1 + 16 * Kb2 + manipulators->getK1() * Kb1) / zeta;
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
		}
		return true;
	}
	inline bool integrateContinuumSegment1(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, const Eigen::Vector4d& vec_v) {
		if (manipulators->getL1o() < 1e-2)
			return false;
		Eigen::Matrix3d Kb1 = manipulators->getKb1();
		Eigen::Matrix3d Ke1 = manipulators->getKe1();
		Eigen::Matrix3d Kb2 = manipulators->getKb2();
		Eigen::Matrix3d Ke2 = manipulators->getKe2();
		Eigen::Matrix34d Q = manipulators->getQ1();
		//double K1 = ksi;
		Eigen::Vector3d r11 = manipulators->getR11();
		Eigen::Vector3d r12 = manipulators->getR12();
		Eigen::Vector3d r21 = manipulators->getR21();
		Eigen::Vector3d r22 = manipulators->getR22();

		Eigen::Matrix34d v = Eigen::Matrix34d::Zero();
		v.row(2) = vec_v.transpose();
		Eigen::Vector3d v1 = v.col(0);
		Eigen::Vector3d v2 = v.col(1);
		Eigen::Vector3d v3 = v.col(2);
		Eigen::Vector3d v4 = v.col(3);

		int N = std::ceil(manipulators->getL1o() / manipulators->getDiscreteElement());
		double step = manipulators->getL1o() * 1e-3 / (double)(N - 1);
		Eigen::Matrix3d Kb = (4 * Kb1 + 16 * Kb2 + manipulators->getK1() * Kb1);
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
				step = manipulators->getLr() * 1e-3;
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
		}
		return true;
	}
	inline bool integrateContinuumSegment2(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, const Eigen::Vector4d& vec_v) {
		Eigen::Matrix3d Kb1 = manipulators->getKb1();
		Eigen::Matrix3d Kb2 = manipulators->getKb2();
		Eigen::Matrix3d Ke2 = manipulators->getKe2();
		Eigen::Matrix32d Q = manipulators->getQ2();
		double K2 = manipulators->getK2();
		Eigen::Vector3d r21 = manipulators->getR21();
		Eigen::Vector3d r22 = manipulators->getR22();

		Eigen::Matrix32d v = Eigen::Matrix32d::Zero();
		v.row(2) = vec_v.tail(2).transpose();
		Eigen::Vector3d v1 = v.col(0);
		Eigen::Vector3d v2 = v.col(1);


		int N = std::ceil(manipulators->getL2() / manipulators->getDiscreteElement());
		double step = manipulators->getL2() * 1e-3 / (double)(N - 1);
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
				step = manipulators->getLg() * 1e-3;
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
		}
		return true;
	}
	bool shootOneManipulator(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, Eigen::Vector6d& strain_phi_l);
	void integrationResult(Eigen::Vector12d& guess, Eigen::Vector12d& Rsd_last, Eigen::Vector6d& qa,
		Eigen::Matrix4d& target, const Eigen::Vector6d FM, const Problem mode = Problem::INVERSE_PROBLEM/*, const bool is_Jacobian = true*/);
	bool shootInverseOptimization(Eigen::Vector12d& guess, Eigen::Vector12d& Rsd_last,
		Eigen::Vector6d& qa, const Eigen::Matrix4d& target, const Eigen::Vector6d wrench);
	bool shootForwardOptimization(Eigen::Vector10d& guess, Eigen::Vector10d& Rsd_last, const Eigen::Vector6d& qa,
		Eigen::Matrix4d& target, Eigen::Vector6d& wrench);
	const Eigen::Vector6d getGripperWrench(const Eigen::Matrix4d& end_pose, const Eigen::Vector6d& wrench,
		const bool flag = 1);

	void setParams(const Eigen::Vector12d& paras);
	void setIRUSWeight(const double);
	void setIRUSBarycenterLength(const double);
	void setIRUSAuxiliarySheathWeight(const double);
	void setIRUSAuxiliarySheathBarycenterLength(const double);
	void setGravityDirection(const Eigen::Vector3d);
	void setMaxIteration(const int);
	void setResolvedIncrement(const double);

	const Eigen::Vector12d getParams();
	const double getIRUSWeight();
	const double getIRUSBarycenterLength();
	const double getIRUSAuxiliarySheathWeight();
	const double getIRUSAuxiliarySheathBarycenterLength();
	const Eigen::Vector3d getGravityDiretcion();
	const int getMaxIteration();
	const double getResolvedIncrement();

public:
	SurgicalContinuumManipulator* manipulators;
	bool is_given_wrench;

private:
	Eigen::Vector12d paras;							// 鞘管与辅助孔位置 
	double IRUS_weight;								// 超声刀重量，N
	double IRUS_barycenter_length;					// 超声刀重心位置，mm （重心与夹持点之间的距离）
	double auxiliary_sheath_weight;					// 辅助鞘管重量，N
	double auxiliary_sheath_barycenter_length;		// 辅助鞘管重心位置，mm（重心与夹持点之间的距离）
	Eigen::Vector3d gravity_direction;				// 世界坐标系下，重力方向
	int MAX_ITER;									// 最大迭代次数
	double increment;								// 逆运动学增量
};