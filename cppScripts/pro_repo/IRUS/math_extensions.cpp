#include "math_extensions.h"
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <iostream>

Eigen::Matrix3d Keith::skew_matrix(Eigen::Vector3d vec)
{
	Eigen::Matrix3d res;
	res << 0, -vec(2), vec(1),
		vec(2), 0, -vec(0),
		-vec(1), vec(0), 0;
	return res;
}

Eigen::Vector3d Keith::invSkewMatrix_keith(Eigen::Matrix3d mat1, Eigen::Matrix3d mat2)
{
	Eigen::Matrix3d mat = mat1 * mat2.transpose() - mat2 * mat1.transpose();
	Eigen::Vector3d p;
	p << mat(2, 1), mat(0, 2), mat(1, 0);
	return p;
}

Eigen::MatrixXd Keith::pinv(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& inMatrix)
{
	//std::cout << inMatrix << std::endl;
	Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(inMatrix);
	Eigen::MatrixXd A_inverse = cod.pseudoInverse();
	return A_inverse;
}

Eigen::MatrixXd Keith::invScale(const Eigen::MatrixXd& inMatrix) {
	// LU分解
	//Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(inMatrix);
	//// 计算逆矩阵
	//Eigen::MatrixXd A_inverse = lu_decomp.inverse();

	Eigen::PartialPivLU<Eigen::MatrixXd> ALU(inMatrix);
	Eigen::MatrixXd A_inverse = ALU.inverse();

	return A_inverse;
}

Eigen::Matrix4d Keith::fromQuat2T(Eigen::Vector7d vec)
{
#if 0 
	Eigen::Vector4d quatvec = vec.tail(4);
	quatvec = quatvec / quatvec.norm();
	double w = quatvec[0];
	quatvec[0] = quatvec[3];
	quatvec[3] = w;
	Eigen::Quaterniond quat(quatvec);
	Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
	res.topLeftCorner(3, 3) = quat.toRotationMatrix();
	res.topRightCorner(3, 1) = vec.head(3);
	return res;
#else
	Eigen::Vector4d quatvec = vec.tail(4);
	quatvec = quatvec / quatvec.norm();
	Eigen::Quaterniond quat;
	quat.w() = quatvec(0);
	quat.x() = quatvec(1);
	quat.y() = quatvec(2);
	quat.z() = quatvec(3);
	Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
	res.topLeftCorner(3, 3) = quat.toRotationMatrix();
	res.topRightCorner(3, 1) = vec.head(3);
	return res;
#endif
}

Eigen::Matrix3d Keith::rpy2Rotation(Eigen::Vector3d rpy) {
	//double cr = std::cos(rpy[0]); double sr = std::sin(rpy[0]);
	//double cp = std::cos(rpy[1]); double sp = std::sin(rpy[1]);
	//double cy = std::cos(rpy[2]); double sy = std::sin(rpy[2]);
	//Eigen::Matrix3d rot;
	//rot << cp*cy, cp*sy, -sp,
	//	sr*sp*cy - cr*sy, sr*sp*sy + cr*cy, sr*cp,
	//	cr*sp*cy + sr*sy, cr*sp*sy - sr*cy, cr*cp;
	Eigen::Matrix3d rot;
	rot = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitX());

	return rot;
}

Eigen::Matrix4d Keith::fromX2T(const Eigen::Vector6d& X)
{
	assert(X.size() == 6);

	double angle = X.segment(3, 3).norm();
	Eigen::Vector3d axis;
	if (angle == 0)
	{
		axis << 0, 0, 1;
	}
	else
	{
		axis = X.segment(3, 3) / angle;
	}
	angle = angle * Keith::PI / 180.0;

	Eigen::Matrix4d T;
	Eigen::AngleAxisd rotation(angle, axis);
	T.topLeftCorner(3, 3) = rotation.toRotationMatrix();
	T.topRightCorner(3, 1) = X.segment(0, 3);
	T.bottomLeftCorner<1, 3>().setZero();
	T(3, 3) = 1.0;

	return T;
}

Eigen::Vector6d Keith::fromT2X(const Eigen::Matrix4d& X)
{
	Eigen::Matrix3d rot = X.topLeftCorner(3, 3);
	Eigen::AngleAxisd axang(rot);
	Eigen::Vector3d axis, vec;
	axis = axang.axis() * axang.angle() * 180 / Keith::PI;
	vec = X.topRightCorner(3, 1);
	Eigen::Vector6d res;
	res.head(3) = vec; res.tail(3) = axis;
	return res;
}

Eigen::Vector6d Keith::calcDeviationFrom2T(const Eigen::Matrix4d& cur_T, const Eigen::Matrix4d& tar_T) {
	Eigen::Matrix3d rot = cur_T.topLeftCorner(3, 3).transpose() * tar_T.topLeftCorner(3, 3);
	Eigen::AngleAxisd d_rot(rot);
	double angle = d_rot.angle();
	Eigen::Vector3d d_p, axis;
	axis = d_rot.axis();
	d_p = tar_T.topRightCorner(3, 1) - cur_T.topRightCorner(3, 1);
	Eigen::Vector6d vec;
	vec.head(3) = d_p;
	vec.tail(3) = axis * angle;
	return vec;

}

Eigen::Matrix3d Keith::axang2rotm(Eigen::Vector3d vec)
{
	Eigen::AngleAxisd axang;
	double angle = vec.norm();
	Eigen::Vector3d axis;
	if (angle == 0.0)
		axis = Eigen::Vector3d{ 0.0, 0.0, 1.0 };
	else
		axis = vec / angle;
	axang.angle() = angle / 180.0 * Keith::PI;
	axang.axis() = axis;
	return axang.toRotationMatrix();
}

Eigen::Vector4d Keith::getQuat(std::vector<Eigen::Vector4d> quat, int quatSize)
{
	Eigen::Vector4d meanTem = Eigen::Vector4d::Zero();
	//meanQuat << 0.f, 0.f, 0.f, 0.f;
	Eigen::Vector4d meanVec = Eigen::Vector4d::Zero();

	Eigen::MatrixXd Q(quatSize, 4), A(4, 4), Qt(4, quatSize);
	for (int i = 0; i < quatSize; i++)
	{
		meanTem = quat[i];
		meanTem = meanTem.normalized();
		Q.row(i) = meanTem;
	}
	Qt = Q.transpose();
	A = Qt * Q;
	//std::cout << Q << "    \n\n\n";
	Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(A);
	Eigen::Vector4d eiVal = eigen_solver.eigenvalues().real();
	Eigen::MatrixXd temMat = eigen_solver.eigenvectors().real();
	int maxVal(0);
	if (eiVal(0) > eiVal(1) && eiVal(0) > eiVal(2) && eiVal(0) > eiVal(3))
		maxVal = 0;
	else if (eiVal(1) > eiVal(0) && eiVal(1) > eiVal(2) && eiVal(1) > eiVal(3))
		maxVal = 1;
	else if (eiVal(2) > eiVal(0) && eiVal(2) > eiVal(1) && eiVal(2) > eiVal(3))
		maxVal = 2;
	else
		maxVal = 3;
	meanVec = temMat.col(maxVal);
	meanVec = meanVec.normalized();
	return meanVec;
}

Eigen::Vector4d Keith::getPose(std::vector<Eigen::Vector4d> pose, int quatSize)
{
	Eigen::Vector4d res(0.f, 0.f, 0.f, 0.f);
	float pMax(0.f), pMin(500.f);
	int x(0), y(0);
	for (int el = 0; el < pose.size(); el++)
	{
		if (pMax < pose[el].norm())
		{
			x = el;
			pMax = pose[el].norm();
		}
		if (pMin > pose[el].norm())
		{
			y = el;
			pMin = pose[el].norm();
		}
	}


	for (int ele = 0; ele < pose.size(); ele++)
	{
		if (ele == x || ele == y)
			continue;
		res += pose[ele];

	}

	int isMarker = 0;
	static int iC = 0;
	for (int el = 0; el < pose.size(); el++)
	{
		if (abs(pose[0].norm() - pose[el].norm()) > 1.f || pose[el].norm() < 1.f)
		{
			isMarker += 0;
			break;
		}
		else
			isMarker += 1;
		//std::cout << pose[el].norm() << "    ";
	}
	//std::cout << std::endl;

	res /= quatSize - 2;
	//std::cout << "getPose execute times:" << iC++ << "    pose.size(): " << pose.size()<<"    isMaker: "<< isMarker
	//	<<std::endl;
	return res;
}

Eigen::Vector6d Keith::Psi2Curvature_keith(const Eigen::Vector6d& psi, const Eigen::Vector3d& L12Zeta) {
	Eigen::Vector6d uc = Eigen::Vector6d::Zero();
	double l1 = psi(0);
	double zeta = L12Zeta(2);
	double L1(0.0), L10(L12Zeta[0]), L2(L12Zeta[1] * 1e-3);

	Eigen::Vector6d temPsi = psi;
	double theta2 = temPsi(4);
	double delta1 = temPsi(3);
	double delta2 = temPsi(5);

	if (l1 <= 0) {
		uc(3) = theta2 / L2 * cos(PI_2 + delta2);
		uc(4) = theta2 / L2 * sin(PI_2 + delta2);
	}
	else if (l1 <= L10)
	{
		L1 = l1 * 1e-3;
		double theta1 = temPsi(2);
		uc(0) = theta1 / L1 * cos(PI_2 + delta1);
		uc(1) = theta1 / L1 * sin(PI_2 + delta1);
		uc(3) = theta2 / L2 * cos(PI_2 + delta2);
		uc(4) = theta2 / L2 * sin(PI_2 + delta2);
	}
	else
	{
		L1 = L10 * 1e-3;
		double theta1 = L1 / (zeta * (l1 * 1e-3 - L1) + L1) * temPsi(2);
		uc(0) = theta1 / L1 * cos(PI_2 + delta1);
		uc(1) = theta1 / L1 * sin(PI_2 + delta1);
		uc(3) = theta2 / L2 * cos(PI_2 + delta2);
		uc(4) = theta2 / L2 * sin(PI_2 + delta2);
	}

	return uc;
}

Eigen::Vector6d Keith::Curvature2Psi_keith(const Eigen::Vector6d& u,
	const Eigen::Vector3d& L12Zeta, const double& l) {
	double L1 = L12Zeta(0) * 1e-3;
	double L2 = L12Zeta(1) * 1e-3;
	double zeta = L12Zeta(2);
	Eigen::Vector6d psi = Eigen::Vector6d::Zero();
	psi(0) = l * 1e-3;
	double L1_new = L1;
	if (l < L12Zeta(0))
	{
		L1_new = l * 1e-3;
		psi(2) = L1_new * u.segment(0, 3).norm();
	}
	else
	{
		psi(2) = (L1_new + zeta * (l * 1e-3 - L1_new)) * u.segment(0, 3).norm();
	}

	psi(3) = -PI_2 + atan2(u(1), u(0));
	psi(4) = L2 * u.segment(3, 3).norm();
	psi(5) = -PI_2 + atan2(u(4), u(3));

	return psi;
}

Eigen::Vector6d Keith::Curvature2Actuation_keith(const Eigen::Vector6d& u, const Eigen::Matrix46d& Gc) {
	Eigen::Vector4d q = Eigen::Vector4d::Zero();
	Eigen::Vector6d qa = Eigen::Vector6d::Zero();
	q = Gc * u;
	qa.tail(4) = q;
	return qa * 1e3;
}

Eigen::Vector6d Keith::Actuation2Curvature_keith(const Eigen::Vector6d& qa, const Eigen::Matrix46d& Gc) {
	Eigen::Vector4d q = qa.tail(4) * 1e-3;
	Eigen::Matrix46d Gc_tem = Gc;
	Eigen::Vector6d u = Keith::pinv(Gc_tem) * q;
	return u;
}

Eigen::Vector6d Keith::Actuation2Psi_keith(const Eigen::Vector6d& qa,
	const Eigen::Vector3d& L12Zeta, const Eigen::Matrix46d& Gc) {
	Eigen::Vector6d u = Actuation2Curvature_keith(qa, Gc);
	Eigen::Vector6d psi = Curvature2Psi_keith(u, L12Zeta, qa(0));
	psi.head(2) = qa.head(2);
	psi.tail(5) = psi.tail(5) / Keith::PI * 180.0;
	return psi;
}

Eigen::Vector6d Keith::Psi2Actuation_keith(const Eigen::Vector6d& psi,
	const Eigen::Vector3d& L12Zeta, const Eigen::Matrix46d& Gc) {
	Eigen::Vector6d psi_tem = psi;
	psi_tem.tail(5) = psi.tail(5) * Keith::PI / 180.0;
	Eigen::Vector6d u = Psi2Curvature_keith(psi_tem, L12Zeta);
	Eigen::Vector6d qa = Curvature2Actuation_keith(u, Gc);
	qa.head(2) = psi_tem.head(2);
	return qa;
}

Eigen::Matrix4d Keith::getSegmentPose(const double L, double theta1, double delta1) {
	double L1 = L;
	double k1 = theta1 / L1;

	Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
	if (L < 1e-4)
		return T2;

	if (theta1 == 0) {
		T2 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, L1,
			0, 0, 0, 1;
	}
	else {
		double cosTHETA1 = cos(theta1);
		double sinTHETA1 = sin(theta1);
		double cosDELTA1 = cos(delta1);
		double sinDELTA1 = sin(delta1);

		T2 << pow(cosDELTA1, 2) * (cosTHETA1 - 1) + 1, sinDELTA1* cosDELTA1* (cosTHETA1 - 1), cosDELTA1* sinTHETA1, cosDELTA1* (1 - cosTHETA1) / k1,
			sinDELTA1* cosDELTA1* (cosTHETA1 - 1), pow(cosDELTA1, 2)* (1 - cosTHETA1) + cosTHETA1, sinDELTA1* sinTHETA1, sinDELTA1* (1 - cosTHETA1) / k1,
			-cosDELTA1 * sinTHETA1, -sinDELTA1 * sinTHETA1, cosTHETA1, sinTHETA1 / k1,
			0, 0, 0, 1;
	}

	return T2;
}
Eigen::Matrix4d Keith::getForwardKinematics(const Eigen::Vector6d& psi, const Eigen::Vector4d& L1r2g, const double Zeta) {
	double l = psi(0);
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	if (l < 1e-6)
		return T;
	Eigen::Vector6d tem_psi = psi;
	tem_psi.tail(5) = psi.tail(5) / 180 * Keith::PI;

	double Lr(L1r2g(1)), L2(L1r2g(2)), Lg(L1r2g(3)), zeta(Zeta), theta2(tem_psi(4)), delta2(tem_psi(5));
	double Ls(0.0), L1(L1r2g(0)), thetas(0.0), deltas(tem_psi(3)), theta1(tem_psi(2)), delta1(tem_psi(3));
	if (l < L1) {
		L1 = l;
	}
	else {
		Ls = l - L1;
		thetas = theta1 * zeta * Ls / (zeta * Ls + L1);
		theta1 = theta1 * L1 / (zeta * Ls + L1);
	}
	Eigen::Matrix4d Ts(T), T1(T), Tr(T), T2(T), Tg(T);
	if (Ls > 0.0)
		Ts = getSegmentPose(Ls, thetas, deltas);
	Eigen::AngleAxisd rotation(tem_psi(1), Eigen::Vector3d::UnitZ());
	T.topLeftCorner(3, 3) = rotation.toRotationMatrix();
	Ts = T * Ts;
	T1 = getSegmentPose(L1, theta1, delta1);
	T2 = getSegmentPose(L2, theta2, delta2);
	Tr(2, 3) = Lr;
	Tg(2, 3) = Lg;
	T = Ts * T1 * Tr * T2 * Tg;
	return T;
}

Eigen::Vector6d Keith::getInverseKinematics(const Eigen::Matrix4d& target, const Eigen::Vector4d& L1r2g,
	const double zeta, Eigen::Vector6d& psi, double& residue) {
	Eigen::Vector6d cur_psi(psi), d_err(Eigen::Vector6d::Zero()), d_psi(Eigen::Vector6d::Zero()), tem;
	if (abs(cur_psi[2]) < 1e-4 && abs(cur_psi[4]) < 1e-4) {
		cur_psi[2] = 30.0;
		cur_psi[4] = 30.0;
	}

	Eigen::Matrix4d cur_T2, cur_tar;
	double err(10.0), thre(1e-6), dp(1e-9), step(2.0);
	int iter(0);
	Eigen::Matrix6d J(Eigen::Matrix6d::Zero()), I66(Eigen::Matrix6d::Identity()),
		J_(Eigen::Matrix6d::Zero());
	Eigen::Matrix4d cur_T = getForwardKinematics(cur_psi, L1r2g, zeta);
	cur_tar = calcMiddleMatrix(cur_T, target, 2.0);
	d_err = calcDeviationFrom2T(cur_T, cur_tar);
	err = d_err.norm();
	while (err > thre) {
		for (size_t i = 0; i < 6; i++) {
			cur_T2 = getForwardKinematics(cur_psi + dp * I66.col(i), L1r2g, zeta);
			J.col(i) = calcDeviationFrom2T(cur_T2, cur_T) / dp;
			tem = J.col(i);
		}
		J_ = invScale((J.transpose() * J + dp * I66)) * J.transpose();
		if (err < step) {
			d_psi = J_ * d_err;
		}
		else {
			d_psi = J_ * d_err / d_err.norm() * step;
		}
		cur_psi -= d_psi;
		cur_T = getForwardKinematics(cur_psi, L1r2g, zeta);
		cur_tar = calcMiddleMatrix(cur_T, target, 2.0);
		d_err = calcDeviationFrom2T(cur_T, cur_tar);
		err = d_err.norm();
		residue = err;
		if (++iter > 50)
			break;
	}
	return cur_psi;
}

const Eigen::Matrix4d Keith::calcMiddleMatrix(const Eigen::Matrix4d& cur_T,
	const Eigen::Matrix4d& tar_T, const double increment) {
	Eigen::Matrix3d rot;
	Eigen::Vector3d pos, e;
	rot = cur_T.topLeftCorner(3, 3).transpose() * tar_T.topLeftCorner(3, 3);
	Eigen::AngleAxisd cur_axang(rot);
	double angle = rad2deg(cur_axang.angle());
	if (angle > increment) {
		cur_axang.angle() = deg2rad(increment);
	}
	else if (angle < -increment) {
		cur_axang.angle() = -deg2rad(increment);
	}
	else {
	}
	rot = cur_T.topLeftCorner(3, 3) * cur_axang.toRotationMatrix();
	double dis = (cur_T.topRightCorner(3, 1) - tar_T.topRightCorner(3, 1)).norm();
	double increment_used(increment);
	if (cur_T(2, 3) < 0.2) {
		increment_used = increment_used / 1e3;
	}
	e = (tar_T.topRightCorner(3, 1) - cur_T.topRightCorner(3, 1)) / dis;
	if (dis > increment_used) {
		pos = cur_T.topRightCorner(3, 1) + e * increment_used;
	}
	else {
		pos = tar_T.topRightCorner(3, 1);
	}
	Eigen::Matrix4d res(Eigen::Matrix4d::Identity());
	res.topLeftCorner(3, 3) = rot;
	res.topRightCorner(3, 1) = pos;
	return res;
}

const Eigen::Vector6d Keith::clacMiddleWrench(const Eigen::Vector6d& cur_W,
	const Eigen::Vector6d& tar_W, const double increment) {
	Eigen::Vector6d res, dev;
	dev = tar_W - cur_W;
	double inc1 = increment / 2;
	double inc2 = increment / 40;
	for (size_t i = 0; i < 3; i++) {
		if (abs(dev[i]) > inc1) {
			res[i] = cur_W[i] + dev[i] / abs(dev[i]) * inc1;
		}
		else {
			res[i] = tar_W[i];
		}
		if (abs(dev[i + 3]) > inc2) {
			res[i + 3] = cur_W[i + 3] + dev[i + 3] / abs(dev[i + 3]) * inc2;
		}
		else {
			res[i + 3] = tar_W[i + 3];
		}
	}
	return res;
}

const Eigen::Vector6d Keith::clacMiddleActuation(const Eigen::Vector6d& cur_Psi, const Eigen::Vector6d& tar_Psi, const double increment) {
	Eigen::Vector6d res, dev;
	dev = tar_Psi - cur_Psi;
	double inc1 = increment / 5;
	if (abs(dev[0]) > increment) {
		res[0] = cur_Psi[0] + dev[0] / abs(dev[0]) * increment;
	}
	else {
		res[0] = tar_Psi[0];
	}
	for (size_t i = 1; i < 6; i++) {
		if (abs(dev[i]) > inc1) {
			res[i] = cur_Psi[i] + dev[i] / abs(dev[i]) * inc1;
		}
		else {
			res[i] = tar_Psi[i];
		}
	}
	return res;
}