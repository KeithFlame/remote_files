#include <assert.h>

#include "SurgicalContinuumManipulator.h"
#include "math_extensions.h"
#include "function_in_DOM.h"

SurgicalContinuumManipulator::SurgicalContinuumManipulator():
	  E(40e9)
	, mu(0.33)
	, d1(0.00095)
	, d2(0.0004)
	, Lstem(0.6)
	, L1(0.1)
	, Lr(0.01)
	, L2(0.02)
	, Lg(0.015)
	, Ldo(0.02)
	, rho1(2.5e-3)
	, rho2(2.7e-3)
	, delta_t1(0)
	, delta_t2(-Keith::PI_4)
	, K1(5)
	, K2(0.6)
	, zeta(0.2)
	, gamma1(0)
	, gamma2(0)
	, gamma3(0)
	, L1o(0.1)
	, Lso(0.04)
	, discrete_element(1e-3)
	, Tgt(Eigen::Matrix4d::Identity())
{
	G = E / 2 / (1 + mu);
	double A1 = Keith::PI * d1 * d1 / 4;
	double A2 = Keith::PI * d2 * d2;
	double I1 = Keith::PI * pow(d1, 4) / 64;
	double I2 = Keith::PI * pow(d2, 4) / 64;
	Kb1 = Eigen::DiagonalMatrix<double, 3>(E * I1, E * I1, 2 * G * I1);
	Kb2 = Eigen::DiagonalMatrix<double, 3>(E * I2, E * I2, 2 * G * I2);
	Ke1 = Eigen::DiagonalMatrix<double, 3>(G * A1, G * A1, E * A1);
	Ke2 = Eigen::DiagonalMatrix<double, 3>(G * A2, G * A2, E * A2);
	
	r11 = Eigen::Vector3d{ cos(delta_t1), sin(delta_t1), 0 }*rho1;
	r12 = Eigen::Vector3d{ cos(delta_t1 + Keith::PI_2), sin(delta_t1 + Keith::PI_2), 0 }*rho1;
	r21 = Eigen::Vector3d{ cos(delta_t2), sin(delta_t2), 0 }*rho1;
	r22 = Eigen::Vector3d{ cos(delta_t2 + Keith::PI_2), sin(delta_t2 + Keith::PI_2), 0 }*rho1;
	Eigen::Vector3d e3 = Eigen::Vector3d{ 0, 0, 1 };
	Q1.col(0) = Keith::skew_matrix(r11) * e3;
	Q1.col(1) = Keith::skew_matrix(r12) * e3;
	Q1.col(2) = Keith::skew_matrix(r21) * e3;
	Q1.col(3) = Keith::skew_matrix(r22) * e3;
	Q2.col(0) = Keith::skew_matrix(r21) * e3;
	Q2.col(1) = Keith::skew_matrix(r22) * e3;
	Gc = Eigen::Matrix46d::Zero();


}
SurgicalContinuumManipulator::SurgicalContinuumManipulator(Eigen::Vector2d MP,
	Eigen::Vector7d SP, Eigen::Vector10d FP) :
	  L1o(0.1)
	, Lso(0.04)
	, discrete_element(1e-3)
	, Tgt(Eigen::Matrix4d::Identity())
{
	E = MP(0);
	mu = MP(1);
	d1 = SP(0) * 1e-3;
	d2 = SP(1) * 1e-3;
	Lstem = SP(2) * 1e-3;
	L1 = SP(3) * 1e-3;
	Lr = SP(4) * 1e-3;
	L2 = SP(5) * 1e-3;
	Lg = SP(6) * 1e-3;
	Ldo = SP(5) * 1e-3;
	rho1 = FP(0) * 1e-3;
	rho2 = FP(1) * 1e-3;
	delta_t1 = FP(2);
	delta_t2 = FP(3);
	K1 = FP(4);
	K2 = FP(5);
	zeta = FP(6);
	gamma1 = FP(7);
	gamma2 = FP(8);
	gamma3 = FP(9);

	G = E / 2 / (1 + mu);
	double A1 = Keith::PI * d1 * d1 / 4;
	double A2 = Keith::PI * d2 * d2;
	double I1 = Keith::PI * pow(d1, 4) / 64;
	double I2 = Keith::PI * pow(d2, 4) / 64;
	Kb1 = Eigen::DiagonalMatrix<double, 3>(E * I1, E * I1, 2 * G * I1);
	Kb2 = Eigen::DiagonalMatrix<double, 3>(E * I2, E * I2, 2 * G * I2);
	Ke1 = Eigen::DiagonalMatrix<double, 3>(G * A1, G * A1, E * A1);
	Ke2 = Eigen::DiagonalMatrix<double, 3>(G * A2, G * A2, E * A2);

	r11 = Eigen::Vector3d{ cos(delta_t1), sin(delta_t1), 0 }*rho1;
	r12 = Eigen::Vector3d{ cos(delta_t1 + Keith::PI_2), sin(delta_t1 + Keith::PI_2), 0 }*rho1;
	r21 = Eigen::Vector3d{ cos(delta_t2), sin(delta_t2), 0 }*rho1;
	r22 = Eigen::Vector3d{ cos(delta_t2 + Keith::PI_2), sin(delta_t2 + Keith::PI_2), 0 }*rho1;
	Eigen::Vector3d e3 = Eigen::Vector3d{0, 0, 1 };
	Q1.col(0) = Keith::skew_matrix(r11) * e3;
	Q1.col(1) = Keith::skew_matrix(r12) * e3;
	Q1.col(2) = Keith::skew_matrix(r21) * e3;
	Q1.col(3) = Keith::skew_matrix(r22) * e3;
	Q2.col(0) = Keith::skew_matrix(r21) * e3;
	Q2.col(1) = Keith::skew_matrix(r22) * e3;
	Gc = Eigen::Matrix46d::Zero();
}

void SurgicalContinuumManipulator::setCalibrationPara(Eigen::Vector10d SL)
{
	Eigen::Index leng_SL = SL.size();
	assert(leng_SL == (Eigen::Index)10);
	// name:  L1 Lr L2 Lg zeta K1  K2   gamma1 Lstem gamma3
	// SL = [100 10 20 15 0.2   5  0.6   pi / 40   600 3];
	L1 = SL(0) * 1e-3;
	Lr = SL(1) * 1e-3;
	L2 = SL(2) * 1e-3;
	Lg = SL(3) * 1e-3;
	zeta = SL(4);
	K1 = SL(5);
	K2 = SL(6);
	gamma1 = SL(7);
	Lstem = SL(8) * 1e-3;
	gamma3 = SL(9);
}

void SurgicalContinuumManipulator::setL1r2g(Eigen::Vector4d SL)
{
	L1 = SL(0) * 1e-3;
	Lr = SL(1) * 1e-3;
	L2 = SL(2) * 1e-3;
	Lg = SL(3) * 1e-3;
}

void SurgicalContinuumManipulator::setK12(double K1, double K2)
{
	this->K1 = K1;
	this->K2 = K2;
}

void SurgicalContinuumManipulator::setK1(double K1)
{
	this->K1 = K1;
}

void SurgicalContinuumManipulator::setZeta(double zeta)
{
	this->zeta = zeta;
}

void SurgicalContinuumManipulator::setGamma1(double gamma1)
{
	this->gamma1 = gamma1;
}

void SurgicalContinuumManipulator::setGamma2(double gamma2)
{
	this->gamma2 = gamma2;
}

void SurgicalContinuumManipulator::setGamma3(double gamma3)
{
	this->gamma3 = gamma3;
}

void SurgicalContinuumManipulator::setLstem(double Lstem)
{
	this->Lstem = Lstem * 1e-3;
}

void SurgicalContinuumManipulator::setLdo(double Ldo)
{
	this->Ldo = Ldo * 1e-3;
}

void SurgicalContinuumManipulator::setLso(double feeding_length)
{
	feeding_length = feeding_length * 1e-3;
	if (feeding_length > L1)
	{
		Lso = feeding_length - L1;
		L1o = L1;
	}
	else
	{
		Lso = 0;
		L1o = feeding_length;
	}
	calcMatrices();
}

void SurgicalContinuumManipulator::setBackboneDiameter(double d1, double d2)
{
	this->d1 = d1 * 1e-3;
	this->d2 = d2 * 1e-3;
}

void SurgicalContinuumManipulator::setPitchCircleRadius(double rho1, double rho2)
{
	this->rho1 = rho1 * 1e-3;
	this->rho2 = rho2 * 1e-3;
}

Eigen::Vector10d SurgicalContinuumManipulator::getCalibrationPara()
{
	Eigen::Vector10d SL;
	SL << L1, Lr, L2, Lg, zeta, K1, K2, gamma1, Lstem, gamma3;
	return SL;
}

Eigen::Vector4d SurgicalContinuumManipulator::getL1r2g()
{
	Eigen::Vector4d SL;
	SL << L1, Lr, L2, Lg;
	return SL;
}

double SurgicalContinuumManipulator::getL1()
{
	return L1 * 1e3;
}

double SurgicalContinuumManipulator::getLr()
{
	return Lr * 1e3;
}

double SurgicalContinuumManipulator::getL2()
{
	return L2 * 1e3;
}

double SurgicalContinuumManipulator::getLg()
{
	return Lg * 1e3;
}

double SurgicalContinuumManipulator::getK1()
{
	return K1;
}

double SurgicalContinuumManipulator::getK2()
{
	return K2;
}

double SurgicalContinuumManipulator::getZeta()
{
	return zeta;
}

double SurgicalContinuumManipulator::getGamma1()
{
	return gamma1;
}

double SurgicalContinuumManipulator::getGamma2()
{
	return gamma2;
}

double SurgicalContinuumManipulator::getGamma3()
{
	return gamma3;
}

double SurgicalContinuumManipulator::getLstem()
{
	return Lstem * 1e3;
}

double SurgicalContinuumManipulator::getLdo()
{
	return Ldo * 1e3;
}

double SurgicalContinuumManipulator::getLso()
{
	return Lso * 1e3;
}

double SurgicalContinuumManipulator::getL1o()
{
	return L1o * 1e3;
}

double SurgicalContinuumManipulator::getSeg1BackboneDiamter()
{
	return d1 * 1e3;
}

double SurgicalContinuumManipulator::getSeg2BackboneDiamter()
{
	return d2 * 1e3;
}

double SurgicalContinuumManipulator::getSeg1PitchCircleRadius()
{
	return rho1 * 1e3;
}

double SurgicalContinuumManipulator::getSeg2PitchCircleRadius()
{
	return rho2 * 1e3;
}

Eigen::Vector3d SurgicalContinuumManipulator::getR11()
{
	return r11;
}

Eigen::Vector3d SurgicalContinuumManipulator::getR12()
{
	return r12;
}

Eigen::Vector3d SurgicalContinuumManipulator::getR21()
{
	return r21;
}

Eigen::Vector3d SurgicalContinuumManipulator::getR22()
{
	return r22;
}

Eigen::Matrix34d SurgicalContinuumManipulator::getQ1()
{
	return Q1;
}

Eigen::Matrix32d SurgicalContinuumManipulator::getQ2()
{
	return Q2;
}

Eigen::Matrix3d SurgicalContinuumManipulator::getKb1()
{
	return Kb1;
}

Eigen::Matrix3d SurgicalContinuumManipulator::getKe1()
{
	return Ke1;
}

Eigen::Matrix3d SurgicalContinuumManipulator::getKb2()
{
	return Kb2;
}

Eigen::Matrix3d SurgicalContinuumManipulator::getKe2()
{
	return Ke2;
}

Eigen::Matrix46d SurgicalContinuumManipulator::calcMatrices()
{
	Eigen::Matrix46d dGamma;
	Eigen::Matrix4d Ell;
	Eigen::MatrixXd Ka(6, 6), THETA(6, 4);
	dGamma.topLeftCorner(4, 3) = (L1o + Lso * zeta) * Q1.transpose();
	dGamma.bottomRightCorner(2, 3) = L2 * Q2.transpose();

	Ell.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity() * (L1 + Lstem);
	Ell.bottomRightCorner(2, 2) = Eigen::Matrix2d::Identity() * (L1 + Lstem + Lr + L2);

	THETA.topLeftCorner(3, 4) = -2 * Ke1(3, 3) * Q1;
	THETA.topRightCorner(3, 2) = Eigen::Matrix32d::Zero();
	THETA.bottomRightCorner(3, 2) = -2 * Ke2(3, 3) * Q2;

	Ka.topLeftCorner(3, 3) = 4 * Kb1 + 16 * Kb2 + K1 * Kb1;
	Ka.topLeftCorner(3, 3) = -16 * Kb2 - K2 * Kb1;
	Ka.topLeftCorner(3, 3) = 16 * Kb2 + K2 * Kb1;
	
	Gc = dGamma - Ell * Keith::pinv(THETA) * Ka;
	return Gc;
}
