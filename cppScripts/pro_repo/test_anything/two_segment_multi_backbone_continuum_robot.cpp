#include "two_segment_multi_backbone_continuum_robot.h"
#include <iostream>

TwoSegmentMultiBackboneContinuumRobot::TwoSegmentMultiBackboneContinuumRobot(ContinuumSegment seg1, ContinuumSegment seg2):
	zeta(0.1f)
	, Lso(0.f)
	, L1o(0.1f)
	, Lstem(0.6f)
	, gamma1(-keith_used::TSCR_PI/4.f)
	, discrete_element(1e-3f)
	, l_do(0.02f)
{
	this->seg1 = seg1;
	this->seg2 = seg2;
	Q1 = Eigen::MatrixXf::Zero(3, 4);
	Q2 = Eigen::MatrixXf::Zero(3, 2);
	calcQ12();
	Gc = Eigen::MatrixXf::Zero(4, 6);
	calcMatrix();
}

TwoSegmentMultiBackboneContinuumRobot::TwoSegmentMultiBackboneContinuumRobot(TwoSegmentMultiBackboneContinuumRobot& tcr)
{
	zeta = tcr.getZeta();
	Lso = tcr.getLso();
	L1o = tcr.getL1o();
	Lstem = tcr.getLstem();
	gamma1 = tcr.getGamma1();
	discrete_element = tcr.getDiscreteElement();
	l_do = tcr.getL_do();
	seg1 = tcr.seg1;
	seg2 = tcr.seg2;
	Q1 = Eigen::MatrixXf::Zero(3, 4);
	Q2 = Eigen::MatrixXf::Zero(3, 2);
	calcQ12();
	Gc = Eigen::MatrixXf::Zero(4, 6);
	calcMatrix();
}

TwoSegmentMultiBackboneContinuumRobot& TwoSegmentMultiBackboneContinuumRobot::operator=(TwoSegmentMultiBackboneContinuumRobot& tcr)
{
	this->zeta = tcr.getZeta();
	this->Lso = tcr.getLso();
	this->L1o = tcr.getL1o();
	this->Lstem = tcr.getLstem();
	this->gamma1 = tcr.getGamma1();
	this->discrete_element = tcr.getDiscreteElement();
	this->l_do = tcr.getL_do();
	this->seg1 = tcr.seg1;
	this->seg2 = tcr.seg2;
	this->Q1 = Eigen::MatrixXf::Zero(3, 4);
	this->Q2 = Eigen::MatrixXf::Zero(3, 2);
	this->calcQ12();
	this->Gc = Eigen::MatrixXf::Zero(4, 6);
	this->calcMatrix();
	return *this;
}

void TwoSegmentMultiBackboneContinuumRobot::resetCalibrationPara(Eigen::VectorXf SL)
{
	// name:  L1 Lr L2 Lg zeta K1  K2   gamma1 Lstem
	// SL = [100 10 20 15 0.2   5  0.6   pi / 40   600];
	Eigen::Index c = SL.size();
	switch (c)
	{
	case 0:
		break;
	case 1:
	{
		seg1.setL(SL(0)); break;
	}
	case 2:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		break;
	}
	case 3:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		seg2.setL(SL(2));
		break;
	}
	case 4:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		seg2.setL(SL(2));
		seg2.setL0(SL(3));
		break;
	}
	case 5:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		seg2.setL(SL(2));
		seg2.setL0(SL(3));
		zeta = SL(4);
		break;
	}
	case 6:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		seg2.setL(SL(2));
		seg2.setL0(SL(3));
		zeta = SL(4);
		seg1.setK(SL(5));
		break;
	}
	case 7:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		seg2.setL(SL(2));
		seg2.setL0(SL(3));
		zeta = SL(4);
		seg1.setK(SL(5));
		seg2.setK(SL(6));
		break;
	}
	case 8:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		seg2.setL(SL(2));
		seg2.setL0(SL(3));
		zeta = SL(4);
		seg1.setK(SL(5));
		seg2.setK(SL(6));
		gamma1 = SL(7);
		break;
	}
	case 9:
	{
		seg1.setL(SL(0));
		seg1.setL0(SL(1));
		seg2.setL(SL(2));
		seg2.setL0(SL(3));
		zeta = SL(4);
		seg1.setK(SL(5));
		seg2.setK(SL(6));
		gamma1 = SL(7);
		Lstem = SL(9);
		break;
	}
	default:
		break;
	}
}

void TwoSegmentMultiBackboneContinuumRobot::resetL1r2g(Eigen::Vector4f SL)
{
	seg1.setL(SL(0));
	seg1.setL0(SL(1));
	seg2.setL(SL(2));
	seg2.setL0(SL(3));
}

void TwoSegmentMultiBackboneContinuumRobot::resetK12(Eigen::Vector2f SL)
{
	seg1.setK(SL(0));
	seg2.setK(SL(1));
}

void TwoSegmentMultiBackboneContinuumRobot::resetZeta(float zeta)
{
	this->zeta = zeta;
}

void TwoSegmentMultiBackboneContinuumRobot::resetGamma1(float gamma1)
{
	this->gamma1 = gamma1;
}

void TwoSegmentMultiBackboneContinuumRobot::resetLstem(float Lstem)
{
	this->Lstem = Lstem;
}

void TwoSegmentMultiBackboneContinuumRobot::resetLso(float l)
{
	if (l > seg1.getL())
	{
		Lso = l - seg1.getL();
		L1o = seg1.getL();
	}
	else
	{
		Lso = 0;
		L1o = l;
	}
	calcMatrix();
}

void TwoSegmentMultiBackboneContinuumRobot::setL_do(float l_do)
{
	this->l_do = l_do;
}

void TwoSegmentMultiBackboneContinuumRobot::calcMatrix()
{
	Eigen::MatrixXf dGamma(4, 6), Ell(4, 4), THETA(6, 4), THETA_(4, 6), Ka(6, 6);
	dGamma = Eigen::MatrixXf::Zero(4, 6);
	dGamma.block(0, 0, 4, 3) = Q1.transpose() * (Lso * zeta + L1o);
	dGamma.block(2, 3, 2, 3) = Q2.transpose() * seg2.getL();

	Ell = Eigen::MatrixXf::Zero(4, 4);
	Ell.block(0, 0, 2, 2) = Eigen::Matrix2f::Identity() * (seg1.getL() + Lstem);
	Ell.block(2, 2, 2, 2) = Eigen::Matrix2f::Identity() * (seg2.getL() + seg1.getL0() + seg1.getL() + Lstem);

	THETA = Eigen::MatrixXf::Zero(6, 4);
	THETA.block(0, 0, 3, 4) = -seg1.getA() * seg1.getE() * Q1;
	THETA.block(0, 2, 3, 2) = 0.f * Q2;
	THETA.block(3, 2, 3, 2) = -seg2.getA() * seg2.getE() * Q2;
	keith_used::pinv(THETA_, THETA);

	Ka = Eigen::MatrixXf::Zero(6, 6);
	Ka.block(0, 0, 3, 3) = 16.f * seg2.getKb() + (seg1.getK() + 4.f) * seg1.getKb();
	Ka.block(0, 3, 3, 3) = -16.f * seg2.getKb() - seg2.getK() * seg1.getKb();
	Ka.block(3, 3, 3, 3) = 16.f * seg2.getKb() + seg2.getK() * seg1.getKb();

	Gc = dGamma - Ell * THETA_ / 2.f * Ka;
}

float TwoSegmentMultiBackboneContinuumRobot::getLso()
{
	return Lso;
}

float TwoSegmentMultiBackboneContinuumRobot::getL1o()
{
	return L1o;
}

void TwoSegmentMultiBackboneContinuumRobot::setDiscreteElement(float discrete_element)
{
	this->discrete_element = discrete_element;
}

float TwoSegmentMultiBackboneContinuumRobot::getDiscreteElement()
{
	return discrete_element;
}

float TwoSegmentMultiBackboneContinuumRobot::getLr()
{
	return seg1.getL0();
}

float TwoSegmentMultiBackboneContinuumRobot::getLg()
{
	return seg2.getL0();
}

float TwoSegmentMultiBackboneContinuumRobot::getLstem()
{
	return Lstem;
}

float TwoSegmentMultiBackboneContinuumRobot::getL1()
{
	return seg1.getL();
}

float TwoSegmentMultiBackboneContinuumRobot::getL2()
{
	return seg2.getL();
}

float TwoSegmentMultiBackboneContinuumRobot::getK1()
{
	return seg1.getK();
}

float TwoSegmentMultiBackboneContinuumRobot::getK2()
{
	return seg2.getK();
}

float TwoSegmentMultiBackboneContinuumRobot::getZeta()
{
	return zeta;
}

float TwoSegmentMultiBackboneContinuumRobot::getL_do()
{
	return l_do;
}

float TwoSegmentMultiBackboneContinuumRobot::getGamma1()
{
	return gamma1;
}

Eigen::Matrix3f TwoSegmentMultiBackboneContinuumRobot::getKb1()
{
	return seg1.getKb();
}

Eigen::Matrix3f TwoSegmentMultiBackboneContinuumRobot::getKe1()
{
	return seg1.getKe();
}

Eigen::Matrix3f TwoSegmentMultiBackboneContinuumRobot::getKb2()
{
	return seg2.getKb();
}

Eigen::Matrix3f TwoSegmentMultiBackboneContinuumRobot::getKe2()
{
	return seg2.getKe();
}

void TwoSegmentMultiBackboneContinuumRobot::showAllParameter()
{
	std::cout << std::endl << "-------------------------- seg1 parameters --------------------------" << std::endl << std::endl;
	seg1.showAllParameters();
	std::cout << std::endl << "-------------------------- seg2 parameters --------------------------" << std::endl << std::endl;
	seg2.showAllParameters();
	std::cout << std::endl << "-------------------------- arm parameters ---------------------------" << std::endl << std::endl;
	std::cout << "zeta   : " << zeta << std::endl;
	std::cout << "Lso    : " << Lso << std::endl;
	std::cout << "L1o    : " << L1o << std::endl;
	std::cout << "Lstem  : " << Lstem << std::endl;
	std::cout << "gamma1 : " << gamma1 << std::endl;
	std::cout << "l_do   : " << l_do << std::endl;
	std::cout << "Gc     : " << std::endl << Gc << std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

}

void TwoSegmentMultiBackboneContinuumRobot::calcQ12()
{
	Eigen::Vector3f e3 = { 0.f,0.f,1.f };
	Eigen::Vector3f q1, q2, q3, q4;
	q1 = seg1.getR1().cross(e3);
	q2 = seg1.getR2().cross(e3);
	q3 = seg2.getR1().cross(e3);
	q4 = seg2.getR2().cross(e3);
	Q1.col(0) = q1;
	Q1.col(1) = q2;
	Q1.col(2) = q3;
	Q1.col(3) = q4;
	Q2.col(0) = q3;
	Q2.col(1) = q4;
}

