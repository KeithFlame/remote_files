#pragma once
#include <Eigen/dense>
#include "eigen_extensions.h"

class SurgicalContinuumManipulator
{
public:
	SurgicalContinuumManipulator();
	SurgicalContinuumManipulator(Eigen::Vector2d MP,
		Eigen::Vector7d SP, Eigen::Vector10d FP);
	void setCalibrationPara(Eigen::Vector10d SL);
	void setL1r2g(Eigen::Vector4d SL);
	void setK12(double K1, double K2);
	void setK1(double K1);
	void setZeta(double zeta);
	void setGamma1(double gamma1);
	void setGamma2(double gamma2);
	void setGamma3(double gamma3);
	void setLstem(double Lstem);
	void setLdo(double Ldo);
	void setLso(double feeding_length);
	void setDiscreteElement(double discrete_element);
	void setBackboneDiameter(double d1, double d2);
	void setPitchCircleRadius(double rho1, double rho2);

	Eigen::Vector10d getCalibrationPara();
	Eigen::Vector4d getL1r2g();
	double getL1();
	double getLr();
	double getL2();
	double getLg();
	double getK1();
	double getK2();
	double getZeta();
	double getGamma1();
	double getGamma2();
	double getGamma3();
	double getLstem();
	double getLdo();
	double getLso();
	double getL1o();
	double getLcnla();
	double getDiscreteElement();
	double getSeg1BackboneDiamter();
	double getSeg2BackboneDiamter();
	double getSeg1PitchCircleRadius();
	double getSeg2PitchCircleRadius();
	Eigen::Vector3d getL12Zeta();
	Eigen::Vector3d getR11();
	Eigen::Vector3d getR12();
	Eigen::Vector3d getR21();
	Eigen::Vector3d getR22();
	Eigen::Matrix34d getQ1();
	Eigen::Matrix32d getQ2();
	Eigen::Matrix3d getKb1();
	Eigen::Matrix3d getKe1();
	Eigen::Matrix3d getKb2();
	Eigen::Matrix3d getKe2();
	Eigen::Matrix46d getGc();


	Eigen::Matrix46d calcMatrices();
private:
	// material properties
	double E;
	double mu;
	double G;

	// structual properties
	double d1;
	double d2;
	double L1;
	double Lr;
	double L2;
	double Lg;
	double Ldo;
	double Lstem;
	double Lcnla;
	Eigen::Matrix3d Ke1;
	Eigen::Matrix3d Ke2;
	Eigen::Matrix3d Kb1;
	Eigen::Matrix3d Kb2;

	// fabrication properties
	double rho1;
	double rho2;
	double delta_t1;
	double delta_t2;
	double K1;
	double K2;
	double zeta;
	double gamma1;
	double gamma2;
	double gamma3;

	//intermediate variable
	Eigen::Vector3d r11;
	Eigen::Vector3d r12;
	Eigen::Vector3d r21;
	Eigen::Vector3d r22;
	Eigen::Matrix34d Q1;
	Eigen::Matrix32d Q2;
	double discrete_element;

	// service properties
	double L1o;
	double Lso;

	// a matrix from curvature to actuation 
	Eigen::Matrix46d Gc;

	// a transformation matrix.
	Eigen::Matrix4d Tgt;
};

