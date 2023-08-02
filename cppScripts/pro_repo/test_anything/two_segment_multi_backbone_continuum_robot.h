#pragma once
#ifndef TWO_SEGMENT_MULTI_BACKBONE_CONTINUUM_ROBOT_H_
#define TWO_SEGMENT_MULTI_BACKBONE_CONTINUUM_ROBOT_H_

#include <Eigen/Dense>
#include "continuum_segment.h"

class TwoSegmentMultiBackboneContinuumRobot {
	
public:
	TwoSegmentMultiBackboneContinuumRobot(ContinuumSegment, ContinuumSegment);
	TwoSegmentMultiBackboneContinuumRobot(TwoSegmentMultiBackboneContinuumRobot& );
	TwoSegmentMultiBackboneContinuumRobot& operator=(TwoSegmentMultiBackboneContinuumRobot& );
	ContinuumSegment seg1;
	ContinuumSegment seg2;
	Eigen::MatrixXf Q1;
	Eigen::MatrixXf Q2;
	Eigen::MatrixXf Gc;
	void resetCalibrationPara(Eigen::VectorXf);
	void resetL1r2g(Eigen::Vector4f);
	void resetK12(Eigen::Vector2f);
	void resetZeta(float);
	void resetGamma1(float);
	void resetLstem(float);
	void resetLso(float);
	void setL_do(float);
	void calcMatrix();

	float getLso();
	float getL1o();
	void setDiscreteElement(float);
	float getDiscreteElement();
	float getLr();
	float getLg();
	float getLstem();
	float getL1();
	float getL2();
	float getK1();
	float getK2();
	float getZeta();
	float getL_do();
	float getGamma1();
	Eigen::Matrix3f getKb1();
	Eigen::Matrix3f getKe1();
	Eigen::Matrix3f getKb2();
	Eigen::Matrix3f getKe2();
	void showAllParameter();
private:
	void calcQ12();
	float zeta;										// Bending performance
	float l_do;
	float Lso;
	float L1o;
	float Lstem;
	float gamma1;
	float discrete_element;
};



#endif // !TWO_SEGMENT_MULTI_BACKBONE_CONTINUUM_ROBOT_H_
