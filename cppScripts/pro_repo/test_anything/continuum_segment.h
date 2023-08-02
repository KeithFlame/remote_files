#pragma once
#ifndef CONTINUUM_SEGMENT_H_
#define CONTINUUM_SEGMENT_H_
#include <Eigen/Core>
#include "math_extentions_keith.h"

class ContinuumSegment {
public:
	ContinuumSegment();
	ContinuumSegment(ContinuumSegment&);
	ContinuumSegment& operator=(ContinuumSegment&);
	~ContinuumSegment() {};
	void refreshWholeStructure();
	void showAllParameters();
	void setE(float);
	void setMu(float);
	void setD(float);
	void setL(float);
	void setL0(float);
	void setRho(float);
	void setDeltaT(float);
	void setK(float);
	void setGamma(float);
	void refreshStiffness();
	void refreshR12();
	Eigen::Vector3f getR1();
	Eigen::Vector3f getR2();
	Eigen::Matrix3f getKe();
	Eigen::Matrix3f getKb();
	float getL();
	float getL0();
	float getA();
	float getE();
	float getK();
	float getMu();
	float getD();
	float getRho();
	float getDelta_t();
	float getGamma();
private:

	// material properties
	float E;										// Elastic modulus
	float mu;										// Possion rate
	float G;										// Shear modulus

	// structural properties
	float d;										// Rod diameter
	float L;										// Segment length
	float L0;										// Segment rigid
	float A;
	Eigen::Matrix3f Ke;								// Stiffness matrix
	Eigen::Matrix3f Kb;								// Stiffness matrix

	// fabrication properties
	float rho;										// Rod pitch circle radius
	float delta_t;									// Angle between first rod and Y-axis
	float K;										// Stiffness coefficient
	float gamma;									// Offset for rotation
	Eigen::Vector3f r1;								// Position for first rod in XY plane
	Eigen::Vector3f r2;								// Position for second rod in XY plane
	

};
#endif // !CONTINUUM_SEGMENT_H_
