#pragma once
#include <Eigen/dense>
#include "continuum_segment.h"

class TwoSegmentTool
{
public:
	TwoSegmentTool();
	TwoSegmentTool(const ContinuumSegment,
		const ContinuumSegment, const ContinuumSegment);
	void setCalibrationPara(const Eigen::Vector10d SL);
	void setL1r2g(const Eigen::Vector4d);
	void setK(const Eigen::Vector3d);
	void setGamma(const Eigen::Vector3d);
	void setBackboneLength(const Eigen::Vector2d);
	void setPitchCircleRadius(const Eigen::Vector2d);
	void setBackboneDiameter(const Eigen::Vector2d);

	const double getL1();
	const Eigen::Vector10d getCalibrationPara();
	const Eigen::Vector4d getL1r2g();
	const Eigen::Vector3d getK();
	const Eigen::Vector3d getGamma();
	const Eigen::Vector2d getBackboneLength();
	const Eigen::Vector2d getPitchCircleRadius();
	const Eigen::Vector2d getBackboneDiameter();

	Eigen::Vector3d r11;
	Eigen::Vector3d r12;
	Eigen::Vector3d r21;
	Eigen::Vector3d r22;
	Eigen::Matrix34d Q1;
	Eigen::Matrix32d Q2;
	Eigen::Matrix3d Kb1;
	Eigen::Matrix3d Ke1;
	Eigen::Matrix3d Kb2;
	Eigen::Matrix3d Ke2;

private:
	void calcVecandMat();
	ContinuumSegment seg0;
	ContinuumSegment seg1;
	ContinuumSegment seg2;
};

