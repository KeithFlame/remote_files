#include "two_segment_tool.h"
#include "math_extensions.h"
#include "constant_definition.h"

TwoSegmentTool::TwoSegmentTool()
{
	ContinuumSegment seg;
	seg0 = seg;
	seg1 = seg;
	seg2 = seg;
	calcVecandMat();
}

TwoSegmentTool::TwoSegmentTool(const ContinuumSegment seg0, const ContinuumSegment seg1, const ContinuumSegment seg2)
{
	this->seg0 = seg0;
	this->seg1 = seg1;
	this->seg2 = seg2;
	calcVecandMat();
}

void TwoSegmentTool::setCalibrationPara(const Eigen::Vector10d SL)
{
	Eigen::Index leng_SL = SL.size();
	assert(leng_SL == (Eigen::Index)10);
	// name:  L1 Lr L2 Lg zeta K1  K2   gamma1 Lstem gamma3
	// SL = [100 10 20 15 0.2   5  0.6   pi / 40   600 3];
	seg1.segment_length = SL(0);
	seg1.rigid_stem_length = SL(1);
	seg2.segment_length = SL(2);
	seg2.rigid_stem_length = SL(3);
	seg0.K = SL(4);
	seg1.K = SL(5);
	seg2.K = SL(6);
	seg0.gamma = SL(7);
	double Lstem = SL(8);
	seg0.segment_length = Lstem;
	seg0.backbone_length = Lstem;
	double Lcnla = 0.0;
	seg1.backbone_length = seg1.segment_length + seg0.backbone_length + seg0.rigid_stem_length;
	seg2.backbone_length = seg2.segment_length + seg1.backbone_length + seg1.rigid_stem_length + Lcnla;
	seg2.gamma = SL(9);
	calcVecandMat();
}

void TwoSegmentTool::setL1r2g(const Eigen::Vector4d SL)
{
	seg1.segment_length = SL(0);
	seg1.rigid_stem_length = SL(1);
	seg2.segment_length = SL(2);
	seg2.rigid_stem_length = SL(3);
}

void TwoSegmentTool::setK(const Eigen::Vector3d K)
{
	seg0.K = K[0];
	seg1.K = K[1];
	seg2.K = K[2];
}

void TwoSegmentTool::setGamma(const Eigen::Vector3d gamma)
{
	seg0.gamma = gamma[0];
	seg1.gamma = gamma[1];
	seg2.gamma = gamma[2];
	calcVecandMat();
}

void TwoSegmentTool::setBackboneLength(const Eigen::Vector2d backbone_length)
{
	seg1.backbone_length = backbone_length[0];
	seg2.backbone_length = backbone_length[1];
}

void TwoSegmentTool::setPitchCircleRadius(const Eigen::Vector2d pitch_circle_radius)
{
	seg1.pitch_circle_radius = pitch_circle_radius[0];
	seg2.pitch_circle_radius = pitch_circle_radius[1];
	calcVecandMat();
}

void TwoSegmentTool::setBackboneDiameter(const Eigen::Vector2d backbone_diameter)
{
	seg1.backbone_diameter = backbone_diameter[0];
	seg2.backbone_diameter = backbone_diameter[1];
	calcVecandMat();
}

const double TwoSegmentTool::getL1()
{
	return seg1.segment_length;
}

const Eigen::Vector10d TwoSegmentTool::getCalibrationPara()
{
	Eigen::Vector10d SL;
	// name:  L1 Lr L2 Lg zeta K1  K2   gamma1 Lstem gamma3
	// SL = [100 10 20 15 0.2   5  0.6   pi / 40   600 3];
	SL(0) = seg1.segment_length;
	SL(1) = seg1.rigid_stem_length;
	SL(2) = seg2.segment_length;
	SL(3) = seg2.rigid_stem_length;
	SL(4) = seg0.K;
	SL(5) = seg1.K;
	SL(6) = seg2.K;
	SL(7) = seg0.gamma;
	SL(8) = seg0.segment_length;
	SL(9) = seg2.gamma;
	return SL;
}

const Eigen::Vector4d TwoSegmentTool::getL1r2g()
{
	Eigen::Vector4d L1r2g;
	L1r2g(0) = seg1.segment_length;
	L1r2g(1) = seg1.rigid_stem_length;
	L1r2g(2) = seg2.segment_length;
	L1r2g(3) = seg2.rigid_stem_length;
	return L1r2g;
}

const Eigen::Vector3d TwoSegmentTool::getK()
{
	Eigen::Vector3d K;
	K(0) = seg0.K;
	K(1) = seg1.K;
	K(2) = seg2.K;
	return K;
}

const Eigen::Vector3d TwoSegmentTool::getGamma()
{
	Eigen::Vector3d gamma;
	gamma(0) = seg0.gamma;
	gamma(1) = seg1.gamma;
	gamma(2) = seg2.gamma;
	return gamma;
}

const Eigen::Vector2d TwoSegmentTool::getBackboneLength()
{
	Eigen::Vector2d backbone_length;
	backbone_length(0) = seg1.backbone_length;
	backbone_length(1) = seg2.backbone_length;
	return backbone_length;
}

const Eigen::Vector2d TwoSegmentTool::getPitchCircleRadius()
{
	Eigen::Vector2d pitch_circle_radius;
	pitch_circle_radius(0) = seg1.pitch_circle_radius;
	pitch_circle_radius(1) = seg2.pitch_circle_radius;
	return pitch_circle_radius;
}

const Eigen::Vector2d TwoSegmentTool::getBackboneDiameter()
{
	Eigen::Vector2d backbone_diameter;
	backbone_diameter(0) = seg1.backbone_diameter;
	backbone_diameter(1) = seg2.backbone_diameter;
	return backbone_diameter;
}


void TwoSegmentTool::calcVecandMat()
{
	r11 = Eigen::Vector3d{ cos(seg0.gamma), sin(seg0.gamma), 0 }*seg1.pitch_circle_radius;
	r12 = Eigen::Vector3d{ cos(seg0.gamma + Keith::PI_2), sin(seg0.gamma + Keith::PI_2), 0 }*seg1.pitch_circle_radius;
	r21 = Eigen::Vector3d{ cos(seg1.gamma), sin(seg1.gamma), 0 }*seg2.pitch_circle_radius;
	r22 = Eigen::Vector3d{ cos(seg1.gamma + Keith::PI_2), sin(seg1.gamma + Keith::PI_2), 0 }*seg2.pitch_circle_radius;
	Eigen::Vector3d e3 = Eigen::Vector3d{ 0, 0, 1 };
	Q1.col(0) = Keith::skew_matrix(r11) * e3;
	Q1.col(1) = Keith::skew_matrix(r12) * e3;
	Q1.col(2) = Keith::skew_matrix(r21) * e3;
	Q1.col(3) = Keith::skew_matrix(r22) * e3;
	Q2.col(0) = Keith::skew_matrix(r21) * e3;
	Q2.col(1) = Keith::skew_matrix(r22) * e3;
	Q1.row(2) = Eigen::Vector4d::Zero();
	Q2.row(2) = Eigen::Vector2d::Zero();
	double A1 = Keith::PI * seg1.backbone_diameter * seg1.backbone_diameter / 4;
	double A2 = Keith::PI * seg2.backbone_diameter * seg2.backbone_diameter;
	double I1 = Keith::PI * pow(seg1.backbone_diameter, 4) / 64;
	double I2 = Keith::PI * pow(seg2.backbone_diameter, 4) / 64;

	MODULE_NITINOL nitinol_module;
	Kb1 = Eigen::DiagonalMatrix<double, 3>(nitinol_module.E * I1, nitinol_module.E * I1, 2 * nitinol_module.G * I1);
	Kb2 = Eigen::DiagonalMatrix<double, 3>(nitinol_module.E * I2, nitinol_module.E * I2, 2 * nitinol_module.G * I2);
	Ke1 = Eigen::DiagonalMatrix<double, 3>(nitinol_module.G * A1, nitinol_module.G * A1, nitinol_module.E * A1);
	Ke2 = Eigen::DiagonalMatrix<double, 3>(nitinol_module.G * A2, nitinol_module.G * A2, nitinol_module.E * A2);
}
