#pragma once
#include "continuum_segment.h"

ContinuumSegment::ContinuumSegment():
	backbone_diameter(0.95),
	pitch_circle_radius(2.5),
	segment_length(100.0),
	segment_diameter(8.0),
	gamma(0.0),
	rigid_stem_length(10.0),
	backbone_length(600.0),
	K(5.0)
{
}

ContinuumSegment::ContinuumSegment(const Eigen::Vector8d para):
	backbone_diameter(para[0]),
	pitch_circle_radius(para[1]),
	segment_length(para[2]),
	segment_diameter(para[3]),
	gamma(para[4]),
	rigid_stem_length(para[5]),
	backbone_length(para[6]),
	K(para[7])
{
}
//
//void ContinuumSegment::setBackboneDiameter(const double backbone_diameter)
//{
//	this->backbone_diameter = backbone_diameter;
//}
//
//void ContinuumSegment::setPitchCircleRadius(const double pitch_circle_radius)
//{
//	this->pitch_circle_radius = pitch_circle_radius;
//}
//
//void ContinuumSegment::setSegmentLength(const double segment_length)
//{
//	this->segment_length = segment_length;
//}
//
//void ContinuumSegment::setSegmentDiameter(const double segment_diameter)
//{
//	this->segment_diameter = segment_diameter;
//}
//
//void ContinuumSegment::setGamma(const double gamma)
//{
//	this->gamma = gamma;
//}
//
//void ContinuumSegment::setRigidStemLength(const double rigid_stem_length)
//{
//	this->rigid_stem_length = rigid_stem_length;
//}
//
//void ContinuumSegment::setBackboneLength(const double backbone_length)
//{
//	this->backbone_length = backbone_length;
//}
//
//void ContinuumSegment::setK(const double K)
//{
//	this->K=K;
//}
//
//const double ContinuumSegment::getBackboneDiameter()
//{
//	return backbone_diameter;
//}
//
//const double ContinuumSegment::getPitchCircleRadius()
//{
//	return pitch_circle_radius;
//}
//
//const double ContinuumSegment::getSegmentLength()
//{
//	return segment_length;
//}
//
//const double ContinuumSegment::getSegmentDiameter()
//{
//	return segment_diameter;
//}
//
//const double ContinuumSegment::getGamma()
//{
//	return gamma;
//}
//
//const double ContinuumSegment::getRigidStemLength()
//{
//	return rigid_stem_length;
//}
//
//const double ContinuumSegment::getBackboneLength()
//{
//	return backbone_length;
//}
//
//const double ContinuumSegment::getK()
//{
//	return K;
//}
