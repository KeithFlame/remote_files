#pragma once
#include <Eigen/dense>
#include "eigen_extensions.h"


struct ContinuumSegment
{
public:
	ContinuumSegment();
	ContinuumSegment(const Eigen::Vector8d);

	//void setBackboneDiameter(const double);
	//void setPitchCircleRadius(const double);
	//void setSegmentLength(const double);
	//void setSegmentDiameter(const double);
	//void setGamma(const double);
	//void setRigidStemLength(const double);
	//void setBackboneLength(const double);
	//void setK(const double);

	//const double getBackboneDiameter();
	//const double getPitchCircleRadius();
	//const double getSegmentLength();
	//const double getSegmentDiameter();
	//const double getGamma();
	//const double getRigidStemLength();
	//const double getBackboneLength();
	//const double getK();

//private:
	// structual properties
	double backbone_diameter;					// diameter of the nitinol backbone in this segment.
	double pitch_circle_radius;					// radius of pitch circle, where all backbones are loacted with respect to cross section. 
	double segment_length;						// length of this segment.
	double segment_diameter;					// diameter of this segment.
	double rigid_stem_length;					// length of rigid stem, connected with the end of continuum segment.
	double backbone_length;						// length of the nitinol backbone of this segment (backbone_length > segment_length).

	// movement properties
	double gamma;								// angle between the ray from center to first backbone and x-axis of {te}.

	// stiffness properties
	double K;									// stiffness ratio parameters from other structure in this segment.
};
