#pragma once

typedef struct _SEGMENT_JOINT_SPACE_ {
	double actuation_length1;						// one actuation length for this continuum segment. [mm]
	double actuation_length2;						// another actuation length for this continuum segment. [mm]
}SegmentJointSpace;

typedef struct _SEGMENT_CONFIGURATION_SPACE_ {
	double bending_angle;							// bending angle for this continuum segment. [rad]
	double bending_plane_angle;						// bending plane angle length for this continuum segment. [rad]
}SegmentConfigurationSpace;

typedef struct _TWO_SEGMENT_TOOL_JOINT_SPACE_ {
	double feeding_length;							// feeding length of the feeding motor. [mm]
	double rotation_angle;							// rotation angle of the rotation motor. [rad]
	SegmentJointSpace seg1;							// the first segment of this tool. [mm, mm]
	SegmentJointSpace seg2;							// the second segment of this tool. [mm, mm]
}TwoSegmentToolJointSpace;

typedef struct _TWO_SEGMENT_TOOL_CONFIGURATION_SPACE_ {
	double feeding_length;							// feeding length of the feeding motor. [mm]
	double rotation_angle;							// rotation angle of the rotation motor. [rad]
	SegmentConfigurationSpace seg1;					// the first segment of this tool. [rad, rad]
	SegmentConfigurationSpace seg2;					// the second segment of this tool. [rad, rad]
}TwoSegmentToolConfigurationSpace;

typedef struct _TASK_SPACE_AXISANGLE_ {
	double x;										// position x in task space. [mm]
	double y;										// position y in task space. [mm]
	double z;										// position z in task space. [mm]
	double axang1;									// rotation axis X rotation angle of the AxisAngle of the Eigen.
	double axang2;									// rotation axis X rotation angle of the AxisAngle of the Eigen.
	double axang3;									// rotation axis X rotation angle of the AxisAngle of the Eigen.
}TaskSpaceAxisAngle;

typedef struct _TASK_SPACE_EULER_ {
	double x;										// position x in task space. [mm]
	double y;										// position y in task space. [mm]
	double z;										// position z in task space. [mm]
	double pitch;									// rotation axis X rotation angle of the AxisAngle of the Eigen.
	double yaw;										// rotation axis X rotation angle of the AxisAngle of the Eigen.
	double roll;									// rotation axis X rotation angle of the AxisAngle of the Eigen.
}TaskSpaceEuler;