#pragma once
#ifndef JOINT_SPACE_H_
#define JOINT_SPACE_H_
struct JointSpace
{
	float q_linear;
	float q_rotation;
	float q_seg1_1;
	float q_seg1_2;
	float q_seg2_1;
	float q_seg2_2;
	float q_clamp;

};
#endif // !JOINT_SPACE_H_
