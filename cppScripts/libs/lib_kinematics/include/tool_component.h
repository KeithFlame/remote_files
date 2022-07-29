#pragma once
#ifndef TOOL_COMPONENT_H_
#define TOOL_COMPONENT_H_
#include <string>
struct ConfigSpaceLimit
{
	float l_max_limit;
	float l_min_limit;
	float phi_limit;
	float theta1_limit;
	float theta2_limit;

};

struct StructurePara
{
	float Lstem;
	float zeta;
	float gamma_1;
	float gamma_3;
	float L1x;
	float d;
	float K1;
	float K2;
};
struct PhisicalPara
{
	float Lcnla;
	float Lprox;
	float d1;
	float d2;
	float E;
	float poisson_ratio;
	float r1;
	float r2;
	float r3;
};
struct Segment
{
	float L1;
	float Lr;
	float L2;
	float Lg;
};
struct PoseError
{
	float epsilon_position;
	float epsilon_direction;
	float epsilon_force;
	float epsilon_moment;
};

struct PoseCalculation
{
	float dt;
	float v_limit;
	float w_limit;
	float v_limit_thred;

};
struct ToolArm
{
	std::string arm_name;
	std::string arm_serial;
	Segment segment;
	StructurePara sp;
	ConfigSpaceLimit csl;
	PoseError pe;
	PhisicalPara pp;
	PoseCalculation pc;
};
#endif // !TOOL_COMPONENT_H_

