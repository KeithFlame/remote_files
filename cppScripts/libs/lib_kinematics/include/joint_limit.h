#pragma once
#ifndef JOINT_LIMIT_H_
#define JOINT_LIMIT_H_
#include <string>
#include "tool_component.h"
#include "configration_space.h"

class JointLimit
{
public:
	static int judgeJointLimit(ConfigSpace&, ConfigSpaceLimit);
	static bool judgePseudoJointLimit(ConfigSpace&);

private:
	static bool limitL(float&, float, float);
	static bool limitPhi(ConfigSpace&, float, float);
	static bool limitTheta(float&, float, float);
	JointLimit();
};

#endif // !JOINT_LIMIT_H_

