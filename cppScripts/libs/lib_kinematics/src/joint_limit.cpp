#include "joint_limit.h"
#include "math_define.h"

int JointLimit::judgeJointLimit(ConfigSpace& psi, ConfigSpaceLimit csl)
{

	return 1000*limitL(psi.l, csl.l_min_limit, csl.l_max_limit) +
		100*limitPhi(psi, -csl.phi_limit, csl.phi_limit) +
		10*limitTheta(psi.theta1, -csl.theta1_limit, csl.theta1_limit) +
		limitTheta(psi.theta2, -csl.theta2_limit, csl.theta2_limit);
}

bool JointLimit::judgePseudoJointLimit(ConfigSpace& psi)
{
	bool is_limit = false;
	while (psi.delta1 > PI)
	{
		psi.delta1 -= 2 * PI;
		is_limit = true;
	}
	while (psi.delta1 < -PI)
	{
		psi.delta1 += 2 * PI;
		is_limit = true;
	}
	while (psi.delta2 > PI)
	{
		psi.delta2 -= 2 * PI;
		is_limit = true;
	}
	while (psi.delta2 < -PI)
	{
		psi.delta2 += 2 * PI;
		is_limit = true;
	}

	if (psi.theta1 < 0.0)
	{
		is_limit = true;
		psi.theta1 *= -1.0;
		if (psi.delta1 < 0)
		{
			psi.delta1 += PI;
		}
		else
		{
			psi.delta1 -= PI;
		}
	}
	if (psi.theta2 < 0.0)
	{
		is_limit = true;
		psi.theta2 *= -1.0;
		if (psi.delta2 < 0)
		{
			psi.delta2 += PI;
		}
		else
		{
			psi.delta2 -= PI;
		}
	}
	return is_limit;
}

bool JointLimit::limitL(float& l, float L_min_limit, float L_max_limit)
{
	bool is_limit = false;
	if (l > L_max_limit)
	{
		l = L_max_limit;
		is_limit = true;
	}
	else if (l < L_min_limit)
	{
		l = L_min_limit;
		is_limit = true;
	}
	return is_limit;
}

bool JointLimit::limitPhi(ConfigSpace& psi, float phi_min_limit, float phi_max_limit)
{
	bool is_limit = false;
	if (psi.phi > phi_max_limit)
	{
		float dphi = psi.phi - phi_max_limit;
		psi.delta1 += dphi;
		psi.delta2 += dphi;

		psi.phi = phi_max_limit;
		is_limit = true;
	}
	else if (psi.phi < phi_min_limit)
	{
		float dphi = psi.phi - phi_min_limit;
		psi.delta1 += dphi;
		psi.delta2 += dphi;

		psi.phi = phi_min_limit;
		is_limit = true;
	}
	return is_limit;
}

bool JointLimit::limitTheta(float& theta, float theta_min_limit, float theta_max_limit)
{
	bool is_limit = false;
	if (theta > theta_max_limit)
	{
		theta = 0.99f * theta_max_limit;
		is_limit = true;
	}
	else if (theta < theta_min_limit)
	{
		theta = 0.99f * theta_min_limit;
		is_limit = true;
	}
	return is_limit;
}
