#include "instrument.h"
#include "math_define.h"
#include "joint_limit.h"
#include <Eigen/Geometry>

namespace EigenAugment
{
void normalizeMatrix3f(Eigen::Matrix3f& mat)
{
	Eigen::Vector3f rot_1, rot_2;
	rot_1 = mat.col(0).normalized();
	rot_2 = mat.col(1).normalized();
	mat.topLeftCorner(3, 1) = rot_1;
	mat.topRightCorner(3, 1) = rot_1.cross(rot_2);
	mat.block(0, 1, 3, 1) = rot_1.cross(rot_2).cross(rot_1);

	rot_1 = mat.row(0).normalized().transpose();
	rot_2 = mat.row(1).normalized().transpose();
	mat.topLeftCorner(1, 3) = rot_1.transpose();
	mat.block(2, 0, 1, 3) = rot_1.cross(rot_2).transpose();
	mat.block(1, 0, 1, 3) = rot_1.cross(rot_2).cross(rot_1).transpose();
}
void normalizeMatrix4f(Eigen::Matrix4f& mat)
{
	Eigen::Matrix3f rot;
	rot = mat.topLeftCorner(3, 3);
	normalizeMatrix3f(rot);
	mat.topLeftCorner(3, 3) = rot;
}
}


namespace
{
	inline Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& vector)
	{
		Eigen::Matrix3f skew;
		skew << 0, -vector(2), vector(1),
			vector(2), 0, -vector(0),
			-vector(1), vector(0), 0;
		return skew;
	}
	inline Eigen::Vector3f ZHat()
	{
		return Eigen::Vector3f(0, 0, 1);
	}
	inline Eigen::MatrixXf JvSeg(const float theta, const float delta, const float L)
	{
		Eigen::MatrixXf Jv(3, 2);
		float cos_delta = cosf(delta);
		float sin_delta = sinf(delta);
		float cos_theta = cosf(theta);
		float sin_theta = sinf(theta);
		if (abs(theta) < 1e-7)
		{
			Jv << L * cos_delta / 2, 0, L* sin_delta / 2, 0, 0, 0;
		}
		else
		{
			Jv << L * cos_delta * (sin_theta / theta + (cos_theta - 1) / (theta * theta)),
				L* sin_delta / theta * (cos_theta - 1),
				L* sin_delta* (sin_theta / theta + (cos_theta - 1) / (theta * theta)),
				-L * cos_delta / theta * (cos_theta - 1),
				L* (cos_theta / theta - sin_theta / (theta * theta)), 0;
		}

		return Jv;
	}

	inline Eigen::MatrixXf JwSeg(const float theta, const float delta)
	{
		Eigen::MatrixXf Jw(3, 2);
		float cos_delta = cosf(delta);
		float sin_delta = sinf(delta);
		float cos_theta = cosf(theta);
		float sin_theta = sinf(theta);
		if (abs(theta) < 1e-7)
		{
			Jw << -sin_delta, 0, cos_delta, 0, 0, 0;
		}
		else
		{
			Jw << -sin_delta, -cos_delta * sin_theta,
				cos_delta, -sin_delta * sin_theta,
				0, 1 - cos_theta;
		}

		return Jw;
	}

	inline Eigen::Vector3f JvL(const float theta, const float delta)
	{
		Eigen::Vector3f Jv_L;
		float cos_delta = cosf(delta);
		float sin_delta = sinf(delta);
		float cos_theta = cosf(theta);
		float sin_theta = sinf(theta);
		if (abs(theta) < 1e-7)
		{
			Jv_L << 0, 0, 1;
		}
		else
		{
			Jv_L << cos_delta * (1 - cos_theta) / theta,
				/*-*/sin_delta* (1 - cos_theta) / theta,
				sin_theta / theta;
		}

		return Jv_L;
	}
	Eigen::Matrix3f rotZ(const float angle)
	{
		Eigen::Matrix3f mat;
		const float cos_angle = cosf(angle);
		const float sin_angle = sinf(angle);
		mat << cos_angle, -sin_angle, 0,
			sin_angle, cos_angle, 0,
			0, 0, 1;

		return mat;
	}
	Eigen::Vector3f calcLinearVelocity(const Eigen::Vector3f& error_pos, float v_limit, float v_limit_thred)
	{
		Eigen::Vector3f v = Eigen::Vector3f::Zero();
		const float scale = error_pos.norm() < v_limit_thred ? v_limit_thred : error_pos.norm();
		v = v_limit / scale * error_pos;

		return v;
	}

	Eigen::Vector3f calcAngularVelocity(const Eigen::Vector3f& error_pos, const Eigen::Matrix3f& error_rot, float w_limit, float v_norm, bool flag = false)
	{
		Eigen::AngleAxisf error_rot_angleaxis;
		error_rot_angleaxis.fromRotationMatrix(error_rot);
		const float error_angle = error_rot_angleaxis.angle();
		float w_norm = 0;

		if (sinf(error_angle) < 5e-4)
		{
			w_norm = 0;
		}
		else
		{
			if (error_pos.norm() < 1e-3)
			{
				w_norm = w_limit;
			}
			else
			{
				if (flag)
				{
					w_norm = v_norm * error_angle;
				}
				else
				{
					w_norm = v_norm * error_angle / error_pos.norm();
				}
			}

			if (w_norm > w_limit)
			{
				w_norm = w_limit;
			}
		}

		Eigen::Vector3f w = Eigen::Vector3f::Zero();
		if (error_angle < 2e-2)
		{
			w << 0, 0, 0;
		}
		else
		{
			w = w_norm * error_rot_angleaxis.axis();
		}

		return w;
	}
}

Instrument::Instrument(ToolArm tool)
{
	this->tool = tool;
}

void Instrument::forwardKinematics(ConfigSpace psi, Eigen::Matrix4f& mat, bool _1st_stiff_seg_is_bent = true)
{
	std::vector<SingleSegment> vss;
	dealConfigurationSpaceValue(psi, vss, _1st_stiff_seg_is_bent);
	Eigen::Matrix4f mb, m0, mb0, m1, m2;
	
	//phi
	mb = Eigen::Matrix4f::Identity();
	mb.topLeftCorner(3, 3) = rotZ(psi.phi);
	//l+1st_bent
	m0 = calcSingleSegment(vss[0]);
	
	//1st bent
	m1 = calcSingleSegment(vss[1]);
	
	//2nd bent
	m2 = calcSingleSegment(vss[2]);


	mb0 = mb * m0;
	kp.R0b_01 = mb0.topLeftCorner(3, 3);
	kp.P0b_01 = mb0.topRightCorner(3, 1);
	

	// Stem tip to Seg1 base (gamma1)
	kp.R01_1b = Eigen::Matrix3f::Identity(); // rotZ(instrument_segment->getGamma1());
	kp.P01_1b << 0, 0, 0;
	

	// Seg1 base to Seg1 end (theta1, delta1)
	kp.R1b_1e = m1.topLeftCorner(3, 3);
	kp.P1b_1e = kp.R1b_1e * Eigen::Vector3f(0, 0, -tool.segment.Lr) + m1.topRightCorner(3, 1);
	

	// Seg1 end to Seg2 base (lr, gamma2)
	kp.R1e_2b = Eigen::Matrix3f::Identity(); // rotZ(instrument_segment->getGamma2());
	kp.P1e_2b << 0, 0, tool.segment.Lr;
	

	// Seg2 base to Seg2 end (theta2, delta2)
	kp.R2b_2e = m2.topLeftCorner(3, 3);
	kp.P2b_2e = kp.R2b_2e * Eigen::Vector3f(0, 0, -tool.segment.Lg) + m2.topRightCorner(3, 1);
	

	// Seg2 end to Gripper (gamma3)
	kp.R2e_g = Eigen::Matrix3f::Identity(); // rotZ(instrument_segment->getGamma3());
	kp.P2e_g << 0, 0, tool.segment.Lg;
	
	mat = m0 * m1 * m2;
}

void Instrument::forwardKinematics(JointSpace q, Eigen::Matrix4f& mat, bool _1st_stiff_seg_is_bent = true)
{
	ConfigSpace psi;
	fromActuation2Psi(q, psi);
	forwardKinematics(psi, mat, _1st_stiff_seg_is_bent);
}

bool Instrument::inverseKinematics(Eigen::Matrix4f target_pose,  ConfigSpace& psi, bool _1st_stiff_seg_is_bent = false)
{
	Eigen::Matrix4f current_pose;
	forwardKinematics(psi, current_pose, _1st_stiff_seg_is_bent);
	float error_pos = KinematicsError::calcErrorDistance(target_pose, current_pose);
	for (int i = 0; i < 10 - 1; ++i)
	{
		doInverseKinematics(target_pose, psi, _1st_stiff_seg_is_bent);
	}
	return doInverseKinematics(target_pose, psi, _1st_stiff_seg_is_bent);
	return true;
}

bool Instrument::inverseKinematics(Eigen::Matrix4f target_pose,  JointSpace& q, bool _1st_stiff_seg_is_bent = false)
{
	ConfigSpace psi;
	fromActuation2Psi(q, psi);
	inverseKinematics(target_pose, psi, _1st_stiff_seg_is_bent);
	fromPsi2Actuation(psi,q);
	return true;
}

bool Instrument::inverseKinematicsPro(Eigen::Matrix4f, ConfigSpace&, bool _1st_stiff_seg_is_bent = false)
{
	return false;
}

bool Instrument::inverseKinematicsPro(Eigen::Matrix4f, JointSpace&, bool _1st_stiff_seg_is_bent = false)
{
	return false;
}

void Instrument::fromPsi2Actuation(ConfigSpace, JointSpace&)
{
}


bool Instrument::doInverseKinematics(Eigen::Matrix4f target_pose, ConfigSpace& psi, bool _1st_stiff_seg_is_bent = false)
{
	Eigen::Matrix4f current_pose;
	forwardKinematics(psi,current_pose,_1st_stiff_seg_is_bent);
	const float error_before_ik = KinematicsError::calcErrorDistance(target_pose, current_pose);
	const ConfigSpace psi_before_ik = psi;

	if (KinematicsError::isSmaller(target_pose, current_pose, tool.pe) == false)
	{
		const Eigen::Matrix6f jacobian = calcJacobian(psi, _1st_stiff_seg_is_bent);
		const Eigen::Vector6f x_dot = calcXdot(target_pose, current_pose);
		Eigen::Vector6f psi_dot = calcPsidot(x_dot, jacobian);
		//const float scale = calcPsidotScale(psi_dot);
		//if (scale > 1.0f)
		//{
		//	psi_dot /= scale;
		//}
		calcPsiFromPsidot(psi, psi_dot);

		forwardKinematics(psi, current_pose, _1st_stiff_seg_is_bent);
		const float error_after_ik = KinematicsError::calcErrorDistance(target_pose, current_pose);
		//const bool is_limit = config.isLimit();
		//is_error_increase = error_increase_judge.isIncrease(error_after_ik, error_before_ik, is_limit, is_error_increase);
		//if (is_error_increase)
		//{
		//	psi = psi_before_ik;
		//}
		return true;
	}
	else
		return false;
}






void Instrument::fromActuation2Psi(JointSpace q, ConfigSpace& psi)
{
	Eigen::MatrixXf G(24, 9), qa(24, 1), ua(9,1);
	calcCoupledMatrix(psi, G);
	qa(0) = q.q_seg1_1; qa(1) = q.q_seg1_2;
	qa(2) = -q.q_seg1_1; qa(3) = -q.q_seg1_2;

	qa(20) = q.q_seg2_1; qa(21) = q.q_seg2_2;
	qa(22) = -q.q_seg2_1; qa(23) = -q.q_seg2_2;
	ua = calcPseudoInverse(G) * qa;

	psi.theta1 = tool.segment.L1 * 1E-3f * (ua.block(0, 0, 3, 1).norm());
	psi.theta2 = tool.segment.L2 * 1E-3f * (ua.block(3, 0, 3, 1).norm());
	psi.delta1 = -(PI / 2.0f - atan2f(ua(1), ua(0)));
	psi.delta2 = -(PI / 2.0f - atan2f(ua(4), ua(3)));
	psi.l = q.q_linear;
	psi.phi = calcRotationAngleFromActuation(q.q_rotation);
	psi.angle_clamp = calcClampAngleFromActuation(q.q_clamp);

}

Eigen::Matrix4f Instrument::calcSingleSegment(SingleSegment ss)
{
	Eigen::Matrix4f mat;
	if (ss.theta < 1e-4f)
	{
		mat << 1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, ss.L + ss.L0,
			0.f, 0.f, 0.f, 1.f;
	}
	else
	{
		float k = ss.theta / ss.L;
		float cosTHETA = cosf(ss.theta); 
		float sinTHETA = sinf(ss.theta);
		float cosDELTA = cosf(ss.delta);
		float sinDELTA = sinf(ss.delta);
		mat << powf(cosDELTA, 2) * (cosTHETA - 1.f) + 1.f, sinDELTA* cosDELTA* (cosTHETA - 1),
			cosDELTA* sinTHETA, cosDELTA* (1 - cosTHETA) / k,
			sinDELTA* cosDELTA* (cosTHETA - 1), powf(cosDELTA, 2)* (1 - cosTHETA) + cosTHETA,
			sinDELTA* sinTHETA, sinDELTA* (1 - cosTHETA) / k,
			-cosDELTA * sinTHETA, -sinDELTA * sinTHETA, cosTHETA, sinTHETA / k,
			0.f, 0.f, 0.f, 1.f;
		EigenAugment::normalizeMatrix4f(mat);
	}
	return mat;
}

void Instrument::dealConfigurationSpaceValue(ConfigSpace psi, std::vector<SingleSegment>& vss, bool _1st_stiff_seg_is_bent = true)
{
	vss.clear();

	float L1 = tool.segment.L1;
	float zeta = tool.sp.zeta;
	SingleSegment ss1;
	SingleSegment ss2;
	SingleSegment ss3;
	ss3.theta = psi.theta2;
	ss3.delta = psi.delta2;
	ss3.L = tool.segment.L2;
	ss3.L0 = tool.segment.Lg;
	if (_1st_stiff_seg_is_bent && psi.l > L1)
	{
		float theta1 = L1 / (zeta * (psi.l - L1) + L1) * psi.theta1;
		float thetas = psi.theta1 - theta1;

		ss1.theta = thetas;
		ss1.delta = psi.delta1;
		ss1.L = psi.l - L1;
		ss1.L0 = 0.f;

		ss2.theta = theta1;
		ss2.delta = psi.delta1;
		ss2.L = L1;
		ss2.L0 = tool.segment.Lr;
	}
	else
	{
		ss1.theta = 0.f;
		ss1.delta = 0.f;
		ss1.L = 0.f;
		ss1.L0 = 0.f;

		ss2.theta = psi.theta1;
		ss2.delta = psi.delta1;
		ss2.L = psi.l > L1 ? L1 : psi.l;
		ss2.L0 = tool.segment.Lr;
	}
	vss.emplace_back(ss1);
	vss.emplace_back(ss2);
	vss.emplace_back(ss3);
}

void Instrument::calcCoupledMatrix(ConfigSpace psi, Eigen::MatrixXf& G)
{
	float L1, Lr, L2, l, L0,Lstem, gammaC, K1, K2;
	L1 = tool.segment.L1;
	L2 = tool.segment.L2;
	Lr = tool.segment.Lr;
	l = psi.l;
	Lstem = tool.sp.Lstem;//除1000是因为换算成m
	gammaC = tool.sp.zeta;
	K1 = tool.sp.K1;
	K2 = tool.sp.K2;


	// 由于工具臂的初始位置决定
	if (l < L1)
	{
		L1 = l;
		L0 = 0.f;
	}
	else
	{
		L0 = l - L1;
	}


	// calc mapping matrix
	Eigen::MatrixXf Q1(3, 4), Q2(3, 16), Q3(3, 4);
	Q1 << 0.0f, 1.0f, 0.0f, -1.0f,
		-1.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f;
	Q3 = Q1;
	Q2 << 0.3584f, 0.6018f, 0.7986f, 0.9336f, 0.9336f, 0.7986f, 0.6018f, 0.3584f, -0.3584f, -0.6018f, -0.7986f, -0.9336f, -0.9336f, -0.7986f, -0.6018f, -0.3584f,
		-0.9336f, -0.7986f, -0.6018f, -0.3584f, 0.3584f, 0.6018f, 0.7986f, 0.9336f, 0.9336f, 0.7986f, 0.6018f, 0.3584f, -0.3584f, -0.6018f, -0.7986f, -0.9336f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
	Q1 = Q1 * 1E-3 * tool.pp.r1;
	Q2 = Q2 * 1E-3 * tool.pp.r2;
	Q3 = Q3 * 1E-3 * tool.pp.r3;
	Eigen::MatrixXf Q1_inv(4, 3), Q2_inv(16, 3), Q3_inv(4, 3), Q2T_inv(3, 16);
	Q1_inv = calcPseudoInverse(Q1);
	Q2_inv = calcPseudoInverse(Q2);
	Q3_inv = calcPseudoInverse(Q3);
	Q2T_inv = calcPseudoInverse(Q2.transpose());

	Eigen::Matrix3f Kb1, Kb2, Kb3;
	float diameter1, diameter2, diameter3, Area1, Area2, Area3, AreaMoment1, AreaMoment2, AreaMoment3,
		Modulus, ShearModulus, Lcnla, Lprox, alpha;
	diameter2 = tool.pp.d2;
	diameter1 = tool.pp.d1;
	diameter3 = tool.pp.d1;
	alpha = tool.pp.r3 / tool.pp.r2; //prox over distal
	Area1 = PI * powf(diameter1, 2) / 4.0f;
	Area2 = PI * powf(diameter2, 2) / 4.0f;
	Area3 = PI * powf(diameter3, 2) / 4.0f;
	Modulus = tool.pp.E; //52.101
	ShearModulus = Modulus / 2.0f / (1.0f + tool.pp.poisson_ratio);//Poisson ratio
	AreaMoment1 = PI * powf(diameter1, 4) / 64.0f;
	AreaMoment2 = PI * powf(diameter2, 4) / 64.0f;
	AreaMoment3 = PI * powf(diameter3, 4) / 64.0f;

	Lcnla = tool.pp.Lcnla;
	Lprox = tool.pp.Lprox;

	Kb1 << Modulus * AreaMoment1, 0.0f, 0.0f, 0.0f, Modulus* AreaMoment1, 0.0f, 0.0f, 0.0f, ShearModulus * 2.0f * AreaMoment1;
	Kb2 << Modulus * AreaMoment2, 0.0f, 0.0f, 0.0f, Modulus* AreaMoment2, 0.0f, 0.0f, 0.0f, ShearModulus * 2.0f * AreaMoment2;
	Kb3 << Modulus * AreaMoment3, 0.0f, 0.0f, 0.0f, Modulus* AreaMoment3, 0.0f, 0.0f, 0.0f, ShearModulus * 2.0f * AreaMoment3;


	Eigen::MatrixXf M_dGamma, M_Ell, M_Theta_pinv, M_Kay;
	M_dGamma = Eigen::MatrixXf::Zero(24, 9);
	M_Ell = Eigen::MatrixXf::Identity(24, 24);
	M_Theta_pinv = Eigen::MatrixXf::Zero(24, 9);
	M_Kay = Eigen::MatrixXf::Zero(9, 9);

	M_dGamma.block(0, 0, 4, 3) = Q1.transpose() * (L1 + L0 * gammaC) * 1E-3;
	M_dGamma.block(4, 0, 16, 3) = Q2.transpose() * (L1 + L0 * gammaC) * 1E-3;
	M_dGamma.block(4, 3, 16, 3) = Q2.transpose() * L2 * 1E-3;
	M_dGamma.block(4, 6, 16, 3) = -alpha * Q2.transpose() * Lprox * 1E-3;
	M_dGamma.block(20, 6, 4, 3) = -Q3.transpose() * Lprox * 1E-3;

	M_Ell.block(0, 0, 4, 4) = M_Ell.block(0, 0, 4, 4) * (tool.segment.L1 * 1E-3 + Lstem);
	M_Ell.block(4, 4, 16, 16) = M_Ell.block(4, 4, 16, 16) * (L2 * 1E-3 + tool.segment.L1 * 1E-3 
		+ Lr * 1E-3 + Lstem * 1E-3 + Lcnla * 1E-3 + Lprox * 1E-3);
	M_Ell.block(20, 20, 4, 4) = M_Ell.block(20, 20, 4, 4) * Lprox * 1E-3;

	M_Theta_pinv.block(0, 0, 4, 3) = -1.0f / Area1 / Modulus * Q1_inv;
	M_Theta_pinv.block(4, 3, 16, 3) = -1.0f / Area2 / Modulus * Q2_inv;
	M_Theta_pinv.block(20, 6, 4, 3) = -1.0f / Area3 / Modulus * Q3_inv;

	M_Kay.block(0, 0, 3, 3) = 4.0f * Kb1 + 16.0f * Kb2 + K1 * Kb1;
	M_Kay.block(0, 3, 3, 3) = -16.0f * Kb2 - K2 * Kb1;
	M_Kay.block(3, 3, 3, 3) = 16.0f * Kb2 + K2 * Kb1;
	M_Kay.block(6, 3, 3, 3) = alpha * (16.0f * Kb2 + K2 * Kb1); //+ tpuKb + srKb + mcKb + cKb + bellowKb
	M_Kay.block(6, 6, 3, 3) = 4.0f * Kb3 + 16.0f * Kb2;

	Eigen::MatrixXf M_U2Qa(24, 9), M_Qa2U(9, 24);
	M_U2Qa = M_dGamma - M_Ell * M_Theta_pinv * M_Kay;
	G = M_U2Qa;
	M_Qa2U = calcPseudoInverse(M_U2Qa);

}

Eigen::MatrixXf Instrument::calcPseudoInverse(const Eigen::MatrixXf mat)
{
	auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	const auto& singlularValues = svd.singularValues();
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(mat.cols(), mat.rows());
	singularValuesInv.setZero();
	double pinvtoler = 1E-8;
	for (unsigned int i = 0; i < singlularValues.size(); i++)
	{
		if (singlularValues(i) > pinvtoler)
			singularValuesInv(i, i) = 1.0f / singlularValues(i);
		else
			singularValuesInv(i, i) = 0.0f;
	}
	Eigen::MatrixXf pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
	return pinvmat;
}


float Instrument::calcClampAngleFromActuation(float qg)
{
	return qg;
}

float Instrument::calcRotationAngleFromActuation(float q_phi)
{
	return q_phi;
}


// private
Eigen::MatrixXf Instrument::calcJacobian(const ConfigSpace psi, bool _1st_stiff_seg_is_bent = false)
{
	/* postion & rotation calculation */
	Eigen::Matrix4f mat;
	forwardKinematics(psi, mat, _1st_stiff_seg_is_bent);

	/* jacobian calculation */
	Eigen::MatrixXf Jv = Eigen::MatrixXf::Zero(3, 6);
	Eigen::MatrixXf Jw = Eigen::MatrixXf::Zero(3, 6);

	// L (global)
	Jv.col(0) = calcJvRL();
	Jw.col(0) = calcJwRL();
	// phi
	Jv.col(1) = calcJvPhi();
	Jw.col(1) = calcJwPhi();
	// seg 1
	Jv.block<3, 2>(0, 2) = calcJv1(psi);
	Jw.block<3, 2>(0, 2) = calcJw1(psi);
	// seg 2
	Jv.block<3, 2>(0, 4) = calcJv2(psi);
	Jw.block<3, 2>(0, 4) = calcJw2(psi);

	Eigen::MatrixXf Jacobian = Eigen::MatrixXf::Zero(6, 6);
	Jacobian.topRows(3) = Jv;
	Jacobian.bottomRows(3) = Jw;

	return Jacobian;
}

Eigen::Vector3f Instrument::calcJvRL()
{
	return ZHat();
}
Eigen::Vector3f Instrument::calcJwRL()
{
	return Eigen::Vector3f::Zero();
}

Eigen::Vector3f Instrument::calcJvPhi()
{
	return -skewSymmetric(kp.P0b_O1b_Og()) * ZHat();
}
Eigen::Vector3f Instrument::calcJwPhi()
{
	return ZHat();
}

Eigen::MatrixXf Instrument::calcJv1(const ConfigSpace psi)
{
	Eigen::MatrixXf Jv1 = JvSeg(psi.theta1, psi.delta1, tool.segment.L1);
	Eigen::MatrixXf Jw1 = JwSeg(psi.theta1, psi.delta1);
	return kp.R0b_1b() * (Jv1 - skewSymmetric(kp.P1b_O1e_Og()) * Jw1);
}
Eigen::MatrixXf Instrument::calcJw1(const ConfigSpace psi)
{
	Eigen::MatrixXf Jw1 = JwSeg(psi.theta1, psi.delta1);
	return kp.R0b_1b() * Jw1;
}

Eigen::MatrixXf Instrument::calcJv2(const ConfigSpace psi)
{
	Eigen::MatrixXf Jv2 = JvSeg(psi.theta2, psi.delta2, tool.segment.L2);
	Eigen::MatrixXf Jw2 = JwSeg(psi.theta2, psi.delta2);
	return kp.R0b_2b() * (Jv2 - skewSymmetric(kp.P2b_O2e_Og()) * Jw2);
}

Eigen::MatrixXf Instrument::calcJw2(const ConfigSpace psi)
{
	Eigen::MatrixXf Jw2 = JwSeg(psi.theta2, psi.delta2);
	return kp.R0b_2b() * Jw2;
}

Eigen::Vector6f Instrument::calcXdot(const Eigen::Matrix4f& target_pose, const Eigen::Matrix4f& current_pose, bool flag)
{
	const Eigen::Matrix3f error_rot = KinematicsError::calcErrorRot(target_pose, current_pose);
	const Eigen::Vector3f error_pos = KinematicsError::calcErrorPos(target_pose, current_pose);

	Eigen::Vector3f v = calcLinearVelocity(error_pos, tool.pc.v_limit, tool.pc.v_limit_thred);
	Eigen::Vector3f w = calcAngularVelocity(error_pos, error_rot, tool.pc.w_limit, v.norm(), flag);
	Eigen::Vector6f x_dot;
	x_dot << v[0], v[1], v[2], w[0], w[1], w[2];

	return x_dot;
}

Eigen::Vector6f Instrument::calcPsidot(const Eigen::Vector6f& x_dot, const Eigen::MatrixXf& jacobian)
{
	Eigen::JacobiSVD <Eigen::MatrixXf> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Vector6f singular_values = svd.singularValues(); // Singular values are always sorted in decreasing order.
	Eigen::Matrix6f u = svd.matrixU();
	Eigen::Matrix6f v = svd.matrixV();
	float min_singular_value = singular_values[5];
	const float pinv_toler = 1e-3f;
	const float lambda = 1e-4f;
	if (min_singular_value < pinv_toler)
	{
		for (int i = 0; i < 6; i++)
		{
			singular_values[i] = singular_values[i] + lambda;
		}
	}
	Eigen::Matrix6f inverse_jacobian_singular_value_matrix = Eigen::Matrix6f::Identity();
	for (int i = 0; i < 6; i++)
	{
		inverse_jacobian_singular_value_matrix(i, i) = 1.0f / singular_values[i];
	}
	Eigen::Matrix6f inverse_jacobian = v * inverse_jacobian_singular_value_matrix * u.transpose();
	Eigen::Vector6f psi_dot = inverse_jacobian * x_dot;

	return psi_dot;
}

void Instrument::calcPsiFromPsidot(ConfigSpace& psi, const Eigen::Vector6f psi_dot)
{
	/* calculate psi */
	float dt = tool.pc.dt;
	//ConfigSpace psi = tool.segment.psi;
	psi.l += psi_dot(0) * dt;
	psi.phi += psi_dot(1) * dt;
	psi.theta1 += psi_dot(2) * dt;
	psi.delta1 += psi_dot(3) * dt;
	psi.theta2 += psi_dot(4) * dt;
	psi.delta2 += psi_dot(5) * dt;

	/* judge joint limit */
	int joint_limit;
	joint_limit = JointLimit::judgeJointLimit(psi,tool.csl);

	/* judge pseudo limit */
	JointLimit::judgePseudoJointLimit(psi);

}