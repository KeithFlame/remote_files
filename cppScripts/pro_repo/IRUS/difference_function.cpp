#include "difference_function.h"

#include <iostream>
#include "timer.h"

DifferenceMethod::DifferenceMethod(QObject* parent) : QObject(parent)
, paras(Eigen::Vector12d::Zero())
, manipulators(new SurgicalContinuumManipulator())
, IRUS_weight(0.0)
, IRUS_barycenter_length(0.0)
, auxiliary_sheath_weight(0.0)
, auxiliary_sheath_barycenter_length(0.0)
, gravity_direction(Eigen::Vector3d::Zero())
, MAX_ITER(60)
, increment(30.0)
, is_given_wrench(true)
{

}


bool DifferenceMethod::shootOneManipulator(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, Eigen::Vector6d& strain_l_phi)
{
	double l = strain_l_phi[4];					// 进给长度，m
	double phi = strain_l_phi[5];				// 旋转关节的角度，rad
	if (l > -1e-6) {
		manipulators->setLso(l * 1e3);
	}
	Eigen::Matrix4d mat;
	mat = y;
	Eigen::Matrix3d R_phi;
	Eigen::AngleAxisd rotation(phi, Eigen::Vector3d::UnitZ());
	mat.topLeftCorner(3, 3) = mat.topLeftCorner(3, 3) * rotation.toRotationMatrix();
	Eigen::Vector4d vec_v = strain_l_phi.head(4);

	bool rc = false;
	// Segment #0
	double energy = 0.0;
	if (manipulators->getLso() > 1e-6) {
		rc = integrateHalfStiffnessSegment(fm0, mat, vec_v);
		if (false == rc) {
			std::cout << "ERROR::INTEGRATION_IN_THE_BASE_STEM (feeding_length = "<< manipulators->getLso() <<")" << std::endl;
			return false;
		}
		else
			rc = false;
	}

	// Segment #1
	rc = integrateContinuumSegment1(fm0, mat, vec_v);
	if (false == rc) {
		std::cout << "ERROR::INTEGRATION_IN_THE_1ST_STEM (feeding_length = " << manipulators->getL1o() << ")" << std::endl;
		return false;
	}
	else
		rc = false;

	// Segment #2
	rc = integrateContinuumSegment2(fm0, mat, vec_v);
	if (false == rc) {
		std::cout << "ERROR::INTEGRATION_IN_THE_2ND_STEM" << std::endl;
		return false;
	}
	else
		rc = false;

	y = mat;
	strain_l_phi.head(4) = vec_v;
	return true;
}

void DifferenceMethod::integrationResult(Eigen::Vector12d& guess, Eigen::Vector12d& Rsd_last,
	Eigen::Vector6d& qa, Eigen::Matrix4d& target, const Eigen::Vector6d FM, const Problem mode/*, const bool is_Jacobian*/)
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	Eigen::Vector6d fm0;
	Eigen::Vector6d strain_l_phi;
	bool rc = false;
	fm0 = paras.head(6);
	mat = Keith::fromX2T(fm0);
	mat.topRightCorner(3, 1) = mat.topRightCorner(3, 1) / 1000.0;
	mat.row(3) = Eigen::Vector4d::Zero().transpose();
	fm0 = guess.head(6);
	strain_l_phi = guess.tail(6);

	rc = shootOneManipulator(fm0, mat, strain_l_phi);
	if (!rc) {
		std::cout << "ERROR::INTEGRATION_ONE_MANIPULATOR" << std::endl;
	}
	Eigen::Vector2d q1, q2, L1, L2;
	L1 = mat.bottomLeftCorner(1, 2).transpose();
	L2 = mat.bottomRightCorner(1, 2).transpose();

	q1 = (manipulators->getLstem() + manipulators->getL1()) * 1e-3 * strain_l_phi.head(2);
	q2 = (manipulators->getLcnla() + manipulators->getLstem() + manipulators->getL1() +
		manipulators->getLr() + manipulators->getL2()) * 1e-3 * strain_l_phi.segment(2, 2);

	Eigen::Vector12d Rsd = Eigen::Vector12d::Zero();
	Rsd.head(3) = -fm0.head(3) + FM.head(3);
	Rsd.segment(3, 3) = -fm0.tail(3) + FM.tail(3);
	if (mode == Problem::INVERSE_PROBLEM) // 逆运动学 1
	{
		Rsd.segment(6, 3) = target.topRightCorner(3, 1) - mat.topRightCorner(3, 1);
		Rsd.segment(9, 3) = Keith::invSkewMatrix_keith(target.topLeftCorner(3, 3), mat.topLeftCorner(3, 3));
		qa[0] = strain_l_phi[4] * 1E3;
		qa[1] = strain_l_phi[5];
		qa.segment(2, 2) = (L1 - q1) * 1e3;
		qa.segment(4, 2) = (L2 - q2) * 1e3;
		target = mat;
	}
	else if (mode == Problem::FORWRAD_PROBLEM) {
		Rsd.segment(6, 2) = L1 - (q1 + 1e-3 * qa.segment(2, 2));
		Rsd.segment(8, 2) = L2 - (q2 + 1e-3 * qa.segment(4, 2));
		//Rsd.segment(6, 4) *= 1e3;
		Rsd.tail(2) = Eigen::Vector2d::Zero();
		target = mat;
	}
	Rsd_last = Rsd;
}

bool DifferenceMethod::shootInverseOptimization(Eigen::Vector12d& guess, Eigen::Vector12d& Rsd_last,
	Eigen::Vector6d& qa, const Eigen::Matrix4d& target, const Eigen::Vector6d FM)
{
	static std::vector<Eigen::Vector12d> guess_lib;
	guess_lib.resize(MAX_ITER);
	static std::vector<Eigen::Vector6d> qa_lib;
	qa_lib.resize(MAX_ITER);
	static std::vector<Eigen::Vector12d> Rsd_lib;
	Rsd_lib.resize(MAX_ITER);
	static std::vector<double> cter;
	cter.resize(MAX_ITER);

	//Eigen::Vector6d FM = getGripperWrench(target);
	Eigen::Vector6d fm = 0.1 * FM;
	Eigen::Vector12d Rsd(Eigen::Vector12d::Zero()), Guess_tem(guess), Rsd_(Eigen::Vector12d::Zero());
	Eigen::Matrix4d tar(target), cur_pose(Eigen::Matrix4d::Identity()),
		tem_pose(Eigen::Matrix4d::Identity());

	double lambda(1e-15), eps(1e-6), del(1e-9), tem_scaled_val(0.0), dt(1.0), err(0.0);
	int iter(0), tem(0), Rsd_size(Rsd.size());

	Eigen::MatrixXd I12(12, 12), J(12, 12), J_(12, 12), J__(12, 12);

	I12 = Eigen::MatrixXd::Identity(12, 12);
	integrationResult(guess, Rsd_last, qa, tar, fm, Problem::INVERSE_PROBLEM);
	Rsd = Rsd_last;
	cur_pose = Keith::calcMiddleMatrix(tar, target, increment);
	while (iter < MAX_ITER) {
		for (size_t i = 0; i < Rsd_size; i++) {
			tem_pose = cur_pose;
			guess[i] = guess[i] + del;
			integrationResult(guess, Rsd_, qa, tem_pose, fm, Problem::INVERSE_PROBLEM);
			J.col(i) = (Rsd_ - Rsd) / del;
			guess[i] = guess[i] - del;
		}
		if (iter < 5)
			dt = 0.1;
		else
			dt = 1.0;
		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I12);
		Guess_tem = J_ * Rsd * dt;

		guess = guess - Guess_tem;
		//if (guess[10] < 0.03) {
		//	guess[10] = 0.03;
		//}
		tem_pose = cur_pose;
		integrationResult(guess, Rsd_last, qa, tem_pose, fm, Problem::INVERSE_PROBLEM);
		Rsd = Rsd_last;
		err = Rsd.norm();
		cter[iter] = err;
		guess_lib[iter] = guess;
		qa_lib[iter] = qa;
		Rsd_lib[iter] = Rsd_last;
		iter++;
		if (abs((fm - FM).sum()) < 1e-4 && abs((cur_pose - target).sum()) < 1e-4) {
			if (err < eps) {
				return true;
			}
		}
		fm = Keith::clacMiddleWrench(fm, FM, increment);
		cur_pose = Keith::calcMiddleMatrix(cur_pose, target, increment);
	}
	auto minElementIter = std::min_element(cter.begin(), cter.end());
	int minIndex = std::distance(cter.begin(), minElementIter);
	guess = guess_lib[minIndex];
	qa = qa_lib[minIndex];
	Rsd_last = Rsd_lib[minIndex];
	return false;
}

bool DifferenceMethod::shootForwardOptimization(Eigen::Vector10d& guess, Eigen::Vector10d& Rsd_last,
	const Eigen::Vector6d& qa, Eigen::Matrix4d& target, Eigen::Vector6d& FM) {
	static std::vector<Eigen::Vector10d> guess_lib(MAX_ITER);
	static std::vector<Eigen::Matrix4d> Target_lib(MAX_ITER);
	static std::vector<Eigen::Vector10d> Rsd_lib(MAX_ITER);
	static std::vector<double> cter(MAX_ITER);
	Eigen::MatrixXd I10(10, 10), J(10, 10), J_(10, 10);
	I10 = Eigen::MatrixXd::Identity(10, 10);
	Eigen::Vector6d qa_f(qa);
	Eigen::Vector6d fm = FM;
	Eigen::Vector10d Rsd, tem_dRsd;
	Eigen::Vector12d guess_f(Eigen::Vector12d::Zero()), Rsd_f(Eigen::Vector12d::Zero()),
		Rsd_(Eigen::Vector12d::Zero()), Guess_tem(guess_f);
	double energy_before(0.0), energy_after(0.0), tem_scaled_val(0.0), dt(1.0),
		lambda(1e-15), eps(1e-6), del(1e-9);
	int iter(0), tem(0), Rsd_size(Rsd.size());

	guess_f.head(10) = guess;
	guess_f.tail(2) = qa.head(2);
	Rsd_f.head(10) = Rsd_last;
	integrationResult(guess_f, Rsd_f, qa_f, target, fm, Problem::FORWRAD_PROBLEM);
	Rsd = Rsd_f.head(10);


	double err = Rsd.norm();

	while (iter < MAX_ITER) {
		for (size_t i = 0; i < Rsd_size; i++) {
			guess_f[i] = guess_f[i] + del;
			integrationResult(guess_f, Rsd_, qa_f, target, fm, Problem::FORWRAD_PROBLEM);
			J.col(i) = (Rsd_.head(10) - Rsd) / del;
			//std::cout << J.col(i).transpose() << std::endl;
			guess_f[i] = guess_f[i] - del;
		}
		if (iter < 3)
			dt = 0.1;
		else
			dt = 1.0;
		J_ = Keith::invScale(J.transpose() * J + lambda * I10) * J.transpose();
		Guess_tem.head(10) = J_ * Rsd * dt;

		guess_f = guess_f - Guess_tem;
		fm = getGripperWrench(target, fm, is_given_wrench);
		integrationResult(guess_f, Rsd_f, qa_f, target, fm, Problem::FORWRAD_PROBLEM);
		Rsd = Rsd_f.head(10);
		err = Rsd.norm();
		cter[iter] = err;
		guess_lib[iter] = guess;
		Target_lib[iter] = target;
		Rsd_lib[iter] = Rsd;
		iter++;
		if (err < eps) {
			FM = fm;
			guess = guess_f.head(10);
			Rsd_last = Rsd;
			return true;
		}

	}
	FM = fm;
	auto minElementIter = std::min_element(cter.begin(), cter.end());
	int minIndex = std::distance(cter.begin(), minElementIter);
	guess = guess_lib[minIndex];
	target = Target_lib[minIndex];
	Rsd_last = Rsd_lib[minIndex];
	return false;
}

const Eigen::Vector6d DifferenceMethod::getGripperWrench(const Eigen::Matrix4d& end_pose, const Eigen::Vector6d& wrench,
	const bool flag) {
	if (flag)
		return wrench;
	else {
		Eigen::Matrix4d auxiliary_port_pose = Keith::fromX2T(paras.tail(6));
		double dis_end2port = (auxiliary_port_pose.topRightCorner(3, 1) - end_pose.topRightCorner(3, 1)).norm();
		double dis_sheathbarycenter2port = auxiliary_sheath_barycenter_length - dis_end2port;
		double dis_IRUSbarycenter2port = IRUS_barycenter_length - dis_end2port;

		double fz_end = (dis_sheathbarycenter2port * auxiliary_sheath_weight
			+ dis_IRUSbarycenter2port * IRUS_weight) / dis_end2port;
		Eigen::Vector6d FM = Eigen::Vector6d::Zero();
		FM.head(3) = gravity_direction * fz_end;
		return FM;
	}

}

void DifferenceMethod::setParams(const Eigen::Vector12d& paras) {
	this->paras = paras;
}

void DifferenceMethod::setIRUSWeight(const double IRUS_weight) {
	this->IRUS_weight = IRUS_weight;
}

void DifferenceMethod::setIRUSBarycenterLength(const double IRUS_barycenter_length) {
	this->IRUS_barycenter_length = IRUS_barycenter_length;
}

void DifferenceMethod::setIRUSAuxiliarySheathWeight(const double auxiliary_sheath_weight) {
	this->auxiliary_sheath_weight = auxiliary_sheath_weight;
}

void DifferenceMethod::setIRUSAuxiliarySheathBarycenterLength(const double auxiliary_sheath_barycenter_length) {
	this->auxiliary_sheath_barycenter_length = auxiliary_sheath_barycenter_length;
}

void DifferenceMethod::setGravityDirection(const Eigen::Vector3d gravity_direction) {
	this->gravity_direction = gravity_direction;
}

void DifferenceMethod::setMaxIteration(const int MAX_ITER) {
	this->MAX_ITER = MAX_ITER;
}

void DifferenceMethod::setResolvedIncrement(const double increment) {
	this->increment = increment;
}


const Eigen::Vector12d DifferenceMethod::getParams() {
	return paras;
}

const double DifferenceMethod::getIRUSWeight() {
	return IRUS_weight;
}

const double DifferenceMethod::getIRUSBarycenterLength() {
	return IRUS_barycenter_length;
}

const double DifferenceMethod::getIRUSAuxiliarySheathWeight() {
	return auxiliary_sheath_weight;
}

const double DifferenceMethod::getIRUSAuxiliarySheathBarycenterLength() {
	return auxiliary_sheath_barycenter_length;
}

const Eigen::Vector3d DifferenceMethod::getGravityDiretcion() {
	return gravity_direction;
}
const int DifferenceMethod::getMaxIteration() {
	return MAX_ITER;
}

const double DifferenceMethod::getResolvedIncrement() {
	return increment;
}