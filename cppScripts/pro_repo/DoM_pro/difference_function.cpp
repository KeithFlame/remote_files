#include "difference_function.h"

#include <iostream>
#include "timer.h"

DifferenceMethod::DifferenceMethod(QObject* parent) : QObject(parent)
{
	Eigen::Vector12d params = Eigen::Vector12d::Zero();
	paras.clear();
	for (size_t i = 0; i < ARM_NUM; i++) {
		manipulators[i] = new SurgicalContinuumManipulator();
		paras.emplace_back(params);
		//tc[i]->initial_pose = Eigen::Matrix4d::Identity();
		//tc[i]->curved_path = Eigen::Vector2d::Zero();
		//tc[i]->feeding_phi = Eigen::Vector2d::Zero();
	}
	Eigen::MatrixXd tem(6, 12);
	Jaco_broy = tem;
	Jaco_broy = Eigen::MatrixXd::Identity(6, 12);
	Jaco_broy.block(0, 6, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
}


bool DifferenceMethod::shootOneSubchain(Eigen::Vector6d& fm0, Eigen::Matrix4d& y, Eigen::Vector8d& v_phi_l, double& energy,
	const Eigen::Matrix3d& Rg_t, const Eigen::Vector13d& ksi, const int arm_num)
{
	double phi = (v_phi_l[4] + v_phi_l[6]);  // 默认0
	double l = v_phi_l[5] + v_phi_l[7];
	if (l > -1e-6) {
		manipulators[arm_num]->setLso(l * 1e3);
		//manipulators[arm_num]->calcMatrices();
	}
	Eigen::Matrix4d mat;
	mat = y;
	Eigen::Matrix3d R_phi;
	Eigen::AngleAxisd rotation(phi, Eigen::Vector3d::UnitZ());
	mat.topLeftCorner(3, 3) = mat.topLeftCorner(3, 3) * rotation.toRotationMatrix();
	Eigen::Vector4d vec_v = v_phi_l.head(4);

	bool rc = false;
	energy += abs(phi) * 1e-4;
	// Segment #0
	if (manipulators[arm_num]->getLso() > 1e-6) {
		rc = integrateHalfStiffnessSegment(fm0, mat, energy, vec_v, ksi[0], arm_num);
		if (false == rc)
			return false;
		else
			rc = false;
	}


	// Segment #1
	rc = integrateContinuumSegment1(fm0, mat, energy, vec_v, ksi[0], arm_num);
	if (false == rc)
		return false;
	else
		rc = false;
	//std::cout << "the " << arm_num << " seg1 pose.vec_v,and fm: " << std::endl << mat << std::endl << std::endl << fm0.transpose() << std::endl;
	// Segment #2
	rc = integrateContinuumSegment2(fm0, mat, energy, vec_v, arm_num);
	if (false == rc)
		return false;
	else
		rc = false;

	//std::cout << "the " << arm_num << " seg2 pose.vec_v,and fm: " << std::endl << mat << std::endl << std::endl << fm0.transpose() << std::endl;

	// Segment #3
	//Eigen::AngleAxisd rotation2(ksi[2], Eigen::Vector3d::UnitX());
	//Eigen::AngleAxisd rotation3(ksi[3], Eigen::Vector3d::UnitY());
	mat.topLeftCorner(3, 3) = mat.topLeftCorner(3, 3) * Rg_t;// *rotation2.toRotationMatrix()
	//	*rotation3.toRotationMatrix();

	//std::cout << Rg_t << std::endl;

	fm0 += ksi.tail(6);
	//Eigen::Vector7d ksi_tem = Eigen::Vector7d::Zero();
	//ksi_tem.head(2) = ksi.head(2);
	//ksi_tem[2] = 0.01; ksi_tem[3] = 0.01;
	rc = integrateElasticRod(fm0, mat, energy, ksi.head(7), arm_num);
	y = mat;
	v_phi_l.head(4) = vec_v;

	//std::cout << "the " << arm_num << " seg3 pose.vec_v,and fm: " << std::endl << mat << std::endl << std::endl << fm0.transpose() << std::endl;

	if (false == rc)
		return false;
	else
		return true;
}

void DifferenceMethod::shootPCKC(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, double& energy,
	std::vector<Eigen::Vector6d>& qa, Eigen::Matrix4d& target, const int arm_num, const int mode, const bool is_Jacobian)
{
	//assert(guess.size()==arm_num);
	//assert(paras.size() == arm_num);
	std::vector<Eigen::Matrix4d> mat;
	Eigen::Matrix3d Rg_t = Eigen::Matrix3d::Identity();
	std::vector<Eigen::Vector6d> fm0;
	std::vector<Eigen::Vector8d> v_phi_l;
	mat.resize(arm_num);
	fm0.resize(arm_num);
	v_phi_l.resize(arm_num);
	Eigen::Vector13d ksi = Eigen::Vector13d::Zero();
	Eigen::Vector3d vec_r = Eigen::Vector3d::Zero();
	//std::vector<int> arm_id{ 1,/*2*/,4 };
	bool rc = false;
	for (size_t i = 0; i < arm_num; i++) {
		fm0[i] = paras[i].head(6);
		mat[i] = Keith::fromX2T(fm0[i]);
		mat[i].topRightCorner(3, 1) = mat[i].topRightCorner(3, 1) / 1000.0;
		mat[i].row(3) = Eigen::Vector4d::Zero().transpose();
		vec_r = paras[i].segment(6, 3);
		Rg_t = Keith::axang2rotm(vec_r);
		fm0[i] = guess[i].head(6);
		v_phi_l[i] = guess[i].segment(6, 8);
		ksi = guess[i].segment(14, 13);
		//ksi = guess[i].segment(14, 10);
		manipulators[i]->setK1(guess[i][14] * 1e3);

		rc = shootOneSubchain(fm0[i], mat[i], v_phi_l[i], energy, Rg_t, ksi, i);
	}
	if (false == is_Jacobian) {
		std::cout << mat[0] << std::endl << std::endl << mat[1] << std::endl;
	}
	Eigen::Vector26d Rsd = Eigen::Vector26d::Zero(); Rsd[25] = 1.0;
	Rsd.head(3) = fm0[0].head(3) + fm0[1].head(3) - guess[0].segment(18, 3);
	Rsd.segment(3, 3) = fm0[0].tail(3) + fm0[1].tail(3) - guess[1].segment(18, 3);
	Rsd.segment(6, 3) = target.topRightCorner(3, 1) * 1e-3 - mat[0].topRightCorner(3, 1);
	vec_r = paras[0].tail(3);
	Eigen::Matrix3d R_tissue = Eigen::Matrix3d::Identity();
	R_tissue = mat[0].topLeftCorner(3, 3) * Keith::axang2rotm(vec_r);
	Rsd.segment(9, 3) = Keith::invSkewMatrix_keith(target.topLeftCorner(3, 3), R_tissue);

	Rsd.segment(12, 3) = target.topRightCorner(3, 1) * 1e-3 - mat[1].topRightCorner(3, 1);
	vec_r = paras[1].tail(3);
	R_tissue = mat[1].topLeftCorner(3, 3) * Keith::axang2rotm(vec_r);
	Rsd.segment(15, 3) = Keith::invSkewMatrix_keith(target.topLeftCorner(3, 3), R_tissue);
	Eigen::Vector2d q11, q12, q21, q22, L11, L12, L21, L22;
	L11 = mat[0].bottomLeftCorner(1, 2).transpose();
	L12 = mat[0].bottomRightCorner(1, 2).transpose();
	L21 = mat[1].bottomLeftCorner(1, 2).transpose();
	L22 = mat[1].bottomRightCorner(1, 2).transpose();

	q11 = (manipulators[0]->getLstem() + manipulators[0]->getL1()) * 1e-3 * v_phi_l[0].head(2);
	q12 = (manipulators[0]->getLcnla() + manipulators[0]->getLstem() + manipulators[0]->getL1() + manipulators[0]->getLr() + manipulators[0]->getL2()) * 1e-3 * v_phi_l[0].segment(2, 2);
	q21 = (manipulators[1]->getLstem() + manipulators[1]->getL1()) * 1e-3 * v_phi_l[1].head(2);
	q22 = (manipulators[0]->getLcnla() + manipulators[1]->getLstem() + manipulators[1]->getL1() + manipulators[1]->getLr() + manipulators[1]->getL2()) * 1e-3 * v_phi_l[1].segment(2, 2);
	if (mode == 1) // 逆运动学
	{
		Rsd.segment(18, 2) = L11 - q11;
		Rsd.segment(20, 2) = L12 - q12;
		Rsd.segment(22, 2) = L21 - q21;
		Rsd.segment(24, 2) = L22 - q22;
		for (size_t m = 0; m < 2; m++) {
			qa[m][0] = (guess[m][11] + guess[m][13]) * 1E3;
			qa[m][1] = (guess[m][10] + guess[m][12]);
		}
		qa[0].segment(2, 4) = 1e3 * (Rsd_last.segment(18, 4));
		qa[1].segment(2, 4) = 1e3 * (Rsd_last.segment(22, 4));
	}
	else if (mode == 2) {
		Rsd.segment(18, 2) = L11 - (q11 + 1e-3 * qa[0].segment(2, 2));
		Rsd.segment(20, 2) = L12 - (q12 + 1e-3 * qa[0].segment(4, 2));
		Rsd.segment(22, 2) = L21 - (q21 + 1e-3 * qa[1].segment(2, 2));
		Rsd.segment(24, 2) = L22 - (q22 + 1e-3 * qa[1].segment(4, 2));
	}
	else if (mode == 0) {
		Rsd.segment(6, 3) = (mat[0].topRightCorner(3, 1) - mat[1].topRightCorner(3, 1));
		Eigen::Matrix3d R_tissue2;
		vec_r = paras[0].tail(3);
		R_tissue2 = mat[0].topLeftCorner(3, 3) * Keith::axang2rotm(vec_r);
		vec_r = paras[1].tail(3);
		R_tissue = mat[1].topLeftCorner(3, 3) * Keith::axang2rotm(vec_r);
		Rsd.segment(9, 3) = Keith::invSkewMatrix_keith(R_tissue2, R_tissue);
		Rsd.segment(12, 2) = L11 - (q11 + 1e-3 * qa[0].segment(2, 2));
		Rsd.segment(14, 2) = L12 - (q12 + 1e-3 * qa[0].segment(4, 2));
		Rsd.segment(16, 2) = L21 - (q21 + 1e-3 * qa[1].segment(2, 2));
		Rsd.segment(18, 2) = L22 - (q22 + 1e-3 * qa[1].segment(4, 2));
		Rsd.tail(6) = Eigen::Vector6d::Zero();
		target.topLeftCorner(3, 3) = R_tissue2;
		target.topRightCorner(3, 1) = mat[0].topRightCorner(3, 1);
	}
	Rsd_last = Rsd;
}

bool DifferenceMethod::shootStiffnessOptimization(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
	Eigen::Matrix4d target)
{
#if 0
	const int max_iter = 40;
	static std::vector<std::vector<Eigen::Vector21d>> guess_lib(max_iter);
	static std::vector<Eigen::Vector26d> Rsd_lib(max_iter);
	static double cter[max_iter] = {};
	Eigen::MatrixXd I14(14, 14), I12(12, 12), J(12, 14), J_(14, 12), J__(14, 12);
	I14 = Eigen::MatrixXd::Identity(14, 14);
	I12 = Eigen::MatrixXd::Identity(12, 12);



	double lambda(1e-5), eps(1e-4), is_energy(0.0002), yita(1e-2), del(1e-4);
	//d_Guess(0, 0) = 1.0; d_Guess(0, 7) = 1.0;
	Eigen::Vector12d Rsd;
	double energy_before(0.0), energy_after(0.0), dt(0.1);
	shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 2);
	Rsd = Rsd_last.segment(6, 12);
	Eigen::Vector14d Energy_rsd, Guess_tem, ratio_val, d_guess;
	Eigen::Vector26d Rsd_;
	int iter(0), tem(0);
	double err = Rsd.norm();
	Guess_tem.head(7) = guess[0].tail(7);
	Guess_tem.tail(7) = guess[1].tail(7);

	while (iter < max_iter) {

		for (size_t i = 0; i < 14; i++) {
			tem = i - 7;
			if (i < 7) {
				guess[0][i + 14] = guess[0][i + 14] - del;
			}
			else
			{
				guess[1][tem + 14] = guess[1][tem + 14] - del;
			}
			shootPCKC(guess, Rsd_, energy_after, qa, target, 2, 2);
			J.col(i) = -(Rsd_.segment(6, 12) - Rsd) / del;

			//std::cout << -(Rsd_.segment(6, 12) - Rsd).transpose() / d_Guess.col(i).norm() << std::endl << std::endl << std::endl << std::endl;

			Energy_rsd[i] = (energy_after - energy_before) / del;
			energy_after = 0.0;
			if (i < 7) {
				guess[0][i + 14] = guess[0][i + 14] + del;
			}
			else
			{
				guess[1][tem + 14] = guess[1][tem + 14] + del;
			}
		}
		if (iter > 20)
			dt = 0.5;
		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I12);

		if (err < is_energy) // yita, 是否使用能量
		{
			J__ = J.transpose() * Keith::pinv(J * J.transpose());
			d_guess = (J_ * Rsd - yita * (I14 - J__ * J) * Energy_rsd);
		}
		else {
			d_guess = J_ * Rsd * dt;
		}
		//d_guess = d_guess.array();// *ratio_val.array();
		Guess_tem = Guess_tem - d_guess;
		for (size_t j = 0; j < 4; j++) {
			if (Guess_tem[j] < 1e-5)
				Guess_tem[j] = 1e-5;
			if (Guess_tem[j + 7] < 1e-5)
				Guess_tem[j + 7] = 1e-5;
		}
		guess[0].tail(7) = Guess_tem.head(7);
		guess[1].tail(7) = Guess_tem.tail(7);
		energy_before = 0.0;
		shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 2);
		Rsd = Rsd_last.segment(6, 12);
		err = Rsd.norm();
		cter[iter] = err;// *energy_before;
		Rsd_lib[iter] = Rsd_last;
		guess_lib[iter] = guess;
		iter++;
		if (err < eps) {
			return true;
		}
	}
	auto minElementIter = std::min_element(cter, cter + max_iter);
	int minIndex = std::distance(cter, minElementIter);
	guess = guess_lib[minIndex];
	Rsd_last = Rsd_lib[minIndex];
	return false;

#else
	auto tic = mmath::timer::getCurrentTimePoint();
	const int max_iter = 50;
	static std::vector<std::vector<Eigen::Vector27d>> guess_lib(max_iter);
	static std::vector<Eigen::Vector26d> Rsd_lib(max_iter);
	static double cter[max_iter] = {};
	Eigen::MatrixXd I34(34, 34), I26(26, 26), J(26, 34), J_(34, 26), J__(34, 26);
	I34 = Eigen::MatrixXd::Identity(34, 34);
	I26 = Eigen::MatrixXd::Identity(26, 26);



	double lambda(1e-6), eps(1e-5), is_energy(0.000), yita(1e-2), del(1e-4);
	//d_Guess(0, 0) = 1.0; d_Guess(0, 7) = 1.0;
	Eigen::Vector26d Rsd;
	double energy_before(0.0), energy_after(0.0), dt(0.2);
	shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 2);
	Rsd = Rsd_last;
	Eigen::Vector34d Energy_rsd, Guess_tem, dd, d_guess;
	Eigen::VectorXd d_guess0 = Eigen::VectorXd::Constant(17, 1e-9);
	//d_guess0.segment(10, 4) = d_guess0.segment(10, 4) * 1e5;
	//d_guess0.tail(6) = d_guess0.tail(6) * 1e-5;
	std::cout << std::endl << std::endl<< "Rsd_last_before_opt: "<< Rsd_last.transpose() << std::endl << std::endl;

	Eigen::VectorXd d_guess2 = Eigen::VectorXd::Constant(17, 1);
	//d_guess2.tail(13) = d_guess2.tail(13) * 1;
	//std::cout << "d_guess2: " << d_guess2 << std::endl;

	dd.head(17) = d_guess2; dd.tail(17) = d_guess2;
	Eigen::Vector26d Rsd_;
	int iter(0), tem(0);
	double err = Rsd.norm();

	std::cout << std::endl << "->err: " << err << std::endl << std::endl;

	Guess_tem.head(10) = guess[0].head(10);
	Guess_tem.segment(10, 7) = guess[0].segment(14, 7);
	Guess_tem.segment(17, 10) = guess[1].head(10);
	Guess_tem.tail(7) = guess[1].segment(14, 7);
	//ratio_val.head(10).setConstant(1e-9);
	//ratio_val.segment(17, 10).setConstant(1e-9);
	int guess_size = Guess_tem.size();
	while (iter < max_iter) {
		for (size_t i = 0; i < guess_size; i++) {
			tem = i - guess_size / 2;
			if (i < guess_size / 2) {
				del = d_guess0[i];
				if (i < 10)
					guess[0][i] = guess[0][i] + d_guess0[i];
				else
					guess[0][i + 4] = guess[0][i + 4] + d_guess0[i];
			}
			else
			{
				if (tem < 10)
					guess[1][tem] = guess[1][tem] + d_guess0[tem];
				else
					guess[1][tem + 4] = guess[1][tem + 4] + d_guess0[tem];
				del = d_guess0[tem];
			}
			shootPCKC(guess, Rsd_, energy_after, qa, target, 2, 2);
			J.col(i) = (Rsd_ - Rsd) / del;

			//std::cout << std::endl << (Rsd_ - Rsd) / del << std::endl << std::endl << std::endl << std::endl;

			Energy_rsd[i] = (energy_after - energy_before) / del;
			energy_after = 0.0;
			if (i < guess_size / 2) {
				if (i < 10)
					guess[0][i] = guess[0][i] - d_guess0[i];
				else
					guess[0][i + 4] = guess[0][i + 4] - d_guess0[i];
			}
			else
			{
				if (tem < 10)
					guess[1][tem] = guess[1][tem] - d_guess0[tem];
				else
					guess[1][tem + 4] = guess[1][tem + 4] - d_guess0[tem];
			}
		}
		//if (iter < 15)
		//	dt = 0.1;
		////else if (iter < 30)
		////	dt = 0.31625;
		//else// if (iter < 70)
		//	dt = 1.0;

		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I26);
		//Eigen::PartialPivLU<Eigen::MatrixXd> ALU(J* J.transpose() + lambda * I26);
		//Eigen::MatrixXd _J_ = J.transpose()* ALU.inverse();

		//std::cout << std::endl << J << std::endl << std::endl << std::endl << std::endl;

		//std::cout << std::endl << J_ << std::endl << std::endl << std::endl << std::endl;

		//std::cout << std::endl << _J_ << std::endl << std::endl << std::endl << std::endl;

		if (err < is_energy) // yita, 是否使用能量
		{
			J__ = J.transpose() * Keith::pinv(J * J.transpose());
			d_guess = (J_ * Rsd + yita * (I34 - J__ * J) * Energy_rsd);
		}
		else {
			d_guess = J_ * Rsd * dt;
		}
		//d_guess = d_guess.array() * dd.array();
		Guess_tem = Guess_tem - d_guess;
		for (size_t j = 0; j < 4; j++) {
			if (Guess_tem[j + 10] < 1e-2)
				Guess_tem[j + 10] = 1e-2;
			if (Guess_tem[j + 27] < 1e-2)
				Guess_tem[j + 27] = 1e-2;
		}
		guess[0].head(10) = Guess_tem.head(10);
		guess[0].segment(14, 7) = Guess_tem.segment(10, 7);
		guess[1].head(10) = Guess_tem.segment(17, 10);
		guess[1].segment(14, 7) = Guess_tem.tail(7);
		energy_before = 0.0;
		shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 2);
		Rsd = Rsd_last;
		err = Rsd.norm();
		cter[iter] = err;// *energy_before;
		Rsd_lib[iter] = Rsd_last;
		guess_lib[iter] = guess;
		iter++;
		if (err < eps) {
			float t1 = mmath::timer::getDurationSince(tic);
			std::cout << "stiffness durations: " << t1 << "\t" << iter << std::endl;
			return true;
		}
	}
	auto minElementIter = std::min_element(cter, cter + max_iter);
	int minIndex = std::distance(cter, minElementIter);
	guess = guess_lib[minIndex];
	Rsd_last = Rsd_lib[minIndex];
	float t1 = mmath::timer::getDurationSince(tic);
	std::cout << "stiffness durations: " << t1 << "\t" << iter << std::endl;
	return false;
#endif
}

bool DifferenceMethod::shootStiffnessOptimization2(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
	Eigen::Matrix4d target)
{
	auto tic = mmath::timer::getCurrentTimePoint();
	const int max_iter = 50;
	int iter = 0;
	static std::vector<std::vector<Eigen::Vector27d>> guess_lib(max_iter);
	static std::vector<Eigen::Vector26d> Rsd_lib(max_iter);
	static double cter[max_iter] = {};
	Eigen::MatrixXd I12(12, 12), I6(6, 6), J(6, 12), J_(12, 6), J__(12, 6);
	I12 = Eigen::MatrixXd::Identity(12, 12);
	I6 = Eigen::MatrixXd::Identity(6, 6);

	double lambda(1e-6), eps(1e-5), is_energy(0.000), yita(1e-2), del(1e-4);
	//d_Guess(0, 0) = 1.0; d_Guess(0, 7) = 1.0;
	Eigen::Vector26d Rsd_;
	double energy_before(0.0), energy_after(0.0), dt(0.2);

	std::vector<Eigen::Vector27d> guess_cur = guess;
	std::vector<Eigen::Vector27d> guess_cur2 = guess;
	Eigen::Matrix4d cur_T(Eigen::Matrix4d::Identity()), cur_tar(target), tem_forward(Eigen::Matrix4d::Identity());
	Eigen::Vector6d drr, dT;
	Eigen::Vector12d dg2;
	Eigen::VectorXd d_guess0 = Eigen::VectorXd::Constant(14, 5e-5);
	d_guess0.head(11) = d_guess0.head(11) * 1e2;

	shootForwardOptimization(guess_cur, Rsd_last, qa, energy_before, cur_T);
	cur_tar.topRightCorner(3, 1) = cur_tar.topRightCorner(3, 1) * 1e-3;
	drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);
	std::cout << cur_T << std::endl << cur_tar << std::endl;
	double err = 10.0;
	int guess_size = 6;
	while (iter < max_iter)
	{
		for (size_t i = 0; i < guess_size; i++)
		{
			guess_cur2 = guess_cur;
			del = d_guess0[i];
			int tem = i - guess_size / 2;
			if (i < guess_size / 2)
			{
				guess_cur2[0][i + 14] += del;
			}
			else {
				guess_cur2[1][tem + 14] += del;
			}
			energy_after = 0.0;
			shootPCKC(guess_cur2, Rsd_, energy_after, qa, tem_forward, 2, 0);
			dT = Keith::calcDeviationFrom2T(cur_T, tem_forward);
			J.col(i) = dT / del;
		}
		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I6);
		dg2 = J_ * drr;

#if 1	// 约束
		Eigen::Vector3d m_dg2 = dg2.head(3);
		double max_ele = m_dg2.cwiseAbs().maxCoeff();
		if (max_ele > 1.0) {
			m_dg2 = m_dg2 / max_ele * 1.0;
			dg2.head(3) = m_dg2;
		}
		Eigen::Vector3d m_dg = dg2.tail(3);
		max_ele = m_dg.cwiseAbs().maxCoeff();
		if (max_ele > 2e-3) {
			m_dg = m_dg / max_ele * 2e-3;
			dg2.tail(3) = m_dg;
		}
#endif 
		guess_cur[0].segment(18, 3) = guess_cur[0].segment(18, 3) + dg2.head(3);
		guess_cur[1].segment(18, 3) = guess_cur[1].segment(18, 3) + dg2.tail(3);
		shootForwardOptimization(guess_cur, Rsd_last, qa, energy_after, cur_T);
		drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);

		double prr(drr.head(3).norm()), arr(drr.tail(3).norm());
		err = prr * 1e3 + arr * 180 / Keith::PI;
		std::cout << "position_error: " << prr * 1e3 << "\t\t" << "angular_error: " << arr * 180 / Keith::PI << std::endl;
		if (err < 1e-1) {
			guess = guess_cur;
			std::cout << "cur_T: " << std::endl << cur_T << std::endl;
			return true;
		}
		cter[iter] = err;
		guess_lib[iter] = guess_cur;
		iter += 1;
	}
	std::cout << iter << std::endl;
	auto minElementIter = std::min_element(cter, cter + max_iter);
	std::cout << minElementIter << std::endl;
	int minIndex = std::distance(cter, minElementIter);
	std::cout << minIndex << std::endl;
	if (err < 3)
		guess = guess_lib[minIndex];
	std::cout << "cur_T: " << std::endl << cur_T << std::endl;
	return false;
}


bool DifferenceMethod::shootInverseOptimization(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
	double& energy_rsd, Eigen::Matrix4d target)
{
	//auto tic = mmath::timer::getCurrentTimePoint();
	const int max_iter = 60;
	static std::vector<std::vector<Eigen::Vector27d>> guess_lib(max_iter);
	static std::vector<std::vector<Eigen::Vector6d>> qa_lib(max_iter);
	static std::vector<Eigen::Vector26d> Rsd_lib(max_iter);
	static std::vector<double> energy_lib(max_iter);
	static double cter[max_iter] = {};
	Eigen::MatrixXd I24(24, 24), I18(18, 18), J(18, 24), J_(24, 18), J__(24, 18);
	I24 = Eigen::MatrixXd::Identity(24, 24);
	I18 = Eigen::MatrixXd::Identity(18, 18);
	double lambda(1e-15), eps(1e-6), is_energy(0.0002), yita(1e-5), del(1e-9);
	Eigen::Vector18d Rsd = Eigen::Vector18d::Zero();
	double energy_before(0.0), energy_before0(10.0), energy_after(0.0), tem_scaled_val(0.0), dt(1.0);
	shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 1, true);
	Rsd = Rsd_last.head(18);
	Eigen::Vector24d Energy_rsd, Guess_tem, tem_w_dU;
	Eigen::Vector26d Rsd_(Rsd_last);

	//std::cout << "Rsd: " << std::endl << Rsd.transpose() << std::endl;

	int iter(0), tem(0);
	double err = Rsd.norm();
	Guess_tem.head(12) = guess[0].head(12);
	Guess_tem.tail(12) = guess[1].head(12);
	while (iter < max_iter) {
		for (size_t i = 0; i < 24; i++) {
			tem = i - 12;
			if (i < 12)
				guess[0][i] = guess[0][i] + del;
			else
				guess[1][tem] = guess[1][tem] + del;
			shootPCKC(guess, Rsd_, energy_after, qa, target, 2, 1);
			J.col(i) = (Rsd_.head(18) - Rsd) / del;
			//std::cout << (Rsd_.head(18) - Rsd).transpose() / del << std::endl;
			Energy_rsd[i] = (energy_after - energy_before) / del;
			energy_after = 0.0;
			if (i < 12)
				guess[0][i] = guess[0][i] - del;
			else
				guess[1][tem] = guess[1][tem] - del;
		}
		if (iter < 2)
			dt = 0.1;
		else
			dt = 1.0;
		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I18);
		if (err < is_energy) // yita, 是否使用能量
		{
			J__ = J.transpose() * Keith::pinv(J * J.transpose());
			//if ((energy_before0 - energy_before) < 1e-3)
			//	yita = 1e-4;
			//else
			//	yita = 1e-5;
			energy_before0 = energy_before;
			Guess_tem = Guess_tem - (J_ * Rsd + yita * (I24 - J__ * J) * Energy_rsd);
		}
		else {
			Guess_tem = Guess_tem - J_ * Rsd * dt;
		}
		guess[0].head(12) = Guess_tem.head(12);
		guess[1].head(12) = Guess_tem.tail(12);
		energy_before = 0.0;
		shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 1);
		Rsd = Rsd_last.head(18);
		err = Rsd.norm();
		cter[iter] = err;
		guess_lib[iter] = guess;
		qa_lib[iter] = qa;
		Rsd_lib[iter] = Rsd_last;
		energy_lib[iter] = energy_before;
		iter++;
		std::cout << "energy_before: " << energy_before << std::endl;
		if ((err < eps)) {
			//float t1 = mmath::timer::getDurationSince(tic);
			//std::cout << "inverse duration: " << t1 << "\t" << iter << std::endl;
			energy_rsd = energy_before;
			return true;
		}
	}
	auto minElementIter = std::min_element(cter, cter + max_iter);
	int minIndex = std::distance(cter, minElementIter);
	guess = guess_lib[minIndex];
	qa = qa_lib[minIndex];
	Rsd_last = Rsd_lib[minIndex];
	energy_rsd = energy_lib[minIndex];
	//float t1 = mmath::timer::getDurationSince(tic);
	//std::cout << "inverse durations: " << t1 << "\t" << std::endl;
	return false;
}

bool DifferenceMethod::shootForwardOptimization(std::vector<Eigen::Vector27d>& guess, Eigen::Vector26d& Rsd_last, std::vector<Eigen::Vector6d>& qa,
	double& energy, Eigen::Matrix4d& target) {
	auto tic = mmath::timer::getCurrentTimePoint();
	const int max_iter = 200;
	static std::vector<std::vector<Eigen::Vector27d>> guess_lib(max_iter);
	static std::vector<Eigen::Matrix4d> Target_lib(max_iter);
	static std::vector<Eigen::Vector26d> Rsd_lib(max_iter);
	std::vector<double> energy_lib(max_iter);
	static double cter[max_iter] = {};
	Eigen::MatrixXd I20(20, 20), J(20, 20), J_(20, 20), J__(20, 20);
	I20 = Eigen::MatrixXd::Identity(20, 20);
	double lambda(1e-15), eps(1e-7), del(1e-9);
	Eigen::Vector20d Rsd, tem_dRsd;
	Eigen::VectorXd d_guess0 = Eigen::VectorXd::Constant(20, 1e-9);
	//d_guess0.segment(6, 4) = d_guess0.segment(6, 4) * 1e-1;
	//d_guess0.tail(4) = d_guess0.tail(4) * 1e-1;
	double energy_before(0.0), energy_after(0.0), tem_scaled_val(0.0), dt(1.0);
	//shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 0);
	Rsd = Rsd_last.head(20);
	Eigen::Vector20d Guess_tem,dguess_tem;
	Eigen::Vector26d Rsd_;

	int iter(0), tem(0);
	double err = Rsd.norm();
	Guess_tem.head(10) = guess[0].head(10);
	Guess_tem.tail(10) = guess[1].head(10);

	while (iter < max_iter) {
		for (size_t i = 0; i < 20; i++) {
			tem = i - 10;
			del = d_guess0[i];
			if (i < 10)
				guess[0][i] = guess[0][i] + del;
			else
				guess[1][tem] = guess[1][tem] + del;
			shootPCKC(guess, Rsd_, energy_after, qa, target, 2, 0);
			J.col(i) = (Rsd_.head(20) - Rsd) / del;
			//tem_dRsd = (Rsd_.head(20) - Rsd) / del;
			//std::cout << tem_dRsd.transpose() << std::endl;
			if (i < 10)
				guess[0][i] = guess[0][i] - del;
			else
				guess[1][tem] = guess[1][tem] - del;
		}
		dt = 1.0;
		if (iter < 9)
			dt = 0.1;
		//else if (iter < 20)
		//	dt = 0.3;
		//else if (iter < 40)
		//	dt = 0.66;
		//else
		//	dt = 1.0;
		J_ = Keith::invScale(J.transpose() * J + lambda * I20) * J.transpose();
		dguess_tem = J_ * Rsd * dt;
		//std::cout << dguess_tem.transpose() << std::endl;
		Guess_tem = Guess_tem - dguess_tem;

		guess[0].head(10) = Guess_tem.head(10);
		guess[1].head(10) = Guess_tem.tail(10);
		energy_before = 0.0;
		shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 0);
		Rsd = Rsd_last.head(20);
		err = Rsd.norm();
		cter[iter] = err;
		guess_lib[iter] = guess;
		Target_lib[iter] = target;
		Rsd_lib[iter] = Rsd_last;
		energy_lib[iter] = energy_before;
		iter++;
		if (err < eps) {
			//float t1 = mmath::timer::getDurationSince(tic);
			//std::cout << "duration: " << t1 << "\t" << iter << "\t" << err << std::endl;
			energy = energy_before;
			//shootPCKC(guess, Rsd_last, energy_before, qa, target, 2, 0);
			return true;
		}
	}
	auto minElementIter = std::min_element(cter, cter + max_iter);
	int minIndex = std::distance(cter, minElementIter);
	guess = guess_lib[minIndex];
	target = Target_lib[minIndex];
	Rsd_last = Rsd_lib[minIndex];
	energy = energy_lib[minIndex];
	float t1 = mmath::timer::getDurationSince(tic);
	std::cout << "durations: " << t1 << "\t" << Rsd_last.head(20).norm() <<'\t'<<"err[100]: "<<cter[99] << '\t' << "err[200]: " << cter[199] 
	 << std::endl;
	return false;
}

void DifferenceMethod::setParams(const std::vector<Eigen::Vector12d>& paras) {
	this->paras = paras;
}

const std::vector<Eigen::Vector12d> DifferenceMethod::getParams() {
	return paras;
}

bool DifferenceMethod::inverseKinematicsPlanB(std::vector<Eigen::Vector6d>& cur_qa, const Eigen::Matrix4d& cur_pose,
	const Eigen::Matrix4d& target, const int iter) {
	static Eigen::Vector12d last_psi = Eigen::Vector12d::Zero();
	static Eigen::Matrix4d last_T = Eigen::Matrix4d::Zero();
	double alpha_J = 0.6;
	double lambda = 1e-6;
	Eigen::MatrixXd J_(12, 6), J__(12, 6), I66(6, 6), I12(12, 12);
	J_ = Eigen::MatrixXd::Zero(12, 6);
	I66 = Eigen::MatrixXd::Identity(6, 6);
	I12 = Eigen::MatrixXd::Identity(12, 12);
	Eigen::Vector12d cur_psi, dPsi, psi_out, next_dpsi, dE;
	cur_psi = Eigen::Vector12d::Zero();
	next_dpsi = Eigen::Vector12d::Zero();
	dE = Eigen::Vector12d::Zero();
	psi_out = Eigen::Vector12d::Zero();
	dPsi = Eigen::Vector12d::Zero();
	cur_psi.head(6) = cur_qa[0];
	cur_psi.tail(6) = cur_qa[1];
	dPsi = cur_psi - last_psi;

	Eigen::Vector6d dT, dTa;
	dT = Keith::calcDeviationFrom2T(last_T, cur_pose);
	//dT.tail(3) = dT.tail(3) * 180 / Keith::PI;
	Jaco_broy = Jaco_broy + alpha_J * (dT - Jaco_broy * dPsi) / (dPsi.transpose() * dPsi) * dPsi.transpose();
	J_ = Jaco_broy.transpose() * Keith::invScale(Jaco_broy * Jaco_broy.transpose() + lambda * I66);

	dTa = Keith::calcDeviationFrom2T(cur_pose, target);
	if (iter > 600)
	{
		J__ = Jaco_broy.transpose() * Keith::pinv(Jaco_broy * Jaco_broy.transpose());
		getEnergyConstant(dE, cur_qa);
		next_dpsi = J_ * dTa - (I12 - J__ * Jaco_broy) * dE;
		std::cout << std::endl << "dE: " << dE.norm() << std::endl << std::endl;
	}
	else {
		next_dpsi = J_ * dTa;
	}
	cur_qa[0] = next_dpsi.head(6) + cur_psi.head(6);
	cur_qa[1] = next_dpsi.tail(6) + cur_psi.tail(6);

	last_psi = cur_psi;
	last_T = cur_pose;
	return true;
}

bool DifferenceMethod::getEnergyConstant(Eigen::Vector12d& dE, const std::vector<Eigen::Vector6d>& cur_qa) {
	Eigen::Vector6d u0, u1;
	std::vector<Eigen::Vector6d> qa = cur_qa;
	double L1(0), L2(0), del(1e-6);
	double energy(0.0), energy_last(0.0);
	manipulators[0]->setLso(cur_qa[0][0]);
	u0 = Keith::Actuation2Curvature_keith(cur_qa[0], manipulators[0]->getGc());
	L1 = manipulators[0]->getL1();
	L2 = manipulators[0]->getL2();
	energy_last = L1 * u0.head(3).dot(manipulators[0]->getKb1() * u0.head(3)) +
		L2 * u0.tail(3).transpose() * manipulators[0]->getKb2() * u0.tail(3);

	manipulators[1]->setLso(cur_qa[1][0]);
	u1 = Keith::Actuation2Curvature_keith(cur_qa[1], manipulators[1]->getGc());
	L1 = manipulators[1]->getL1();
	L2 = manipulators[1]->getL2();
	energy_last += L1 * u1.head(3).dot(manipulators[1]->getKb1() * u1.head(3)) +
		L2 * u1.tail(3).transpose() * manipulators[1]->getKb2() * u1.tail(3);

	Eigen::Vector12d d_e = Eigen::Vector12d::Zero();
	for (size_t i = 0; i < 12; i++) {
		int tem = i - 6;
		if (i < 6)
			qa[0][i] = qa[0][i] + del;
		else
			qa[1][tem] = qa[1][tem] + del;

		manipulators[0]->setLso(qa[0][0]);
		u0 = Keith::Actuation2Curvature_keith(qa[0], manipulators[0]->getGc());
		L1 = manipulators[0]->getL1();
		L2 = manipulators[0]->getL2();
		energy = L1 * u0.head(3).dot(manipulators[0]->getKb1() * u0.head(3)) +
			L2 * u0.tail(3).transpose() * manipulators[0]->getKb2() * u0.tail(3);

		manipulators[1]->setLso(qa[1][0]);
		u1 = Keith::Actuation2Curvature_keith(qa[1], manipulators[1]->getGc());
		L1 = manipulators[1]->getL1();
		L2 = manipulators[1]->getL2();
		energy += L1 * u1.head(3).dot(manipulators[1]->getKb1() * u1.head(3)) +
			L2 * u1.tail(3).transpose() * manipulators[1]->getKb2() * u1.tail(3);

		d_e[i] = energy - energy_last;

		if (i < 6)
			qa[0][i] = qa[0][i] - del;
		else
			qa[1][tem] = qa[1][tem] - del;
	}
	dE = d_e;
	return true;
}
