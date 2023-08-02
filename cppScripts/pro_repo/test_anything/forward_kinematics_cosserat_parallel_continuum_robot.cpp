#include "forward_kinematics_cosserat_parallel_continuum_robot.h"
#include <Eigen/Geometry>
#include <iostream>

FKCoPCR::FKCoPCR()
{
	guess0 = Eigen::VectorXd::Zero(20, 1);
	guess0 << 2.585357, 0.86919, 1.231203, 0.053751, 0.013668,
		-0.009253, -0.001128, -0.000963, -0.000221, -7.8e-05,
		-2.572132, -0.910867, -1.229551, 0.077246, -0.20064,
		-0.026386, -0.000417, -0.000142, -5.1e-05, -9e-06;
}


Eigen::VectorXd FKCoPCR::shootOptimization(Eigen::VectorXd qa, Eigen::Matrix4d T_base, Eigen::Matrix4d T_base2, 
	Eigen::VectorXd FMfm, TwoSegmentMultiBackboneContinuumRobot tcr1,
	TwoSegmentMultiBackboneContinuumRobot tcr2, Eigen::VectorXd ksi, Eigen::VectorXd& yend)
{
	Eigen::MatrixXd dGuess(20, 20),I20(20,20),tem(20,20),J(20,20);
	dGuess = Eigen::MatrixXd::Identity(20, 20) * 1e-9;
	I20 = Eigen::MatrixXd::Identity(20, 20);
	double lambda(1e-6);
	double eps(2e-5);
	Eigen::VectorXd guess(20,1),Rsd(20,1),Rsd_(20,1);
	//guess = guess0;
	guess = Eigen::MatrixXd::Zero(20, 1);
	Rsd = shootMethod(guess,qa, T_base, T_base2,FMfm,tcr1,tcr2,ksi,yend);
	unsigned int iter = 0;
	Eigen::Index rsd_size = Rsd.size();
	//std::cout << " ------> ksi :" << ksi.transpose() <<"  "<<tcr1.getK1() << "  " << tcr1.getK2()
	//	<< "  " << tcr2.getK1() << "  " << tcr2.getK2() << std::endl;

	while (Rsd.norm() > eps)
	{
		for (Eigen::Index i = 0; i < rsd_size; i++)
		{
			Rsd_ = shootMethod(guess + dGuess.block(0, i, rsd_size, 1), qa, T_base, T_base2, FMfm, tcr1, tcr2, ksi, yend);
			//std::cout << "Rsd_ :" << ((Rsd_ - Rsd) / dGuess.block(0, i, Rsd.size(), 1).norm()).transpose() << std::endl;
			J.block(0, i, rsd_size, 1) = (Rsd_ - Rsd) / dGuess.block(0, i, rsd_size, 1).norm();
		}
		tem = J.transpose() * J + lambda * I20;
		keith_used::pinv(tem, tem);
		guess = guess - I20 * tem * J.transpose() * Rsd;
		Rsd = shootMethod(guess, qa, T_base, T_base2, FMfm, tcr1, tcr2, ksi, yend);
		guess0 = guess;
		if (++iter > 10)
		{
			std::cout << "WARNING::UNCONVERGENCE" << iter << std::endl;
			break;
		}
	}
	return Rsd;
}

Eigen::VectorXd FKCoPCR::shootMethod(Eigen::VectorXd guess, Eigen::VectorXd qa, Eigen::Matrix4d T_base,
	Eigen::Matrix4d T_base2, Eigen::VectorXd FMfm, TwoSegmentMultiBackboneContinuumRobot tcr1, 
	TwoSegmentMultiBackboneContinuumRobot tcr2, Eigen::VectorXd ksi, Eigen::VectorXd& yend)
{
	Eigen::VectorXd nm1(6, 1), nm2(6, 1);
	nm1 = guess.block<6, 1>(0, 0);
	nm2 = guess.block<6, 1>(10, 0);
	Eigen::MatrixXd v1(3, 4),v2(3,4);
	v1 = Eigen::MatrixXd::Zero(3, 4); v2 = Eigen::MatrixXd::Zero(3, 4);
	v1.block<1, 4>(2, 0) = guess.block<4, 1>(6, 0).transpose();
	v2.block<1, 4>(2, 0) = guess.block<4, 1>(16, 0).transpose();
	Eigen::VectorXd fm = FMfm.block<6, 1>(6, 0);
	Eigen::Vector4d ksi1,ksi2;
	Eigen::VectorXd qaaa1(8,1), qaaa2(8, 1);
	ksi1 = ksi.block<4, 1>(0, 0); ksi2 = ksi.block<4, 1>(4, 0);
	Eigen::VectorXd y10(22, 1), y20(22, 1), y13(19, 1), y23(19, 1);

	Eigen::Matrix3d R_phi;
	R_phi = Eigen::AngleAxisd(qa(0), Eigen::Vector3d::UnitZ());
	T_base.block<3, 3>(0, 0) = T_base.block<3, 3>(0, 0) * R_phi;
	y10 << T_base.block<3, 1>(0, 3), T_base.block<3, 1>(0, 0), T_base.block<3, 1>(0, 1), T_base.block<3, 1>(0, 2), nm1, 0, 0, 0, 0;
	y13 = shootOneRobot(y10, fm, v1, 1, tcr1, ksi1, qaaa1);
	//std::cout << "y10 :" << y10.transpose() << std::endl;
	//std::cout << "y13 :" << y13.transpose() << std::endl;
	R_phi = Eigen::AngleAxisd(qa(6), Eigen::Vector3d::UnitZ());
	T_base2.block<3, 3>(0, 0) = T_base2.block<3, 3>(0, 0) * R_phi;
	y20 << T_base2.block<3, 1>(0, 3), T_base2.block<3, 1>(0, 0), T_base2.block<3, 1>(0, 1), T_base2.block<3, 1>(0, 2), nm2, 0, 0, 0, 0;
	y23 = shootOneRobot(y20, fm, v2, 0, tcr2, ksi2, qaaa2);
	//std::cout << "y23 :" << y23.transpose() << std::endl;

	Eigen::VectorXd Rsd(20, 1);
	Eigen::Matrix3d R1, R2, Rtem;
	Eigen::Vector3d r1, r2, r3;
	r1 = y13.block<3, 1>(3, 0); r2 = y13.block<3, 1>(6, 0); r3 = y13.block<3, 1>(9, 0);
	R1.col(0) = r1; R1.col(1) = r2; R1.col(2) = r3;
	Eigen::Quaterniond tem;
	tem= R1;
	tem = tem.normalized();
	R1 = tem.toRotationMatrix();


	r1 = y23.block<3, 1>(3, 0); r2 = y23.block<3, 1>(6, 0); r3 = y23.block<3, 1>(9, 0);
	R2.col(0) = r1; R2.col(1) = r2; R2.col(2) = r3;
	tem = R2;
	tem = tem.normalized();
	R2 = tem.toRotationMatrix();

	Rtem = Eigen::AngleAxisd(keith_used::TSCR_PI, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd axang(R2.transpose() * R1 * Rtem);

	Eigen::Vector3d f_err, m_err, p_err, o_err;
	Eigen::Vector2d v11_err, v12_err, v21_err, v22_err;
	f_err = FMfm.block<3, 1>(0, 0) - y13.block<3, 1>(12, 0) - y23.block<3, 1>(12, 0);
	m_err = FMfm.block<3, 1>(3, 0) - y13.block<3, 1>(15, 0) - y23.block<3, 1>(15, 0);
	p_err = y13.block<3, 1>(0, 0) - y23.block<3, 1>(0, 0);
	o_err = axang.axis() * axang.angle();
	v11_err = qaaa1.block<2, 1>(0, 0) - (qa.block<2, 1>(2, 0).cast <double>() + qaaa1.block<2, 1>(2, 0));
	v12_err = qaaa1.block<2, 1>(4, 0) - (qa.block<2, 1>(4, 0).cast <double>() + qaaa1.block<2, 1>(6, 0));
	v21_err = qaaa2.block<2, 1>(0, 0) - (qa.block<2, 1>(8, 0).cast <double>() + qaaa2.block<2, 1>(2, 0));
	v22_err = qaaa2.block<2, 1>(4, 0) - (qa.block<2, 1>(10, 0).cast <double>() + qaaa2.block<2, 1>(6, 0));
	Rsd.block<3, 1>(0, 0) = f_err;
	Rsd.block<3, 1>(3, 0) = m_err;
	Rsd.block<3, 1>(6, 0) = p_err;
	Rsd.block<3, 1>(9, 0) = o_err;
	Rsd.block<2, 1>(12, 0) = v11_err;
	Rsd.block<2, 1>(14, 0) = v12_err;
	Rsd.block<2, 1>(16, 0) = v21_err;
	Rsd.block<2, 1>(18, 0) = v22_err;
	yend = y23;
	return Rsd;
}

Eigen::VectorXd FKCoPCR::shootOneRobot(Eigen::VectorXd y, Eigen::VectorXd fm, Eigen::MatrixXd v,bool is_main,
	TwoSegmentMultiBackboneContinuumRobot tcr, Eigen::VectorXd ksi, Eigen::VectorXd& qaaa)
{
	Eigen::VectorXd t0, t1, t11, t2, t22, t3;
	Eigen::MatrixXd u0, u1, u2, u3, y0, y1,y11, y2,y22, y3;
	Eigen::VectorXd y0_(22,1), y1_(20, 1), y2_(19, 1), y_new1(22,1), y_new2(20,1);
	Eigen::Matrix3d R1, R2, Rtem, R2_;
	Eigen::VectorXd qaa(8, 1);
	Eigen::Quaterniond tem;

	// 切缝钢管段
	if (tcr.getLso() > 0)
	{
		t0 = integrateCosseratRod(y,v,0,tcr,fm,ksi,y0,u0);
		y0_ = y0.bottomRows<1>().transpose();
	}
	else
	{
		Eigen::VectorXd  tt(1,1);
		tt << 0;
		t0 = tt;
		y0_ = y;
	}

	// 第一柔性段
	t11 = integrateCosseratRod(y0_, v, 1, tcr, fm, ksi, y11, u1);	
	y_new1 = y11.bottomRows<1>().transpose();

	std::cout << "y :" << y.transpose() << std::endl;

	R1 << y_new1.block<3, 1>(3, 0), y_new1.block<3, 1>(6, 0), y_new1.block<3, 1>(9, 0);
	tem = R1;tem = tem.normalized();R1 = tem.toRotationMatrix();

	y_new1.head(3) = y_new1.head(3) + R1 * Eigen::Vector3d{0.f,0.f,tcr.getLr()};

	y1 = Eigen::MatrixXd::Zero(y11.rows() + 1, y11.cols());
	y1.topRows(y11.rows()) = y11; y1.bottomRows<1>() = y_new1.transpose();
	t1 = Eigen::VectorXd::Zero(t11.size() + 1, 1);
	t1 << t11, tcr.getLr();
	Eigen::Vector3d moment1, moment2, moment3, moment;
	Eigen::Vector3d v0=v.block<3, 1>(0, 0), v1=v.block<3, 1>(0, 1),
		v_lr=Eigen::Vector3d{ 0.f,0.f,tcr.getLr() },y_new1_12_3=y_new1.block<3, 1>(12, 0);
	moment1 = (R1 * v_lr).cross(y_new1_12_3);
	moment2 = (R1 * tcr.seg1.getR1().cast <double>()).cross(R1 * tcr.getKe1().cast <double>() * v0);
	moment3 =(R1 * tcr.seg1.getR2().cast <double>()).cross(R1 * tcr.getKe1().cast <double>() * v1);
	moment = y_new1.block<3, 1>(15, 0) + moment1 + 2 * moment2 + 2 * moment3;
	y1_ << y_new1.head(15), moment, y_new1.block<2, 1>(20, 0);
	//std::cout << "y1_: " << y1_.transpose() << std::endl;
	// 第二柔性段
	t22 = integrateCosseratRod(y1_, v, 2, tcr, fm, ksi, y22, u2);
	y_new2 = y22.bottomRows<1>().transpose();

	//std::cout << "y_new2 :" << y_new2.transpose() << std::endl;

	R2 << y_new2.block<3, 1>(3, 0), y_new2.block<3, 1>(6, 0), y_new2.block<3, 1>(9, 0);
	tem = R2;
	tem = tem.normalized();
	R2 = tem.toRotationMatrix();

	y_new2.head(3) = y_new2.head(3) + R2 * Eigen::Vector3d{ 0.f,0.f,tcr.getLg() };
	y2 = Eigen::MatrixXd::Zero(y22.rows() + 1, y22.cols());
	y2.topRows(y22.rows()) = y22; y2.bottomRows<1>() = y_new2.transpose();
	t2 = Eigen::VectorXd::Zero(t22.size() + 1, 1);
	t2 << t22, tcr.getLg();
	Eigen::Vector3d y_new2_12_3(y_new2.block<3, 1>(12, 0)),v2(v.block<3, 1>(0, 2)),v3(v.block<3, 1>(0, 3));
	moment1 = (R2 * Eigen::Vector3d{ 0.f,0.f,tcr.getLg() }).cross(y_new2_12_3);
	moment2 = (R2 * tcr.seg2.getR1().cast <double>()).cross(R2 * tcr.getKe2().cast <double>() * v2);
	moment3 = (R2 * tcr.seg2.getR2().cast <double>()).cross(R2 * tcr.getKe2().cast <double>() * v3);
	moment = y_new2.block<3, 1>(15, 0) + moment1 + 2 * moment2 + 2 * moment3;

	if (is_main)
		Rtem = Eigen::AngleAxisd(-keith_used::TSCR_PI / 2, Eigen::Vector3d::UnitY());
	else
		Rtem = Eigen::AngleAxisd(keith_used::TSCR_PI / 2, Eigen::Vector3d::UnitY());
	R2_ = R2 * Rtem;

	y2_ << y_new2.head(3), R2_.col(0), R2_.col(1), R2_.col(2),
		y_new2.block<3, 1>(12, 0), moment, 0;
	//std::cout << "y2_: " << y2_.transpose() << std::endl;
	// 第三柔性段
	t3 = integrateCosseratRod(y2_, v, 3, tcr, fm, ksi, y3, u3);

	qaa.head(2) = y_new1.block<2, 1>(18, 0);
	qaa.block<2, 1>(2, 0) = ((tcr.getLstem() + tcr.getL1())) * v.block<1, 2>(2, 0).transpose();
	qaa.block<2, 1>(4, 0) = y_new2.block(18, 0, 2, 1);
	qaa.block<2, 1>(6, 0) = ((tcr.getLstem() + tcr.getL1() + tcr.getLr() + tcr.getL2())) * v.block<1, 2>(2, 2).transpose();
	qaaa = qaa;
	//std::cout << "y3_: " << y3.bottomRows<1>() << std::endl;
	return y3.bottomRows<1>().transpose();
}

Eigen::VectorXd FKCoPCR::integrateCosseratRod(Eigen::VectorXd y0, Eigen::MatrixXd v, int segIdx, 
	TwoSegmentMultiBackboneContinuumRobot tcr, Eigen::VectorXd fm, Eigen::VectorXd ksi, Eigen::MatrixXd& y, Eigen::MatrixXd& U)
{
	Eigen::Index DoF = y0.size();
	Eigen::Matrix3d Kb1, Ke1, Kb2, Ke2, Kb, Ke, Kb_, Ke_;
	Eigen::MatrixXd Q;
	int N, mod_SegIdx;
	double step, zeta;
	Kb1 = tcr.getKb1().cast <double>();
	Kb2 = tcr.getKb2().cast <double>();
	Ke1 = tcr.getKe1().cast <double>();
	Ke2 = tcr.getKe2().cast <double>();
	switch (segIdx)
	{
	case 0:
	{
		Q = Eigen::MatrixXd::Zero(3, 4);
		Q = tcr.Q1.cast <double>();
		N = ceil(tcr.getLso() / tcr.getDiscreteElement());
		mod_SegIdx = 1;
		zeta = tcr.getZeta();
		Kb = (4 * Kb1 + 16 * Kb2 + tcr.getK1() * Kb1) / zeta;
		Kb_ = Kb.inverse();
		step = (double)(tcr.getLso() / (N - 1));
		break;
	}
	case 1:
	{
		Q = Eigen::MatrixXd::Zero(3, 4);
		Q = tcr.Q1.cast <double>();
		N = ceil(tcr.getL1o() / tcr.getDiscreteElement());
		mod_SegIdx = 1;
		Kb = 4 * Kb1 + 16 * Kb2 + tcr.getK1() * Kb1;
		Kb_ = Kb.inverse();
		step = (double)(tcr.getL1o() / (N - 1));
		break;
	}
	case 2:
	{
		Q = Eigen::MatrixXd::Zero(3, 2);
		Q = tcr.Q2.cast <double>();
		N = ceil(tcr.getL2() / tcr.getDiscreteElement());
		mod_SegIdx = 0;
		Kb = 16 * Kb2 + tcr.getK2() * Kb1;
		Kb_ = Kb.inverse();
		step = (double)(tcr.getL2() / (N - 1));
		break;
	}
	case 3:
	{
		Q = Eigen::MatrixXd::Zero(3, 2);
		Q = tcr.Q2.cast <double>();
		N = ceil(tcr.getL_do() / tcr.getDiscreteElement());
		Kb = Eigen::Matrix3d::Identity();
		Kb(0, 0) = ksi(1); Kb(1, 1) = ksi(2); Kb(2, 2) = ksi(3);
		Kb_ = Kb.inverse();
		Ke = Eigen::Matrix3d::Identity();
		Ke(0, 0) = ksi(0) * 10000; Ke(1, 1) = ksi(0) * 10000; Ke(2, 2) = ksi(0);
		Ke_ = Ke.inverse();
		step = (double)(tcr.getL_do() / (N - 1));
		break;
	}
	}
	std::cout << "step: "<<step << std::endl;
	std::cout << "N: " << N << std::endl;
	Eigen::MatrixXd yc(N, DoF), Uc(N, 3);
	yc.topRows<1>() = y0.transpose();

	Eigen::Vector3d p, n, m, p_dot, n_dot, m_dot;
	Eigen::VectorXd q, q_dot, t(N, 1);
	Eigen::Matrix3d R, R_dot;
	Eigen::Vector3d u;
	Eigen::Quaterniond tem;

	double v_do(0), theta(0), delta(0), phi(0);
	Eigen::Vector3d R11(tcr.seg1.getR1().cast <double>()), R12(tcr.seg1.getR2().cast <double>()),
		R21(tcr.seg2.getR1().cast <double>()), R22(tcr.seg2.getR2().cast <double>());
	Eigen::Vector3d ee1, ee2, ee3, ee4, ee5, ee6, ee7, ee8;
	Eigen::Vector3d v0(v.col(0)), v1(v.col(1)), v2(v.col(2)), v3(v.col(3));
	v0 = Ke1 * v0;
	v1 = Ke1 * v1;
	v2 = Ke2 * v2;
	v3 = Ke2 * v3;

	for (size_t i = 0; i < N - 1; i++)
	{
		p = yc.block(i, 0, 1, 3).transpose();
		R.col(0) = yc.block<1, 3>(i, 3).transpose(); R.col(1) = yc.block<1, 3>(i, 6).transpose();
		R.col(2) = yc.block<1, 3>(i, 9).transpose();
		tem = R;
		tem = tem.normalized();
		R = tem.toRotationMatrix();

		n = yc.block<1, 3>(i, 12).transpose();
		m = yc.block<1, 3>(i, 15).transpose();
		q = yc.block(i, 18, 1, DoF - 18).transpose();

		if (segIdx < 3)
		{
			u = Kb_ * R.transpose() * m; u(2) = 0;
			double theta = step * u.norm();
			double delta = -atan2(u(1), u(0)) + keith_used::TSCR_PI / 2;
			double ct, st, cd, sd;
			ct = cos(theta); st = sin(theta); cd = cos(delta); sd = sin(delta);
			if (abs(theta) > 1e-15)
			{
				p_dot = R * Eigen::Vector3d{ cd * (1 - ct), sd * (ct - 1), st }*(step / theta);
				R_dot << pow(cd, 2) * ct + pow(sd, 2), -sd * cd * (ct - 1), cd* st,
					sd* cd* (1 - ct), pow(cd, 2) + ct * pow(sd, 2), -sd * st,
					-cd * st, sd* st, ct;
				tem = R_dot;
				tem = tem.normalized();
				R_dot = tem.toRotationMatrix();
			}
			else
			{
				p_dot = R * Eigen::Vector3d{ 0.f,0.f,step };
				R_dot = Eigen::Matrix3d::Identity();
			}
			p = p + p_dot;
			R = R * R_dot;
			n_dot = Eigen::Vector3d::Zero();
			ee1 = u.cross(R11); ee1 = ee1.cross(v0);
			ee2 = R11.cross(u.cross(v0));
			ee3 = u.cross(R12); ee3 = ee3.cross(v1);
			ee4 = R12.cross(u.cross(v1));

			ee5 = u.cross(R21); ee5 = ee5.cross(v2);
			ee6 = R21.cross(u.cross(v2));
			ee7 = u.cross(R22); ee7 = ee7.cross(v3);
			ee8 = R22.cross(u.cross(v3));
			m_dot = -p_dot.cross(n) -
				2 * R * ((ee1 + ee2 + ee3 + ee4) * mod_SegIdx + ee5 + ee6 + ee7 + ee8) * step;
			n = n + n_dot;
			m = m + m_dot;
			q_dot = Q.transpose() * u * step;
			q = q + q_dot;
		}
		else
		{
			u = Kb_ * R.transpose() * m;
			Eigen::Vector3d v = Ke_ * R.transpose() * n;
			v_do = v(2)*step;
			theta = step * u.head(2).norm();
			delta = -atan2(u(1), u(0)) + keith_used::TSCR_PI / 2;
			phi = step * u(2);
			Eigen::Matrix3d R_phi;
			R_phi = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ());
			double ct, st, cd, sd;
			ct = cos(theta); st = sin(theta); cd = cos(delta); sd = sin(delta);
			if (abs(theta) > 1e-15)
			{
				p_dot = R * Eigen::Vector3d{ cd * (1 - ct), sd * (ct - 1), st }*((step + v_do) / theta);
				R_dot << pow(cd, 2) * ct + pow(sd, 2), -sd * cd * (ct - 1), cd* st,
					sd* cd* (1 - ct), pow(cd, 2) + ct * pow(sd, 2), -sd * st,
					-cd * st, sd* st, ct;
				R_dot = R_dot * R_phi;
				tem = R_dot;
				tem = tem.normalized();
				R_dot = tem.toRotationMatrix();
			}
			else
			{
				p_dot = R * Eigen::Vector3d{ 0.f,0.f,step + v_do };
				R_dot = Eigen::Matrix3d::Identity() * R_phi;
			}
			p = p + p_dot;
			R = R * R_dot;
			n_dot = Eigen::Vector3d::Zero();
			m_dot = -p_dot.cross(n);
			n = n + n_dot;
			m = m + m_dot;
			q_dot = Eigen::VectorXd::Zero(1, 1);
			q_dot(0) = v_do;
			q = q + q_dot;
		}
		yc.block<1, 3>(i + 1, 0) = p.transpose();
		yc.block<1, 3>(i + 1, 3) = R.col(0).transpose();
		yc.block<1, 3>(i + 1, 6) = R.col(1).transpose();
		yc.block<1, 3>(i + 1, 9) = R.col(2).transpose();
		yc.block<1, 3>(i + 1, 12) = n.transpose();
		yc.block<1, 3>(i + 1, 15) = m.transpose();
		yc.block(i + 1, 18, 1, DoF - 18) = q.transpose();
		Uc.block<1, 3>(i, 0) = u.transpose();
		t(i) = u.norm();
		//std::cout << "yc.row("<<i+1<<"): " << std::endl << yc.row(i+1) << std::endl;
	}
	y = yc;
	U = Uc;
	return t;
}

void FKCoPCR::setGuess0(Eigen::VectorXd guess0)
{
	this->guess0 = guess0;
}

Eigen::VectorXd FKCoPCR::optimizeStiffness(Eigen::VectorXd pose,Eigen::VectorXd qa, Eigen::Matrix4d T_base, Eigen::Matrix4d T_base2,
	Eigen::VectorXd FMfm, TwoSegmentMultiBackboneContinuumRobot tcr1,
	TwoSegmentMultiBackboneContinuumRobot tcr2, Eigen::VectorXd ksi, Eigen::VectorXd& yend)
{
	Eigen::MatrixXd I20, tem(12, 12), tem_inv(12,12), J(6, 12),aksi(12,1), aksi_(12, 1),ksi_(8,1), dKsi(12, 1);
	Eigen::VectorXd k_ratio;
	k_ratio = Eigen::VectorXd::Ones(12, 1) * 1e-3;
	k_ratio(0) = k_ratio(0) * 1e5; k_ratio(4) = k_ratio(4) * 1e2;
	double lambda(5e-7);
	double eps(1e-4);
	Eigen::VectorXd guess(20, 1), Rsd(6, 1), Rsd_(6, 1), Rsd__(6, 1);
	Eigen::AngleAxisd axang;
	Eigen::Matrix3d rot;
	I20 = Eigen::MatrixXd::Identity(12, 12);
	tcr1.resetK12(ksi.block<2, 1>(8, 0).cast<float>()); tcr2.resetK12(ksi.block<2, 1>(10, 0).cast<float>());
	shootOptimization(qa, T_base, T_base2, FMfm, tcr1, tcr2, ksi, yend);
	guess = guess0;
	Rsd_ = getPosefromVec6(yend);
	Rsd = getPoseDeviationfromVec6(Rsd_, pose);
	std::cout << "Rsd_ :" << Rsd_.transpose() << std::endl;
	std::cout << "pose :" << pose.transpose() << std::endl;
	std::cout << "Rsd :" << Rsd.transpose() << std::endl;
	Rsd.tail(3) /= 10;
	unsigned int iter_ = 0;
	Eigen::Index aksi_size = aksi.size(), rsd_size = Rsd.size();
	aksi = ksi;
	aksi_ = aksi;
	while (Rsd.norm() > eps)
	{
		for (Eigen::Index i = 0; i < aksi_size; i++)
		{
			aksi_(i) += k_ratio(i)* aksi(i);
			tcr1.resetK12(aksi_.block<2, 1>(8, 0).cast<float>());
			tcr2.resetK12(aksi_.block<2, 1>(10, 0).cast<float>());
			shootOptimization(qa, T_base, T_base2, FMfm, tcr1, tcr2, aksi_, yend);
			Rsd__ = getPosefromVec6(yend);
			//std::cout << "Rsd :" << Rsd_.transpose() << std::endl;
			Rsd__ = getPoseDeviationfromVec6(Rsd__, Rsd_);
			std::cout << "two error :" << Rsd__.transpose() << std::endl;
			J.block<6, 1>(0, i) = Rsd__ / k_ratio(i);
			aksi_(i) = aksi(i);
			tcr1.resetK12(aksi.block<2, 1>(8, 0).cast<float>());
			tcr2.resetK12(aksi.block<2, 1>(10, 0).cast<float>());
		}
		tem = J.transpose() * J + lambda * I20;
		keith_used::pinv(tem_inv, tem);
		dKsi = (I20 * tem_inv * J.transpose() * Rsd).array()*aksi.array();
		aksi = aksi - dKsi;
		aksi_ = aksi;
		for (Eigen::Index i = 0; i < aksi_size; i++)
			if (aksi(i) < 1e-3)
				aksi(i) = 1e-3;
		std::cout << "dKsi :" << dKsi.transpose() << std::endl;
		tcr1.resetK12(aksi.block<2, 1>(8, 0).cast<float>());
		tcr2.resetK12(aksi.block<2, 1>(10, 0).cast<float>());
		guess0 = guess;
		shootOptimization(qa, T_base, T_base2, FMfm, tcr1, tcr2, aksi, yend);
		Rsd_ = getPosefromVec6(yend);
		Rsd = getPoseDeviationfromVec6(Rsd_, pose);
		Rsd.tail(3) /= 10;
		if (++iter_ > 10)
		{
			std::cout << "WARNING::UNCONVERGENCE" << iter_ << std::endl;
			//break;
		}
		guess = guess0;

		std::cout << " -> Iteration: " << iter_ << ". And the residue is :" << Rsd.norm() << std::endl;
	}
	
	return aksi;
}

Eigen::VectorXd FKCoPCR::optimizeActuation(Eigen::VectorXd pose, Eigen::VectorXd qa, Eigen::Matrix4d T_base, Eigen::Matrix4d T_base2,
	Eigen::VectorXd FMfm, TwoSegmentMultiBackboneContinuumRobot tcr1,
	TwoSegmentMultiBackboneContinuumRobot tcr2, Eigen::VectorXd ksi, Eigen::VectorXd& yend)
{
	Eigen::MatrixXd I20, tem(12, 12), J(6, 12), aqa(12, 1), aqa_(12, 1), dQa(12, 1);
	Eigen::VectorXd qa_ratio;
	qa_ratio = Eigen::VectorXd::Ones(12, 1) * 1e-5;
	double lambda(5e-7);
	double eps(1e-4);
	Eigen::VectorXd guess(20, 1), Rsd(6, 1), Rsd_(6, 1), Rsd__(6,1);
	Eigen::AngleAxisd axang;
	Eigen::Matrix3d rot;
	I20 = Eigen::MatrixXd::Identity(12, 12);
	tcr1.resetLso(qa(1));	tcr2.resetLso(qa(7));
	shootOptimization(qa, T_base, T_base2, FMfm, tcr1, tcr2, ksi, yend);
	guess = guess0;
	Rsd_ = getPosefromVec6(yend);
	Rsd = getPoseDeviationfromVec6(Rsd_, pose);
	//Rsd.tail(3) /= 10;
	unsigned int _iter_ = 0;
	Eigen::Index rsd_size = Rsd.size();
	aqa = qa;
	aqa_ = aqa;
	while (Rsd.norm() > eps)
	{
		for (Eigen::Index i = 0; i < rsd_size; i++)
		{
			aqa_(i) = qa_ratio(i) + aqa(i);
			tcr1.resetLso(aqa_(1));
			tcr2.resetLso(aqa_(7));
			guess0 = guess;
			shootOptimization(aqa_, T_base, T_base2, FMfm, tcr1, tcr2, ksi, yend);
			Rsd__ = getPosefromVec6(yend);
			Rsd__ = getPoseDeviationfromVec6(Rsd__, Rsd_);
			J.block<6, 1>(0, i) = Rsd__ / qa_ratio(i);
			aqa_(i) = aqa(i);
			tcr1.resetLso(aqa(1));
			tcr2.resetLso(aqa(7));
		}
		tem = J.transpose() * J + lambda * I20;
		keith_used::pinv(tem, tem);
		dQa = tem * J.transpose() * Rsd;
		aqa = aqa - 0.1 * dQa;
		aqa_ = aqa;
		std::cout << "->dQa :" << 0.1 * dQa.transpose() << std::endl;
		std::cout << "->aqa :" << aqa.transpose() << std::endl;
		tcr1.resetLso(aqa(1));
		tcr2.resetLso(aqa(7));
		guess0 = guess;
		shootOptimization(aqa, T_base, T_base2, FMfm, tcr1, tcr2, ksi, yend);
		Rsd_ = getPosefromVec6(yend);
		Rsd = getPoseDeviationfromVec6(Rsd_, pose);
		//Rsd.tail(3) /= 10;
		if (++_iter_ > 10)
		{
			std::cout << "WARNING::UNCONVERGENCE" << _iter_ << std::endl;
			//break;
		}
		guess = guess0;
		std::cout << " -> Iteration: " << _iter_ << ". And the residue is :" << Rsd.norm() << std::endl;
	}
	qa = aqa;
	return aqa;
}

Eigen::VectorXd FKCoPCR::getPosefromVec6(Eigen::VectorXd yend)
{
	Eigen::VectorXd Rsd_(6, 1);
	Eigen::Matrix3d rot;
	Rsd_.head(3) = yend.head(3);
	rot << yend.block<3, 1>(3, 0), yend.block<3, 1>(6, 0), yend.block<3, 1>(9, 0);
	Eigen::AngleAxisd axang(keith_used::normalizeRotation(rot));
	Rsd_.tail(3) = axang.axis() * axang.angle();
	return Rsd_;
}

Eigen::VectorXd FKCoPCR::getPoseDeviationfromVec6(Eigen::VectorXd vec1, Eigen::VectorXd vec2)
{
	Eigen::VectorXd Rsd(6, 1);
	Rsd.head(3) = vec2.head(3) - vec1.head(3);
	double ang1 = vec1.tail(3).norm();
	double ang2 = vec2.tail(3).norm();
	Eigen::AngleAxisd axang1(ang1, vec1.tail(3) / ang1), axang2(ang2, vec2.tail(3) / ang2),axang3;
	axang3 = axang1.toRotationMatrix().transpose() * axang2.toRotationMatrix();
	Rsd.tail(3) = axang3.axis() * axang3.angle();
	return Rsd;
}
