#include "forward_kinematics_cosserat_parallel_continuum_robot.h"
//#include "continuum_segment.h"
#include <iostream>

int _main()
{
	std::string path("qa.log");
	Eigen::MatrixXd qa_mat;
	qa_mat  = keith_used::readData(path);
	Eigen::VectorXd qa1(6,1),qa2(6,1),qa(12, 1);
	qa1 = qa_mat.block(0, 0, 1, 6).transpose(); qa.head(6) = qa1;
	qa2 = qa_mat.block(1, 0, 1, 6).transpose(); qa.tail(6) = qa2;
	qa = qa / 1000.f;
	qa(0) = qa(0) * 1000.f / 180.f * keith_used::TSCR_PI;
	qa(6) = qa(6) * 1000.f / 180.f * keith_used::TSCR_PI;
	keith_used::TimerAvrg Fps;


	//Eigen::Vector3d t1, t2, t3;
	//t1 << 1, 0, 0;
	//t2 << 1, 0, 0;
	//t3 = t1.cross(t2);
	//std::cout << "t1: " << t1.transpose() << std::endl;
	//std::cout << "t2: " << t2.transpose() << std::endl;
	//std::cout << "t3: " << t3.transpose() << std::endl;
	//std::cout << "tttt: " << t1.cross(t2*1e9)/1e-9 << std::endl;



	ContinuumSegment seg1;
	ContinuumSegment seg2;
	seg2.setD(0.4e-3f);
	seg2.setK(0.6f);
	seg2.setDeltaT(-keith_used::TSCR_PI / 4.f);
	seg2.setL(0.02f);
	seg2.setL0(0.015f);
	seg2.setRho(2.7e-3f);
	seg2.refreshWholeStructure();
	
	
	Eigen::VectorXd FMfm(12, 1), ksi(8, 1),Rsd, yend,stiff,QA;
	Eigen::Matrix4d T_base, T_base2,Tend;
	
	
	T_base = keith_used::readData("T_base1.log");
	T_base2 = keith_used::readData("T_base2.log");
	FMfm = Eigen::VectorXd::Zero(12,1);
	ksi = Eigen::VectorXd::Zero(12, 1);
	Eigen::Matrix3f Ke(seg1.getKe()), Kb(seg1.getKb());
	ksi(0) = Ke(2, 2); ksi(1) = Kb(0, 0); ksi(2) = Kb(1, 1); ksi(3) = Kb(2, 2);
	ksi(4) = Ke(2, 2); ksi(5) = Kb(0, 0); ksi(6) = Kb(1, 1); ksi(7) = Kb(2, 2);
	ksi(8) = seg1.getK(); ksi(9) = seg2.getK(); ksi(10) = ksi(8); ksi(11) = ksi(9);
	TwoSegmentMultiBackboneContinuumRobot tcr1(seg1, seg2);
	tcr1.showAllParameter();
	tcr1.resetLso(qa(1));
	TwoSegmentMultiBackboneContinuumRobot tcr2 = tcr1;
	tcr2.showAllParameter();
	tcr2.resetLso(qa(7));
	FKCoPCR fkco;
	Fps.start();
	Rsd = fkco.shootOptimization(qa,T_base,T_base2,FMfm,tcr1,tcr2,ksi,yend);
	Fps.stop();
	std::cout << "\rTime Optimization =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl;
	Tend = Eigen::Matrix4d::Identity();
	Tend.block(0, 0, 3, 1) = yend.block(3, 0, 3, 1);
	Tend.block(0, 1, 3, 1) = yend.block(6, 0, 3, 1);
	Tend.block(0, 2, 3, 1) = yend.block(9, 0, 3, 1);
	Tend.block(0, 3, 3, 1) = yend.block(0, 0, 3, 1);
	std::cout << "Tend: " << Tend << std::endl;

	Eigen::VectorXd pose = fkco.getPosefromVec6(yend);
	//Fps.start();
	//stiff = fkco.optimizeStiffness(pose, qa, T_base, T_base2, FMfm, tcr1, tcr2, ksi*2, yend);
	//Fps.stop();
	//std::cout << "\rTime Optimization =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl;
	//std::cout << "ksi: " << ksi.transpose() << std::endl;
	//std::cout << "stiff: " << stiff.transpose() << std::endl;

	Fps.start();
	QA = fkco.optimizeActuation(pose, qa*1.1, T_base, T_base2, FMfm, tcr1, tcr2, ksi, yend);
	Fps.stop();
	std::cout << "\rTime Optimization =" << Fps.getAvrg() * 1000 << " ms\n" << std::endl;
	std::cout << "ksi: " << ksi.transpose() << std::endl;
	std::cout << "stiff: " << stiff.transpose() << std::endl;

	return 0;
}