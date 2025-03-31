#pragma once
#include <Eigen/Core>
//#include
class FContinuum {

public:
	FContinuum() {};
	void forwardStage();
	void backwardStage();
	void setL1r2g();

private:
	double L1 = 100.0;
	double Lr = 10.0;
	double L2 = 20.0;
	double Lg = 15.0;
};