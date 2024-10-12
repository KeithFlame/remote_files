#pragma once
//#include <Eigen/dense>

typedef struct _MODULE_NITINOL_ {
	double E = 50e9;
	double mu = 0.25;
	double G = E / 2 / (1 + mu);
}MODULE_NITINOL;
