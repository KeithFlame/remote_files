#include "function_in_DOM.h"

Eigen::Vector12d Keith::shootingStiffOpt_keith(Eigen::Vector34d& Guess_sf, Eigen::Vector12d qa, Eigen::Matrix4d Tc, SurgicalContinuumManipulator& MBP1, SurgicalContinuumManipulator& MBP2, Eigen::Vector2d err)
{
	Eigen::Vector34d dGuess(Eigen::Vector34d::Zero());
	double lamda = 1e-6;
	double thre = 1e-4;


	return Eigen::Vector12d();
}
