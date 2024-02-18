#include "function_in_DOM.h"

Eigen::Matrix3d Keith::skew_matrix(Eigen::Vector3d vec)
{
	Eigen::Matrix3d res;
	res << 0, -vec(2), vec(1),
		vec(2), 0, -vec(0),
		-vec(1), vec(0), 0;
	return res;
}

Eigen::Vector3d Keith::invSkewMatrix_keith(Eigen::Matrix3d mat1, Eigen::Matrix3d mat2)
{
	Eigen::Matrix3d mat = mat1 * mat2.transpose() - mat2 * mat1.transpose();
	Eigen::Vector3d p;
	p << mat(2, 1), mat(0, 2), mat(1, 0);
	return p;
}

Eigen::MatrixXd Keith::pinv(const Eigen::MatrixXd inMatrix)
{
	{
		double pinvtoler = 1e-6; // choose your tolerance wisely!
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(inMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::VectorXd singularValues_inv = svd.singularValues();
		Eigen::VectorXd sv = svd.singularValues();
		for (Eigen::Index i = 0; i < svd.cols(); ++i)
		{
			if (sv(i) > pinvtoler)
			{
				singularValues_inv(i) = 1.f / sv(i);
			}
			else
			{
				singularValues_inv(i) = 0;
			}
		}
		Eigen::MatrixXd outMatrix = (svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose());
		return outMatrix;
	}
}

Eigen::Matrix4d Keith::fromQuat2T(Eigen::Vector7d vec)
{
	Eigen::Quaterniond quat(vec.tail(4));
	Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
	res.topLeftCorner(3, 3) = quat.toRotationMatrix();
	res.topRightCorner(3, 1) = vec.head(3);
	return res;
}

Eigen::Vector12d Keith::shootingStiffOpt_keith(Eigen::Vector32d& Guess_sf, Eigen::Vector12d qa, Eigen::Matrix4d Tc, SurgicalContinuumManipulator& MBP1, SurgicalContinuumManipulator& MBP2, Eigen::Vector2d err)
{
	Eigen::Vector32d dGuess(Eigen::Vector32d::Zero());
	double lamda = 1e-6;
	double thre = 1e-4;


	return Eigen::Vector12d();
}
