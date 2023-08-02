#include "math_extentions_keith.h"
#include <iostream>

void keith_used::pinv(Eigen::MatrixXf& outMatrix, const Eigen::MatrixXf inMatrix)
{
	float pinvtoler = 1.e-6f; // choose your tolerance wisely!
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(inMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXf singularValues_inv = svd.singularValues();
	Eigen::VectorXf sv = svd.singularValues();
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
	outMatrix = (svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose());
	//return outMatrix;
}

void keith_used::pinv(Eigen::MatrixXd& outMatrix, const Eigen::MatrixXd& inMatrix)
{
	double pinvtoler = 1.e-6; // choose your tolerance wisely!
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
	outMatrix = (svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose());
	//return outMatrix;
}

std::vector<std::string> keith_used::split(const std::string& s, const std::string& seperator)
{
	std::vector<std::string> result;
	typedef std::string::size_type string_size;
	string_size i = 0;

	while (i != s.size()) {
		//找到字符串中首个不等于分隔符的字母；
		int flag = 0;
		while (i != s.size() && flag == 0) {
			flag = 1;
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[i] == seperator[x]) {
					++i;
					flag = 0;
					break;
				}
		}

		//找到又一个分隔符，将两个分隔符之间的字符串取出；
		flag = 0;
		string_size j = i;
		while (j != s.size() && flag == 0) {
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[j] == seperator[x]) {
					flag = 1;
					break;
				}
			if (flag == 0)
				++j;
		}
		if (i != j) {
			result.push_back(s.substr(i, j - i));
			i = j;
		}
	}
	return result;
}

Eigen::MatrixXd keith_used::readData(std::string path)
{
	std::fstream fread;
	fread.open(path);
	if (!fread.is_open())
	{
		std::cerr << "ERROR::OPEN_FILE_FAILED-> "<<path<<"!!!" << std::endl;
		return Eigen::Matrix2d::Zero();
	}
	char buf[500];
	Eigen::MatrixXd mat(100, 100);
	std::vector<std::string> res;
	unsigned int row_num,col_num,iter(0);
	while (fread.getline(buf, sizeof(buf) / sizeof(char)))
	{
		res = split(buf, " \t,;");
		col_num = res.size();
		Eigen::VectorXd vec(col_num, 1);
		for (size_t i = 0; i < res.size(); i++)
			vec[i] = std::atof(res[i].c_str());
		mat.block(iter++, 0, 1, col_num) = vec.transpose();
	}
	row_num = iter;
	return mat.topLeftCorner(row_num,col_num);
}

keith_used::TimerAvrg::TimerAvrg(int _n)
{
	 n = _n;
	times.reserve(n); 
}

double keith_used::TimerAvrg::getAvrg()
{
	double sum = 0; for (auto t : times) sum += t;
	return sum / double(times.size());
}
